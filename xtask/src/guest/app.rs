//! Stela.app, the macOS utility in `guest/stela-app`, as the StelaGPU kext
//! delivers it to `/Applications` (`guest/stela-gpu/kext/app_delivery.h`).
//!
//! SwiftUI does not cross-build on Linux, so the app comes prebuilt: the
//! bundle `STELA_APP` names, else the newest
//! `target/stela-app/package.*/Stela.app` that `guest/stela-app/build.sh`
//! or [`build`] left. Without one the guest build still succeeds and the
//! kext reports that its build carries no app. A package is checked against
//! its own code signature, then written to `stela/app.h` as the kext embeds
//! it: each file's digest, a zip of the bundle with its modes, and the plist
//! of the one-shot launchd job that installs it.
use super::{c_bytes, c_digest};
use anyhow::{Context, Result, bail, ensure};
use std::{
    collections::BTreeMap,
    env,
    ffi::OsString,
    fmt::Write as _,
    fs,
    io::{Cursor, Write as _},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::SystemTime,
};

mod signature;

/// The app's bundle identifier, which a package must have.
pub const IDENTIFIER: &str = "org.stela.Stela";
/// Names the app bundle to embed instead of the newest built package.
pub const VARIABLE: &str = "STELA_APP";
/// The Data volume, where the kext finds what user space sees at `/`.
const DATA: &str = "/System/Volumes/Data";
const APP: &str = "/Applications/Stela.app";
/// The staged archive, in a directory only root writes.
const ARCHIVE: &str = "/private/var/db/org.stela.Stela.zip";
const JOB: &str = "/Library/LaunchDaemons/org.stela.Stela.install.plist";
const LABEL: &str = "org.stela.Stela.install";
/// The install job's own record of each run, readable without `log show`.
const LOG: &str = "/Library/Logs/org.stela.Stela.install.log";
/// `app_policy.h`'s registry property and the report that permits the install.
const PROPERTY: &str = "StelaApp";
const STAGED: &str = "staged";
/// How long the job waits for the kext's report.
const WAIT_SECONDS: u32 = 300;
/// The longest identifier or version the kext compares (`kStringBytes` - 1).
const STRING_BYTES: usize = 63;

/// A checked Stela.app.
pub struct Package {
    pub version: String,
    pub revision: String,
    executable: String,
    /// Each regular file's path in the bundle and bytes.
    files: BTreeMap<String, Vec<u8>>,
}

/// The app to embed: `STELA_APP`'s, else the newest built package, if any.
pub fn locate(root: &Path) -> Result<Option<PathBuf>> {
    locate_in(root, env::var_os(VARIABLE))
}

fn locate_in(root: &Path, given: Option<OsString>) -> Result<Option<PathBuf>> {
    if let Some(given) = given.filter(|given| !given.is_empty()) {
        let path = PathBuf::from(given);
        ensure!(
            path.join("Contents/Info.plist").is_file(),
            "{VARIABLE} names {}, which is not an app bundle",
            path.display()
        );
        return Ok(Some(path));
    }
    let packages = root.join("target/stela-app");
    if !packages.is_dir() {
        return Ok(None);
    }
    let mut newest: Option<(SystemTime, PathBuf)> = None;
    for entry in fs::read_dir(&packages)? {
        let entry = entry?;
        let app = entry.path().join("Stela.app");
        if !entry.file_name().to_string_lossy().starts_with("package.")
            || !app.join("Contents/Info.plist").is_file()
        {
            continue;
        }
        let built = entry.metadata()?.modified()?;
        if newest.as_ref().is_none_or(|(time, _)| built > *time) {
            newest = Some((built, app));
        }
    }
    Ok(newest.map(|(_, app)| app))
}

impl Package {
    /// Reads and checks the bundle at `app`.
    pub fn read(app: &Path) -> Result<Self> {
        let mut files = BTreeMap::new();
        let mut directories = vec![app.to_path_buf()];
        while let Some(directory) = directories.pop() {
            for entry in fs::read_dir(&directory)? {
                let entry = entry?;
                let kind = entry.file_type()?;
                let path = entry.path();
                if kind.is_dir() {
                    directories.push(path);
                    continue;
                }
                ensure!(
                    kind.is_file(),
                    "{} is neither a file nor a directory",
                    path.display()
                );
                let name = path
                    .strip_prefix(app)?
                    .to_str()
                    .context("a file name is not UTF-8")?
                    .replace('\\', "/");
                ensure!(
                    name.bytes()
                        .all(|byte| { byte.is_ascii_graphic() && byte != b'"' && byte != b'\\' }),
                    "{name:?} is not a path the kext spells"
                );
                ensure!(
                    format!("{DATA}{APP}/{name}").len() < super::KERNEL_PATH_BYTES,
                    "{name:?} is longer than the kext's {}-byte paths",
                    super::KERNEL_PATH_BYTES
                );
                files.insert(name, fs::read(&path)?);
            }
        }
        let info = files
            .get("Contents/Info.plist")
            .context("the app has no Contents/Info.plist")?;
        let plist: plist::Value =
            plist::from_bytes(info).context("Contents/Info.plist is not a property list")?;
        let plist = plist
            .as_dictionary()
            .context("Contents/Info.plist is not a dictionary")?;
        let string = |key: &str| -> Result<String> {
            plist
                .get(key)
                .and_then(plist::Value::as_string)
                .map(str::to_owned)
                .with_context(|| format!("Contents/Info.plist has no {key}"))
        };
        let identifier = string("CFBundleIdentifier")?;
        ensure!(
            identifier == IDENTIFIER,
            "the app is {identifier}, not Stela.app ({IDENTIFIER})"
        );
        let version = string("CFBundleShortVersionString")?;
        ensure!(
            is_version(&version),
            "the app's version {version:?} is not dotted decimal"
        );
        // The kext reads an installed Info.plist's text, so the app it
        // installs must spell these as it reads them, or it would never
        // find its own install current.
        for (key, value) in [
            ("CFBundleIdentifier", &identifier),
            ("CFBundleShortVersionString", &version),
        ] {
            ensure!(
                plist_string(info, key) == Some(value.as_str()),
                "Contents/Info.plist is not XML spelling {key} as the kext reads it"
            );
        }
        let revision = string("StelaSourceRevision")?;
        ensure!(
            !revision.is_empty()
                && revision.len() <= STRING_BYTES
                && revision
                    .bytes()
                    .all(|byte| byte.is_ascii_alphanumeric() || b".-".contains(&byte)),
            "the app's revision {revision:?} is malformed"
        );
        let executable = string("CFBundleExecutable")?;
        let code = files
            .get(&format!("Contents/MacOS/{executable}"))
            .with_context(|| format!("the app has no Contents/MacOS/{executable}"))?;
        let resources = files
            .get("Contents/_CodeSignature/CodeResources")
            .context("the app has no resource seal: is it signed?")?;
        signature::check_executable(code, IDENTIFIER, info, resources)
            .context("the app's executable signature")?;
        signature::check_resources(resources, &files, &executable)
            .context("the app's resource seal")?;
        Ok(Self {
            version,
            revision,
            executable,
            files,
        })
    }

    /// A zip of the bundle as `Stela.app/...`, which `ditto -x -k` extracts:
    /// its directories, then its files, with fixed times, 0755 for the
    /// directories and the executable and 0644 for every other file.
    pub fn archive(&self) -> Result<Vec<u8>> {
        use zip::{CompressionMethod, DateTime, ZipWriter, write::SimpleFileOptions};
        let options = |mode: u32| {
            SimpleFileOptions::default()
                .last_modified_time(DateTime::default())
                .unix_permissions(mode)
        };
        let mut directories = std::collections::BTreeSet::from([String::new()]);
        for path in self.files.keys() {
            let mut parent = Path::new(path).parent();
            while let Some(directory) = parent.filter(|directory| !directory.as_os_str().is_empty())
            {
                directories.insert(format!("{}/", directory.to_str().unwrap()));
                parent = directory.parent();
            }
        }
        let mut zip = ZipWriter::new(Cursor::new(Vec::new()));
        for directory in &directories {
            zip.add_directory(
                format!("Stela.app/{directory}"),
                options(0o755).compression_method(CompressionMethod::Stored),
            )?;
        }
        let executable = format!("Contents/MacOS/{}", self.executable);
        for (path, bytes) in &self.files {
            let mode = if *path == executable { 0o755 } else { 0o644 };
            zip.start_file(
                format!("Stela.app/{path}"),
                options(mode)
                    .compression_method(CompressionMethod::Deflated)
                    .compression_level(Some(9)),
            )?;
            zip.write_all(bytes)?;
        }
        Ok(zip.finish()?.into_inner())
    }

    /// `stela/app.h`: what the kext stages and how it recognises an install.
    pub fn header(&self) -> Result<String> {
        let archive = self.archive()?;
        let job = job()?;
        let mut header = String::from(
            "/* Stela.app as `cargo xtask guest` packaged it, which the kext stages\n \
             * for its install job (guest/stela-gpu/kext/app_delivery.h). */\n\
             #pragma once\n\
             #define STELA_APP_PRESENT 1\n",
        );
        for (name, value) in [
            ("IDENTIFIER", IDENTIFIER.to_owned()),
            ("VERSION", self.version.clone()),
            ("REVISION", self.revision.clone()),
            ("PATH", format!("{DATA}{APP}")),
            ("ARCHIVE_PATH", format!("{DATA}{ARCHIVE}")),
            ("JOB_PATH", format!("{DATA}{JOB}")),
        ] {
            writeln!(header, "#define STELA_APP_{name} \"{value}\"")?;
        }
        header.push_str(
            "struct stela_app_file {\n\
             \tconst char *path;\n\
             \tunsigned char sha256[32];\n\
             \tunsigned long length;\n\
             };\n\
             static const struct stela_app_file stela_app_files[] = {\n",
        );
        for (path, bytes) in &self.files {
            writeln!(
                header,
                "\t{{\"{path}\", {{{}}}, {}}},",
                c_digest(bytes),
                bytes.len()
            )?;
        }
        header.push_str("};\n");
        c_bytes(&mut header, "stela_app_archive", &archive)?;
        writeln!(
            header,
            "static const unsigned char stela_app_archive_sha256[32] = {{{}}};",
            c_digest(&archive)
        )?;
        c_bytes(&mut header, "stela_app_job", &job)?;
        writeln!(
            header,
            "static const unsigned char stela_app_job_sha256[32] = {{{}}};",
            c_digest(&job)
        )?;
        Ok(header)
    }

    /// The bundle's total size and the archive's, for the build's log.
    pub fn sizes(&self) -> Result<(usize, usize)> {
        Ok((
            self.files.values().map(Vec::len).sum(),
            self.archive()?.len(),
        ))
    }
}

/// `stela/app.h` for a build without an app: the kext reports it and
/// installs nothing.
pub fn absent_header() -> String {
    "/* This build carries no Stela.app: the kext reports that once per boot\n \
     * and installs or changes nothing (guest/stela-gpu/kext/app_delivery.h). */\n\
     #pragma once\n\
     #define STELA_APP_PRESENT 0\n"
        .to_owned()
}

/// Whether `version` is the dotted decimal `stela_app_compare_versions`
/// accepts: one to four components of one to nine digits.
fn is_version(version: &str) -> bool {
    let parts: Vec<&str> = version.split('.').collect();
    version.len() <= STRING_BYTES
        && (1..=4).contains(&parts.len())
        && parts.iter().all(|part| {
            (1..=9).contains(&part.len()) && part.bytes().all(|byte| byte.is_ascii_digit())
        })
}

/// The `<string>` after the first `<key>key</key>` of XML `text`, read as
/// `stela_app_plist_string` reads it.
fn plist_string<'a>(text: &'a [u8], key: &str) -> Option<&'a str> {
    let text = std::str::from_utf8(text).ok()?;
    let marker = format!("<key>{key}</key>");
    let after = &text[text.find(&marker)? + marker.len()..];
    let value = after
        .trim_start_matches([' ', '\t', '\n', '\r'])
        .strip_prefix("<string>")?;
    let end = value.find('<')?;
    (value[end..].starts_with("</string>") && !value[..end].contains('&')).then(|| &value[..end])
}

/// The install job's script, for `/bin/sh` as root at boot, using only the
/// tools macOS ships. It installs only after this boot's kext reports that
/// it staged the archive, and removes the archive and its own plist after
/// any run, so the kext stages them again on a later boot if still needed.
fn script() -> String {
    format!(
        r#"app={APP}
new=/Applications/.Stela.app.new
old=/Applications/.Stela.app.old
archive={ARCHIVE}
report=
waited=0
while [ "$waited" -lt {WAIT_SECONDS} ]; do
  report=$(/usr/sbin/ioreg -r -c StelaGPU -d 1 | /usr/bin/sed -n 's/.*"{PROPERTY}" = "\([a-z-]*\)".*/\1/p' | /usr/bin/head -n 1)
  [ -n "$report" ] && break
  /bin/sleep 1
  waited=$((waited + 1))
done
result="not installed: the kext reports ${{report:-nothing}}"
if [ "$report" = {STAGED} ]; then
  result="failed: the old app is kept"
  /bin/rm -rf "$new" "$old"
  if /bin/mkdir -m 0700 "$new" && /usr/bin/ditto -x -k "$archive" "$new" &&
    /usr/sbin/chown -R root:admin "$new/Stela.app" &&
    /usr/bin/codesign --verify --strict "$new/Stela.app"; then
    if {{ [ ! -e "$app" ] && [ ! -L "$app" ] || /bin/mv "$app" "$old"; }} &&
      /bin/mv "$new/Stela.app" "$app"; then
      result=installed
    elif [ -e "$old" ] && [ ! -e "$app" ]; then
      /bin/mv "$old" "$app"
    fi
  fi
  /bin/rm -rf "$new" "$old"
fi
echo "$(/bin/date -u +%Y-%m-%dT%H:%M:%SZ) $result"
/bin/rm -f "$archive" {JOB}
"#
    )
}

/// The install job's launchd plist: run once at load, record each run in
/// its log, and name the app the job belongs to for macOS's background item
/// list.
fn job() -> Result<Vec<u8>> {
    let mut job = plist::Dictionary::new();
    job.insert("Label".into(), LABEL.into());
    job.insert(
        "ProgramArguments".into(),
        plist::Value::Array(vec!["/bin/sh".into(), "-c".into(), script().into()]),
    );
    job.insert("RunAtLoad".into(), true.into());
    job.insert("StandardOutPath".into(), LOG.into());
    job.insert("StandardErrorPath".into(), LOG.into());
    job.insert(
        "AssociatedBundleIdentifiers".into(),
        plist::Value::Array(vec![IDENTIFIER.into()]),
    );
    let mut bytes = Vec::new();
    plist::to_writer_xml(&mut bytes, &plist::Value::Dictionary(job))?;
    Ok(bytes)
}

/// The sources `guest/stela-app/build.sh` reads, relative to the root.
const SOURCES: [&str; 7] = [
    "Cargo.toml",
    "assets/stela-icon.png",
    "guest/stela-app/Package.swift",
    "guest/stela-app/build.sh",
    "guest/stela-app/Packaging",
    "guest/stela-app/Sources",
    "guest/stela-app/Tests",
];
/// The Mac's working directory for the build, under its home.
const REMOTE_DIR: &str = "stela-app";

/// Builds Stela.app with `guest/stela-app/build.sh`: here on macOS without
/// `host`, else on the Mac `host` names over ssh, whose package lands in
/// `target/stela-app/package.<time>-mac`. Answers the app's path.
pub fn build(root: &Path, host: Option<&str>) -> Result<PathBuf> {
    let Some(host) = host else {
        ensure!(
            cfg!(target_os = "macos"),
            "Stela.app builds only on macOS: pass --mac or set {}",
            crate::tools::MAC_VARIABLE
        );
        crate::tools::exec(Command::new("bash").arg(root.join("guest/stela-app/build.sh")))?;
        return locate_in(root, None)?.context("build.sh left no package");
    };
    let revision = revision(root)?;
    let mut send = Command::new("tar");
    send.arg("-C").arg(root).args(["-cf", "-"]).args(SOURCES);
    pipe(
        send,
        ssh(
            host,
            &format!("rm -rf {REMOTE_DIR} && mkdir -p {REMOTE_DIR} && tar -C {REMOTE_DIR} -xf -"),
        ),
    )?;
    let output = root.join("target/stela-app").join(format!(
        "package.{}-mac",
        SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)?
            .as_secs()
    ));
    fs::create_dir_all(&output)?;
    let mut receive = Command::new("tar");
    receive.arg("-C").arg(&output).arg("-xf").arg("-");
    // The Mac's full Xcode, where present, supplies XCTest for build.sh's
    // tests; its tar leaves out macOS metadata files, which no seal lists.
    pipe(
        ssh(
            host,
            &format!(
                "cd {REMOTE_DIR} && rm -rf target/stela-app && \
                 {{ [ ! -d /Applications/Xcode.app/Contents/Developer ] || \
                 export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer; }} && \
                 STELA_SOURCE_REVISION={revision} bash guest/stela-app/build.sh >&2 && \
                 cd target/stela-app/package.* && COPYFILE_DISABLE=1 tar -cf - Stela.app"
            ),
        ),
        receive,
    )?;
    let app = output.join("Stela.app");
    let package = Package::read(&app)?;
    println!(
        "Stela.app {} ({}): {}",
        package.version,
        package.revision,
        app.display()
    );
    Ok(app)
}

/// The revision build.sh records: HEAD, marked when the app's inputs changed.
fn revision(root: &Path) -> Result<String> {
    let git = |arguments: &[&str]| -> Result<String> {
        let output = Command::new("git")
            .arg("-C")
            .arg(root)
            .args(arguments)
            .output()?;
        ensure!(output.status.success(), "git {arguments:?} failed");
        Ok(String::from_utf8(output.stdout)?.trim().to_owned())
    };
    let mut revision = git(&["rev-parse", "--short=12", "HEAD"])?;
    let changed = git(&[
        "status",
        "--porcelain",
        "--untracked-files=normal",
        "--",
        "guest/stela-app",
        "assets",
        "Cargo.toml",
    ])?;
    if !changed.is_empty() {
        revision.push_str("-modified");
    }
    Ok(revision)
}

fn ssh(host: &str, script: &str) -> Command {
    let mut command = Command::new("ssh");
    command.arg(host).arg(script);
    command
}

/// Pipes `source`'s output into `sink`.
fn pipe(mut source: Command, mut sink: Command) -> Result<()> {
    println!("+ {source:?} | {sink:?}");
    let mut running = source.stdout(Stdio::piped()).spawn().context("tar")?;
    let status = sink.stdin(running.stdout.take().unwrap()).status()?;
    let source_status = running.wait()?;
    if !status.success() || !source_status.success() {
        bail!("{source:?} | {sink:?} failed: {source_status}, {status}");
    }
    Ok(())
}

#[cfg(test)]
mod tests;
