//! The host and optional target LLVM toolsets, each chosen in one
//! place as one set: Clang and its C++ driver, libclang for bindgen (whose
//! built-in headers must be the compiler's), and a linker, archiver and
//! symbol lister at least as new.
//!
//! `CLANG` names the compiler. Otherwise the newest supported Clang among
//! the versioned installs (`/usr/lib/llvm-NN`, `/usr/lib/llvmNN`,
//! `/usr/lib64/llvmNN`, Homebrew's `llvm@NN`) and `clang`/`clang-NN` on PATH
//! is chosen. Xcode's Apple clang counts by its own release number, and
//! LLVM's Clang goes first of the same number. The linker, archiver and
//! symbol lister come from the same install when it has them, else from the
//! active Rust toolchain's `llvm-tools`, else from PATH; each must be at
//! least the compiler's version. The nested Cargo builds get the choice through `CLANG`, so every
//! build script uses the host Clang, or `STELA_TARGET_CLANG` for cross builds.
use anyhow::{Result, bail};
use std::{
    cmp::Ordering,
    env,
    ffi::OsString,
    fmt,
    ops::RangeInclusive,
    path::{Path, PathBuf},
    process::Command,
    sync::OnceLock,
};

/// The Clang majors Stela builds with: libc++ of the pinned LLVM needs 21,
/// and 23 is the newest checked.
pub const SUPPORTED: RangeInclusive<u32> = 21..=23;
/// The environment variable that names the compiler.
pub const VARIABLE: &str = "CLANG";
/// Optional compiler for cross-compiled code.
pub const TARGET_VARIABLE: &str = "STELA_TARGET_CLANG";
/// Where versioned LLVM installs live, and the prefixes their directories'
/// names start with: Debian's `llvm-NN`, Arch's, Fedora's and openSUSE's
/// `llvmNN`, Homebrew's `llvm@NN`.
const INSTALLS: [(&str, &str); 6] = [
    ("/usr/lib", "llvm"),
    ("/usr/lib64", "llvm"),
    ("/usr/local/lib", "llvm"),
    ("/opt/homebrew/opt", "llvm"),
    ("/usr/local/opt", "llvm"),
    ("/opt", "llvm"),
];
/// Directories distributions keep a libclang outside its LLVM install.
const LIBRARY_DIRECTORIES: [&str; 4] = [
    "/usr/lib64",
    "/usr/lib/x86_64-linux-gnu",
    "/usr/lib/aarch64-linux-gnu",
    "/usr/lib",
];

/// An LLVM release.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

/// The release a tool's `--version` reports on its first line that names
/// one: `clang version 21.1.8`, `LLD 21.1.8`, `LLVM version 21.1.8`, and
/// Apple's `Apple clang version 21.0.0`, which numbers its own releases.
pub fn version(report: &str) -> Option<Version> {
    let number = report.lines().find_map(|line| {
        let words: Vec<&str> = line.split_whitespace().collect();
        words
            .windows(2)
            .find(|pair| matches!(pair[0], "version" | "LLD"))
            .map(|pair| pair[1])
    })?;
    let mut parts = number
        .split(|c: char| !c.is_ascii_digit())
        .map(|part| part.parse::<u32>().ok());
    Some(Version {
        major: parts.next()??,
        minor: parts.next().flatten().unwrap_or(0),
        patch: parts.next().flatten().unwrap_or(0),
    })
}

/// What the choice reads of this computer.
pub trait Host {
    fn var(&self, name: &str) -> Option<OsString>;
    /// PATH's directories, in order.
    fn path(&self) -> Vec<PathBuf>;
    fn is_file(&self, path: &Path) -> bool;
    /// The path with its links resolved, or itself.
    fn canonical(&self, path: &Path) -> PathBuf;
    /// The names in a directory.
    fn list(&self, directory: &Path) -> Vec<String>;
    /// What `program --version` prints.
    fn version_report(&self, program: &Path) -> Option<String>;
    /// The active Rust toolchain's `llvm-tools` directory and its LLVM
    /// release.
    fn rust_tools(&self) -> Option<(PathBuf, Version)>;
    /// What `clang -print-resource-dir` prints: its built-in headers'
    /// directory, under the library directory of its own install.
    fn resource_dir(&self, clang: &Path) -> Option<PathBuf>;
}

/// A compiler found, and where.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    pub clang: PathBuf,
    /// `None` for a Clang that reports no release.
    pub version: Option<Version>,
    /// Apple's Clang, whose release numbers are Xcode's: Apple clang 21 is
    /// what libc++ of the pinned LLVM accepts as its oldest, as LLVM's 21.
    pub apple: bool,
}

/// The tools of one LLVM.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Toolset {
    pub version: Version,
    pub clang: PathBuf,
    pub clangxx: PathBuf,
    pub linker: PathBuf,
    pub archiver: PathBuf,
    pub symbols: PathBuf,
    /// The libclang shared library bindgen loads, by file.
    pub libclang: PathBuf,
}

impl Toolset {
    /// What a build made with these tools depends on: each tool, and the
    /// compiler's release.
    pub fn identity(&self) -> String {
        format!(
            "clang {} {}\nclang++ {}\nlinker {}\narchiver {}\nsymbols {}\nlibclang {}\n",
            self.version,
            self.clang.display(),
            self.clangxx.display(),
            self.linker.display(),
            self.archiver.display(),
            self.symbols.display(),
            self.libclang.display(),
        )
    }

    /// Environment for a nested build: the compiler for every build script
    /// and the libclang for bindgen.
    pub fn apply(&self, command: &mut Command) {
        command
            .env(VARIABLE, &self.clang)
            .env("LIBCLANG_PATH", &self.libclang);
    }
}

fn exe(name: &str) -> String {
    format!("{name}{}", env::consts::EXE_SUFFIX)
}

/// The suffix of a versioned driver's name: `-21` of `clang-21`.
fn suffix(clang: &Path) -> String {
    let name = clang
        .file_stem()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    name.strip_prefix("clang").unwrap_or_default().to_owned()
}

/// Whether a file's name is a Clang driver's, not a wrapper's such as
/// ccache's.
fn is_clang(path: &Path) -> bool {
    path.file_stem()
        .is_some_and(|stem| stem.to_string_lossy().starts_with("clang"))
}

/// The compiler `clang` runs, with links resolved: through a wrapper that
/// a link of Clang's name leads to, as ccache's directory of links on PATH
/// does, the Clang of the same name later on PATH. The build still runs
/// `clang` by the name it was found under; this locates its install.
fn real_compiler(host: &dyn Host, clang: &Path) -> PathBuf {
    let real = host.canonical(clang);
    if is_clang(&real) {
        return real;
    }
    let Some(name) = clang.file_name() else {
        return real;
    };
    host.path()
        .into_iter()
        .map(|directory| directory.join(name))
        .filter(|path| path != clang && host.is_file(path))
        .map(|path| host.canonical(&path))
        .find(|path| is_clang(path))
        .unwrap_or(real)
}

/// Every Clang this computer has: `CLANG` alone when it is set, else the
/// versioned installs' and PATH's, each once by the compiler it runs.
pub fn candidates(host: &dyn Host) -> Vec<Found> {
    candidates_for(host, VARIABLE)
}

fn candidates_for(host: &dyn Host, variable: &str) -> Vec<Found> {
    let mut paths = Vec::new();
    let explicit = host.var(variable).filter(|value| !value.is_empty());
    let named = explicit.is_some();
    if let Some(clang) = explicit {
        let clang = PathBuf::from(clang);
        // A bare name is looked up on PATH.
        let clang = if clang.components().count() == 1 {
            host.path()
                .into_iter()
                .map(|directory| directory.join(&clang))
                .find(|path| host.is_file(path))
                .unwrap_or(clang)
        } else {
            clang
        };
        paths.push(clang);
    } else {
        for directory in host.path() {
            let mut names: Vec<String> = host
                .list(&directory)
                .into_iter()
                .filter(|name| {
                    let stem = name.strip_suffix(env::consts::EXE_SUFFIX).unwrap_or(name);
                    stem == "clang"
                        || stem
                            .strip_prefix("clang-")
                            .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
                })
                .collect();
            names.sort();
            paths.extend(names.into_iter().map(|name| directory.join(name)));
        }
        for (base, prefix) in INSTALLS {
            let mut installs: Vec<String> = host
                .list(Path::new(base))
                .into_iter()
                .filter(|name| {
                    name.strip_prefix(prefix).is_some_and(|rest| {
                        let digits = rest.trim_start_matches(['-', '@']);
                        rest.is_empty() || digits.bytes().all(|b| b.is_ascii_digit())
                    })
                })
                .collect();
            installs.sort();
            paths.extend(
                installs
                    .into_iter()
                    .map(|install| Path::new(base).join(install).join("bin").join(exe("clang"))),
            );
        }
    }
    let mut found: Vec<Found> = Vec::new();
    let mut seen = Vec::new();
    for clang in paths {
        if !host.is_file(&clang) {
            // Only a compiler `CLANG` names is listed when it is missing.
            if named {
                found.push(Found {
                    clang,
                    version: None,
                    apple: false,
                });
            }
            continue;
        }
        let real = real_compiler(host, &clang);
        if seen.contains(&real) {
            continue;
        }
        seen.push(real);
        let report = host.version_report(&clang).unwrap_or_default();
        found.push(Found {
            clang,
            version: version(&report),
            apple: report.contains("Apple"),
        });
    }
    found
}

/// What stops the build from choosing a toolset, each as a line saying what
/// to install or set.
#[derive(Debug, PartialEq, Eq)]
pub struct Problems(pub Vec<String>);

impl fmt::Display for Problems {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0.join("\n"))
    }
}

fn range() -> String {
    format!("{} to {}", SUPPORTED.start(), SUPPORTED.end())
}

/// The newest supported Clang of `found`: LLVM's before Apple's of the same
/// release, else the first of equals.
fn newest(found: &[Found]) -> Option<&Found> {
    let rank = |f: &Found| (f.version, !f.apple);
    found
        .iter()
        .filter(|f| f.version.is_some_and(|v| SUPPORTED.contains(&v.major)))
        .fold(None, |best: Option<&Found>, f| match best {
            Some(best) if rank(best).cmp(&rank(f)) != Ordering::Less => Some(best),
            _ => Some(f),
        })
}

/// Chooses the toolset, or says everything that is missing.
pub fn select(host: &dyn Host) -> Result<Toolset, Problems> {
    select_for(host, VARIABLE)
}

fn select_for(host: &dyn Host, variable: &str) -> Result<Toolset, Problems> {
    let found = candidates_for(host, variable);
    let explicit = host.var(variable).is_some_and(|value| !value.is_empty());
    let Some(chosen) = newest(&found) else {
        return Err(Problems(vec![no_compiler(&found, explicit, variable)]));
    };
    let version = chosen.version.expect("a supported compiler has a version");
    let mut problems = Vec::new();
    let clang = chosen.clang.clone();
    let directory = clang.parent().unwrap_or(Path::new("")).to_path_buf();
    // The install's tools and libclang are found beside the compiler it
    // runs, never beside a wrapper.
    let real = real_compiler(host, &clang);
    let real_directory = real.parent().unwrap_or(Path::new("")).to_path_buf();
    let suffix = suffix(&clang);

    let clangxx = [
        directory.join(exe(&format!("clang++{suffix}"))),
        real_directory.join(exe("clang++")),
    ]
    .into_iter()
    .find(|path| host.is_file(path));
    if clangxx.is_none() {
        problems.push(format!(
            "install Clang {}'s C++ driver: no clang++{suffix} beside {}",
            version.major,
            clang.display()
        ));
    }

    let tool = |name: &str, rust_name: &str| -> Option<PathBuf> {
        let install = [
            real_directory.join(exe(name)),
            directory.join(exe(&format!("{name}{suffix}"))),
            directory.join(exe(name)),
        ];
        let rust = host
            .rust_tools()
            .filter(|(_, rust)| rust.major >= version.major)
            .map(|(directory, _)| directory.join(exe(rust_name)));
        let path = host.path().into_iter().flat_map(|directory| {
            [
                directory.join(exe(&format!("{name}-{}", version.major))),
                directory.join(exe(name)),
            ]
        });
        let new_enough = |path: &PathBuf| {
            host.version_report(path)
                .as_deref()
                .and_then(self::version)
                .is_some_and(|found| found.major >= version.major)
        };
        install
            .into_iter()
            .filter(|path| host.is_file(path) && new_enough(path))
            .chain(rust.filter(|path| host.is_file(path)))
            .chain(path.filter(|path| host.is_file(path) && new_enough(path)))
            .next()
    };
    let linker = tool("ld.lld", "gcc-ld/ld.lld");
    let archiver = tool("llvm-ar", "llvm-ar");
    let symbols = tool("llvm-nm", "llvm-nm");
    for (found, name, package) in [
        (&linker, "ld.lld", "LLD"),
        (&archiver, "llvm-ar", "LLVM's tools"),
        (&symbols, "llvm-nm", "LLVM's tools"),
    ] {
        if found.is_none() {
            problems.push(format!(
                "install {package} {} or newer ({name}), or the Rust llvm-tools component",
                version.major
            ));
        }
    }

    // The install's library directory, which the built-in headers are in:
    // the one way to it from Xcode's `/usr/bin/clang`, a launcher.
    let library = host
        .resource_dir(&clang)
        .and_then(|resource| Some(resource.parent()?.parent()?.to_path_buf()));
    let libclang = libclang(host, &real_directory, library.as_deref(), version);
    let libclang = match libclang {
        Ok(path) => Some(path),
        Err(problem) => {
            problems.push(problem);
            None
        }
    };
    match (clangxx, linker, archiver, symbols, libclang) {
        (Some(clangxx), Some(linker), Some(archiver), Some(symbols), Some(libclang))
            if problems.is_empty() =>
        {
            Ok(Toolset {
                version,
                // Both drivers run by the names they were found under: a
                // wrapper such as ccache stays in front of the compiler, and
                // Clang picks C++ by the name it runs under (Homebrew's
                // `clang++` is a link to `clang-NN`, which run by its own
                // name links no C++ library).
                clang,
                clangxx,
                linker,
                archiver,
                symbols,
                libclang: host.canonical(&libclang),
            })
        }
        _ => Err(Problems(problems)),
    }
}

fn no_compiler(found: &[Found], explicit: bool, variable: &str) -> String {
    let listed: Vec<String> = found
        .iter()
        .map(|f| match f.version {
            Some(version) if f.apple => {
                format!("Apple clang {version} ({})", f.clang.display())
            }
            Some(version) => format!("Clang {version} ({})", f.clang.display()),
            None if explicit => format!("{} (missing, or reports no release)", f.clang.display()),
            None => format!("{} (an unknown Clang)", f.clang.display()),
        })
        .collect();
    let found = if listed.is_empty() {
        "no Clang was found".to_owned()
    } else {
        format!("found {}", listed.join(", "))
    };
    if explicit {
        format!(
            "{variable} names no supported compiler: {found}. Stela builds with Clang {}: \
             install one and name it with {variable}, or unset {variable}",
            range()
        )
    } else {
        format!(
            "{found}. Stela builds with Clang {}: install one (clang, lld and libclang of the \
             same LLVM), or name it with {VARIABLE}=/path/to/clang",
            range()
        )
    }
}

/// The major a libclang's file name carries: `libclang.so.21.1.8`,
/// `libclang-21.so.1`, `libclang-21.so`.
fn library_major(name: &str) -> Option<u32> {
    let digits = |text: &str| -> Option<u32> {
        let number: String = text.chars().take_while(char::is_ascii_digit).collect();
        number.parse().ok()
    };
    if let Some(rest) = name.strip_prefix("libclang-") {
        return digits(rest);
    }
    name.strip_prefix("libclang.so.").and_then(digits)
}

fn is_libclang(name: &str) -> bool {
    name == "libclang.so"
        || name == "libclang.dylib"
        || name == "libclang.dll"
        || name.starts_with("libclang.so.")
        // Not libclang-cpp, the C++ library.
        || name
            .strip_prefix("libclang-")
            .is_some_and(|rest| rest.starts_with(|c: char| c.is_ascii_digit()) && rest.contains(".so"))
}

/// The libclang of Clang `version`: `LIBCLANG_PATH`'s when it is set (a file
/// or a directory), else its install's, else a distribution directory's
/// with the same major. A library whose name carries a release must carry
/// `version`'s major; one without is taken from `LIBCLANG_PATH` or the
/// install.
fn libclang(
    host: &dyn Host,
    clang_directory: &Path,
    library: Option<&Path>,
    version: Version,
) -> Result<PathBuf, String> {
    let matches = |path: &Path, unversioned: bool| -> bool {
        let name = host
            .canonical(path)
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_default();
        match library_major(&name) {
            Some(major) => major == version.major,
            None => unversioned,
        }
    };
    let search = |directory: &Path, unversioned: bool| -> Option<PathBuf> {
        let mut names: Vec<String> = host
            .list(directory)
            .into_iter()
            .filter(|name| is_libclang(name))
            .collect();
        names.sort();
        names
            .into_iter()
            .map(|name| directory.join(name))
            .find(|path| host.is_file(path) && matches(path, unversioned))
    };
    if let Some(given) = host.var("LIBCLANG_PATH").filter(|value| !value.is_empty()) {
        let given = PathBuf::from(given);
        // A library named without its release, as macOS's libclang.dylib
        // and Windows's libclang.dll are, is taken as given.
        let found = if host.is_file(&given) {
            matches(&given, true).then_some(given.clone())
        } else {
            search(&given, true)
        };
        return found.ok_or_else(|| {
            format!(
                "LIBCLANG_PATH ({}) holds no libclang {}, which bindgen needs to match Clang {}: \
                 point it at that LLVM's libclang, or unset it",
                given.display(),
                version.major,
                version
            )
        });
    }
    let prefix = clang_directory.parent().unwrap_or(Path::new(""));
    library
        .map(Path::to_path_buf)
        .into_iter()
        .chain([
            prefix.join("lib"),
            prefix.join("lib64"),
            clang_directory.to_path_buf(),
        ])
        .collect::<Vec<_>>()
        .iter()
    .find_map(|directory| search(directory, true))
    .or_else(|| {
        LIBRARY_DIRECTORIES
            .iter()
            .find_map(|directory| search(Path::new(directory), false))
    })
    .ok_or_else(|| {
        format!(
            "install libclang {} (the shared library of Clang {}), which bindgen loads, or name \
             it with LIBCLANG_PATH",
            version.major, version
        )
    })
}

/// This computer.
pub struct System;

impl Host for System {
    fn var(&self, name: &str) -> Option<OsString> {
        env::var_os(name)
    }

    fn path(&self) -> Vec<PathBuf> {
        env::var_os("PATH")
            .map(|path| env::split_paths(&path).collect())
            .unwrap_or_default()
    }

    fn is_file(&self, path: &Path) -> bool {
        path.is_file()
    }

    fn canonical(&self, path: &Path) -> PathBuf {
        std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf())
    }

    fn list(&self, directory: &Path) -> Vec<String> {
        std::fs::read_dir(directory)
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|entry| entry.file_name().into_string().ok())
            .collect()
    }

    fn version_report(&self, program: &Path) -> Option<String> {
        let output = Command::new(program).arg("--version").output().ok()?;
        output
            .status
            .success()
            .then(|| String::from_utf8_lossy(&output.stdout).into_owned())
    }

    fn resource_dir(&self, clang: &Path) -> Option<PathBuf> {
        let output = Command::new(clang)
            .arg("-print-resource-dir")
            .output()
            .ok()?;
        let text = String::from_utf8(output.stdout).ok()?;
        (output.status.success() && !text.trim().is_empty()).then(|| PathBuf::from(text.trim()))
    }

    fn rust_tools(&self) -> Option<(PathBuf, Version)> {
        let rustc = env::var_os("RUSTC").unwrap_or_else(|| "rustc".into());
        let report = Command::new(&rustc).arg("-vV").output().ok()?;
        let report = String::from_utf8(report.stdout).ok()?;
        let host = report
            .lines()
            .find_map(|line| line.strip_prefix("host: "))?;
        let llvm = report
            .lines()
            .find_map(|line| line.strip_prefix("LLVM version: "))
            .and_then(|number| version(&format!("version {number}")))?;
        let sysroot = Command::new(&rustc)
            .args(["--print", "sysroot"])
            .output()
            .ok()?;
        let sysroot = PathBuf::from(String::from_utf8(sysroot.stdout).ok()?.trim());
        Some((sysroot.join("lib/rustlib").join(host).join("bin"), llvm))
    }
}

/// The toolset of this computer, chosen once per run.
pub fn toolset() -> Result<&'static Toolset> {
    static CHOSEN: OnceLock<Result<Toolset, String>> = OnceLock::new();
    match CHOSEN.get_or_init(|| select(&System).map_err(|problems| problems.to_string())) {
        Ok(toolset) => Ok(toolset),
        Err(problems) => bail!("{problems}"),
    }
}

/// The cross compiler, or the host toolset when no override was provided.
pub fn target_toolset() -> Result<&'static Toolset> {
    static CHOSEN: OnceLock<Result<Toolset, String>> = OnceLock::new();
    match CHOSEN.get_or_init(|| {
        if env::var_os(TARGET_VARIABLE).is_some_and(|value| !value.is_empty()) {
            select_for(&System, TARGET_VARIABLE).map_err(|problems| problems.to_string())
        } else {
            toolset().cloned().map_err(|error| error.to_string())
        }
    }) {
        Ok(toolset) => Ok(toolset),
        Err(problems) => bail!("{problems}"),
    }
}

#[cfg(test)]
mod tests;
