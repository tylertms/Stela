use super::*;
use std::collections::BTreeMap;

/// A computer of a test's choosing: files, links, `--version` reports and
/// environment.
#[derive(Default)]
struct Fake {
    files: BTreeMap<PathBuf, Option<String>>,
    links: BTreeMap<PathBuf, PathBuf>,
    vars: BTreeMap<&'static str, String>,
    path: Vec<PathBuf>,
    rust: Option<(PathBuf, Version)>,
    resources: BTreeMap<PathBuf, PathBuf>,
}

impl Fake {
    fn new(path: &[&str]) -> Self {
        Self {
            path: path.iter().map(PathBuf::from).collect(),
            ..Self::default()
        }
    }

    /// A program reporting `report` to `--version`.
    fn tool(mut self, path: &str, report: &str) -> Self {
        self.files.insert(path.into(), Some(report.into()));
        self
    }

    /// The LLVM `major` tools under `bin`.
    fn llvm(self, bin: &str, major: u32) -> Self {
        let clang = format!("clang version {major}.1.8\nTarget: x86_64-pc-linux-gnu\n");
        let lld = format!("LLD {major}.1.8 (compatible with GNU linkers)\n");
        let llvm = format!("LLVM (http://llvm.org/):\n  LLVM version {major}.1.8\n");
        self.tool(&format!("{bin}/clang"), &clang)
            .tool(&format!("{bin}/clang++"), &clang)
            .tool(&format!("{bin}/ld.lld"), &lld)
            .tool(&format!("{bin}/llvm-ar"), &llvm)
            .tool(&format!("{bin}/llvm-nm"), &llvm)
    }

    fn file(mut self, path: &str) -> Self {
        self.files.insert(path.into(), None);
        self
    }

    fn link(mut self, link: &str, target: &str) -> Self {
        self.links.insert(link.into(), target.into());
        self
    }

    fn var(mut self, name: &'static str, value: &str) -> Self {
        self.vars.insert(name, value.into());
        self
    }
}

impl Host for Fake {
    fn var(&self, name: &str) -> Option<OsString> {
        self.vars.get(name).map(OsString::from)
    }
    fn path(&self) -> Vec<PathBuf> {
        self.path.clone()
    }
    fn is_file(&self, path: &Path) -> bool {
        self.files.contains_key(&self.canonical(path))
    }
    fn canonical(&self, path: &Path) -> PathBuf {
        let mut path = path.to_path_buf();
        while let Some(target) = self.links.get(&path) {
            path = target.clone();
        }
        path
    }
    fn list(&self, directory: &Path) -> Vec<String> {
        let mut names: Vec<String> = self
            .files
            .keys()
            .chain(self.links.keys())
            .filter_map(|path| {
                let rest = path.strip_prefix(directory).ok()?;
                Some(rest.components().next()?.as_os_str().to_str()?.to_owned())
            })
            .collect();
        names.dedup();
        names
    }
    fn version_report(&self, program: &Path) -> Option<String> {
        self.files.get(&self.canonical(program)).cloned().flatten()
    }
    fn rust_tools(&self) -> Option<(PathBuf, Version)> {
        self.rust.clone()
    }
    fn resource_dir(&self, clang: &Path) -> Option<PathBuf> {
        self.resources.get(clang).cloned()
    }
}

fn path(text: &str) -> PathBuf {
    PathBuf::from(text)
}

#[test]
fn versions_from_tool_reports() {
    let v = |report| version(report).map(|v| v.to_string());
    assert_eq!(
        v("Ubuntu clang version 21.1.8 (6ubuntu1)\nTarget: x86_64").as_deref(),
        Some("21.1.8")
    );
    assert_eq!(v("clang version 23.1.0").as_deref(), Some("23.1.0"));
    assert_eq!(
        v("LLD 22.1.2 (compatible with GNU linkers)").as_deref(),
        Some("22.1.2")
    );
    assert_eq!(
        v("LLVM (http://llvm.org/):\n  LLVM version 23.1.1-rust-1.99.1-stable\n").as_deref(),
        Some("23.1.1")
    );
    assert_eq!(
        v("llvm-nm, compatible with GNU nm\nUbuntu LLVM version 21.1.8\n").as_deref(),
        Some("21.1.8")
    );
    assert_eq!(
        v("Apple clang version 21.0.0 (clang-2100.3.34.2)").as_deref(),
        Some("21.0.0")
    );
    assert_eq!(v("clang version 22.0.0git").as_deref(), Some("22.0.0"));
    assert_eq!(v(""), None);
}

/// The newest supported Clang of several installs, with its own tools and
/// libclang: Debian's layout, where `/usr/bin/clang` links into
/// `/usr/lib/llvm-NN`.
#[test]
fn debian_takes_the_newest_supported_install() {
    let host = Fake::new(&["/usr/bin"])
        .llvm("/usr/lib/llvm-19/bin", 19)
        .llvm("/usr/lib/llvm-21/bin", 21)
        .llvm("/usr/lib/llvm-22/bin", 22)
        .link("/usr/bin/clang", "/usr/lib/llvm-19/bin/clang")
        .link("/usr/bin/clang-21", "/usr/lib/llvm-21/bin/clang")
        .file("/usr/lib/llvm-21/lib/libclang-21.so.1")
        .file("/usr/lib/llvm-22/lib/libclang-cpp.so.22.1")
        .file("/usr/lib/llvm-22/lib/libclang-22.so.1");
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.version.major, 22);
    assert_eq!(toolset.clang, path("/usr/lib/llvm-22/bin/clang"));
    // Debian's clang++ links to clang, which acts as the C++ driver only
    // under a C++ name.
    let host = host.link("/usr/lib/llvm-22/bin/clang++", "/usr/lib/llvm-22/bin/clang");
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clangxx, path("/usr/lib/llvm-22/bin/clang++"));
    assert_eq!(toolset.clangxx, path("/usr/lib/llvm-22/bin/clang++"));
    assert_eq!(toolset.linker, path("/usr/lib/llvm-22/bin/ld.lld"));
    assert_eq!(toolset.archiver, path("/usr/lib/llvm-22/bin/llvm-ar"));
    assert_eq!(
        toolset.libclang,
        path("/usr/lib/llvm-22/lib/libclang-22.so.1")
    );
}

/// Arch: the current Clang in `/usr/bin` and an older one under
/// `/usr/lib/llvmNN`. `CLANG` chooses the older one, with its own libclang,
/// whatever libclang `/usr/lib` holds.
#[test]
fn arch_honors_clang_and_matches_its_libclang() {
    let host = Fake::new(&["/usr/bin"])
        .llvm("/usr/bin", 24)
        .llvm("/usr/lib/llvm21/bin", 21)
        .file("/usr/lib/libclang.so.24.1")
        .link("/usr/lib/libclang.so", "/usr/lib/libclang.so.24.1")
        .file("/usr/lib/llvm21/lib/libclang.so.21.1")
        .link(
            "/usr/lib/llvm21/lib/libclang.so",
            "/usr/lib/llvm21/lib/libclang.so.21.1",
        );
    // Clang 24 is newer than the supported range, so 21 is chosen.
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clang, path("/usr/lib/llvm21/bin/clang"));
    assert_eq!(toolset.linker, path("/usr/lib/llvm21/bin/ld.lld"));
    assert_eq!(
        toolset.libclang,
        path("/usr/lib/llvm21/lib/libclang.so.21.1")
    );
    let host = host.var("CLANG", "/usr/lib/llvm21/bin/clang");
    assert_eq!(
        select(&host).unwrap().clang,
        path("/usr/lib/llvm21/bin/clang")
    );
}

#[test]
fn a_named_compiler_out_of_range_is_refused_with_its_version_and_the_range() {
    let host = Fake::new(&["/usr/bin"])
        .llvm("/usr/bin", 24)
        .llvm("/usr/lib/llvm21/bin", 21)
        .var("CLANG", "/usr/bin/clang");
    let problems = select(&host).unwrap_err().to_string();
    assert!(
        problems.contains("Clang 24.1.8 (/usr/bin/clang)"),
        "{problems}"
    );
    assert!(
        problems.contains(&format!(
            "Clang {} to {}",
            SUPPORTED.start(),
            SUPPORTED.end()
        )),
        "{problems}"
    );
    assert!(problems.contains("CLANG"), "{problems}");
    let host = Fake::new(&[]).var("CLANG", "/nowhere/clang");
    let problems = select(&host).unwrap_err().to_string();
    assert!(problems.contains("/nowhere/clang (missing"), "{problems}");
}

#[test]
fn too_old_and_apple_compilers_are_named() {
    let host = Fake::new(&["/usr/bin"]).tool(
        "/usr/bin/clang",
        "Apple clang version 17.0.0 (clang-1700.0.13.5)",
    );
    let problems = select(&host).unwrap_err().to_string();
    assert!(
        problems.contains("Apple clang 17.0.0 (/usr/bin/clang)"),
        "{problems}"
    );
    let host = Fake::new(&["/usr/bin"]).llvm("/usr/bin", 18);
    let problems = select(&host).unwrap_err().to_string();
    assert!(problems.contains("found Clang 18.1.8"), "{problems}");
    let problems = select(&Fake::new(&[])).unwrap_err().to_string();
    assert!(problems.starts_with("no Clang was found"), "{problems}");
}

/// A Nix shell: Clang, LLD and the LLVM tools are separate store paths on
/// PATH, the Rust toolchain has no `llvm-tools`, and `LIBCLANG_PATH` names
/// the library.
#[test]
fn nix_shell_takes_tools_from_path_and_libclang_from_the_environment() {
    let clang = "/nix/store/a-clang-wrapper-21.1.8/bin";
    let lld = "/nix/store/b-lld-21.1.8/bin";
    let tools = "/nix/store/c-llvm-binutils-21.1.8/bin";
    let host = Fake::new(&[clang, lld, tools, "/nix/store/d-lld-19.1.7/bin"])
        .tool(&format!("{clang}/clang"), "clang version 21.1.8")
        .tool(&format!("{clang}/clang++"), "clang version 21.1.8")
        .tool(&format!("{lld}/ld.lld"), "LLD 21.1.8")
        .tool(&format!("{tools}/llvm-ar"), "llvm-ar, version 21.1.8")
        .tool(&format!("{tools}/llvm-nm"), "llvm-nm, version 21.1.8")
        .file("/nix/store/e-clang-21.1.8-lib/lib/libclang.so.21.1")
        .var("LIBCLANG_PATH", "/nix/store/e-clang-21.1.8-lib/lib");
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.linker, path(&format!("{lld}/ld.lld")));
    assert_eq!(toolset.archiver, path(&format!("{tools}/llvm-ar")));
    assert_eq!(
        toolset.libclang,
        path("/nix/store/e-clang-21.1.8-lib/lib/libclang.so.21.1")
    );
    // Another LLVM's libclang is refused, by name.
    let host = host.var("LIBCLANG_PATH", "/usr/lib/libclang.so.23.1");
    let host = host.file("/usr/lib/libclang.so.23.1");
    let problems = select(&host).unwrap_err().to_string();
    assert!(
        problems.contains("LIBCLANG_PATH (/usr/lib/libclang.so.23.1)"),
        "{problems}"
    );
}

/// `LIBCLANG_PATH` naming macOS's `libclang.dylib`, whose name carries no
/// release, as a file or by its directory: taken as given.
#[test]
fn an_unversioned_libclang_path_is_taken_as_given() {
    let lib = "/opt/homebrew/opt/llvm@21/lib";
    let mut host = Fake::new(&["/opt/homebrew/opt/llvm@21/bin"])
        .llvm("/opt/homebrew/opt/llvm@21/bin", 21)
        .file(&format!("{lib}/libclang.dylib"));
    for given in [format!("{lib}/libclang.dylib"), lib.to_owned()] {
        host = host.var("LIBCLANG_PATH", &given);
        let toolset = select(&host).unwrap();
        assert_eq!(toolset.libclang, path(&format!("{lib}/libclang.dylib")));
    }
}

/// An empty `CLANG` is unset: an install directory without a `clang` is
/// not listed as a missing compiler.
#[test]
fn an_empty_clang_lists_only_compilers_that_exist() {
    let host = Fake::new(&["/usr/bin"])
        .var("CLANG", "")
        .file("/usr/lib/llvm-21/lib/libLLVM.so.21.1")
        .llvm("/usr/bin", 18);
    let problems = select(&host).unwrap_err().to_string();
    assert!(
        problems.starts_with("found Clang 18.1.8 (/usr/bin/clang)."),
        "{problems}"
    );
    assert!(!problems.contains("llvm-21"), "{problems}");
}

/// A linker or archiver older than the compiler is passed over for the Rust
/// toolchain's, which is new enough; without either, every missing tool is
/// listed at once.
#[test]
fn older_tools_are_passed_over_and_every_missing_one_is_listed() {
    let host = Fake::new(&["/usr/bin"])
        .tool("/usr/bin/clang", "clang version 22.1.0")
        .tool("/usr/bin/clang++", "clang version 22.1.0")
        .tool("/usr/bin/ld.lld", "LLD 21.1.8")
        .file("/usr/lib64/libclang.so.22.1");
    let problems = select(&host).unwrap_err().0;
    assert_eq!(problems.len(), 3, "{problems:?}");
    assert!(problems[0].contains("LLD 22 or newer (ld.lld)"));
    assert!(problems[1].contains("(llvm-ar)"));
    assert!(problems[2].contains("(llvm-nm)"));
    let mut host = host
        .file("/rust/bin/gcc-ld/ld.lld")
        .file("/rust/bin/llvm-ar")
        .file("/rust/bin/llvm-nm");
    host.rust = Some((
        path("/rust/bin"),
        Version {
            major: 23,
            minor: 1,
            patch: 1,
        },
    ));
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.linker, path("/rust/bin/gcc-ld/ld.lld"));
    assert_eq!(toolset.symbols, path("/rust/bin/llvm-nm"));
    assert_eq!(toolset.libclang, path("/usr/lib64/libclang.so.22.1"));
}

/// Fedora's compat package `clang21` beside the current Clang: the libclang
/// in its own `lib64`.
#[test]
fn fedora_compat_install_has_its_own_libclang() {
    let host = Fake::new(&["/usr/bin"])
        .llvm("/usr/lib64/llvm21/bin", 21)
        .file("/usr/lib64/llvm21/lib64/libclang.so.21.1")
        .file("/usr/lib64/libclang.so.25.1");
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clang, path("/usr/lib64/llvm21/bin/clang"));
    assert_eq!(
        toolset.libclang,
        path("/usr/lib64/llvm21/lib64/libclang.so.21.1")
    );
    let host = Fake::new(&["/usr/bin"]).llvm("/usr/bin", 21);
    let problems = select(&host).unwrap_err().to_string();
    assert!(problems.contains("install libclang 21"), "{problems}");
}

/// A versioned driver on PATH finds its suffixed siblings.
#[test]
fn versioned_drivers_find_suffixed_siblings() {
    let report = "clang version 21.1.2";
    let host = Fake::new(&["/usr/bin"])
        .tool("/usr/bin/clang-21", report)
        .tool("/usr/bin/clang++-21", report)
        .tool("/usr/bin/ld.lld-21", "LLD 21.1.2")
        .tool("/usr/bin/llvm-ar-21", "LLVM version 21.1.2")
        .tool("/usr/bin/llvm-nm-21", "LLVM version 21.1.2")
        .file("/usr/lib64/libclang.so.21.1");
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clangxx, path("/usr/bin/clang++-21"));
    assert_eq!(toolset.linker, path("/usr/bin/ld.lld-21"));
}

#[test]
fn identity_names_every_tool() {
    let host = Fake::new(&["/usr/bin"])
        .llvm("/usr/lib/llvm-21/bin", 21)
        .file("/usr/lib/llvm-21/lib/libclang-21.so.1");
    let toolset = select(&host).unwrap();
    let identity = toolset.identity();
    for tool in [
        "clang 21.1.8 /usr/lib/llvm-21/bin/clang",
        "ld.lld",
        "llvm-ar",
        "libclang-21",
    ] {
        assert!(identity.contains(tool), "{identity}");
    }
}

/// macOS: an Apple clang older than 21 is passed over for Homebrew's LLVM,
/// with its libclang; Homebrew keeps LLD apart, so the linker comes from
/// the Rust toolchain.
#[test]
fn macos_passes_over_xcode_for_homebrew() {
    let mut host = Fake::new(&["/usr/bin", "/opt/homebrew/bin"])
        .tool(
            "/usr/bin/clang",
            "Apple clang version 17.0.0 (clang-1700.0.13.5)",
        )
        .tool(
            "/opt/homebrew/opt/llvm/bin/clang",
            "Homebrew clang version 21.1.8",
        )
        .tool(
            "/opt/homebrew/opt/llvm/bin/clang++",
            "Homebrew clang version 21.1.8",
        )
        .tool(
            "/opt/homebrew/opt/llvm/bin/llvm-ar",
            "Homebrew LLVM version 21.1.8",
        )
        .tool(
            "/opt/homebrew/opt/llvm/bin/llvm-nm",
            "Homebrew LLVM version 21.1.8",
        )
        .file("/opt/homebrew/opt/llvm/lib/libclang.dylib")
        .file("/rust/bin/gcc-ld/ld.lld");
    host.rust = Some((
        path("/rust/bin"),
        Version {
            major: 23,
            minor: 1,
            patch: 1,
        },
    ));
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clang, path("/opt/homebrew/opt/llvm/bin/clang"));
    assert_eq!(toolset.linker, path("/rust/bin/gcc-ld/ld.lld"));
    assert_eq!(toolset.archiver, path("/opt/homebrew/opt/llvm/bin/llvm-ar"));
    assert_eq!(
        toolset.libclang,
        path("/opt/homebrew/opt/llvm/lib/libclang.dylib")
    );
}

/// macOS with the Command Line Tools alone: Apple clang 21, whose
/// `/usr/bin/clang` is a launcher, so its libclang is found through its
/// built-in headers' directory; the linker, archiver and symbol lister are
/// the Rust toolchain's.
#[test]
fn macos_command_line_tools_alone() {
    let tools = "/Library/Developer/CommandLineTools/usr";
    let report = "Apple clang version 21.0.0 (clang-2100.3.34.2)\nTarget: arm64-apple-darwin";
    let mut host = Fake::new(&["/usr/bin"])
        .tool("/usr/bin/clang", report)
        .tool("/usr/bin/clang++", report)
        .file(&format!("{tools}/lib/libclang.dylib"))
        .file("/rust/bin/gcc-ld/ld.lld")
        .file("/rust/bin/llvm-ar")
        .file("/rust/bin/llvm-nm");
    host.resources.insert(
        path("/usr/bin/clang"),
        path(&format!("{tools}/lib/clang/21")),
    );
    host.rust = Some((
        path("/rust/bin"),
        Version {
            major: 23,
            minor: 1,
            patch: 1,
        },
    ));
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clang, path("/usr/bin/clang"));
    assert_eq!(toolset.linker, path("/rust/bin/gcc-ld/ld.lld"));
    assert_eq!(toolset.archiver, path("/rust/bin/llvm-ar"));
    assert_eq!(
        toolset.libclang,
        path(&format!("{tools}/lib/libclang.dylib"))
    );
}

/// Homebrew's `clang++` is a link to `clang`, and that to `clang-NN`: the
/// C++ driver keeps the name it is run by, since `clang-NN` links no C++
/// library.
#[test]
fn the_cxx_driver_keeps_its_name() {
    let cellar = "/opt/homebrew/Cellar/llvm/23.1.4/bin";
    let mut host = Fake::new(&["/usr/bin"])
        .tool(
            &format!("{cellar}/clang-23"),
            "Homebrew clang version 23.1.4\n",
        )
        .link(
            "/opt/homebrew/opt/llvm/bin/clang",
            &format!("{cellar}/clang-23"),
        )
        .link(
            "/opt/homebrew/opt/llvm/bin/clang++",
            "/opt/homebrew/opt/llvm/bin/clang",
        )
        .file("/opt/homebrew/Cellar/llvm/23.1.4/lib/libclang.dylib")
        .file("/rust/bin/gcc-ld/ld.lld")
        .file("/rust/bin/llvm-ar")
        .file("/rust/bin/llvm-nm");
    host.rust = Some((
        path("/rust/bin"),
        Version {
            major: 23,
            minor: 1,
            patch: 1,
        },
    ));
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clang, path("/opt/homebrew/opt/llvm/bin/clang"));
    assert_eq!(toolset.clangxx, path("/opt/homebrew/opt/llvm/bin/clang++"));
    assert_eq!(
        toolset.libclang,
        path("/opt/homebrew/Cellar/llvm/23.1.4/lib/libclang.dylib")
    );
}

/// ccache's directory of links first on PATH, as Fedora sets it: the build
/// runs the compilers through ccache, by the names found there, and finds
/// the tools and libclang beside the Clang behind it.
#[test]
fn a_ccache_wrapper_stays_in_front_of_the_real_install() {
    let host = Fake::new(&["/usr/lib64/ccache", "/usr/bin"])
        // ccache answers `--version` with the compiler's report.
        .tool("/usr/bin/ccache", "clang version 21.1.8\n")
        .link("/usr/lib64/ccache/clang", "/usr/bin/ccache")
        .link("/usr/lib64/ccache/clang++", "/usr/bin/ccache")
        .llvm("/usr/lib64/llvm21/bin", 21)
        .link("/usr/bin/clang", "/usr/lib64/llvm21/bin/clang")
        .file("/usr/lib64/llvm21/lib64/libclang.so.21.1");
    let toolset = select(&host).unwrap();
    assert_eq!(toolset.clang, path("/usr/lib64/ccache/clang"));
    assert_eq!(toolset.clangxx, path("/usr/lib64/ccache/clang++"));
    assert_eq!(toolset.linker, path("/usr/lib64/llvm21/bin/ld.lld"));
    assert_eq!(
        toolset.libclang,
        path("/usr/lib64/llvm21/lib64/libclang.so.21.1")
    );
    // The wrapper and the compiler it runs are one candidate.
    let found = candidates(&host);
    assert_eq!(found.len(), 1, "{found:?}");
}

#[test]
fn host_and_target_compilers_are_selected_independently() {
    let host = Fake::new(&["/host/bin", "/target/bin"])
        .llvm("/host/bin", 21)
        .llvm("/target/bin", 21)
        .file("/host/lib/libclang.so.21.1")
        .file("/target/lib/libclang.so.21.1")
        .var("CLANG", "/host/bin/clang")
        .var("STELA_TARGET_CLANG", "/target/bin/clang");
    assert_eq!(select(&host).unwrap().clang, path("/host/bin/clang"));
    let target = select_for(&host, TARGET_VARIABLE).unwrap();
    assert_eq!(target.clang, path("/target/bin/clang"));
    assert_eq!(target.clangxx, path("/target/bin/clang++"));
    assert_eq!(target.archiver, path("/target/bin/llvm-ar"));
    let missing = host.var("STELA_TARGET_CLANG", "/missing/clang");
    let error = select_for(&missing, TARGET_VARIABLE)
        .unwrap_err()
        .to_string();
    assert!(error.contains("STELA_TARGET_CLANG"), "{error}");
    assert!(error.contains("/missing/clang"), "{error}");
}
