//! The pinned tools the GPU builds run, installed under `.tools/gpu`: Meson,
//! Ninja and Mesa's Python generator modules in a virtual environment, and the
//! Rust binding generators NAK's build calls.
use crate::tools::{self, exec};
use anyhow::{Context, Result, bail, ensure};
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

/// Mesa 26.2 needs Python 3.10 or newer for its generators.
const PYTHON_MINOR: u32 = 10;
/// The python-build-standalone CPython installed when no Python 3.10 or newer
/// is on PATH: release, version, and each host's archive SHA-256.
const STANDALONE_RELEASE: &str = "20261003";
const STANDALONE_VERSION: &str = "3.13.16";
const STANDALONE: [(&str, &str); 3] = [
    (
        "x86_64-apple-darwin",
        "b4dad38ba6a344555ccb71a1b08caad0a6c0dda88c5803658bc95bd7f04e9f5c",
    ),
    (
        "aarch64-apple-darwin",
        "9e01f63bbb08576cd9c8bc2d0564d098cb30c8453a0cd4bcf6aef458f6d2a147",
    ),
    (
        "x86_64-unknown-linux-gnu",
        "4595c5589fff7bf0cb158d9a88a797e0d791fa33830770fcb7bf3f4b104feeae",
    ),
];
const PYTHON_PACKAGES: [&str; 6] = [
    "meson==1.11.2",
    "ninja==1.13.2",
    "pkgconf==2.5.1.post2",
    "Mako==1.4.3",
    "packaging==26.3",
    "PyYAML==6.0.3",
];
const RUST_PACKAGES: [&str; 2] = ["bindgen-cli@0.73.2", "cbindgen@0.29.4"];
/// The pins an installation was made from, beside it.
const STAMP: &str = "stela-pins";
/// Where a virtual environment keeps its programs, under `.tools/gpu`.
pub const GENERATORS: &str = if cfg!(windows) {
    "generators/Scripts"
} else {
    "generators/bin"
};
/// The virtual environment's Python, under `.tools/gpu/generators`.
const VENV_PYTHON: &str = if cfg!(windows) {
    "Scripts/python.exe"
} else {
    "bin/python3"
};

fn directory() -> PathBuf {
    tools::root().join(".tools/gpu")
}

/// Meson's pkg-config. The wheel's launcher loses its output to whoever
/// captures it, so Meson runs the program it wraps, which needs no system one.
pub fn pkg_config() -> Result<String> {
    let generators = directory().join("generators");
    let packages = if cfg!(windows) {
        generators.join("Lib/site-packages")
    } else {
        fs::read_dir(generators.join("lib"))?
            .next()
            .context("the generators name no Python")??
            .path()
            .join("site-packages")
    };
    Ok(super::support::slashed(&packages.join(format!(
        "pkgconf/.bin/pkgconf{}",
        std::env::consts::EXE_SUFFIX
    ))))
}

/// Installs whatever is missing or pinned differently, then returns.
pub fn install() -> Result<()> {
    let generators = directory().join("generators");
    let pins = PYTHON_PACKAGES.join("\n") + "\n";
    if fs::read_to_string(generators.join(STAMP)).ok().as_deref() != Some(pins.as_str()) {
        let python = generators.join(VENV_PYTHON);
        if !python.is_file() {
            exec(
                Command::new(base_python()?)
                    .args(["-m", "venv"])
                    .arg(&generators),
            )?;
        }
        exec(
            Command::new(&python)
                .args(["-m", "pip", "install", "-q", "--disable-pip-version-check"])
                .args(PYTHON_PACKAGES),
        )?;
        fs::write(generators.join(STAMP), pins)?;
    }
    let rust = directory().join("rust");
    let pins = RUST_PACKAGES.join("\n") + "\n";
    if fs::read_to_string(rust.join(STAMP)).ok().as_deref() != Some(pins.as_str()) {
        exec(
            tools::cargo()
                .args(["install", "--locked", "--quiet", "--root"])
                .arg(&rust)
                .args(RUST_PACKAGES),
        )?;
        fs::write(rust.join(STAMP), pins)?;
    }
    Ok(())
}

/// `STELA_GPU_PYTHON`, else the first Python 3.10 or newer on PATH, else the
/// pinned standalone CPython.
fn base_python() -> Result<PathBuf> {
    match found_python()? {
        Some(python) => Ok(python),
        None => standalone_python(),
    }
}

/// What the preflight says of Python: nothing when `STELA_GPU_PYTHON` or a
/// Python 3.10 or newer is there, or the standalone one can be fetched.
pub(crate) fn python_problem() -> Option<String> {
    if found_python().ok().flatten().is_some() {
        return None;
    }
    let missing: Vec<&str> = ["curl", "tar"]
        .into_iter()
        .filter(|tool| tools::which(tool).is_none())
        .collect();
    (!missing.is_empty()).then(|| {
        format!(
            "install Python 3.{PYTHON_MINOR} or newer, or {} to fetch one",
            missing.join(" and ")
        )
    })
}

/// `STELA_GPU_PYTHON`, else the first Python 3.10 or newer on PATH.
fn found_python() -> Result<Option<PathBuf>> {
    if let Some(python) = std::env::var_os("STELA_GPU_PYTHON") {
        return Ok(Some(python.into()));
    }
    for name in [
        "python3",
        "python3.14",
        "python3.13",
        "python3.12",
        "python3.11",
        "python3.10",
    ] {
        let Some(python) = tools::which(name) else {
            continue;
        };
        let check = format!("import sys; sys.exit(sys.version_info < (3, {PYTHON_MINOR}))");
        if Command::new(&python)
            .args(["-c", &check])
            .status()?
            .success()
        {
            return Ok(Some(python));
        }
    }
    Ok(None)
}

/// Downloads and unpacks the pinned standalone CPython for this host into
/// `.tools/gpu/python`, checking its archive digest first.
fn standalone_python() -> Result<PathBuf> {
    let root = directory().join("python");
    let python = root.join("bin/python3");
    if python.is_file() {
        return Ok(python);
    }
    let host = format!(
        "{}-{}",
        std::env::consts::ARCH,
        match std::env::consts::OS {
            "macos" => "apple-darwin",
            "linux" => "unknown-linux-gnu",
            os => os,
        }
    );
    let Some((_, digest)) = STANDALONE.iter().find(|(target, _)| *target == host) else {
        bail!("install Python 3.{PYTHON_MINOR} or newer, or name one with STELA_GPU_PYTHON");
    };
    let name = format!(
        "cpython-{STANDALONE_VERSION}+{STANDALONE_RELEASE}-{host}-install_only_stripped.tar.gz"
    );
    let archive = directory().join(&name);
    fs::create_dir_all(directory())?;
    exec(
        Command::new("curl")
            .args(["-fsSL", "--retry", "3", "-o"])
            .arg(&archive)
            .arg(format!(
                "https://github.com/astral-sh/python-build-standalone/releases/download/{STANDALONE_RELEASE}/{}",
                name.replace('+', "%2B")
            )),
    )?;
    let actual = format!("{:x}", Sha256::digest(fs::read(&archive)?));
    ensure!(
        actual == *digest,
        "{name} has SHA-256 {actual}, not {digest}"
    );
    // The archive's single top-level directory is `python`.
    exec(
        Command::new("tar")
            .arg("-xzf")
            .arg(&archive)
            .arg("-C")
            .arg(directory()),
    )?;
    fs::remove_file(&archive)?;
    ensure!(python.is_file(), "{name} has no bin/python3");
    Ok(python)
}

/// The LLVM tools the GPU builds run: [`crate::clang`]'s one choice.
pub(super) type Toolchain = crate::clang::Toolset;

/// The chosen tools, with the GPU build trees that other tools or flags
/// left removed.
pub(super) fn discover() -> Result<Toolchain> {
    let toolchain = crate::clang::target_toolset()?.clone();
    rebuild_if_changed(
        &toolchain,
        crate::clang::toolset()?,
        &super::support::build_root(),
    )?;
    Ok(toolchain)
}

/// What the GPU build trees under `target/gpu` were made with, beside them.
const IDENTITY: &str = "toolchain";

/// Every flag, define and Meson option the GPU builds pass on their own,
/// or drop from Meson's commands, whose change must rebuild their objects
/// and Meson's probe results as a compiler change does.
fn flags() -> String {
    use super::{llvm, nvk, sysroot};
    [
        nvk::OPTIONS.as_slice(),
        nvk::TARGET_FLAGS.as_slice(),
        nvk::LIBC_TEST_FLAGS.as_slice(),
        nvk::HOST_FLAGS.as_slice(),
        nvk::HOST_DEFINES.as_slice(),
        nvk::LATE_DEFINES.as_slice(),
        nvk::ADDED_DEFINES.as_slice(),
        &[sysroot::TARGET, sysroot::POSITION_INDEPENDENT],
        sysroot::FREESTANDING.as_slice(),
        sysroot::PROBE_LINK.as_slice(),
        llvm::CXX_FLAGS.as_slice(),
        llvm::CXX_ONLY.as_slice(),
        llvm::TARGET_CXX_FLAGS.as_slice(),
        llvm::TOOL_WINDOWS_FLAGS.as_slice(),
        llvm::TOOL_LIBRARIES,
        llvm::BLAKE3_FLAGS.as_slice(),
    ]
    .concat()
    .join(" ")
}

/// Removes the build trees under `root` when the tools or flags they were made
/// with differ from these: their objects, Meson's configuration and the probe
/// results it recorded all depend on them, and only the sources' times are
/// watched otherwise.
fn rebuild_if_changed(toolchain: &Toolchain, host: &Toolchain, root: &Path) -> Result<()> {
    let identity = format!(
        "target:
{}host:
{}flags {}\n",
        toolchain.identity(),
        host.identity(),
        flags()
    );
    let record = root.join(IDENTITY);
    let recorded = fs::read_to_string(&record).ok();
    if recorded.as_deref() == Some(identity.as_str()) {
        return Ok(());
    }
    if let Some(recorded) = &recorded {
        let changed: Vec<&str> = identity
            .lines()
            .filter(|line| !recorded.lines().any(|old| old == *line))
            .collect();
        eprintln!(
            "The GPU builds' tools or flags changed ({}); rebuilding {}",
            changed.join("; "),
            root.display()
        );
    }
    let mut announced = recorded.is_some();
    for entry in fs::read_dir(root).into_iter().flatten() {
        let path = entry?.path();
        if path.is_dir() {
            if !announced {
                eprintln!(
                    "{} records no tools it was built with; rebuilding it",
                    root.display()
                );
                announced = true;
            }
            fs::remove_dir_all(&path)
                .with_context(|| format!("cannot remove {}", path.display()))?;
        }
    }
    fs::create_dir_all(root)?;
    fs::write(record, identity)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn toolset(clang: &str) -> Toolchain {
        Toolchain {
            version: crate::clang::Version {
                major: 21,
                minor: 1,
                patch: 8,
            },
            clang: clang.into(),
            clangxx: "/llvm/bin/clang++".into(),
            linker: "/llvm/bin/ld.lld".into(),
            archiver: "/llvm/bin/llvm-ar".into(),
            symbols: "/llvm/bin/llvm-nm".into(),
            libclang: "/llvm/lib/libclang.so.21.1".into(),
        }
    }

    /// A change of compiler removes the trees built with the old one; the
    /// same compiler keeps them.
    #[test]
    fn a_compiler_change_rebuilds_the_trees() {
        let root = tempfile::tempdir().unwrap();
        let tree = root.path().join("nvk/sysroot");
        fs::create_dir_all(&tree).unwrap();
        // A tree from before the record existed.
        rebuild_if_changed(
            &toolset("/llvm/bin/clang"),
            &toolset("/llvm/bin/clang"),
            root.path(),
        )
        .unwrap();
        assert!(!tree.exists());
        fs::create_dir_all(&tree).unwrap();
        rebuild_if_changed(
            &toolset("/llvm/bin/clang"),
            &toolset("/llvm/bin/clang"),
            root.path(),
        )
        .unwrap();
        assert!(tree.exists());
        rebuild_if_changed(
            &toolset("/home/me/wrapper/clang"),
            &toolset("/llvm/bin/clang"),
            root.path(),
        )
        .unwrap();
        assert!(!tree.exists());
        fs::create_dir_all(&tree).unwrap();
        rebuild_if_changed(
            &toolset("/home/me/wrapper/clang"),
            &toolset("/new/host/clang"),
            root.path(),
        )
        .unwrap();
        assert!(!tree.exists());
        let record = fs::read_to_string(root.path().join(IDENTITY)).unwrap();
        assert!(record.contains("/home/me/wrapper/clang"), "{record}");
        assert!(record.contains("-fno-stack-protector"), "{record}");
        // The probes' link flags and Meson's options key the trees too.
        for flag in [
            super::super::sysroot::PROBE_LINK.join(" ").as_str(),
            super::super::sysroot::POSITION_INDEPENDENT,
            "-Dbuildtype=release",
            "-mtls-dialect=gnu2",
        ] {
            assert!(record.contains(flag), "{flag}: {record}");
        }
    }

    /// A distribution's Clang that hardens by default, as Arch's turns on
    /// the stack protector, simulated by a configuration file read first:
    /// the build's own flags still leave every object free of the
    /// protector, whose `__stack_chk_fail` Stela's C library lacks.
    #[test]
    #[ignore = "manual: opt-in"]
    fn explicit_flags_override_a_hardened_default() {
        let toolset = crate::clang::toolset().unwrap();
        let scratch = tempfile::tempdir().unwrap();
        let config = scratch.path().join("hardened.cfg");
        fs::write(
            &config,
            "-fstack-protector-strong\n-fstack-clash-protection\n-fcf-protection=full\n\
             -D_FORTIFY_SOURCE=3\n",
        )
        .unwrap();
        let source = scratch.path().join("probe.c");
        // A local array is what the strong protector guards.
        fs::write(
            &source,
            "void fill(char *);\nint probe(void) { char buffer[64]; fill(buffer); return \
             buffer[3]; }\n#ifdef _FORTIFY_SOURCE\n#error fortified\n#endif\n",
        )
        .unwrap();
        let compile = |flags: &[&str], object: &str| {
            let object = scratch.path().join(object);
            let status = Command::new(&toolset.clang)
                .arg(format!("--config={}", config.display()))
                .args(["--target=x86_64-unknown-linux-gnu", "-O2", "-c"])
                .args(flags)
                .arg(&source)
                .arg("-o")
                .arg(&object)
                .status()
                .unwrap();
            (status.success(), object)
        };
        let symbols = |object: &Path| {
            let output = Command::new(&toolset.symbols).arg(object).output().unwrap();
            String::from_utf8(output.stdout).unwrap()
        };
        let endbr64 = |object: &Path| {
            let bytes = fs::read(object).unwrap();
            bytes.windows(4).any(|w| w == [0xf3, 0x0f, 0x1e, 0xfa])
        };
        // The configuration takes effect without the build's flags.
        let (built, hardened) = compile(&["-U_FORTIFY_SOURCE"], "hardened.o");
        assert!(built);
        assert!(symbols(&hardened).contains("__stack_chk_fail"));
        assert!(endbr64(&hardened));
        for flags in [
            super::super::sysroot::FREESTANDING.as_slice(),
            super::super::nvk::TARGET_FLAGS.as_slice(),
            super::super::nvk::LIBC_TEST_FLAGS.as_slice(),
        ] {
            // The target's own machine flags are another target's.
            let flags: Vec<&str> = flags
                .iter()
                .copied()
                .filter(|flag| !flag.starts_with("-m") && !flag.starts_with("--target"))
                .collect();
            let (built, object) = compile(&flags, "free.o");
            assert!(built, "{flags:?}");
            let listed = symbols(&object);
            assert!(!listed.contains("__stack_chk"), "{flags:?}: {listed}");
            assert!(!endbr64(&object), "{flags:?}");
        }
    }
}
