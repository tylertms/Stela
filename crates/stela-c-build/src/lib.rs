//! Shared C tool selection, freestanding target ABI and hosted sanitizer builds.
//! Callers may select tools through cc's standard environment; COFF archives use
//! the active Rust toolchain's LLVM archiver.
use std::{env, path::PathBuf, process::Command};

#[cfg(feature = "linux")]
pub mod linux;
#[cfg(feature = "native")]
pub mod linux_build;
#[cfg(feature = "linux-registry")]
pub mod linux_capture;
#[cfg(feature = "linux-registry")]
pub mod linux_radio;
#[cfg(feature = "native")]
pub mod native;

#[cfg(feature = "linux-registry")]
pub mod inputs;
#[cfg(feature = "linux-registry")]
pub mod linux_registry;

/// Whether the caller chose a tool through cc's standard environment.
pub fn selected(tool: &str) -> bool {
    let target = env::var("TARGET").expect("Cargo supplies TARGET");
    let names = [
        format!("{tool}_{target}"),
        format!("{tool}_{}", target.replace('-', "_")),
        format!("TARGET_{tool}"),
        tool.into(),
    ];
    for name in &names {
        println!("cargo:rerun-if-env-changed={name}");
    }
    names.iter().any(|name| env::var_os(name).is_some())
}

/// What freestanding C for the mount compiles with whatever the host's Clang
/// defaults to: distributions turn on hardening the mount's runtime has no
/// support for (Arch's and Nix's stack protector calls `__stack_chk_fail`;
/// others add CET branch markers or `_FORTIFY_SOURCE`), and their
/// configuration files can add any flag, stack clash probes among them.
/// Clang offers no stack clash probes for the mount's COFF target, and
/// rejects the flag that turns them off there.
pub const UNHARDENED: [&str; 4] = [
    "--no-default-config",
    "-fno-stack-protector",
    "-fcf-protection=none",
    "-U_FORTIFY_SOURCE",
];

/// The Clang to compile with when the caller chose no compiler: for cross
/// builds, `STELA_TARGET_CLANG` if set; otherwise `CLANG`, which xtask sets
/// to its host choice, or `clang` on PATH. The
/// build reruns when either changes, so objects never mix compilers.
pub fn clang() -> std::ffi::OsString {
    println!("cargo:rerun-if-env-changed=CLANG");
    println!("cargo:rerun-if-env-changed=STELA_TARGET_CLANG");
    let target_clang = (env::var_os("TARGET") != env::var_os("HOST"))
        .then(|| env::var_os("STELA_TARGET_CLANG"))
        .flatten()
        .filter(|clang| !clang.is_empty());
    target_clang
        .or_else(|| env::var_os("CLANG"))
        .filter(|clang| !clang.is_empty())
        .unwrap_or_else(|| {
            println!("cargo:rerun-if-env-changed=PATH");
            "clang".into()
        })
}

/// Apple's ar produces a BSD archive even for COFF inputs; the active Rust
/// toolchain's LLVM archiver does not.
pub fn rust_archiver() -> PathBuf {
    let output = Command::new(env::var_os("RUSTC").expect("Cargo supplies RUSTC"))
        .args(["--print", "sysroot"])
        .output()
        .expect("cannot locate the active Rust sysroot");
    assert!(output.status.success(), "rustc --print sysroot failed");
    let sysroot = String::from_utf8(output.stdout).expect("Rust sysroot is not UTF-8");
    let host = env::var("HOST").expect("Cargo supplies HOST");
    let archiver = PathBuf::from(sysroot.trim())
        .join("lib/rustlib")
        .join(host)
        .join(format!("bin/llvm-ar{}", env::consts::EXE_SUFFIX));
    assert!(
        archiver.is_file(),
        "install llvm-tools-preview for the active Rust toolchain"
    );
    archiver
}
