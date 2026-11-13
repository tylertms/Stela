//! The ARM64e macOS components Stela installs in the guest, built with the
//! host's clang against xnu's open-source headers: no macOS SDK.
//!
//! The StelaGPU kext compiles to relocatable objects beside its
//! `Info.plist` in `target/guest/StelaGPU.kext`, whose final link is against
//! one boot kernel collection. The Metal plugin links here, into
//! `target/guest/StelaGPU.bundle`. No released `ld64.lld` links arm64e's
//! authenticated pointers, so `stela-loader` links both. The kext also
//! carries Stela.app when a built package is present ([`app`]).
use crate::tools::{self, exec};
use anyhow::{Context, Result, ensure};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};
use stela_loader::{aux_kc::TRUSTED_CODE_FILE, trust_cache::CodeHash};

pub mod app;
pub mod sdk;
pub mod wire;

/// The macOS the guest components target, as clang's `-target` names it.
const TARGET: &str = "arm64e-apple-macos27.0";
/// The kext's path buffers, `char path[256]`, NUL included.
const KERNEL_PATH_BYTES: usize = 256;
/// Where the kext installs the plugin's files (`install.cpp`'s `kPlugin`).
const PLUGIN_PATH: &str = "/System/Volumes/Data/Library/GPUBundles/StelaGPU.bundle/";
/// What the Metal plugin's `Info.plist` and signature identify it as.
const BUNDLE_IDENTIFIER: &str = "com.stela.StelaGPU.Metal";
/// The Metal plugin's executable in its bundle.
const BUNDLE_EXECUTABLE: &str = "Contents/MacOS/StelaGPU";
/// The Metal plugin's slices: Metal loads it into processes of either.
const BUNDLE_TARGETS: [(&str, &str); 2] = [("arm64", "arm64-apple-macos27.0"), ("arm64e", TARGET)];

/// Builds every guest component.
pub fn run() -> Result<()> {
    let root = tools::root();
    let xnu = root.join("third_party/xnu");
    let output = root.join("target/guest");
    let include = output.join("include");
    sdk::generate(&xnu, &include)?;
    wire::generate(&root.join("crates/stela-driver-gpu/src/metal"), &include)?;
    let component = root.join("guest/stela-gpu");
    let bundle = build_bundle(&component, &output, &include)?;
    println!("StelaGPU bundle: {}", bundle.display());
    fs::write(include.join("stela/bundle.h"), bundle_header(&bundle)?)?;
    fs::write(include.join("stela/app.h"), app_header(root)?)?;
    let kext = build_kext(&component, &output, &xnu, &include)?;
    let code = stela_loader::plugin::code_hashes(&fs::read(bundle.join(BUNDLE_EXECUTABLE))?)?;
    fs::write(kext.join(TRUSTED_CODE_FILE), code.concat())?;
    embed_kext(&kext, &code, &output.join("StelaGPU.kext.rs"))?;
    stela_cli::gpu::Payload::write(&output)?;
    println!("StelaGPU kext: {}", kext.display());
    Ok(())
}

/// `stela/app.h`: the Stela.app package [`app::locate`] finds, or a build
/// without one, which the kext reports at boot.
fn app_header(root: &Path) -> Result<String> {
    let Some(path) = app::locate(root)? else {
        println!(
            "Stela.app: no package (set {} or run `cargo xtask app`); the kext installs no app",
            app::VARIABLE
        );
        return Ok(app::absent_header());
    };
    let package = app::Package::read(&path)
        .with_context(|| format!("the Stela.app package {}", path.display()))?;
    let (bytes, archived) = package.sizes()?;
    println!(
        "Stela.app {} ({}): {bytes} bytes, {archived} archived, from {}",
        package.version,
        package.revision,
        path.display()
    );
    package.header()
}

/// Compiles each source of `<component>/bundle` once per slice, under
/// `<output>/StelaGPU.objects`, and links the slices into
/// `<output>/StelaGPU.bundle`.
fn build_bundle(component: &Path, output: &Path, include: &Path) -> Result<PathBuf> {
    let source = component.join("bundle");
    let objects = output.join("StelaGPU.objects");
    let bundle = output.join("StelaGPU.bundle");
    for directory in [&objects, &bundle] {
        if directory.exists() {
            fs::remove_dir_all(directory)?;
        }
    }
    let clang = &crate::clang::target_toolset()?.clang;
    let resources = Command::new(clang).arg("-print-resource-dir").output()?;
    ensure!(
        resources.status.success(),
        "clang names no resource directory"
    );
    let resources = PathBuf::from(String::from_utf8(resources.stdout)?.trim());
    let mut sources = Vec::new();
    for entry in fs::read_dir(&source)? {
        let path = entry?.path();
        if path.extension().is_some_and(|extension| extension == "m") {
            sources.push(path);
        }
    }
    sources.sort();
    let mut slices = Vec::new();
    for (architecture, target) in BUNDLE_TARGETS {
        let directory = objects.join(architecture);
        fs::create_dir_all(&directory)?;
        let mut compiled = Vec::with_capacity(sources.len());
        for path in &sources {
            let object = directory
                .join(path.file_stem().unwrap())
                .with_extension("o");
            let mut command = Command::new(clang);
            command
                .args(BUNDLE_FLAGS)
                .arg(format!("--target={target}"))
                .arg("-isystem")
                .arg(resources.join("include"))
                .arg("-I")
                .arg(&source)
                .arg("-I")
                .arg(component.join("include"))
                .arg("-I")
                .arg(include);
            exec(command.arg("-c").arg(path).arg("-o").arg(&object))?;
            compiled.push(fs::read(&object)?);
        }
        slices.push(compiled);
    }
    let slices: Vec<Vec<&[u8]>> = slices
        .iter()
        .map(|slice| slice.iter().map(Vec::as_slice).collect())
        .collect();
    let executable = stela_loader::plugin::link(BUNDLE_IDENTIFIER, &slices)?;
    let contents = bundle.join("Contents");
    fs::create_dir_all(contents.join("MacOS"))?;
    fs::copy(source.join("Info.plist"), contents.join("Info.plist"))?;
    fs::write(bundle.join(BUNDLE_EXECUTABLE), executable)?;
    Ok(bundle)
}

/// Objective-C against the bundle's own declarations of macOS: no SDK.
const BUNDLE_FLAGS: &[&str] = &[
    // No distribution configuration file reaches another platform's code.
    "--no-default-config",
    "-xobjective-c",
    "-std=gnu17",
    "-O2",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-Wno-unused-parameter",
    "-Wno-objc-protocol-method-implementation",
    "-fobjc-arc",
    "-fblocks",
    "-fobjc-runtime=macosx-27.0",
    "-fno-common",
    "-fno-unwind-tables",
    "-fno-asynchronous-unwind-tables",
    "-nostdinc",
];

/// Compiles each source of `<component>/kext` into `<output>/StelaGPU.kext`
/// beside its `Info.plist`.
fn build_kext(component: &Path, output: &Path, xnu: &Path, include: &Path) -> Result<PathBuf> {
    let source = component.join("kext");
    let bundle = output.join("StelaGPU.kext");
    if bundle.exists() {
        fs::remove_dir_all(&bundle)?;
    }
    fs::create_dir_all(&bundle)?;
    let info = fs::read_to_string(source.join("Info.plist"))?;
    let (identifier, version) = identity(&info)?;
    fs::write(bundle.join("Info.plist"), &info)?;
    let clang = &crate::clang::target_toolset()?.clangxx;
    for entry in fs::read_dir(&source)? {
        let path = entry?.path();
        if path.extension().is_none_or(|extension| extension != "cpp") {
            continue;
        }
        let object = bundle.join(path.file_stem().unwrap()).with_extension("o");
        let mut command = Command::new(clang);
        command
            .args(KEXT_FLAGS)
            .arg(format!("--target={TARGET}"))
            .arg(format!("-DSTELA_KEXT_IDENTIFIER=\"{identifier}\""))
            .arg(format!("-DSTELA_KEXT_VERSION=\"{version}\""))
            .arg("-I")
            .arg(component.join("include"));
        for headers in XNU_HEADERS {
            command.arg("-isystem").arg(xnu.join(headers));
        }
        command.arg("-isystem").arg(include);
        exec(command.arg("-c").arg(&path).arg("-o").arg(&object))?;
    }
    Ok(bundle)
}

/// `stela/bundle.h`: the Metal plugin `bundle` as the kext installs it, each
/// regular file's path in the bundle, mode, SHA-256 and bytes, in path order.
/// Its executable is the code the boot's trust cache lists, so the kext and
/// the trust cache always name the same plugin.
fn bundle_header(bundle: &Path) -> Result<String> {
    use std::fmt::Write as _;
    let mut files = Vec::new();
    let mut directories = vec![bundle.to_path_buf()];
    while let Some(directory) = directories.pop() {
        for entry in fs::read_dir(&directory)? {
            let entry = entry?;
            let kind = entry.file_type()?;
            ensure!(
                kind.is_dir() || kind.is_file(),
                "{} is neither a file nor a directory",
                entry.path().display()
            );
            if kind.is_dir() {
                directories.push(entry.path());
            } else {
                files.push(entry.path());
            }
        }
    }
    let mut files: Vec<(String, Vec<u8>)> = files
        .into_iter()
        .map(|path| {
            let name = path
                .strip_prefix(bundle)?
                .to_str()
                .context("a file name is not UTF-8")?;
            ensure!(
                name.bytes()
                    .all(|byte| byte.is_ascii_graphic() && byte != b'"' && byte != b'\\'),
                "{name:?} is not a path the kext spells"
            );
            ensure!(
                PLUGIN_PATH.len() + name.len() < KERNEL_PATH_BYTES,
                "{name:?} is longer than the kext's {KERNEL_PATH_BYTES}-byte paths"
            );
            let bytes = fs::read(&path)?;
            ensure!(!bytes.is_empty(), "{name} is empty");
            Ok((name.to_owned(), bytes))
        })
        .collect::<Result<_>>()?;
    files.sort();
    let mut header = String::from(
        "/* StelaGPU.bundle as `cargo xtask guest` built it, which the kext installs:\n \
         * each file's path in the bundle, mode, SHA-256 and bytes. */\n\
         #pragma once\n\
         struct stela_bundle_file {\n\
         \tconst char *path;\n\
         \tunsigned mode;\n\
         \tunsigned char sha256[32];\n\
         \tconst unsigned char *bytes;\n\
         \tunsigned long length;\n\
         };\n",
    );
    for (index, (_, bytes)) in files.iter().enumerate() {
        c_bytes(&mut header, &format!("stela_bundle_bytes{index}"), bytes)?;
    }
    header.push_str("static const struct stela_bundle_file stela_bundle_files[] = {\n");
    for (index, (name, bytes)) in files.iter().enumerate() {
        let mode = if name == BUNDLE_EXECUTABLE {
            0o755
        } else {
            0o644
        };
        let digest = c_digest(bytes);
        writeln!(
            header,
            "\t{{\"{name}\", 0{mode:o}, {{{digest}}}, stela_bundle_bytes{index}, sizeof stela_bundle_bytes{index}}},"
        )?;
    }
    header.push_str(
        "};\n#define STELA_BUNDLE_FILES (sizeof stela_bundle_files / sizeof stela_bundle_files[0])\n",
    );
    Ok(header)
}

/// Appends `static const unsigned char <name>[]` holding `bytes`.
fn c_bytes(header: &mut String, name: &str, bytes: &[u8]) -> Result<()> {
    use std::fmt::Write as _;
    write!(header, "static const unsigned char {name}[] = {{")?;
    for (offset, byte) in bytes.iter().enumerate() {
        if offset % 24 == 0 {
            header.push_str("\n\t");
        }
        write!(header, "{byte},")?;
    }
    header.push_str("\n};\n");
    Ok(())
}

/// `bytes`' SHA-256 as the body of a C array initializer.
fn c_digest(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte},"))
        .collect()
}

/// Writes the Rust `stela.efi` includes for `kext`'s `INFO` and `OBJECTS`, as
/// it links them against each boot's own kernel, and `TRUSTED_CODE`, `code`
/// as that boot's trust cache lists it.
fn embed_kext(kext: &Path, code: &[CodeHash], source: &Path) -> Result<()> {
    let mut objects: Vec<_> = fs::read_dir(kext)?
        .map(|entry| Ok(entry?.path()))
        .collect::<Result<_>>()?;
    objects.retain(|path| path.extension().is_some_and(|extension| extension == "o"));
    objects.sort();
    let includes: String = objects
        .iter()
        .map(|path| format!("    include_bytes!({:?}),\n", path.display().to_string()))
        .collect();
    let info = kext.join("Info.plist");
    fs::write(
        source,
        format!(
            "pub static INFO: &[u8] = include_bytes!({:?});\n\
             pub static OBJECTS: &[&[u8]] = &[\n{includes}];\n\
             pub static TRUSTED_CODE: &[[u8; 20]] = &{code:?};\n",
            info.display().to_string()
        ),
    )?;
    Ok(())
}

/// A kext's code generation: Apple clang's `-mkernel` set, which upstream
/// clang leaves to the build, and Apple clang's name for its
/// pointer-authentication intrinsics, which xnu's `ptrauth.h` tests. The kernel
/// authenticates every function pointer by its type, `kmod_info`'s first.
const KEXT_FLAGS: &[&str] = &[
    "--no-default-config",
    "-std=gnu++20",
    "-O2",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-mkernel",
    "-fapple-kext",
    "-fptrauth-function-pointer-type-discrimination",
    "-fno-rtti",
    "-fno-exceptions",
    "-fno-builtin",
    "-fno-common",
    "-fno-stack-protector",
    "-nostdinc",
    "-nostdinc++",
    "-DKERNEL",
    "-D__PTRAUTH_INTRINSICS__=1",
    // xnu's `OS_PTRAUTH_SIGNED_PTR_AUTH_NULL` passes Apple clang's options
    // string, which upstream clang's `__ptrauth` lacks. The options change
    // only how a load of the field authenticates null, never its layout,
    // and the kext loads no field so signed.
    "-D__ptrauth(key,address,discriminator,...)=__ptrauth(key,address,discriminator)",
];

/// xnu's header roots, in the precedence the kernel SDK's flat tree has.
const XNU_HEADERS: [&str; 6] = [
    "EXTERNAL_HEADERS",
    "bsd",
    "osfmk",
    "iokit",
    "libkern",
    "pexpert",
];

/// The `CFBundleIdentifier` and `CFBundleVersion` of an `Info.plist`, which
/// the kext's `kmod_info` repeats.
fn identity(info: &str) -> Result<(String, String)> {
    let plist: plist::Value = plist::from_bytes(info.as_bytes())?;
    let dictionary = plist
        .as_dictionary()
        .context("Info.plist is not a dictionary")?;
    let [identifier, version] = ["CFBundleIdentifier", "CFBundleVersion"].map(|key| {
        let value = dictionary
            .get(key)
            .and_then(plist::Value::as_string)
            .with_context(|| format!("Info.plist has no {key}"))?;
        ensure!(
            value
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || b".-".contains(&byte)),
            "{key} {value:?} is not a kext's"
        );
        Ok(value.to_owned())
    });
    Ok((identifier?, version?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_identity_comes_from_the_info_plist() {
        let info = r#"<?xml version="1.0" encoding="UTF-8"?>
<plist version="1.0"><dict>
<key>CFBundleIdentifier</key><string>com.stela.Test</string>
<key>CFBundleVersion</key><string>1.2.3</string>
</dict></plist>"#;
        let identity = identity(info).unwrap();
        assert_eq!(identity, ("com.stela.Test".into(), "1.2.3".into()));
        assert!(super::identity(&info.replace("1.2.3", "1\"2")).is_err());
    }

    #[test]
    fn the_kext_carries_each_file_of_the_plugin_with_its_digest() {
        let bundle = tempfile::tempdir().unwrap();
        let macos = bundle.path().join("Contents/MacOS");
        fs::create_dir_all(&macos).unwrap();
        fs::write(bundle.path().join("Contents/Info.plist"), b"abc").unwrap();
        fs::write(macos.join("StelaGPU"), [0xff, 1]).unwrap();
        let header = bundle_header(bundle.path()).unwrap();
        // SHA-256("abc") begins ba 78 16 bf.
        assert!(header.contains("\t{\"Contents/Info.plist\", 0644, {186,120,22,191,"));
        assert!(header.contains("stela_bundle_bytes0[] = {\n\t97,98,99,\n};"));
        assert!(header.contains("\t{\"Contents/MacOS/StelaGPU\", 0755, {"));
        assert!(header.contains("stela_bundle_bytes1[] = {\n\t255,1,\n};"));
        fs::write(bundle.path().join("Contents/empty"), b"").unwrap();
        assert!(bundle_header(bundle.path()).is_err());
        fs::remove_file(bundle.path().join("Contents/empty")).unwrap();
        let long = "x".repeat(KERNEL_PATH_BYTES - PLUGIN_PATH.len());
        fs::write(bundle.path().join(&long), b"x").unwrap();
        assert!(bundle_header(bundle.path()).is_err());
        let install = include_str!("../../guest/stela-gpu/kext/install.cpp");
        assert!(install.contains(&format!("kPlugin[] = \"{PLUGIN_PATH}\";")));
        assert!(install.contains(&format!("char path[{KERNEL_PATH_BYTES}];")));
    }
}
