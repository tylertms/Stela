//! NVK, NIR and NAK for `stela.efi`.
//!
//! Meson configures Mesa for `sysroot`'s machine, the same from every build
//! host, which leaves its generated sources and its compile plan. This task
//! then compiles that plan for the boot image's C target against `compat/libc`,
//! and archives it where `crates/stela-mesa`
//! links it. Mesa's Rust builds as ordinary Cargo dependencies of the image,
//! from sources generated here. Mesa itself stays unpatched: what differs is
//! the C library, the configuration below and the units in `compat/mesa`.
use super::{
    support::{NATIVE_TARGET, build_directory, capture, gpu, slashed, third_party, tool},
    sysroot::Sysroot,
    toolchain::{GENERATORS, Toolchain},
};
use crate::tools::{self, exec};
use anyhow::{Context, Result, bail, ensure};
use serde_json::Value;
use std::{
    collections::BTreeSet,
    fs,
    path::{Path, PathBuf},
    process::Command,
    sync::{
        Mutex,
        atomic::{AtomicUsize, Ordering},
    },
    time::SystemTime,
};

/// What NVK needs and nothing else: no window system, no cache on
/// disk and no host compiler. NVK's OpenCL kernels are `compat/mesa/nvkcl.c`, so
/// the tools that would build them are never run. A release build: NDEBUG drops
/// NIR validation after every pass and NAK's debug assertions, which cost the
/// image compile time and heap; the host compiler lane keeps them.
pub(super) const OPTIONS: [&str; 21] = [
    "-Dcpp_rtti=false",
    "-Dvulkan-drivers=nouveau",
    "-Dgallium-drivers=",
    "-Dplatforms=",
    "-Dopengl=false",
    "-Dgles1=disabled",
    "-Dgles2=disabled",
    "-Dglx=disabled",
    "-Degl=disabled",
    "-Dgbm=disabled",
    "-Dllvm=disabled",
    "-Dzlib=disabled",
    "-Dzstd=disabled",
    "-Dshader-cache=disabled",
    "-Dmesa-clc=system",
    "-Dprecomp-compiler=system",
    "-Dvulkan-layers=",
    "-Dtools=",
    "-Dvideo-codecs=",
    "-Dexpat=disabled",
    "-Dbuildtype=release",
];
/// The unit that keeps NDEBUG off: only then does ralloc record each block's
/// size, which `ralloc_total_size` sums for the Metal compiler's NIR budget.
/// The block header is private to the unit and its size is the same either way.
const SIZED_UNIT: &str = "src/util/ralloc.c";
/// The Vulkan drivers the image links, as Meson's targets.
const DRIVERS: [&str; 1] = ["src/nouveau/vulkan/libvulkan_nouveau.so"];
/// Tools the configuration asks for that nothing runs.
const UNUSED_TOOLS: [&str; 2] = ["mesa_clc", "vtn_bindgen2"];
/// What Mesa's configuration asks of `glslangValidator`, only for the compute
/// shaders that build acceleration structures and decode ASTC in Vulkan's
/// runtime. The device enables neither, as no Metal path uses them, so they
/// compile to an empty SPIR-V module.
const GLSLANG: &str = r#"#!/usr/bin/env python3
import sys
arguments = sys.argv[1:]
if "--version" in arguments:
    print("Glslang Version: 11:15.1.0")
    raise SystemExit
output = arguments[arguments.index("-o") + 1]
with open(output, "w") as file:
    file.write("0x07230203, 0x00010500, 0, 1, 0,\n")
if "--depfile" in arguments:
    with open(arguments[arguments.index("--depfile") + 1], "w") as file:
        file.write(output + ":\n")
"#;
/// `.pc` files for what Mesa's configuration requires: libdrm's headers are
/// vendored and its calls are `compat/mesa/drm.c`; the one user of libelf is
/// `compat/mesa/nv_cubin.c`.
const LIBDRM_VERSION: &str = "2.4.133";
const LIBELF_VERSION: &str = "0.190";

/// Units not built, by a part of their source path: each either has a stand-in
/// in `compat/mesa`, or needs what the image lacks and nothing here calls it.
/// The libraries Meson links whole, by their objects' names.
const WHOLE_LIBRARIES: [&str; 4] = [
    "libvulkan_lite_runtime_a",
    "libvulkan_runtime_a",
    "libvulkan_instance_a",
    "libnvk_a",
];

const SKIPPED: [&str; 11] = [
    "nvkcl",
    "build_id",
    "nv_cubin.c",
    "texcompress_astc",
    "u_qsort",
    "os_file_notify",
    "os_socket",
    "disk_cache",
    "mesa_cache_db",
    "u_debug_stack",
    "wsi_common_display",
];
/// Stela's stand-ins, under `compat/mesa`, built with an NVK unit's flags.
const STAND_INS: [&str; 7] = [
    "nvkcl",
    "build_id",
    "nv_cubin",
    "drm",
    "soft_float",
    "nir_options",
    "submit_thread",
];
/// The Metal compiler service, under `compiler`, built with an NVK unit's flags.
const AIR: [&str; 16] = [
    "air_call",
    "air_calls",
    "air_commands",
    "air_constants",
    "air_log",
    "air_math",
    "air_precise",
    "air_matrix",
    "air_metadata",
    "air_simd",
    "air_stitch",
    "air_texture",
    "air_to_nir",
    "metal_abi",
    "metal_compiler",
    "threadgroup",
];
const MODEL_UNIT: &str = "nvk_query_pool.c";
/// The C library, under `compat/libc`.
pub(super) const LIBC: [&str; 6] = ["libc", "stdio", "pthread", "math", "system", "wchar"];
/// Original LLVM units behind `compat/libc/regex.h`, also present in the
/// freestanding LLVM archive.
pub(super) const REGEX: [&str; 5] = ["regcomp", "regexec", "regfree", "regerror", "regstrlcpy"];

/// What Meson's probes find that names a path the image does not take. SSE4.1
/// and BLAKE3's assembly are host features; the image runs on any x86-64.
pub(super) const HOST_DEFINES: [&str; 10] = [
    "ALLOW_KCMP",
    "HAS_SCHED_GETAFFINITY",
    "HAVE_CET_H",
    "HAVE_DIRENT_D_TYPE",
    "HAVE_DLADDR",
    "HAVE_DRI2",
    "HAVE_FLOCK",
    "HAVE_FUNC_ATTRIBUTE_VISIBILITY",
    "HAVE_MKOSTEMP",
    "HAVE_RENDERDOC_INTEGRATION=1",
];
pub(super) const LATE_DEFINES: [&str; 3] = [
    "USE_SSE41",
    "VK_USE_PLATFORM_DISPLAY_KHR",
    "VK_USE_PLATFORM_HEADLESS_EXT",
];
/// What the image has without the probes finding it: `HAVE_DL_ITERATE_PHDR`
/// declares the build ID that `compat/mesa/build_id.c` provides.
pub(super) const ADDED_DEFINES: [&str; 6] = [
    "STELA_LIBC",
    "HAVE_DL_ITERATE_PHDR",
    "BLAKE3_NO_SSE2",
    "BLAKE3_NO_SSE41",
    "BLAKE3_NO_AVX2",
    "BLAKE3_NO_AVX512",
];
/// Flags of Meson's that do not apply to a freestanding COFF object.
pub(super) const HOST_FLAGS: [&str; 9] = [
    "-g",
    "-MD",
    "-pthread",
    "-fPIC",
    "-mtls-dialect=gnu2",
    "-fvisibility=hidden",
    "-pipe",
    "-fno-trapping-math",
    "-D_FORTIFY_SOURCE=2",
];
/// The C target's flags: `stela-driver-gpu/build/native.rs`'s, with SSE, which
/// Mesa uses, in place of its general registers only. A kernel's stacks follow.
/// Hardening a distribution's Clang turns on by default is turned off by
/// name, as `sysroot::FREESTANDING` explains.
pub(super) const TARGET_FLAGS: [&str; 17] = [
    "-mno-stack-arg-probe",
    "-mno-incremental-linker-compatible",
    "--no-default-config",
    "-fno-stack-protector",
    "-fcf-protection=none",
    "-mno-red-zone",
    "-msse2",
    "-nostdinc",
    "-U_FORTIFY_SOURCE",
    "-U__CYGWIN__",
    "-U__unix__",
    "-D__linux__",
    "-fno-exceptions",
    "-ffunction-sections",
    "-fdata-sections",
    "-fno-asynchronous-unwind-tables",
    "-femulated-tls",
];
/// The C library's own test, on the build host.
pub(super) const LIBC_TEST_FLAGS: [&str; 16] = [
    "-Qunused-arguments",
    "--target=x86_64-unknown-linux-gnu",
    "-O1",
    "-msse2",
    "-nostdinc",
    "-ffreestanding",
    "-fno-builtin",
    "--no-default-config",
    "-fno-stack-protector",
    "-fno-stack-clash-protection",
    "-fcf-protection=none",
    "-U_FORTIFY_SOURCE",
    "-femulated-tls",
    "-fno-pic",
    "-Wall",
    "-Werror",
];
/// Bindgen's C wrappers of the conversions that pass floats to Rust, which the
/// soft-float versions in `compat/mesa/soft_float.c` replace.
const FLOAT_WRAPPERS: [&str; 3] = [
    "_mesa_half_to_float",
    "_mesa_float_to_float16_rtne",
    "_mesa_double_to_float16_rtne",
];
/// Mesa's Rust, for the check of what crosses its C boundary.
const RUST_SOURCES: [&str; 5] = [
    "src/nouveau/compiler/nak",
    "src/nouveau/nil",
    "src/compiler/rust",
    "src/nouveau/rust",
    "src/nouveau/headers/nv_push_rs",
];

/// One unit to compile, with what Clang needs.
pub(super) struct Unit {
    pub(super) output: PathBuf,
    pub(super) arguments: Vec<String>,
}

fn paths(directory: &Path) -> Result<(PathBuf, PathBuf)> {
    let stubs = directory.join("tools");
    let local = tools::root().join(".tools/gpu");
    let path = std::env::join_paths(
        [
            stubs.clone(),
            local.join(GENERATORS),
            local.join("rust/bin"),
        ]
        .into_iter()
        .chain(std::env::split_paths(
            &std::env::var_os("PATH").unwrap_or_default(),
        )),
    )?;
    Ok((stubs, path.into()))
}

fn setup(
    toolchain: &Toolchain,
    sysroot: &Sysroot,
    directory: &Path,
    build: &Path,
    path: &Path,
) -> Result<()> {
    let packages = directory.join("pkgconfig");
    let machine = directory.join("machine.ini");
    let description = sysroot.machine_file(toolchain, &packages)?;
    let plan = build.join("build.ninja");
    let modified = |path: &Path| fs::metadata(path).and_then(|file| file.modified()).ok();
    if modified(&plan) > modified(&sysroot.libc())
        && fs::read_to_string(&machine).is_ok_and(|current| current == description)
    {
        return Ok(());
    }
    if build.is_dir() {
        fs::remove_dir_all(build)?;
    }
    fs::write(&machine, description)?;
    let (stubs, _) = paths(directory)?;
    fs::create_dir_all(&stubs)?;
    let unused = UNUSED_TOOLS.map(|name| (name, "#!/usr/bin/env python3\nraise SystemExit(1)\n"));
    for (name, script) in unused.into_iter().chain([("glslangValidator", GLSLANG)]) {
        let stub = stubs.join(name);
        fs::write(&stub, script)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&stub, fs::Permissions::from_mode(0o755))?;
        }
    }
    fs::create_dir_all(&packages)?;
    let libdrm = third_party("libdrm");
    fs::write(
        packages.join("libdrm.pc"),
        format!(
            "Name: libdrm\nDescription: vendored headers\nVersion: {LIBDRM_VERSION}\n\
             Cflags: -I{0} -I{0}/include/drm\n",
            slashed(&libdrm)
        ),
    )?;
    fs::write(
        packages.join("libelf.pc"),
        format!("Name: libelf\nDescription: unused\nVersion: {LIBELF_VERSION}\nCflags:\n"),
    )?;
    let mut setup = Command::new(tool("meson", GENERATORS)?);
    setup
        .arg("setup")
        .arg(build)
        .arg(third_party("mesa"))
        .arg("--cross-file")
        .arg(machine)
        .args(OPTIONS)
        .env("PATH", path);
    // Rust on Windows does not link through the C compiler Meson finds.
    if cfg!(windows) {
        setup.env("RUSTC_LD_FOR_BUILD", "lld-link");
    }
    exec(&mut setup)?;
    let commands = fs::read_to_string(build.join("compile_commands.json"))?;
    let missing = missing_probes(&commands)?;
    if !missing.is_empty() {
        // The next run configures again rather than keep these results.
        let log = build.join("meson-logs/meson-log.txt");
        let kept = directory.join("meson-log-failed-probes.txt");
        let _ = fs::copy(&log, &kept);
        fs::remove_dir_all(build)?;
        bail!(
            "Mesa's configuration found no {} in Stela's C library, which has them: its link \
             probes failed, usually from a default of this computer's Clang that the build \
             does not turn off. Meson's log is {}",
            missing.join(", "),
            kept.display()
        );
    }
    Ok(())
}

/// What Meson's probes must find in Stela's C library, by the define each
/// adds to every compile: a link probe that fails turns a feature off without
/// an error, and the units built without it then miss symbols at link time.
const PROBED: [&str; 11] = [
    "HAVE_GNU_QSORT_R",
    "HAVE_STRTOF",
    "HAVE_STRTOK_R",
    "HAVE_SECURE_GETENV",
    "HAVE_SYSCONF",
    "HAVE_MEMFD_CREATE",
    "HAVE_POSIX_MEMALIGN",
    "HAVE_STRUCT_TIMESPEC",
    "HAVE_PTHREAD",
    "HAVE_ENDIAN_H",
    "HAVE_DLFCN_H",
];

/// The [`PROBED`] defines that Meson's compile of [`SIZED_UNIT`] lacks, in
/// `compile_commands.json`'s text.
fn missing_probes(commands: &str) -> Result<Vec<&'static str>> {
    let commands: Value = serde_json::from_str(commands)?;
    let command = commands
        .as_array()
        .into_iter()
        .flatten()
        .find(|entry| {
            entry["file"]
                .as_str()
                .is_some_and(|file| file.ends_with(SIZED_UNIT))
        })
        .and_then(|entry| entry["command"].as_str())
        .with_context(|| format!("Meson's plan has no compile of {SIZED_UNIT}"))?;
    let words = shlex::split(command).context("Meson's compile command")?;
    Ok(PROBED
        .into_iter()
        .filter(|define| !words.iter().any(|word| word == &format!("-D{define}")))
        .collect())
}

/// Every source Meson generates, and the structured copy of the compiler crate.
fn generate(sysroot: &Sysroot, build: &Path, path: &Path, libclang: &Path) -> Result<()> {
    let ninja = tool("ninja", GENERATORS)?;
    let mut targets = BTreeSet::new();
    for rule in ["CUSTOM_COMMAND", "CUSTOM_COMMAND_DEP"] {
        let listed = capture(
            Command::new(&ninja)
                .args(["-t", "targets", "rule", rule])
                .current_dir(build)
                .env("PATH", path),
        )?;
        targets.extend(listed.lines().map(str::to_owned));
    }
    let plan = fs::read_to_string(build.join("build.ninja"))?;
    targets.extend(plan.lines().filter_map(|line| {
        let target = line.strip_prefix("build ")?.split(':').next()?;
        target
            .contains(".rlib.p/structured/")
            .then(|| target.to_owned())
    }));
    targets.retain(|target| {
        !target.contains("nvkcl")
            && ![
                "all",
                "test",
                "benchmark",
                "clean",
                "dist",
                "reconfigure",
                "PHONY",
            ]
            .contains(&target.as_str())
            && !target.starts_with("meson-")
            && !target.ends_with("install")
    });
    // bindgen splits these as a shell does, and a path may hold a space.
    let quoted: Vec<_> = sysroot
        .compile
        .iter()
        .map(|argument| format!("\"{}\"", argument.replace('\\', "/")))
        .collect();
    exec(
        Command::new(&ninja)
            .args(&targets)
            .current_dir(build)
            .env("BINDGEN_EXTRA_CLANG_ARGS", quoted.join(" "))
            .env("LIBCLANG_PATH", libclang)
            .env("PATH", path),
    )
}

pub(super) fn resource_directory(clang: &Path) -> Result<String> {
    Ok(capture(Command::new(clang).arg("-print-resource-dir"))?
        .trim()
        .to_owned())
}

fn include_flags(clang: &Path, libc: &Path) -> Result<Vec<String>> {
    Ok(vec![
        format!("--target={NATIVE_TARGET}"),
        "-isystem".to_owned(),
        libc.display().to_string(),
        "-isystem".to_owned(),
        format!("{}/include", resource_directory(clang)?),
    ])
}

/// Bindgen's output against the C library the units compile with: Meson's run
/// saw the build host's, which lays out its types differently.
fn bindings(build: &Path, path: &Path, flags: &[String], libclang: &Path) -> Result<()> {
    let plan = fs::read_to_string(build.join("build.ninja"))?;
    for command in plan
        .lines()
        .filter_map(|line| line.strip_prefix(" COMMAND = "))
    {
        if !command.contains("/bindgen ") || command.contains("nvidia/hwref") {
            continue;
        }
        let words = shlex::split(&command.replace("$ ", " ")).context("bindgen command")?;
        let (program, arguments) = words.split_first().context("bindgen command")?;
        let (ours, theirs) = arguments.split_at(
            arguments
                .iter()
                .position(|word| word == "--")
                .context("bindgen flags")?
                + 1,
        );
        exec(
            Command::new(program)
                .args(ours)
                .args(flags)
                .args(["-nostdinc", "-U__CYGWIN__", "-D__linux__", "-U__unix__"])
                .arg("-DSTELA_LIBC")
                .args(
                    theirs
                        .iter()
                        .filter(|word| *word != "-pthread" && !word.starts_with("--target="))
                        .filter(|word| !host_define(word)),
                )
                .current_dir(build)
                .env("LIBCLANG_PATH", libclang)
                .env("PATH", path),
        )?;
    }
    fs::copy(
        build.join("src/compiler/rust/bindings.rs"),
        build.join("src/compiler/rust/libcompiler.rlib.p/structured/bindings.rs"),
    )?;
    Ok(())
}

/// Rust's UEFI target is soft-float, so a C function with a float in its
/// signature that Mesa's Rust calls needs a stand-in in `soft_float.c`.
fn check_float_crossings(build: &Path) -> Result<()> {
    let mut crossing = BTreeSet::new();
    for bindings in [
        "src/compiler/rust/bindings.rs",
        "src/nouveau/compiler/nak_bindings.rs",
        "src/nouveau/nil/nil_bindings.rs",
    ] {
        let text = fs::read_to_string(build.join(bindings))?;
        for signature in text.split("pub fn ").skip(1) {
            let signature = signature.split(';').next().unwrap_or_default();
            if signature.contains("f32") || signature.contains("f64") {
                let name = signature
                    .split(|c: char| !c.is_alphanumeric() && c != '_')
                    .next();
                crossing.extend(name.map(str::to_owned));
            }
        }
    }
    let mut sources = Vec::new();
    for root in RUST_SOURCES {
        collect_rust(&third_party("mesa").join(root), &mut sources)?;
    }
    for source in sources {
        let text = fs::read_to_string(&source)?;
        for name in &crossing {
            ensure!(
                FLOAT_WRAPPERS.contains(&name.as_str()) || !mentions(&text, name),
                "{} passes a float to C through {name}: add it to soft_float.c",
                source.display()
            );
        }
    }
    Ok(())
}

fn mentions(text: &str, name: &str) -> bool {
    text.match_indices(name).any(|(at, _)| {
        let identifier = |c: char| c.is_alphanumeric() || c == '_';
        !text[..at].ends_with(identifier) && !text[at + name.len()..].starts_with(identifier)
    })
}

fn collect_rust(directory: &Path, sources: &mut Vec<PathBuf>) -> Result<()> {
    for entry in fs::read_dir(directory)? {
        let path = entry?.path();
        if path.is_dir() {
            collect_rust(&path, sources)?;
        } else if path.extension().is_some_and(|extension| extension == "rs") {
            sources.push(path);
        }
    }
    Ok(())
}

fn host_define(flag: &str) -> bool {
    flag.strip_prefix("-D")
        .is_some_and(|define| HOST_DEFINES.contains(&define) || LATE_DEFINES.contains(&define))
}

fn host_only(flag: &str) -> bool {
    HOST_FLAGS.contains(&flag)
        || flag.starts_with("-W")
        || flag.starts_with("--target=")
        || host_define(flag)
}

/// Meson's flags for a unit, for the image's target.
fn retarget(arguments: &[String], common: &[String]) -> Vec<String> {
    let mut result = common.to_vec();
    let mut words = arguments.iter().skip(1);
    while let Some(word) = words.next() {
        if matches!(word.as_str(), "-MQ" | "-MF" | "-o") {
            words.next();
        } else if !host_only(word) {
            result.push(word.clone());
        }
    }
    result
}

fn object_name(output: &str) -> String {
    output.replace(['/', '.'], "_") + ".o"
}

/// The units the drivers link, with Stela's stand-ins and C library.
fn units(
    build: &Path,
    objects: &Path,
    ninja: &Path,
    path: &Path,
    common: &[String],
    cxx: &[String],
) -> Result<Vec<Unit>> {
    let linked = capture(
        Command::new(ninja)
            .args(["-t", "inputs"])
            .args(DRIVERS)
            .current_dir(build)
            .env("PATH", path),
    )?;
    let linked: BTreeSet<&str> = linked
        .lines()
        .filter(|input| input.ends_with(".o"))
        .collect();
    let database: Value =
        serde_json::from_str(&fs::read_to_string(build.join("compile_commands.json"))?)?;
    let mut units = Vec::new();
    let mut model = None;
    let entries = database.as_array().context("compile database")?;
    // A source two libraries compile is the one's that is linked whole.
    let whole: BTreeSet<&str> = entries
        .iter()
        .filter(|entry| entry["output"].as_str().is_some_and(linked_whole))
        .filter_map(|entry| entry["file"].as_str())
        .collect();
    for entry in entries {
        let file = entry["file"].as_str().context("unit file")?;
        let words = shlex::split(entry["command"].as_str().context("unit command")?)
            .context("unit command")?;
        let Some(at) = words.iter().position(|word| word == "-o") else {
            continue;
        };
        let output = &words[at + 1];
        if !linked.contains(output.as_str()) || (whole.contains(file) && !linked_whole(output)) {
            continue;
        }
        if file.ends_with(MODEL_UNIT) {
            model = Some((file.to_owned(), words.clone()));
        }
        if file.ends_with(".S")
            || SKIPPED
                .iter()
                .any(|skip| output.contains(skip) || file.contains(skip))
        {
            continue;
        }
        let is_cxx = file.ends_with(".cpp");
        ensure!(
            is_cxx || file.ends_with(".c"),
            "{file} is neither C nor C++"
        );
        let mut arguments = retarget(&words, if is_cxx { cxx } else { common });
        if file.ends_with("rust/bindings.c") {
            arguments.extend(
                FLOAT_WRAPPERS
                    .iter()
                    .map(|name| format!("-D{name}__extern={name}__hard_float")),
            );
        }
        let mut name = object_name(output);
        if file.replace('\\', "/").ends_with(SIZED_UNIT) {
            arguments.push("-UNDEBUG".to_owned());
            // Its own object, so one compiled with NDEBUG is never reused.
            name.insert_str(0, "sized_");
        }
        units.push(Unit {
            output: objects.join(name),
            arguments,
        });
    }
    let (file, words) = model.context("no model unit")?;
    let compat = gpu().join("compat");
    let like = |file: &str, words: &[String], source: &Path, name: &str, added: &[String]| {
        let words: Vec<_> = words
            .iter()
            .map(|word| {
                if *word == file {
                    source.display().to_string()
                } else {
                    word.clone()
                }
            })
            .collect();
        let mut arguments = retarget(&words, common);
        arguments.extend_from_slice(added);
        Unit {
            output: objects.join(format!("stela_{name}.o")),
            arguments,
        }
    };
    let model_unit =
        |source: &Path, name: &str, added: &[String]| like(&file, &words, source, name, added);
    for name in STAND_INS {
        let source = compat.join("mesa").join(format!("{name}.c"));
        units.push(model_unit(&source, name, &[]));
    }
    let compiler = gpu().join("compiler");
    let added = [
        compiler.clone(),
        compiler.join("llvm/include"),
        third_party("llvm").join("llvm/include"),
        gpu(),
    ]
    .map(|include| format!("-I{}", include.display()));
    for name in AIR {
        let source = compiler.join(format!("{name}.c"));
        units.push(model_unit(&source, name, &added));
    }
    for name in LIBC {
        let mut arguments = common.to_vec();
        arguments.extend(["-O2", "-ffreestanding", "-c"].map(str::to_owned));
        arguments.push(
            compat
                .join("libc")
                .join(format!("{name}.c"))
                .display()
                .to_string(),
        );
        units.push(Unit {
            output: objects.join(format!("libc_{name}.o")),
            arguments,
        });
    }
    Ok(units)
}

/// Whether anything the last compile read has changed, by its dependency file.
fn stale(output: &Path, build: &Path) -> bool {
    let (Ok(built), Ok(depends)) = (
        fs::metadata(output).and_then(|file| file.modified()),
        fs::read_to_string(output.with_extension("d")),
    ) else {
        return true;
    };
    depends
        .split_whitespace()
        .skip(1)
        .filter(|word| *word != "\\")
        .any(|input| {
            fs::metadata(build.join(input))
                .and_then(|file| file.modified())
                .map_or(true, |modified: SystemTime| modified > built)
        })
}

pub(super) fn compile(clang: &Path, build: &Path, units: &[Unit], jobs: u16) -> Result<()> {
    let next = AtomicUsize::new(0);
    let failures = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..jobs {
            scope.spawn(|| {
                while let Some(unit) = units.get(next.fetch_add(1, Ordering::Relaxed)) {
                    if !stale(&unit.output, build) {
                        continue;
                    }
                    let output = Command::new(clang)
                        .args(&unit.arguments)
                        .arg("-w")
                        .arg("-MD")
                        .arg("-MF")
                        .arg(unit.output.with_extension("d"))
                        .arg("-o")
                        .arg(&unit.output)
                        .current_dir(build)
                        .output();
                    let report = match output {
                        Ok(output) if output.status.success() => continue,
                        Ok(output) => String::from_utf8_lossy(&output.stderr).into_owned(),
                        Err(error) => error.to_string(),
                    };
                    let error = report.lines().find(|line| line.contains("error:"));
                    failures.lock().unwrap().push(format!(
                        "{}: {}",
                        unit.output.display(),
                        error.unwrap_or(&report)
                    ));
                }
            });
        }
    });
    let failures = failures.into_inner().unwrap();
    ensure!(
        failures.is_empty(),
        "{} units failed:\n{}",
        failures.len(),
        failures.join("\n"),
    );
    Ok(())
}

/// Moves the static constructors of a COFF `object` between the bounds
/// `compat/llvm/runtime.cpp` walks: the linker sorts a section's parts by what
/// follows its `$`, and a plain `.ctors` would come before the first bound.
/// False if there are none.
fn rename_constructors(object: &mut [u8]) -> Option<bool> {
    let field = |at: usize| Some(u16::from_le_bytes(object.get(at..at + 2)?.try_into().ok()?));
    let sections = usize::from(field(2)?);
    let table = 20 + usize::from(field(16)?);
    let headers = object.get_mut(table..table + sections * 40)?;
    let mut renamed = false;
    for header in headers.as_chunks_mut::<40>().0 {
        if header.starts_with(b".ctors\0\0") {
            header[..8].copy_from_slice(b".ctors$m");
            renamed = true;
        }
    }
    Some(renamed)
}

/// [`rename_constructors`] for every unit.
pub(super) fn gather_constructors(units: &[Unit]) -> Result<()> {
    for unit in units {
        let mut object = fs::read(&unit.output)?;
        let renamed = rename_constructors(&mut object)
            .with_context(|| format!("{} is no COFF object", unit.output.display()))?;
        if renamed {
            fs::write(&unit.output, object)?;
        }
    }
    Ok(())
}

/// Whether Meson links `unit`'s library whole: its entry points are reached
/// only by weak reference, which pulls no member from an archive.
fn linked_whole(object: &str) -> bool {
    let name = object_name(object);
    WHOLE_LIBRARIES.iter().any(|library| name.contains(library))
}

pub fn build(jobs: u16) -> Result<()> {
    let toolchain = super::toolchain::discover()?;
    let directory = build_directory("nvk");
    let build = directory.join("build");
    let (_, path) = paths(&directory)?;
    fs::create_dir_all(&directory)?;
    super::llvm::build(&toolchain, jobs, false)?;
    let sysroot = Sysroot::build(&toolchain, &directory, jobs)?;
    setup(&toolchain, &sysroot, &directory, &build, &path)?;
    generate(&sysroot, &build, &path, &toolchain.libclang)?;
    let libc = gpu().join("compat/libc");
    let includes = include_flags(&toolchain.clang, &libc)?;
    bindings(&build, &path, &includes, &toolchain.libclang)?;
    check_float_crossings(&build)?;

    let objects = directory.join("objects");
    fs::create_dir_all(&objects)?;
    let mut common = includes;
    common.extend(TARGET_FLAGS.map(str::to_owned));
    common.extend(ADDED_DEFINES.map(|define| format!("-D{define}")));
    common.push(format!("-I{}", gpu().join("compat/mesa").display()));
    let ninja = tool("ninja", GENERATORS)?;
    let cxx = super::llvm::cxx_flags(&common);
    let units = units(&build, &objects, &ninja, &path, &common, &cxx)?;
    compile(&toolchain.clang, &build, &units, jobs)?;

    gather_constructors(&units)?;
    let (whole, lazy): (Vec<_>, Vec<_>) = units.iter().partition(|unit| {
        linked_whole(
            &unit
                .output
                .file_name()
                .unwrap_or_default()
                .to_string_lossy(),
        )
    });
    let archive = directory.join("libstela-nvk.a");
    for (name, members) in [("libstela-vulkan.a", &whole), ("libstela-nvk.a", &lazy)] {
        let archive = directory.join(name);
        if archive.is_file() {
            fs::remove_file(&archive)?;
        }
        super::support::archive(
            &toolchain.archiver,
            &archive,
            members.iter().map(|unit| &unit.output),
        )?;
    }
    println!("{} objects in {}", units.len(), archive.display());
    // The test is a static Linux program.
    if cfg!(all(target_os = "linux", target_arch = "x86_64")) {
        test_libc(&toolchain, &sysroot, &directory)?;
    }
    Ok(())
}

/// Builds `compat/libc/tests/libc_test.c` with the C library as a static Linux
/// program, and runs it.
fn test_libc(toolchain: &Toolchain, sysroot: &Sysroot, directory: &Path) -> Result<()> {
    let libc = gpu().join("compat/libc");
    let output = directory.join("libc-test");
    fs::create_dir_all(&output)?;
    let resource = format!("{}/include", resource_directory(&toolchain.clang)?);
    let test = output.join("libc_test.o");
    exec(
        Command::new(&toolchain.clang)
            .args(LIBC_TEST_FLAGS)
            .arg("-isystem")
            .arg(&libc)
            .arg("-isystem")
            .arg(&resource)
            .arg("-c")
            .arg(libc.join("tests/libc_test.c"))
            .arg("-o")
            .arg(&test),
    )?;
    let program = output.join("libc_test");
    exec(
        Command::new(&toolchain.linker)
            .args(["-static", "-o"])
            .arg(&program)
            .arg(&test)
            .arg(sysroot.libc()),
    )?;
    exec(&mut Command::new(&program)).context("the C library's test failed")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructors_move_between_the_bounds() {
        let mut object = vec![0; 20 + 2 + 2 * 40];
        object[2] = 2;
        object[16] = 2;
        object[22..28].copy_from_slice(b".text\0");
        object[62..68].copy_from_slice(b".ctors");
        assert_eq!(rename_constructors(&mut object), Some(true));
        assert_eq!(&object[62..70], b".ctors$m");
        assert_eq!(rename_constructors(&mut object), Some(false));
        assert_eq!(&object[22..28], b".text\0");
        assert_eq!(rename_constructors(&mut object[..30]), None);
    }

    #[test]
    fn retargeting_keeps_a_units_own_flags_and_drops_the_hosts() {
        let words = [
            "cc",
            "-Iinclude",
            "--target=probe",
            "-DMESA_DEBUG=0",
            "-DHAVE_FLOCK",
            "-Wall",
            "-MD",
            "-MQ",
            "out",
            "-MF",
            "out.d",
            "-fPIC",
            "-c",
            "a.c",
            "-o",
            "a.o",
        ]
        .map(str::to_owned);
        let common = ["--target=x".to_owned()];
        assert_eq!(
            retarget(&words, &common),
            ["--target=x", "-Iinclude", "-DMESA_DEBUG=0", "-c", "a.c"]
        );
    }

    #[test]
    fn a_probe_that_failed_is_named() {
        let all = PROBED.map(|define| format!("-D{define}")).join(" ");
        let plan = |defines: &str| {
            serde_json::json!([
                {"file": "../mesa/src/util/u_math.c", "command": "clang -c u_math.c"},
                {"file": "../mesa/src/util/ralloc.c",
                 "command": format!("clang -DHAVE_OPENGL=0 {defines} -c ralloc.c")},
            ])
            .to_string()
        };
        assert!(missing_probes(&plan(&all)).unwrap().is_empty());
        // As when every link probe fails: no qsort_r and no library calls.
        let compile_only = "-DHAVE_STRUCT_TIMESPEC -DHAVE_ENDIAN_H -DHAVE_DLFCN_H";
        let missing = missing_probes(&plan(compile_only)).unwrap();
        assert_eq!(missing[0], "HAVE_GNU_QSORT_R");
        assert_eq!(missing.len(), PROBED.len() - 3);
        assert!(missing_probes("[]").is_err());
    }

    #[test]
    fn a_mention_is_a_whole_identifier() {
        assert!(mentions(
            "x = _mesa_half_to_float(y)",
            "_mesa_half_to_float"
        ));
        assert!(!mentions(
            "_mesa_half_to_float_slow(y)",
            "_mesa_half_to_float"
        ));
    }
}
