//! Mesa's Vulkan drivers for `stela.efi`: NVK with NAK, and ANV with BRW and
//! RADV with ACO when the build host has the tools their builds run
//! (`clc.rs`). A release build (`STELA_RELEASE`) fails instead of leaving out
//! a driver the image gives its GPUs.
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

/// What the drivers need and nothing else: no window system, no cache on
/// disk and no compiler of its own for the host tools, which `clc.rs` builds
/// when a driver needs them. `vulkan-drivers` is the drivers chosen. A
/// release build: NDEBUG drops NIR validation after every pass and NAK's
/// debug assertions, which cost the image compile time and heap; the host
/// compiler lane keeps them.
pub(super) const OPTIONS: [&str; 20] = [
    "-Dcpp_rtti=false",
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
/// A Vulkan driver the image may link.
pub(super) struct Driver {
    /// Its name in Meson's `vulkan-drivers`.
    pub(super) name: &'static str,
    /// Its Meson target.
    pub(super) target: &'static str,
    /// Where Meson puts its own objects, under the build directory.
    objects: &'static str,
    /// The libraries Meson links whole into it, by their objects' names.
    whole: &'static [&'static str],
    /// The name its `vk_icdGetInstanceProcAddr` takes in the image. Every
    /// Mesa driver defines that loader entry; one image holds them all, so
    /// each but the first has its own compiled under this name.
    pub(super) entry: &'static str,
    /// Other symbols an earlier driver's units also define, with the names
    /// this driver's own units compile them under: Mesa names some of each
    /// driver's application layers alike.
    renamed: &'static [(&'static str, &'static str)],
    /// Meson options only this driver reads.
    pub(super) options: &'static [&'static str],
    /// Whether its build runs the host tools (`clc.rs`): its internal
    /// kernels are OpenCL C and its libraries GLSL.
    host_tools: bool,
    /// Whether a release image must hold it: true once its GPU stack gives
    /// it to its GPUs (`stela-driver-gpu/src/stack.rs`), as an image
    /// without it then draws nothing on them. Change both together.
    released: bool,
    /// One of its own units, whose flags its stand-ins compile with.
    pub(super) model: &'static str,
    /// Its stand-ins and driver-internal calls, under `compat/mesa`.
    stand_ins: &'static [&'static str],
    /// The libraries it calls that Mesa's configuration takes from the
    /// system, built from their unchanged sources.
    pub(super) libraries: &'static [&'static Library],
}

/// A library Mesa's configuration takes from the system, which the image
/// builds from its unchanged sources under `third_party` with the calling
/// driver's model unit's flags.
pub(super) struct Library {
    /// Its name, which its objects' names start with.
    pub(super) name: &'static str,
    /// Its sources, under `third_party`.
    pub(super) sources: &'static [&'static str],
    /// What its own build defines.
    pub(super) defines: &'static [&'static str],
    /// Its functions that a kernel driver in the same image also defines,
    /// with the names every unit of the build gives them instead.
    renamed: &'static [(&'static str, &'static str)],
}

/// libdrm's amdgpu library, which RADV and AMD's common code call for the
/// device, its buffers, its GPU address space and its queries, over DRM's
/// ioctls (`compat/mesa/drm.c`). The command-submission source is built for
/// the timeout conversion the buffer calls share; RADV submits through its
/// own ioctls (`ac_linux_drm.c`). The VM and user-queue sources are not
/// called.
const LIBDRM_AMDGPU: Library = Library {
    name: "libdrm_amdgpu",
    sources: &[
        "libdrm/amdgpu/amdgpu_asic_id.c",
        "libdrm/amdgpu/amdgpu_bo.c",
        "libdrm/amdgpu/amdgpu_cs.c",
        "libdrm/amdgpu/amdgpu_device.c",
        "libdrm/amdgpu/amdgpu_gpu_info.c",
        "libdrm/amdgpu/amdgpu_vamgr.c",
        "libdrm/amdgpu/handle_table.c",
    ],
    // What libdrm's configuration (its `config.h`) and its amdgpu build set
    // for a Linux target with GCC's atomic builtins. The table of marketing
    // names lies under its data directory; the image holds no file system,
    // so a device has no marketing name and libdrm says once per device
    // which file it lacks.
    defines: &[
        "HAVE_LIBDRM_ATOMIC_PRIMITIVES=1",
        "HAVE_LIB_ATOMIC_OPS=0",
        "HAVE_ALLOCA_H=1",
        "HAVE_VISIBILITY=1",
        "HAVE___BUILTIN_CLZLL=1",
        "AMDGPU_ASIC_ID_TABLE=\"/usr/share/libdrm/amdgpu.ids\"",
    ],
    // amdgpu's own `amdgpu_bo_list.c` and `amdgpu_object.c` name these
    // too. Nothing in Mesa calls them; any later caller reaches libdrm's.
    renamed: &[
        ("amdgpu_bo_list_create", "libdrm_amdgpu_bo_list_create"),
        ("amdgpu_bo_set_metadata", "libdrm_amdgpu_bo_set_metadata"),
    ],
};

/// The loader entry every Mesa Vulkan driver defines.
const ENTRY: &str = "vk_icdGetInstanceProcAddr";
/// NVK. Its OpenCL kernels are `compat/mesa/nvkcl.c`, so it needs no host
/// tool.
const NVK: Driver = Driver {
    name: "nouveau",
    target: "src/nouveau/vulkan/libvulkan_nouveau.so",
    objects: "src/nouveau/vulkan/",
    whole: &["libnvk_a"],
    entry: ENTRY,
    renamed: &[],
    options: &[],
    host_tools: false,
    released: true,
    model: "nvk_query_pool.c",
    stand_ins: &[
        "nvkcl",
        "build_id",
        "nv_cubin",
        "drm",
        "soft_float",
        "nir_options",
        "submit_thread",
    ],
    libraries: &[],
};
/// The Meson options only ANV reads, which the host test (`anv.rs`) builds
/// with too.
pub(super) const ANV_OPTIONS: [&str; 2] = ["-Dintel-rt=disabled", "-Dintel-elk=false"];
/// ANV, with BRW, for Intel GPUs from Gen9. Ray tracing is left out: no
/// Metal path the image serves builds acceleration structures, and NVK has
/// none either. ELK compiles for Gen8 and older, which only hasvk drives.
const ANV: Driver = Driver {
    name: "intel",
    target: "src/intel/vulkan/libvulkan_intel.so",
    objects: "src/intel/vulkan/",
    whole: &["libanv_common_a", "libanv_per_hw_ver"],
    entry: "anv_icdGetInstanceProcAddr",
    renamed: &[],
    options: &ANV_OPTIONS,
    host_tools: true,
    // Intel's stack gives no Vulkan driver yet.
    released: false,
    model: "anv_util.c",
    stand_ins: &["anv_nir_options"],
    libraries: &[],
};
/// RADV, with ACO, for AMD GPUs from GFX6. Mesa's configuration has it
/// compile the shaders that build acceleration structures from GLSL, which
/// needs the host's glslang; without it RADV would report ray tracing it
/// cannot build. Its objects are its shared library's own, not a library's,
/// so they are linked whole for the entry points its tables reach by weak
/// reference. ACO and addrlib are C++, built with LLVM's runtime
/// (`llvm::cxx_flags`). No LLVM backend: the image's LLVM serves AIR only.
pub(super) const RADV: Driver = Driver {
    name: "amd",
    target: "src/amd/vulkan/libvulkan_radeon.so",
    objects: "src/amd/vulkan/",
    whole: &["libvulkan_radeon_so"],
    entry: "radv_icdGetInstanceProcAddr",
    renamed: &[(
        "metro_exodus_GetSemaphoreCounterValue",
        "radv_metro_exodus_GetSemaphoreCounterValue",
    )],
    // RADV keys its pipeline caches by its build ID, which it finds in its
    // ELF notes or a file's timestamp; the image has neither, so it is given
    // the note `compat/mesa/build_id.c` holds, as #163's build did.
    options: &[
        "-Damd-use-llvm=false",
        "-Dradv-build-id=5374656c6120737461746963616c6c79",
    ],
    host_tools: true,
    // AMD's stack gives no Vulkan driver yet.
    released: false,
    model: "radv_query.c",
    stand_ins: &["radv_nir_options", "radv_submit"],
    libraries: &[&LIBDRM_AMDGPU],
};
/// Every driver the image may link, the first keeping Mesa's entry name.
pub(super) const DRIVERS: [&Driver; 3] = [&NVK, &ANV, &RADV];
/// Where the build lists the drivers it linked, beside the archives, for the
/// crates that name their entries.
pub(super) const LINKED: &str = "vulkan-drivers";
/// Tools the configuration asks for that nothing runs when no driver needs
/// the host tools.
const UNUSED_TOOLS: [&str; 2] = ["mesa_clc", "vtn_bindgen2"];
/// What Mesa's configuration asks of `glslangValidator` on a host that has
/// none, only for the compute shaders that build acceleration structures and
/// decode ASTC in Vulkan's runtime. The device enables neither, as no Metal
/// path uses them, so they compile to an empty SPIR-V module. No driver that
/// compiles a GLSL library it uses (ANV's software doubles) is built then.
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
/// vendored and its calls are `compat/mesa/drm.c`, libdrm_amdgpu is
/// [`LIBDRM_AMDGPU`]; the one user of libelf is `compat/mesa/nv_cubin.c`,
/// and AMD's code takes its path without it ([`HOST_DEFINES`]).
pub(super) const LIBDRM_VERSION: &str = "2.4.133";
const LIBELF_VERSION: &str = "0.190";

/// The libraries of Vulkan's runtime Meson links whole, by their objects'
/// names; each driver names its own.
const WHOLE_RUNTIME: [&str; 3] = [
    "libvulkan_lite_runtime_a",
    "libvulkan_runtime_a",
    "libvulkan_instance_a",
];

/// Units not built, by a part of their source path: each either has a stand-in
/// in `compat/mesa`, or needs what the image lacks and nothing here calls it.
const SKIPPED: [&str; 12] = [
    "nvkcl",
    "build_id",
    "nv_cubin.c",
    "u_qsort",
    "os_file_notify",
    "os_socket",
    "disk_cache",
    "mesa_cache_db",
    "u_debug_stack",
    "wsi_common_display",
    "ac_rtld",
    "ac_rgp_elf_object_pack",
];
/// The Metal compiler service, under `compiler`, built with the first
/// driver's model unit's flags.
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
/// The C library, under `compat/libc`.
pub(super) const LIBC: [&str; 6] = ["libc", "stdio", "pthread", "math", "system", "wchar"];
/// Original LLVM units behind `compat/libc/regex.h`, also present in the
/// freestanding LLVM archive.
pub(super) const REGEX: [&str; 5] = ["regcomp", "regexec", "regfree", "regerror", "regstrlcpy"];

/// What Meson's probes find that names a path the image does not take. SSE4.1
/// and BLAKE3's assembly are host features; the image runs on any x86-64.
/// libelf's `.pc` is a stand-in, so AMD's code takes its path without it:
/// no LLVM-made code objects (`ac_rtld`) and no ELF chunk in an RGP capture.
pub(super) const HOST_DEFINES: [&str; 9] = [
    "ALLOW_KCMP",
    "HAVE_CET_H",
    "HAVE_DIRENT_D_TYPE",
    "HAVE_DLADDR",
    "HAVE_DRI2",
    "HAVE_FLOCK",
    "HAVE_FUNC_ATTRIBUTE_VISIBILITY",
    "HAVE_RENDERDOC_INTEGRATION=1",
    "USE_LIBELF",
];
pub(super) const LATE_DEFINES: [&str; 3] = [
    "USE_SSE41",
    "VK_USE_PLATFORM_DISPLAY_KHR",
    "VK_USE_PLATFORM_HEADLESS_EXT",
];
/// What the image has without the probes finding it. Meson's probe finds the
/// C library's `dl_iterate_phdr` and then declares the build ID that
/// `compat/mesa/build_id.c` provides.
pub(super) const ADDED_DEFINES: [&str; 5] = [
    "STELA_LIBC",
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
pub(super) const LIBC_TEST_FLAGS: [&str; 15] = [
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

/// The directory of Mesa's tools for the image's configuration, the PATH it
/// runs with, which finds those first, and the PATH without them, with which
/// the host tools are found and built.
fn paths(directory: &Path) -> Result<(PathBuf, PathBuf, PathBuf)> {
    let stubs = directory.join("tools");
    let local = tools::root().join(".tools/gpu");
    let host: Vec<PathBuf> = [local.join(GENERATORS), local.join("rust/bin")]
        .into_iter()
        .chain(std::env::split_paths(
            &std::env::var_os("PATH").unwrap_or_default(),
        ))
        .collect();
    let path = std::env::join_paths(std::iter::once(stubs.clone()).chain(host.iter().cloned()))?;
    Ok((stubs, path.into(), std::env::join_paths(host)?.into()))
}

/// The drivers this build links and the host tools they run: every driver
/// when the host has the tools, else those that need none.
pub(super) struct Choice {
    pub(super) drivers: Vec<&'static Driver>,
    tools: Option<super::clc::Tools>,
}

impl Choice {
    /// [`DRIVERS`], given what [`super::clc::tools`] found and whether this
    /// is a release build.
    fn new(tools: std::result::Result<super::clc::Tools, String>, release: bool) -> Result<Self> {
        Self::of(&DRIVERS, tools, release)
    }

    /// Every driver of `drivers`, or without the host tools those that need
    /// none: the reason each is left out is printed once. A release build
    /// fails instead when one left out is [`Driver::released`].
    fn of(
        drivers: &[&'static Driver],
        tools: std::result::Result<super::clc::Tools, String>,
        release: bool,
    ) -> Result<Self> {
        let tools = match tools {
            Ok(tools) => Some(tools),
            Err(reason) => {
                let left_out = || drivers.iter().filter(|driver| driver.host_tools);
                let required: Vec<&str> = left_out()
                    .filter(|driver| driver.released)
                    .map(|driver| driver.name)
                    .collect();
                ensure!(
                    !release || required.is_empty(),
                    "a release image must hold Vulkan driver {}, which this host cannot build: \
                     {reason}",
                    required.join(" and ")
                );
                for driver in left_out() {
                    println!(
                        "Vulkan driver {} is not built: {reason}. Its GPUs show only frames \
                         another GPU draws.",
                        driver.name
                    );
                }
                None
            }
        };
        let drivers = drivers
            .iter()
            .copied()
            .filter(|driver| tools.is_some() || !driver.host_tools)
            .collect();
        Ok(Self { drivers, tools })
    }

    /// Mesa's Meson arguments for these drivers, after [`OPTIONS`].
    fn arguments(&self) -> Vec<String> {
        let names: Vec<&str> = self.drivers.iter().map(|driver| driver.name).collect();
        let mut arguments: Vec<String> = OPTIONS.map(str::to_owned).into();
        arguments.push(format!("-Dvulkan-drivers={}", names.join(",")));
        for driver in &self.drivers {
            arguments.extend(driver.options.iter().map(|option| (*option).to_owned()));
        }
        arguments
    }

    /// What Mesa's configuration record holds: its arguments and the host
    /// tools it found.
    fn record(&self) -> String {
        let mut record = self.arguments().join("\n") + "\n";
        if let Some(tools) = &self.tools {
            record.push_str(&tools.identity());
        }
        record
    }

    /// The defines that give the chosen drivers' libraries' clashing
    /// functions their own names, for every unit of the build.
    fn library_names(&self) -> Vec<String> {
        self.drivers
            .iter()
            .flat_map(|driver| driver.libraries)
            .flat_map(|library| library.renamed)
            .map(|(name, renamed)| format!("-D{name}={renamed}"))
            .collect()
    }

    /// Whether `output`, a unit of the drivers, is `driver`'s own.
    fn owner(&self, output: &str) -> Option<&'static Driver> {
        let output = output.replace('\\', "/");
        self.drivers
            .iter()
            .copied()
            .find(|driver| output.starts_with(driver.objects))
    }

    /// The driver whose own unit compiled the image object `object`.
    fn owner_of_object(&self, object: &Path) -> Option<&'static Driver> {
        let name = object.file_name()?.to_string_lossy();
        self.drivers
            .iter()
            .copied()
            .find(|driver| name.starts_with(&driver.objects.replace(['/', '.'], "_")))
    }

    /// Whether Meson links `object`'s library whole: its entry points are
    /// reached only by weak reference, which pulls no member from an archive.
    fn linked_whole(&self, object: &str) -> bool {
        let name = object_name(object);
        WHOLE_RUNTIME
            .iter()
            .chain(self.drivers.iter().flat_map(|driver| driver.whole))
            .any(|library| name.contains(library))
    }
}

/// Where Mesa's configuration records the Meson arguments it was made with.
const CONFIGURATION: &str = "configuration";

/// Whether Mesa's configuration in `build` is current: made after the C
/// library its probes linked, for the machine `description` and with the
/// Meson `arguments`, as `directory` records them.
fn configured(
    directory: &Path,
    build: &Path,
    libc: &Path,
    description: &str,
    arguments: &str,
) -> bool {
    let modified = |path: &Path| fs::metadata(path).and_then(|file| file.modified()).ok();
    let recorded = |name: &str, expected: &str| {
        fs::read_to_string(directory.join(name)).is_ok_and(|current| current == expected)
    };
    modified(&build.join("build.ninja")) > modified(libc)
        && recorded("machine.ini", description)
        && recorded(CONFIGURATION, arguments)
}

fn setup(
    toolchain: &Toolchain,
    sysroot: &Sysroot,
    directory: &Path,
    build: &Path,
    path: &Path,
    choice: &Choice,
) -> Result<()> {
    let packages = directory.join("pkgconfig");
    let machine = directory.join("machine.ini");
    let description = sysroot.machine_file(toolchain, &packages)?;
    let record = choice.record();
    if configured(directory, build, &sysroot.libc(), &description, &record) {
        return Ok(());
    }
    if build.is_dir() {
        fs::remove_dir_all(build)?;
    }
    // A configuration that fails leaves no record that it was made.
    let _ = fs::remove_file(directory.join(CONFIGURATION));
    fs::write(&machine, description)?;
    let (stubs, _, _) = paths(directory)?;
    if stubs.is_dir() {
        fs::remove_dir_all(&stubs)?;
    }
    fs::create_dir_all(&stubs)?;
    match &choice.tools {
        // The real tools, found first; glslang is found on PATH.
        Some(tools) => {
            for program in &tools.programs {
                fs::copy(
                    program,
                    stubs.join(program.file_name().context("host tool")?),
                )?;
            }
        }
        None => {
            let unused =
                UNUSED_TOOLS.map(|name| (name, "#!/usr/bin/env python3\nraise SystemExit(1)\n"));
            for (name, script) in unused.into_iter().chain([("glslangValidator", GLSLANG)]) {
                let stub = stubs.join(name);
                fs::write(&stub, script)?;
                #[cfg(unix)]
                {
                    use std::os::unix::fs::PermissionsExt;
                    fs::set_permissions(&stub, fs::Permissions::from_mode(0o755))?;
                }
            }
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
        packages.join("libdrm_amdgpu.pc"),
        format!(
            "Name: libdrm_amdgpu\nDescription: built from its sources\nVersion: {LIBDRM_VERSION}\n\
             Cflags: -I{0} -I{0}/amdgpu -I{0}/include/drm\n",
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
        .args(choice.arguments())
        .env("PATH", path)
        // The image's packages are the machine file's only: a host's own,
        // such as SPIRV-Tools for the host tools, must not be found.
        .env_remove("PKG_CONFIG_PATH");
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
    fs::write(directory.join(CONFIGURATION), record)?;
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
            .env("PATH", path)
            // As in `setup`, should Ninja configure again.
            .env_remove("PKG_CONFIG_PATH"),
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
    choice: &Choice,
) -> Result<Vec<Unit>> {
    let linked = capture(
        Command::new(ninja)
            .args(["-t", "inputs"])
            .args(choice.drivers.iter().map(|driver| driver.target))
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
    let mut models = Vec::new();
    let entries = database.as_array().context("compile database")?;
    // A source two libraries compile is the one's that is linked whole.
    let whole: BTreeSet<&str> = entries
        .iter()
        .filter(|entry| {
            entry["output"]
                .as_str()
                .is_some_and(|output| choice.linked_whole(output))
        })
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
        if !linked.contains(output.as_str())
            || (whole.contains(file) && !choice.linked_whole(output))
        {
            continue;
        }
        if let Some(driver) = choice
            .owner(output)
            .filter(|driver| file.ends_with(driver.model))
        {
            models.push((driver.name, file.to_owned(), words.clone()));
        }
        if file.ends_with(".S")
            || SKIPPED
                .iter()
                .any(|skip| output.contains(skip) || file.contains(skip))
        {
            continue;
        }
        let is_cxx = file.ends_with(".cpp") || file.ends_with(".cc");
        ensure!(
            is_cxx || file.ends_with(".c"),
            "{file} is neither C nor C++"
        );
        let mut arguments = retarget(&words, if is_cxx { cxx } else { common });
        arguments.extend(choice.library_names());
        if file.ends_with("rust/bindings.c") {
            arguments.extend(
                FLOAT_WRAPPERS
                    .iter()
                    .map(|name| format!("-D{name}__extern={name}__hard_float")),
            );
        }
        if let Some(driver) = choice.owner(output) {
            if driver.entry != ENTRY {
                arguments.push(format!("-D{ENTRY}={}", driver.entry));
            }
            arguments.extend(
                driver
                    .renamed
                    .iter()
                    .map(|(name, renamed)| format!("-D{name}={renamed}")),
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
    let model = |driver: &Driver| {
        models
            .iter()
            .find(|(name, _, _)| *name == driver.name)
            .map(|(_, file, words)| (file, words))
            .with_context(|| format!("{} has no unit {}", driver.name, driver.model))
    };
    for driver in &choice.drivers {
        let (file, words) = model(driver)?;
        for name in driver.stand_ins {
            let source = compat.join("mesa").join(format!("{name}.c"));
            units.push(like(file, words, &source, name, &[]));
        }
        for library in driver.libraries {
            let mut defines: Vec<String> = library
                .defines
                .iter()
                .map(|define| format!("-D{define}"))
                .collect();
            defines.extend(choice.library_names());
            for source in library.sources {
                let source = third_party(source);
                let stem = source
                    .file_stem()
                    .context("library source")?
                    .to_string_lossy();
                let name = format!("{}_{stem}", library.name);
                units.push(like(file, words, &source, &name, &defines));
            }
        }
    }
    let (file, words) = model(choice.drivers.first().context("no Vulkan driver")?)?;
    let model_unit =
        |source: &Path, name: &str, added: &[String]| like(file, words, source, name, added);
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

/// The record of the command a unit was last compiled with, beside its object.
fn command_record(output: &Path) -> PathBuf {
    output.with_extension("command")
}

/// Whether the unit's command or anything its last compile read has changed,
/// by the command's record and the dependency file. A new result of Meson's
/// probes changes a command and no file the unit read.
fn stale(unit: &Unit, build: &Path) -> bool {
    let output = &unit.output;
    let (Ok(built), Ok(depends), Ok(command)) = (
        fs::metadata(output).and_then(|file| file.modified()),
        fs::read_to_string(output.with_extension("d")),
        fs::read_to_string(command_record(output)),
    ) else {
        return true;
    };
    command != unit.arguments.join("\n")
        || depends
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
                    if !stale(unit, build) {
                        continue;
                    }
                    let _ = fs::remove_file(command_record(&unit.output));
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
                        Ok(output) if output.status.success() => {
                            match fs::write(command_record(&unit.output), unit.arguments.join("\n"))
                            {
                                Ok(()) => continue,
                                Err(error) => error.to_string(),
                            }
                        }
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

/// The external symbols `object` defines.
/// Weak definitions and those in a COMDAT group the linker keeps one copy of
/// (COFF's `.refptr` indirections and C++'s inline functions) are left out:
/// two drivers may each have one.
fn definitions(object: &Path) -> Result<BTreeSet<String>> {
    use object::{ComdatKind, Object, ObjectComdat, ObjectSymbol};
    let data = fs::read(object)?;
    let file = object::File::parse(&*data)
        .with_context(|| format!("{} is no object", object.display()))?;
    let merged: BTreeSet<_> = file
        .comdats()
        .filter(|comdat| comdat.kind() != ComdatKind::NoDuplicates)
        .flat_map(|comdat| {
            comdat
                .sections()
                .map(|section| section.0)
                .collect::<Vec<_>>()
        })
        .collect();
    Ok(file
        .symbols()
        .filter(|symbol| symbol.is_definition() && symbol.is_global() && !symbol.is_weak())
        .filter(|symbol| {
            symbol
                .section_index()
                .is_none_or(|section| !merged.contains(&section.0))
        })
        .filter_map(|symbol| symbol.name().ok().map(str::to_owned))
        .collect())
}

/// Checks what the drivers' own units define, by driver: each defines its
/// loader entry, and no symbol is defined by two, which one image could not
/// link.
fn check_definitions(defined: &[(&Driver, BTreeSet<String>)]) -> Result<()> {
    for (index, (driver, symbols)) in defined.iter().enumerate() {
        ensure!(
            symbols.contains(driver.entry),
            "Vulkan driver {}'s units define no {}",
            driver.name,
            driver.entry
        );
        for (other, theirs) in &defined[..index] {
            let shared: Vec<&String> = symbols.intersection(theirs).collect();
            ensure!(
                shared.is_empty(),
                "Vulkan drivers {} and {} both define {}",
                other.name,
                driver.name,
                shared
                    .iter()
                    .map(|name| name.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
    }
    Ok(())
}

/// [`check_definitions`] over the compiled units.
fn check_entries(choice: &Choice, units: &[Unit]) -> Result<()> {
    let mut defined: Vec<(&Driver, BTreeSet<String>)> = choice
        .drivers
        .iter()
        .map(|driver| (*driver, BTreeSet::new()))
        .collect();
    for unit in units {
        if let Some(driver) = choice.owner_of_object(&unit.output) {
            let symbols = definitions(&unit.output)?;
            if let Some((_, all)) = defined
                .iter_mut()
                .find(|(owner, _)| core::ptr::eq(*owner, driver))
            {
                all.extend(symbols);
            }
        }
    }
    check_definitions(&defined)
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

pub fn build(jobs: u16) -> Result<()> {
    let toolchain = super::toolchain::discover()?;
    let directory = build_directory("nvk");
    let build = directory.join("build");
    let (_, path, host) = paths(&directory)?;
    fs::create_dir_all(&directory)?;
    // The list names what the archives hold, so it goes until they are made.
    let linked = directory.join(LINKED);
    let _ = fs::remove_file(&linked);
    super::llvm::build(&toolchain, jobs, false)?;
    let sysroot = Sysroot::build(&toolchain, &directory, jobs)?;
    let choice = Choice::new(
        super::clc::tools(&host, jobs)?,
        crate::tools::release_build(),
    )?;
    setup(&toolchain, &sysroot, &directory, &build, &path, &choice)?;
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
    let units = units(&build, &objects, &ninja, &path, &common, &cxx, &choice)?;
    compile(&toolchain.clang, &build, &units, jobs)?;

    gather_constructors(&units)?;
    check_entries(&choice, &units)?;
    let (whole, lazy): (Vec<_>, Vec<_>) = units.iter().partition(|unit| {
        choice.linked_whole(
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
    let names: Vec<&str> = choice.drivers.iter().map(|driver| driver.name).collect();
    fs::write(&linked, names.join("\n") + "\n")?;
    println!(
        "{} objects of Vulkan drivers {} in {}",
        units.len(),
        names.join(", "),
        archive.display()
    );
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

    /// Without the host tools only NVK is built; with them ANV and RADV too,
    /// whose own units compile their loader entries under their own names
    /// and whose libraries Meson links whole are linked whole.
    #[test]
    fn the_drivers_follow_the_host_tools() {
        let without = Choice::new(Err("no glslangValidator on PATH".to_owned()), false).unwrap();
        assert_eq!(without.drivers.len(), 1);
        assert_eq!(without.drivers[0].name, "nouveau");
        assert!(
            without
                .arguments()
                .contains(&"-Dvulkan-drivers=nouveau".to_owned())
        );
        assert!(!without.record().contains("intel"));
        assert!(!without.record().contains("amd"));
        assert!(
            without
                .owner("src/intel/vulkan/libanv_common.a.p/anv_instance.c.o")
                .is_none()
        );

        let tools = super::super::clc::Tools {
            programs: vec!["/build/mesa_clc".into(), "/build/vtn_bindgen2".into()],
            glslang: ("/usr/bin/glslangValidator".into(), "16.2.0".to_owned()),
        };
        let with = Choice::new(Ok(tools), false).unwrap();
        assert_eq!(
            with.drivers
                .iter()
                .map(|driver| driver.name)
                .collect::<Vec<_>>(),
            ["nouveau", "intel", "amd"]
        );
        let arguments = with.arguments();
        assert!(arguments.contains(&"-Dvulkan-drivers=nouveau,intel,amd".to_owned()));
        assert!(arguments.contains(&"-Dintel-rt=disabled".to_owned()));
        assert!(arguments.contains(&"-Damd-use-llvm=false".to_owned()));
        let radv = with
            .owner("src/amd/vulkan/libvulkan_radeon.so.p/radv_instance.c.o")
            .unwrap();
        assert_eq!(radv.entry, "radv_icdGetInstanceProcAddr");
        assert_eq!(radv.libraries[0].name, "libdrm_amdgpu");
        assert!(
            with.library_names()
                .contains(&"-Damdgpu_bo_list_create=libdrm_amdgpu_bo_list_create".to_owned())
        );
        assert!(without.library_names().is_empty());
        assert!(
            with.owner("src/amd/compiler/libaco.a.p/aco_ir.cpp.o")
                .is_none()
        );
        assert!(
            with.record()
                .contains("glslang /usr/bin/glslangValidator 16.2.0")
        );
        let anv = with
            .owner("src/intel/vulkan/libanv_common.a.p/anv_instance.c.o")
            .unwrap();
        assert_eq!(anv.entry, "anv_icdGetInstanceProcAddr");
        assert_eq!(
            with.owner("src/nouveau/vulkan/libnvk.a.p/nvk_instance.c.o")
                .unwrap()
                .entry,
            ENTRY
        );
        assert!(with.owner("src/compiler/nir/libnir.a.p/nir.c.o").is_none());
        for whole in [
            "src/intel/vulkan/libanv_common.a.p/anv_device.c.o",
            "src/intel/vulkan/libanv_per_hw_ver120.a.p/genX_cmd_buffer.c.o",
            "src/vulkan/runtime/libvulkan_runtime.a.p/vk_device.c.o",
            "src/amd/vulkan/libvulkan_radeon.so.p/radv_device.c.o",
        ] {
            assert!(with.linked_whole(whole), "{whole}");
            assert_eq!(
                without.linked_whole(whole),
                !whole.contains("anv") && !whole.contains("radv"),
                "{whole}"
            );
        }
        assert!(!with.linked_whole("src/intel/vulkan/libvulkan_intel.so.p/anv_gem.c.o"));
        assert!(!with.linked_whole("src/amd/compiler/libaco.a.p/aco_ir.cpp.o"));
    }

    /// A release build without the host tools fails, naming the driver its
    /// image must hold and why it cannot be built; another build leaves the
    /// driver out, and a release leaves out one its image does not give.
    #[test]
    fn a_release_fails_without_a_driver_it_must_hold() {
        const GIVEN: Driver = Driver {
            released: true,
            ..RADV
        };
        let missing = || Err("no glslangValidator on PATH".to_owned());
        let error = Choice::of(&[&NVK, &ANV, &GIVEN], missing(), true)
            .err()
            .expect("a release without RADV")
            .to_string();
        assert_eq!(
            error,
            "a release image must hold Vulkan driver amd, which this host cannot build: \
             no glslangValidator on PATH"
        );
        let local = Choice::of(&[&NVK, &ANV, &GIVEN], missing(), false).unwrap();
        assert_eq!(local.drivers.len(), 1);
        let release = Choice::of(&[&NVK, &ANV], missing(), true).unwrap();
        assert_eq!(release.drivers[0].name, "nouveau");
        assert_eq!(release.drivers.len(), 1);
    }

    /// Two drivers' units defining one symbol, or a driver without its
    /// entry, stop the build with the names.
    #[test]
    fn colliding_or_missing_entries_are_named() {
        let set = |names: &[&str]| names.iter().map(|name| (*name).to_owned()).collect();
        let good = [
            (&NVK, set(&[ENTRY, "nvk_create_device"])),
            (
                &ANV,
                set(&["anv_icdGetInstanceProcAddr", "anv_create_device"]),
            ),
        ];
        check_definitions(&good).unwrap();
        let collide = [
            (&NVK, set(&[ENTRY])),
            (&ANV, set(&["anv_icdGetInstanceProcAddr", ENTRY])),
        ];
        let error = check_definitions(&collide).unwrap_err().to_string();
        assert!(
            error.contains("both define vk_icdGetInstanceProcAddr"),
            "{error}"
        );
        let missing = [(&NVK, set(&[ENTRY])), (&ANV, set(&[ENTRY]))];
        let error = check_definitions(&missing).unwrap_err().to_string();
        assert!(
            error.contains("define no anv_icdGetInstanceProcAddr"),
            "{error}"
        );
        let choice = Choice::new(
            Ok(super::super::clc::Tools {
                programs: Vec::new(),
                glslang: ("glslangValidator".into(), "16.2.0".to_owned()),
            }),
            false,
        )
        .unwrap();
        let object = Path::new("/o/src_intel_vulkan_libanv_common_a_p_anv_instance_c_o.o");
        assert_eq!(choice.owner_of_object(object).unwrap().name, "intel");
        let object = Path::new("/o/src_amd_vulkan_libvulkan_radeon_so_p_radv_instance_c_o.o");
        assert_eq!(choice.owner_of_object(object).unwrap().name, "amd");
        assert!(
            choice
                .owner_of_object(Path::new("/o/stela_libdrm_amdgpu_amdgpu_bo.o"))
                .is_none()
        );
        assert!(
            choice
                .owner_of_object(Path::new("/o/stela_anv_nir_options.o"))
                .is_none()
        );
    }

    /// Mesa is configured again when its Meson arguments, its machine or
    /// the C library its probes linked change, and only then.
    #[test]
    fn a_change_of_meson_arguments_configures_mesa_again() {
        let root = tempfile::tempdir().unwrap();
        let (directory, build) = (root.path(), root.path().join("build"));
        let libc = directory.join("libc.a");
        fs::write(&libc, "").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(20));
        fs::create_dir_all(&build).unwrap();
        fs::write(build.join("build.ninja"), "").unwrap();
        fs::write(directory.join("machine.ini"), "machine").unwrap();
        let current = |arguments: &str| configured(directory, &build, &libc, "machine", arguments);
        // A configuration from before the record existed.
        assert!(!current("-Da=1\n"));
        fs::write(directory.join(CONFIGURATION), "-Da=1\n").unwrap();
        assert!(current("-Da=1\n"));
        assert!(!current("-Da=1\n-Db=2\n"));
        assert!(!configured(directory, &build, &libc, "other", "-Da=1\n"));
        std::thread::sleep(std::time::Duration::from_millis(20));
        fs::write(&libc, "newer").unwrap();
        assert!(!current("-Da=1\n"));
    }

    /// A unit is compiled again when its command changes, as when Meson's
    /// probes find more, though none of the files it read did.
    #[test]
    fn a_unit_whose_command_changed_is_stale() {
        let root = tempfile::tempdir().unwrap();
        let build = root.path();
        fs::write(build.join("futex.c"), "").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(20));
        let unit = |arguments: &[&str]| Unit {
            output: build.join("futex.o"),
            arguments: arguments
                .iter()
                .map(|argument| (*argument).to_owned())
                .collect(),
        };
        let old = unit(&["-c", "futex.c"]);
        fs::write(&old.output, "").unwrap();
        fs::write(old.output.with_extension("d"), "futex.o: futex.c\n").unwrap();
        // An object from before the record existed.
        assert!(stale(&old, build));
        fs::write(command_record(&old.output), "-c\nfutex.c").unwrap();
        assert!(!stale(&old, build));
        assert!(stale(
            &unit(&["-DHAVE_LINUX_FUTEX_H", "-c", "futex.c"]),
            build
        ));
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
