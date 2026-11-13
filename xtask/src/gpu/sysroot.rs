//! The machine Meson configures Mesa for, the same from every build host: Linux
//! on x86-64, whose system libraries are Stela's C library and libc++. Mesa's
//! probes compile and link against the libraries `stela.efi` contains, so what
//! they find is true of the image, and no build host chooses its contents.
use super::{
    llvm,
    nvk::{LIBC, LIBC_TEST_FLAGS, REGEX, Unit, compile, resource_directory},
    support::{gpu, slashed, third_party},
    toolchain::Toolchain,
};
use anyhow::Result;
use std::{
    fs,
    path::{Path, PathBuf},
};

pub(super) const TARGET: &str = "--target=x86_64-unknown-linux-gnu";
/// The Rust of the machine: nothing builds with it, and Meson checks that it
/// links programs. This target carries the C library that takes.
const RUST_TARGET: &str = "x86_64-unknown-linux-musl";
/// Some of Mesa's probes link shared libraries.
pub(super) const POSITION_INDEPENDENT: &str = "-fPIC";
/// What the machine's C library is, said to every compile and probe against
/// it rather than left to the build host's Clang: distributions change its
/// defaults (Arch and Nix turn on the stack protector, whose
/// `__stack_chk_fail` Stela's C library lacks, so every link probe fails and
/// Mesa decides the library has no `qsort_r`; others add stack clash
/// probes, CET branch markers, `_FORTIFY_SOURCE` or libstdc++ assertions),
/// and configuration files can add anything, which `--no-default-config`
/// drops.
pub(super) const FREESTANDING: [&str; 7] = [
    "--no-default-config",
    "-Qunused-arguments",
    "-fno-stack-protector",
    "-fno-stack-clash-protection",
    "-fcf-protection=none",
    "-U_FORTIFY_SOURCE",
    "-U_GLIBCXX_ASSERTIONS",
];
/// Mesa's probes link programs and shared libraries: position-independent
/// code ([`POSITION_INDEPENDENT`]) linked as a position-independent
/// executable, whichever the host's Clang makes by default.
pub(super) const PROBE_LINK: [&str; 2] = ["--no-default-config", "-pie"];

pub(super) struct Sysroot {
    directory: PathBuf,
    /// What Mesa's C compiles and its bindings parse with.
    pub compile: Vec<String>,
    compile_cxx: Vec<String>,
}

impl Sysroot {
    /// Builds the libraries under `<parent>/sysroot/lib`.
    pub fn build(toolchain: &Toolchain, parent: &Path, jobs: u16) -> Result<Self> {
        let directory = parent.join("sysroot");
        let objects = directory.join("objects");
        let libraries = directory.join("lib");
        fs::create_dir_all(&libraries)?;
        fs::create_dir_all(&objects)?;

        let libc = gpu().join("compat/libc");
        let resource = format!("{}/include", resource_directory(&toolchain.clang)?);
        let llvm = third_party("llvm");
        let c_unit = |source: PathBuf, name: &str| {
            let mut arguments = LIBC_TEST_FLAGS.map(str::to_owned).to_vec();
            arguments.extend([POSITION_INDEPENDENT.to_owned(), "-isystem".to_owned()]);
            arguments.push(libc.display().to_string());
            arguments.extend(["-isystem".to_owned(), resource.clone()]);
            arguments.push(format!("-I{}", gpu().join("compat/llvm/include").display()));
            arguments.push(format!("-I{}", llvm.join("llvm/include").display()));
            arguments.extend(["-c".to_owned(), source.display().to_string()]);
            Unit {
                output: objects.join(format!("{}.o", name.replace('/', "-"))),
                arguments,
            }
        };
        let units: Vec<_> = LIBC
            .into_iter()
            .chain(["tests/host"])
            .map(|name| c_unit(libc.join(format!("{name}.c")), name))
            .chain(REGEX.map(|name| c_unit(llvm.join(format!("llvm/lib/Support/{name}.c")), name)))
            .collect();
        let libcxx = llvm::libcxx_units(
            toolchain,
            &[TARGET, POSITION_INDEPENDENT],
            &objects.join("libcxx"),
        )?;
        compile(&toolchain.clang, &directory, &units, jobs)?;
        compile(&toolchain.clang, &directory, &libcxx, jobs)?;
        archive(toolchain, &libraries.join("libc.a"), &units)?;
        archive(toolchain, &libraries.join("libc++.a"), &libcxx)?;

        // The compiler's own directory is a default one, which Meson drops
        // from `-isystem`.
        let installed = format!("-idirafter{}", resource.replace('\\', "/"));
        let system = |include: &Path| format!("-isystem{}", slashed(include));
        let mut shared = vec![TARGET.to_owned(), "-nostdinc".to_owned()];
        shared.extend(FREESTANDING.map(str::to_owned));
        shared.push(POSITION_INDEPENDENT.to_owned());
        let mut compile = shared.to_vec();
        compile.extend([system(&libc), installed.clone(), "-DSTELA_LIBC".to_owned()]);
        let compat = gpu().join("compat/llvm");
        let mut compile_cxx = shared.to_vec();
        compile_cxx.extend([
            "-nostdinc++".to_owned(),
            system(&compat.join("libcxx")),
            system(&llvm.join("libcxx/include")),
            system(&libc),
            installed,
            "-DSTELA_LIBC".to_owned(),
        ]);
        Ok(Self {
            directory,
            compile,
            compile_cxx,
        })
    }

    /// The C library, which Mesa's configuration depends on.
    pub fn libc(&self) -> PathBuf {
        self.directory.join("lib/libc.a")
    }

    /// Meson's description of the machine, with `.pc` files in `packages`.
    pub fn machine_file(&self, toolchain: &Toolchain, packages: &Path) -> Result<String> {
        let list = |flags: &[String]| {
            flags
                .iter()
                .map(|flag| format!("'{flag}'"))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let mut link = vec![
            TARGET.to_owned(),
            format!("--ld-path={}", slashed(&toolchain.linker)),
            format!("--sysroot={}", slashed(&self.directory)),
            "-nostdlib".to_owned(),
        ];
        link.extend(PROBE_LINK.map(str::to_owned));
        let mut link_cxx = link.clone();
        link.push("-lc".to_owned());
        link_cxx.extend(["-lc++".to_owned(), "-lc".to_owned()]);
        Ok(format!(
            "[binaries]\n\
             c = '{clang}'\n\
             cpp = '{clangxx}'\n\
             ar = '{archiver}'\n\
             nm = '{symbols}'\n\
             rust = ['rustc', '--target', '{RUST_TARGET}', '-C', 'link-arg={TARGET}', \
             '-C', 'link-arg=--ld-path={linker}']\n\
             pkg-config = '{pkg_config}'\n\
             [built-in options]\n\
             c_args = [{compile}]\n\
             cpp_args = [{compile_cxx}]\n\
             c_link_args = [{link}]\n\
             cpp_link_args = [{link_cxx}]\n\
             [properties]\n\
             needs_exe_wrapper = true\n\
             pkg_config_libdir = ['{packages}']\n\
             [host_machine]\n\
             system = 'linux'\n\
             cpu_family = 'x86_64'\n\
             cpu = 'x86_64'\n\
             endian = 'little'\n",
            clang = slashed(&toolchain.clang),
            clangxx = slashed(&toolchain.clangxx),
            archiver = slashed(&toolchain.archiver),
            symbols = slashed(&toolchain.symbols),
            linker = slashed(&toolchain.linker),
            pkg_config = super::toolchain::pkg_config()?,
            compile = list(&self.compile),
            compile_cxx = list(&self.compile_cxx),
            link = list(&link),
            link_cxx = list(&link_cxx),
            packages = slashed(packages),
        ))
    }
}

/// Archives `units` unless `archive` is newer than each of them.
fn archive(toolchain: &Toolchain, archive: &Path, units: &[Unit]) -> Result<()> {
    let built = |path: &Path| fs::metadata(path).and_then(|file| file.modified()).ok();
    let current = built(archive);
    if current.is_some() && units.iter().all(|unit| built(&unit.output) < current) {
        return Ok(());
    }
    if archive.is_file() {
        fs::remove_file(archive)?;
    }
    super::support::archive(
        &toolchain.archiver,
        archive,
        units.iter().map(|unit| &unit.output),
    )
}
