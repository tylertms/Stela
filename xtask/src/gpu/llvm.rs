//! LLVM for `stela.efi`: the AIR compiler's bitcode reader and writer,
//! verifier, linker and passes, unpatched, over libc++ and `compat/libc`.
//!
//! LLVM's CMake only states what these steps state here, and the build host
//! need not have it. `llvm-min-tblgen` builds natively and generates what
//! `include/llvm` lacks, `llvm-tblgen` generates the x86 target's tables from
//! that, and each library compiles for the boot image's C target. The image's
//! linker resolves every archive member it pulls before it discards any, so
//! [`SKIPPED`] cuts what would reach past those needs at its root, and
//! `compat/llvm` answers what is left of those units.
use super::{
    nvk::{
        LIBC, LIBC_TEST_FLAGS, TARGET_FLAGS, Unit, compile, gather_constructors, resource_directory,
    },
    support::{NATIVE_TARGET, build_directory, gpu, third_party},
    toolchain::Toolchain,
};
use crate::tools::exec;
use anyhow::{Context, Result, ensure};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

/// What the natively built `llvm-min-tblgen` is made of, under `llvm/`.
const TOOL_SOURCES: [&str; 5] = [
    "lib/Support",
    "lib/Demangle",
    "lib/TableGen",
    "utils/TableGen/Basic",
    "utils/TableGen",
];
/// Of `utils/TableGen`, only the small tool's own unit.
const TOOL_MAIN: &str = "llvm-min-tblgen.cpp";
/// The `include/llvm` directories whose `CMakeLists.txt` names generated files.
const GENERATED: [&str; 4] = ["IR", "Analysis", "CodeGen", "TargetParser"];
/// Every source file of these libraries, under `llvm/lib`.
const AIR_LIBRARIES: [&str; 12] = [
    "Support",
    "Demangle",
    "BinaryFormat",
    "Bitstream/Reader",
    "Bitcode/Reader",
    "Bitcode/Writer",
    "IR",
    "Remarks",
    "Linker",
    "Analysis",
    "Transforms/Utils",
    "ProfileData",
];
/// The passes the AIR compiler runs, by source, with the C API's data layouts
/// and the bitcode writer's string table.
const UNITS: [&str; 12] = [
    "MC/StringTableBuilder.cpp",
    "Target/Target.cpp",
    "TargetParser/AMDGPUTargetParser.cpp",
    "TargetParser/ARMTargetParser.cpp",
    "TargetParser/ARMTargetParserCommon.cpp",
    "TargetParser/Triple.cpp",
    "Transforms/IPO/AlwaysInliner.cpp",
    "Transforms/IPO/GlobalOpt.cpp",
    "Transforms/Scalar/DCE.cpp",
    "Transforms/Scalar/InstSimplifyPass.cpp",
    "Transforms/Scalar/SCCP.cpp",
    "Transforms/Scalar/SROA.cpp",
];
/// libc++'s out-of-line code, under `libcxx/src`.
const LIBCXX: [&str; 23] = [
    "algorithm",
    "bind",
    "call_once",
    "charconv",
    "chrono",
    "condition_variable",
    "condition_variable_destructor",
    "error_category",
    "functional",
    "future",
    "hash",
    "memory",
    "mutex",
    "mutex_destructor",
    "new_handler",
    "optional",
    "shared_mutex",
    "stdexcept",
    "string",
    "system_error",
    "thread",
    "vector",
    "verbose_abort",
];
/// Units of [`LIBRARIES`] that bring what the image lacks: processes, signals
/// and sockets, or object files through a profile's reader and the legacy
/// passes through `Analysis.cpp`. `compat/llvm` has what nothing reaches of them.
const SKIPPED: [&str; 22] = [
    "Analysis/Analysis.cpp",
    "CodeGen/AssignmentTrackingAnalysis.cpp",
    "ExecutionEngine/GDBRegistrationListener.cpp",
    "Object/IRObjectFile.cpp",
    "Object/IRSymtab.cpp",
    "Object/ModuleSymbolTable.cpp",
    "Object/OffloadBinary.cpp",
    "Object/RecordStreamer.cpp",
    "ProfileData/InstrProfCorrelator.cpp",
    "ProfileData/InstrProfReader.cpp",
    "Support/CachePruning.cpp",
    "Support/CrashRecoveryContext.cpp",
    "Support/DynamicLibrary.cpp",
    "Support/ExponentialBackoff.cpp",
    "Support/LockFileManager.cpp",
    "Support/Memory.cpp",
    "Support/Mustache.cpp",
    "Support/Program.cpp",
    "Support/Signals.cpp",
    "Support/raw_os_ostream.cpp",
    "Support/raw_socket_stream.cpp",
    "Transforms/Instrumentation/AddressSanitizer.cpp",
];
/// Stela's own, under `compat/llvm`.
const STELA: [&str; 3] = ["passes", "runtime", "support"];
pub(super) const CXX_FLAGS: [&str; 8] = [
    "-std=c++17",
    "-O2",
    "-DNDEBUG",
    "-DLLVM_BUILD_STATIC",
    "-D__STDC_CONSTANT_MACROS",
    "-D__STDC_FORMAT_MACROS",
    "-D__STDC_LIMIT_MACROS",
    "-fno-exceptions",
];
/// What LLVM's Windows support code expects of a unit that builds on Windows.
pub(super) const TOOL_WINDOWS_FLAGS: [&str; 4] = [
    "-DUNICODE",
    "-D_UNICODE",
    "-D_HAS_EXCEPTIONS=0",
    "-D_WIN32_WINNT=0x0A00",
];
/// The system libraries `llvm-min-tblgen` links.
pub(super) const TOOL_LIBRARIES: &[&str] = if cfg!(windows) {
    &[
        "-lole32",
        "-lpsapi",
        "-lshell32",
        "-lws2_32",
        "-lntdll",
        "-luuid",
        "-ladvapi32",
    ]
} else {
    &["-lm"]
};
/// What a C unit drops of the flags above.
pub(super) const CXX_ONLY: [&str; 4] = [
    "-std=c++17",
    "-fno-rtti",
    "-nostdinc++",
    "-fno-threadsafe-statics",
];
/// BLAKE3's portable C.
pub(super) const BLAKE3_FLAGS: [&str; 5] = [
    "-DBLAKE3_NO_AVX2",
    "-DBLAKE3_NO_AVX512",
    "-DBLAKE3_NO_SSE2",
    "-DBLAKE3_NO_SSE41",
    "-DBLAKE3_USE_NEON=0",
];
pub(super) const TARGET_CXX_FLAGS: [&str; 4] = [
    "-fno-rtti",
    "-fno-threadsafe-statics",
    "-nostdinc++",
    "-D__unix__",
];

fn sources(directory: &Path, extensions: &[&str], found: &mut Vec<PathBuf>) -> Result<()> {
    let mut entries: Vec<_> = fs::read_dir(directory)
        .with_context(|| directory.display().to_string())?
        .collect::<Result<_, _>>()?;
    entries.sort_by_key(fs::DirEntry::path);
    found.extend(entries.iter().map(fs::DirEntry::path).filter(|path| {
        path.extension()
            .is_some_and(|extension| extensions.iter().any(|known| extension == *known))
    }));
    Ok(())
}

fn unit(source: &Path, objects: &Path, root: &Path, flags: &[String]) -> Result<Unit> {
    let relative = source.strip_prefix(root).unwrap_or(source);
    let name = relative.to_string_lossy().replace(['/', '\\', '.'], "_") + ".o";
    let mut arguments = flags.to_vec();
    arguments.push(
        if source.extension().is_some_and(|extension| extension == "c") {
            "-xc".to_owned()
        } else {
            "-xc++".to_owned()
        },
    );
    arguments.extend(["-c".to_owned(), source.display().to_string()]);
    Ok(Unit {
        output: objects.join(name),
        arguments,
    })
}

/// The statements of a `CMakeLists.txt` that matter here, each on one line.
fn statements(plan: &str) -> Vec<String> {
    let mut statements = Vec::new();
    let mut open = String::new();
    for line in plan.lines().map(str::trim) {
        open.push_str(line);
        if open.starts_with("tablegen(") && !open.ends_with(')') {
            open.push(' ');
        } else {
            statements.push(std::mem::take(&mut open));
        }
    }
    statements
}

/// Runs the `tablegen()` calls of `source`'s `CMakeLists.txt` into `output`.
fn tablegen_directory(tool: &Path, llvm: &Path, source: &Path, output: &Path) -> Result<()> {
    fs::create_dir_all(output)?;
    let plan = fs::read_to_string(source.join("CMakeLists.txt"))?;
    let mut definitions = None;
    for line in statements(&plan) {
        if let Some(rest) = line.strip_prefix("set(LLVM_TARGET_DEFINITIONS ") {
            definitions = rest.strip_suffix(')').map(str::to_owned);
        }
        let Some(call) = line
            .strip_prefix("tablegen(LLVM ")
            .and_then(|call| call.strip_suffix(')'))
        else {
            continue;
        };
        let mut words = call.split_whitespace();
        let target = output.join(words.next().context("a tablegen output")?);
        let definitions = definitions.as_deref().context("tablegen without input")?;
        // A definition outside the checkout belongs to a target Stela omits.
        let input = match definitions.strip_prefix("${PROJECT_SOURCE_DIR}/") {
            Some(path) => llvm.join(path),
            None => source.join(definitions),
        };
        if !input.is_file() {
            continue;
        }
        if target.is_file()
            && fs::metadata(&target)?.modified()? > fs::metadata(tool)?.modified()?
        {
            continue;
        }
        exec(
            Command::new(tool)
                .args(words.map(|word| word.replace('"', "")))
                .arg("-I")
                .arg(llvm.join("include"))
                .arg("-I")
                .arg(source)
                .arg("-I")
                .arg(llvm.join("lib/Target"))
                .arg("-o")
                .arg(&target)
                .arg(&input),
        )?;
    }
    Ok(())
}

/// LLVM's generated headers, from the `tablegen()` calls of its own `CMakeLists`.
fn tablegen(tool: &Path, llvm: &Path, generated: &Path) -> Result<()> {
    for directory in GENERATED {
        let source = llvm.join("include/llvm").join(directory);
        tablegen_directory(tool, llvm, &source, &generated.join("llvm").join(directory))?;
    }
    Ok(())
}

/// Builds `llvm-min-tblgen` for this machine.
fn build_tool(toolchain: &Toolchain, llvm: &Path, directory: &Path, jobs: u16) -> Result<PathBuf> {
    let objects = directory.join("tool");
    fs::create_dir_all(&objects)?;
    let mut flags: Vec<String> = CXX_FLAGS.map(str::to_owned).to_vec();
    flags.push("-O1".to_owned());
    if cfg!(windows) {
        flags.extend(TOOL_WINDOWS_FLAGS.map(str::to_owned));
    }
    for include in [gpu().join("compat/llvm/include"), llvm.join("include")] {
        flags.push(format!("-I{}", include.display()));
    }
    let mut files = Vec::new();
    for source in TOOL_SOURCES {
        let source = llvm.join(source);
        sources(&source, &["cpp", "c"], &mut files)?;
    }
    sources(&llvm.join("lib/Support/BLAKE3"), &["c"], &mut files)?;
    files.retain(|file| {
        let name = file.file_name().unwrap_or_default().to_string_lossy();
        let parent = file.parent().and_then(Path::file_name).unwrap_or_default();
        (parent != "TableGen" || !file.starts_with(llvm.join("utils")) || name == TOOL_MAIN)
            && (parent != "BLAKE3"
                || !name.contains("avx") && !name.contains("sse") && !name.contains("neon"))
    });
    let units = files
        .iter()
        .map(|file| {
            let mut flags = flags.clone();
            flags.push(format!("-I{}", llvm.join("utils/TableGen").display()));
            flags.push(format!("-I{}", llvm.join("lib/Support").display()));
            flags.push(format!("-I{}", llvm.join("../libc").display()));
            flags.push(format!(
                "-I{}",
                llvm.join("../third-party/siphash/include").display()
            ));
            if file.extension().is_some_and(|extension| extension == "c") {
                flags.retain(|flag| flag != "-std=c++17");
                flags.push("-DBLAKE3_USE_NEON=0".to_owned());
                flags.extend(BLAKE3_FLAGS.map(str::to_owned));
            }
            unit(file, &objects, llvm, &flags)
        })
        .collect::<Result<Vec<_>>>()?;
    compile(&toolchain.clang, directory, &units, jobs)?;
    let tool = directory.join(format!("llvm-min-tblgen{}", std::env::consts::EXE_SUFFIX));
    let newest = units
        .iter()
        .map(|unit| fs::metadata(&unit.output)?.modified())
        .collect::<std::io::Result<Vec<_>>>()?
        .into_iter()
        .max();
    let current = fs::metadata(&tool)
        .ok()
        .and_then(|file| file.modified().ok());
    if current < newest {
        exec(
            Command::new(&toolchain.clangxx)
                .arg("-o")
                .arg(&tool)
                .args(units.iter().map(|unit| &unit.output))
                .args(TOOL_LIBRARIES),
        )?;
    }
    Ok(tool)
}

/// What every unit of the libraries and libc++ compiles with for `target`.
fn target_flags(toolchain: &Toolchain, checkout: &Path, target: &[&str]) -> Result<Vec<String>> {
    let compat = gpu().join("compat");
    let resource = resource_directory(&toolchain.clang)?;
    let mut flags: Vec<String> = target.iter().map(|&flag| flag.to_owned()).collect();
    flags.extend(TARGET_FLAGS.map(str::to_owned));
    flags.retain(|flag| flag != "-U__unix__");
    flags.extend(CXX_FLAGS.map(str::to_owned));
    flags.extend(TARGET_CXX_FLAGS.map(str::to_owned));
    for include in [
        compat.join("llvm/libcxx"),
        compat.join("llvm/include"),
        checkout.join("libcxx/include"),
        compat.join("libc"),
        PathBuf::from(format!("{resource}/include")),
    ] {
        flags.extend(["-isystem".to_owned(), include.display().to_string()]);
    }
    Ok(flags)
}

/// What a C++ unit of Mesa's compiles with for the image: libc++ ahead of the C
/// library in `common`, and LLVM's headers from its own configuration.
pub(super) fn cxx_flags(common: &[String]) -> Vec<String> {
    let compat = gpu().join("compat");
    let mut flags = Vec::new();
    for include in [
        compat.join("llvm/libcxx"),
        compat.join("llvm/include"),
        third_party("llvm").join("libcxx/include"),
    ] {
        flags.extend(["-isystem".to_owned(), include.display().to_string()]);
    }
    flags.extend_from_slice(common);
    flags.extend(CXX_FLAGS.map(str::to_owned));
    flags.extend(TARGET_CXX_FLAGS.map(str::to_owned));
    flags
}

/// The units of libc++, with `flags`, in `objects`.
fn out_of_line_units(checkout: &Path, objects: &Path, flags: &[String]) -> Result<Vec<Unit>> {
    let libcxx = checkout.join("libcxx/src");
    let mut library_flags = flags.to_vec();
    library_flags.retain(|flag| flag != "-std=c++17");
    library_flags.extend(["-std=c++23", "-D_LIBCPP_BUILDING_LIBRARY"].map(str::to_owned));
    library_flags.push(format!("-I{}", libcxx.display()));
    library_flags.push(format!("-I{}", checkout.join("libc").display()));
    LIBCXX
        .into_iter()
        .map(|name| {
            let file = libcxx.join(format!("{name}.cpp"));
            unit(&file, objects, checkout, &library_flags)
        })
        .collect()
}

/// The units of libc++ for `target`, in `objects`.
pub(super) fn libcxx_units(
    toolchain: &Toolchain,
    target: &[&str],
    objects: &Path,
) -> Result<Vec<Unit>> {
    fs::create_dir_all(objects)?;
    let checkout = third_party("llvm");
    let flags = target_flags(toolchain, &checkout, target)?;
    out_of_line_units(&checkout, objects, &flags)
}

/// Every unit of the libraries, libc++ and `compat/llvm` for `target`, in
/// `objects`.
fn library_units(
    toolchain: &Toolchain,
    checkout: &Path,
    generated: &Path,
    objects: &Path,
    target: &[&str],
) -> Result<Vec<Unit>> {
    fs::create_dir_all(objects)?;
    let llvm = checkout.join("llvm");
    let compat = gpu().join("compat");
    let mut flags = target_flags(toolchain, checkout, target)?;
    for include in [generated.to_owned(), llvm.join("include")] {
        flags.push(format!("-I{}", include.display()));
    }

    let mut units = Vec::new();
    let library = llvm.join("lib");
    let mut files = Vec::new();
    for name in AIR_LIBRARIES {
        sources(&library.join(name), &["cpp", "c"], &mut files)?;
    }
    sources(&library.join("Support/BLAKE3"), &["c"], &mut files)?;
    files.retain(|file| {
        let name = file.file_name().unwrap_or_default().to_string_lossy();
        file.parent().and_then(Path::file_name).unwrap_or_default() != "BLAKE3"
            || !name.contains("avx") && !name.contains("sse") && !name.contains("neon")
    });
    files.retain(|file| !SKIPPED.iter().any(|name| file.ends_with(name)));
    files.extend(UNITS.map(|name| library.join(name)));
    files.sort();
    files.dedup();
    for file in &files {
        let mut flags = flags.clone();
        flags.push(format!("-I{}", library.join("Support").display()));
        flags.push(format!("-I{}", checkout.join("libc").display()));
        flags.push(format!(
            "-I{}",
            checkout.join("third-party/siphash/include").display()
        ));
        if file.extension().is_some_and(|extension| extension == "c") {
            flags.retain(|flag| !CXX_ONLY.contains(&flag.as_str()));
            flags.extend(BLAKE3_FLAGS.map(str::to_owned));
        }
        units.push(unit(file, objects, checkout, &flags)?);
    }
    units.extend(out_of_line_units(checkout, objects, &flags)?);
    for name in STELA {
        let file = compat.join(format!("llvm/{name}.cpp"));
        units.push(unit(&file, objects, &compat, &flags)?);
    }
    Ok(units)
}

/// Builds `target/gpu/nvk/libstela-llvm.a`, and with `test` checks the same
/// units as a Linux program (`compat/llvm/tests/llvm_test.c`).
pub(super) fn build(toolchain: &Toolchain, jobs: u16, test: bool) -> Result<()> {
    let directory = build_directory("llvm");
    fs::create_dir_all(&directory)?;
    let checkout = third_party("llvm");
    let llvm = checkout.join("llvm");
    ensure!(
        llvm.join("lib/IR").is_dir(),
        "{} has no LLVM sources: run `cargo xtask gpu fetch`",
        llvm.display()
    );
    let generated = directory.join("generated");
    let tool = build_tool(crate::clang::toolset()?, &llvm, &directory, jobs)?;
    tablegen(&tool, &llvm, &generated)?;

    let target = format!("--target={NATIVE_TARGET}");
    let objects = directory.join("objects");
    let units = library_units(toolchain, &checkout, &generated, &objects, &[&target])?;
    println!("compiling {} units", units.len());
    compile(&toolchain.clang, &directory, &units, jobs)?;
    gather_constructors(&units)?;

    let archive = build_directory("nvk").join("libstela-llvm.a");
    fs::create_dir_all(archive.parent().context("the archive's directory")?)?;
    if archive.is_file() {
        fs::remove_file(&archive)?;
    }
    super::support::archive(
        &toolchain.archiver,
        &archive,
        units.iter().map(|unit| &unit.output),
    )?;
    println!("{} objects in {}", units.len(), archive.display());
    if test {
        test_hosted(toolchain, &checkout, &generated, &directory, jobs)?;
    }
    Ok(())
}

/// Builds and runs `compat/llvm/tests/llvm_test.c` over the libraries, libc++
/// and the C library.
fn test_hosted(
    toolchain: &Toolchain,
    checkout: &Path,
    generated: &Path,
    directory: &Path,
    jobs: u16,
) -> Result<()> {
    let output = directory.join("hosted");
    let units = library_units(
        toolchain,
        checkout,
        generated,
        &output.join("objects"),
        &["--target=x86_64-unknown-linux-gnu", "-fno-pic"],
    )?;
    println!("compiling {} units for the test", units.len());
    compile(&toolchain.clang, directory, &units, jobs)?;

    let compat = gpu().join("compat");
    let libc = compat.join("libc");
    let tests = compat.join("llvm/tests");
    let fixture = output.join("fixture.bc");
    exec(
        Command::new(&toolchain.clang)
            .args(["--target=x86_64-unknown-linux-gnu", "-O0"])
            .args([
                "-Xclang",
                "-disable-O0-optnone",
                "-Xclang",
                "-disable-llvm-passes",
            ])
            .args(["-emit-llvm", "-c"])
            .arg(tests.join("fixture.c"))
            .arg("-o")
            .arg(&fixture),
    )?;
    let resource = format!("{}/include", resource_directory(&toolchain.clang)?);
    let mut objects: Vec<_> = units.iter().map(|unit| unit.output.clone()).collect();
    let sources = LIBC
        .iter()
        .map(|name| libc.join(format!("{name}.c")))
        .chain([libc.join("tests/host.c"), tests.join("llvm_test.c")]);
    for source in sources {
        let stem = source.file_stem().unwrap_or_default().to_string_lossy();
        let object = output.join(format!("{stem}.o"));
        exec(
            Command::new(&toolchain.clang)
                .args(LIBC_TEST_FLAGS)
                .arg(format!("-DFIXTURE=\"{}\"", fixture.display()))
                .arg("-isystem")
                .arg(&libc)
                .arg("-isystem")
                .arg(&resource)
                .arg(format!("-I{}", libc.join("tests").display()))
                .arg(format!(
                    "-I{}",
                    gpu().join("compiler/llvm/include").display()
                ))
                .arg(format!("-I{}", checkout.join("llvm/include").display()))
                .arg("-c")
                .arg(&source)
                .arg("-o")
                .arg(&object),
        )?;
        objects.push(object);
    }
    let program = output.join("llvm_test");
    exec(
        Command::new(&toolchain.linker)
            .args(["-static", "--gc-sections", "-o"])
            .arg(&program)
            .args(&objects),
    )?;
    exec(&mut Command::new(&program)).context("LLVM's test failed")
}
