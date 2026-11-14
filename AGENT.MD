# AGENT.MD

Guidance for AI agents and new contributors working in this repository.

## What this project is

Stela boots unmodified Apple silicon macOS on an x86-64 PC. It is a UEFI
program (`stela.efi`) containing an arm64-to-x86-64 binary translator, a
register-level model of an Apple M2 Mac, its own PC hardware drivers, and a
Metal driver for NVIDIA GPUs. The `stela` command prepares the boot USB.

## Learn the codebase first

The learning material is an Obsidian vault in [`agents/`](agents/). Start at
[`agents/Home.md`](agents/Home.md). Notes are short (400 words at most), written
in plain language for a junior engineer, and link to each other with
`[[wikilinks]]`.

| If you are about to touch | Read |
|---|---|
| Anything, for the first time | `agents/Overview/Big Picture.md`, `Life of a Boot.md`, `Life of an Instruction.md`, `Design Values.md` |
| `crates/stela-compiler` | `agents/Translator/` |
| `crates/stela-engine` | `agents/Engine/` |
| `crates/stela-boot`, `stela-boot-core` | `agents/Boot/` |
| `crates/stela-soc*`, `stela-runtime`, `stela-loader` | `agents/Machine/` |
| `crates/stela-driver*`, `stela-drivers`, `stela-linux` | `agents/PC/` |
| `crates/stela-driver-gpu`, `stela-mesa`, `guest/stela-gpu` | `agents/Graphics/` |
| `crates/stela-cli`, `xtask`, builds, tests | `agents/Tooling/` |
| Style, errors, commits, vocabulary | `agents/Reference/` |

Not sure where something lives: `agents/Reference/Where to Look.md`.

## Commands

```sh
cargo xtask check              # the full gate: format, Clippy (host, UEFI, cross), tests
cargo xtask check --fix        # format first
cargo xtask check --low-memory # same checks, one at a time
cargo xtask dist               # release `stela` with `stela.efi` embedded
cargo xtask boot run           # boot stela.efi in QEMU with the test kernel
cargo test -p <crate>          # narrow loop while developing
```

`crates/stela-boot` and `crates/stela-mesa` are outside the workspace; build
them through `cargo xtask`. The first full build fetches and compiles Linux,
Mesa and LLVM and takes a long time.

## Rules that matter here

1. **Refuse, never guess.** Unknown registers, options and encodings return a
   `Refusal` (`refuse!`, `require!`) naming the values that decided it. No
   silent zeros, no defaults, no panics on guest-controlled input.
2. **The guest is unchanged.** Never patch Apple binaries. Adapt Stela.
3. **No Apple software in the repository.** No IPSW, firmware or extracted
   Apple code or data.
4. **Cite sources** in doc comments: Arm ARM section, Intel SDM volume, XNU
   file and line, m1n1, Linux behavior. Mark what is inferred.
5. **Keep module `//!` headers true.** They are the project's documentation.
6. **`no_std` first.** Code linked into `stela.efi` cannot use `std`.
7. **Every `unsafe` block has a `// SAFETY:` comment.**
8. **Do not hand-edit generated code** such as
   `crates/stela-engine/src/machine/system/generated.rs`; use
   `cargo xtask sysregs`.
9. **Contracts are versioned.** Changing `stela.cfg` options, the boot-bundle
   schema, `GuestState` or the StelaGPU wire interface affects another
   component; update both sides and the revision.
10. **Take extra care near disk selection and writes**
    (`crates/stela-runtime/src/platform/pc/storage.rs`): that code protects
    the user's data.

## Commits

Conventional prefix, a scope, and one specific sentence:
`perf(engine): index a pack record's entries and decode only the entry a fetch installs`.
See `agents/Reference/Commit Style.md`. Commit or push only when asked.

## Keeping the learning material current

When a change makes a note in `agents/` wrong, fix the note in the same
change. Keep each note at 400 words or fewer, in plain language a junior
engineer can follow (explain or link every special term), and keep its
wikilinks resolving. The vault is documentation and context only: no
exercises or quizzes.
