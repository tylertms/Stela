---
tags: [stela, overview, reference]
---
# Repo Map

What is in each top-level folder.

| Path | What it is |
|---|---|
| `Cargo.toml` | The list of all Rust packages (*crates*) in the project. Its comments are worth reading |
| `rust-toolchain.toml` | Which Rust version to use (1.99.1). Rust installs it automatically |
| `.cargo/config.toml` | Defines the `cargo xtask` shortcut and a few build flags |
| `crates/` | All the Rust code, one folder per crate |
| `guest/stela-gpu/` | Graphics code that runs **inside macOS** (C++ and Objective-C) |
| `guest/stela-app/` | A small Mac app, Stela.app (Swift), also inside macOS |
| `xtask/` | Helper commands for developers, like "run all checks" |
| `third_party/` | Other people's projects that Stela uses: Linux, Mesa, LLVM, and more. Stela's changes to them are in `third_party/patches/` |
| `.github/workflows/` | Automatic checks that run on GitHub |

## How big is it?
About 1.8 million lines of Rust. Nobody reads all of it. The five biggest crates:

| Crate | Lines | What |
|---|---|---|
| `stela-driver-net` | 278k | Network card drivers |
| `stela-engine` | 240k | Runs translated code |
| `stela-compiler` | 206k | The translator |
| `stela-drivers` | 189k | USB, Bluetooth, input |
| `stela-soc` | 179k | The pretend Mac hardware |

> [!warning] Two crates are special
> `crates/stela-boot` and `crates/stela-mesa` are **not** part of the normal workspace. A plain `cargo build` skips them, because `stela.efi` is built for a different target than your computer. Use `cargo xtask` to build them. See [[Build and Check]].

More detail: [[Crates - Foundations]], [[Crates - CPU and Mac]], [[Crates - PC]], [[Crates - Programs]]. How they connect: [[Dependency Shape]]. Quick lookup: [[Where to Look]].
