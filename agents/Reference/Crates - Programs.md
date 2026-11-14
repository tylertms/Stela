---
tags: [stela, reference, crates]
---
# Crates - Programs

These crates produce things you can actually run.

| Crate | Lines | Produces | In plain words |
|---|---|---|---|
| `stela-boot` | 68k | `stela.efi` | The program the PC starts from the USB |
| `stela-boot-core` | 47k | a library | The parts of `stela.efi` that can also run and be tested on your computer |
| `stela-cli` | 38k | `stela` | The command users run to prepare the USB |
| `stela-dev` | 4k | `stela-dev` | Experiment commands for developers |
| `stela-gpu-maintenance` | 1k | a small tool | Installs Stela's graphics parts from inside macOS |
| `xtask` | 55k | `cargo xtask` | Helper tasks: build, check, run in QEMU |

## Why are there two "boot" crates?
`stela.efi` runs on a bare PC, which makes it hard to test: you would need to reboot a machine for every change.

So the team moved everything that does **not** need the PC's firmware into `stela-boot-core`. That code can run in ordinary unit tests on your laptop. Only the truly hardware-bound code stays in `stela-boot`.

> [!tip] A rule for new boot code
> Ask: "does this need the firmware or real hardware?" If no, put it in `stela-boot-core` and write a test.

## How `stela` carries `stela.efi`
The released `stela` command contains a copy of `stela.efi` inside it, so users download only one file. `cargo xtask dist` builds `stela.efi` first, then embeds it.

## What a plain `cargo build` builds
The crates shipped to users. It skips `stela-dev` and `xtask` (add `-p stela-dev` to build one), and it skips `stela-boot`, which needs `cargo xtask boot build`.

See [[Loader Phase]], [[Setup and USB]], [[Dev Commands]], [[Build and Check]].
