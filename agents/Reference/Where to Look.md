---
tags: [stela, reference]
---
# Where to Look

A quick lookup table. Find your topic on the left, open the file on the right.

| I want to understand | Start at |
|---|---|
| One Mac (Arm) instruction | `crates/stela-compiler/src/aarch64/<family>.rs` (decode), `src/x86_64/<family>.rs` (emit) |
| How PC instructions are written out | `crates/stela-compiler/src/x86_64/asm.rs` |
| A standard CPU setting (System register) | `crates/stela-engine/src/machine/system.rs` |
| An Apple-only CPU setting | `crates/stela-engine/src/machine/registers.rs`, `impdef.rs` |
| Reading macOS's page tables | `crates/stela-engine/src/machine/mmu.rs` |
| Errors and interrupts (exceptions) | `crates/stela-engine/src/machine/exception.rs`, `cpu.rs` |
| The engine's main loop | `crates/stela-engine/src/machine/engine.rs` |
| An Apple device | `crates/stela-soc/src/<device>.rs` and `crates/stela-soc-data/src/<device>.rs` |
| How the pretend Mac is assembled | `crates/stela-runtime/src/machine/mount.rs` |
| A PC driver | `crates/stela-drivers/src/probe.rs`, then the class crate |
| A setting in `stela.cfg` | `crates/stela-boot-core/src/options.rs` |
| What is on the USB | `crates/stela-cli/src/setup.rs`, `crates/stela-loader/src/mount_interface.rs` |
| A graphics (Metal) call | `guest/stela-gpu/bundle/`, `crates/stela-driver-gpu/src/metal/service.rs` |
| How macOS gets installed | `crates/stela-runtime/src/restore.rs` |
| A `cargo xtask` task | `xtask/src/main.rs` |

> [!tip] Getting oriented in any crate
> Open `src/lib.rs` first. It lists the crate's modules, and each module file starts with a `//!` comment saying what it is for. Together they work like a table of contents.

Related: [[Repo Map]], [[Home]].
