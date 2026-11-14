---
tags: [stela, drivers, gpu]
---
# Linux Environment

## The problem
The driver for NVIDIA graphics cards is enormous and complicated. An open-source one already exists inside Linux, called **nouveau**. Rewriting it in Rust would take years.

## The solution
Stela takes the Linux driver's C source code **unchanged** and compiles it into `stela.efi`.

But that code expects to run inside the Linux kernel. It calls Linux functions to get memory, start background tasks, load firmware files, and so on. Those don't exist in Stela.

So `crates/stela-linux` provides **stand-ins for the Linux functions the driver needs**. The driver calls what it thinks is Linux; it is really Stela's Rust code.

## What is provided
| File | Stands in for |
|---|---|
| `heap.rs`, `pages.rs` | Linux's memory allocation |
| `task.rs`, `threads.rs` | Linux's background tasks |
| `srcu.rs` | One of Linux's locking tools |
| `firmware.rs` | Loading firmware files |
| `resources.rs`, `owner.rs` | Tracking which driver owns which hardware |

Rust functions that C code must call are marked `extern "C"` and `#[unsafe(no_mangle)]`, which gives them a plain name C can find.

## Where the Linux code lives
- `third_party/linux` — the Linux source, pulled in as a git submodule
- `third_party/patches/linux/` — Stela's small changes, kept as patch files
- `crates/stela-c-build` — compiles the C code with the right settings

## Working with the patches
```
cargo xtask gpu fetch     # download the sources and apply Stela's patches
cargo xtask gpu export    # save your new changes back as patch files
```

> [!warning] Licensing
> Linux is under the GPL-2.0 license, and so is Stela. Some files carry a line like `SPDX-License-Identifier: GPL-2.0-only`. Never remove these lines. See `LICENSE` and `NOTICE`.

Related: [[GPU and Metal]], [[Host Drivers]], [[Build and Check]].
