---
tags: [stela, engine]
---
# Engine Overview

**In one sentence:** `crates/stela-engine` runs the code that the translator produced, and does everything a Mac chip does *besides* executing plain instructions.

## What "everything else" means
A CPU is more than "run instructions". It also:
- maps virtual addresses to memory ([[Shadow Paging]])
- separates apps from the kernel with privilege levels
- handles errors and interrupts
- keeps timers
- has hundreds of settings ([[Exceptions and System Registers]])

The translator handles ordinary instructions. The engine implements all the rest in Rust.

## Two ways to run code
| Style | Folder | Used for |
|---|---|---|
| **System mode** | `machine/` | Running all of macOS. This is the important one |
| Simple runtime | `blocks.rs`, `execution.rs`, `dispatch.rs` | Running small pieces of code in tests and tools |

## Inside `machine/`
| File | Job |
|---|---|
| `engine.rs` | The main loop. See [[Dispatch Loop]] |
| `cpu.rs` | Everything that describes the state of one CPU core |
| `mmu.rs` | Reads macOS's page tables |
| `exception.rs` | What happens on an error or interrupt |
| `system.rs` | Standard CPU settings |
| `registers.rs`, `impdef.rs` | Apple-only CPU settings |
| `identification.rs` | What the CPU says about itself when asked |
| `timer.rs` | Timers |

## Other folders
- `floating/` — floating-point math done in software, exactly as a Mac chip does it
- `pauth/` — see [[Pointer Authentication]]
- `translations/` — see [[Translation Store]]

> [!info] It does not know about PCs or Apple devices
> The engine talks to the outside world through two interfaces. That keeps it testable on any computer. See [[Host and Devices Traits]].

> [!info] Runs with and without an OS
> The engine is written without Rust's standard library (`no_std`), because inside `stela.efi` there is no operating system. In tests, the same code runs as a normal program.

Related: [[Parallel Execution]], [[Translator Overview]].
