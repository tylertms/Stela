---
tags: [stela, boot, engine]
---
# Engine Phase

The second half of startup. The firmware is gone, so Stela now does everything itself. The code is in `crates/stela-boot/src/engine.rs` and the folder `crates/stela-boot/src/engine/`.

The file `engine/start.rs` walks from "just took over" to "first macOS instruction".

## Steps in order
1. **Set up the CPU.** Tell the CPU where to jump on errors and hardware interrupts (`paging.rs`, `native_irq.rs`).
2. **Find the PC's hardware.** Scan for disks, network cards, USB controllers, keyboards and the graphics card, and start a driver for each. See [[Host Drivers]].
3. **Get the macOS boot files.** Either use the package prepared by `stela setup`, or build it now from the installed macOS on disk. See [[Boot Bundle]].
4. **Build the pretend Mac.** Create every pretend Apple device and connect each to real PC hardware. See [[Board Mount]].
5. **Copy the boot files** into the memory macOS will use.
6. **Start the processors.** Each pretend Mac CPU core gets one real PC core. See [[Parallel Execution]].
7. **Run.** `engine/run.rs` is the main loop: run macOS for a short time slice, do housekeeping, repeat. It also prints a regular "heartbeat" line so you can see it is alive.

## The glue file
`engine/host.rs` is where the engine (which knows nothing about PCs) meets the real machine. It provides memory, a place to store translated code, and timers. See [[Host and Devices Traits]] and [[Shadow Paging]].

## Other jobs in this folder
| Job | File |
|---|---|
| Let idle cores sleep to save power | `native_idle.rs` |
| Receive hardware interrupts | `native_irq.rs` |
| Save translations for next boot | `engine/translations.rs` |
| Graphics service | `engine/gpu/` |
| Help macOS install itself | `engine/restore.rs` |

Related: [[Loader Phase]], [[Dispatch Loop]].
