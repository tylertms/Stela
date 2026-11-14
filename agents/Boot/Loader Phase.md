---
tags: [stela, boot]
---
# Loader Phase

The first half of Stela's startup. The PC's firmware is still running and can help, so Stela uses it to read files and get memory.

Read along in `crates/stela-boot/src/main.rs`, then `crates/stela-boot/src/system.rs`.

## Steps in order
1. **Keep a clean copy of itself.** Before doing anything else, Stela copies its own program image. Later it will run from a moved copy of this. (`system::take_snapshot`)
2. **Set up text output and memory.** Stela opens the serial port for log messages (`serial.rs`) and asks the firmware for memory to use as its *heap* (the pool that `Vec`, `String` and `Box` allocate from).
3. **Start the clock and the log file.** Stela measures how fast the CPU's timer ticks, and opens `boot.log` on the USB. Every message goes to the serial port and this file (`output.rs`). The screen shows one short status line per step (`status.rs`).
4. **Read settings and show the menu.** See [[Boot Modes]].
5. **Prepare the takeover** (`system::boot`). Decide how memory addresses will be shared with macOS, reserve memory for the engine, and build *page tables* (the tables a CPU uses to map addresses to real memory). Everything the next phase needs is packed into one plain struct called `Handoff` (`handoff.rs`).
6. **Say goodbye to the firmware.** This call is named `ExitBootServices`. After it, the firmware's helpers are gone for good.
7. **Switch to Stela's own page tables** and jump to the engine.

> [!info] What is LA57?
> Newer CPUs support bigger address spaces (five levels of page tables instead of four). The feature is called LA57. Turning it on needs an awkward little detour, in `la57.rs`. Why Stela wants it: [[Address Space Layout]].

## Extra jobs during install
The loader can also show the "which disk should macOS go on?" screen (`installer.rs`).

Next: [[Address Space Layout]], then [[Engine Phase]].
