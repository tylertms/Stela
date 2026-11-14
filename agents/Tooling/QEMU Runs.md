---
tags: [stela, tooling, debug]
---
# QEMU Runs

**QEMU** is a free program that simulates a whole computer. With it you can run Stela on your development machine without rebooting or a spare PC.

```
cargo xtask boot run
```
This builds `stela.efi`, creates a pretend USB with it, and starts a simulated PC that boots from it — just as a real PC would. Code: `xtask/src/boot.rs`.

## What runs inside by default
Not macOS. Stela comes with its own **tiny test kernel**, written in assembly: `xtask/src/boot/kernel.S`. It is small but exercises the important paths: memory mapping, privilege levels, errors, a timer, and two small "apps" that take turns.

It is great for learning and testing, because it needs no Apple files.

## Useful flags
| Flag | Purpose |
|---|---|
| `--bundle DIR` | Start real macOS boot files instead of the test kernel |
| `--preboot-disk IMAGE` | Attach a disk image with macOS installed on it |
| `--storage-disk IMAGE` | Attach a disk image as an extra disk |
| `--append "..."` | Extra settings for `stela.efi` |
| `--memory`, `--host-cores`, `--cpu` | Choose the simulated PC's RAM, core count and CPU type |
| `--no-la57` | Pretend to be an older CPU. See [[Address Space Layout]] |
| `--gdb PORT` | Wait for a debugger (GDB) to connect |
| `--console PORT` | Let you type into the guest's text console |
| `--icount SHIFT` | Make simulated time follow work done, so slow simulation does not cause timeouts |
| `--kvm` | Use your real CPU for speed (Linux only) |

Run `cargo xtask boot run --help` for the full list.

## Testing the installer
`--payload` tests install mode with a **fake macOS download** generated on the spot, so no Apple files are needed.

## Where the results go
Each run writes a folder containing the pretend USB, the log of everything Stela printed, and a short report.

> [!info] Expect it to be slow
> By default QEMU simulates the PC's CPU in software, and Stela translates Mac code on top of that: two layers of translation. Booting real macOS this way takes tens of minutes. On Linux, `--kvm` lets QEMU use your real CPU directly, which is much faster.

Related: [[Build and Check]], [[Engine Phase]], [[Oracles]].
