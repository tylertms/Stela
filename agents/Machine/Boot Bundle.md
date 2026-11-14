---
tags: [stela, machine, loader]
---
# Boot Bundle

## Background: how a Mac starts
On a real Mac, a small Apple program called **iBoot** (the bootloader) runs first. It:
1. loads the macOS kernel into memory,
2. prepares the hardware list (the DeviceTree),
3. prepares startup settings (the boot arguments),
4. jumps to the kernel.

## What Stela does instead
Stela has no iBoot. It does iBoot's job itself, and does most of it **in advance**. The result is the **boot bundle**: a package containing everything already laid out as it must appear in memory.

At boot, `stela.efi` only has to:
1. check the package is undamaged (every part has a checksum),
2. copy the parts into macOS's memory,
3. fill in a few values only known at run time,
4. jump to the starting point.

Source: `crates/stela-loader/src/boot_bundle.rs` (building) and `crates/stela-formats/src/boot_bundle.rs` (the file format).

## What is inside
- the macOS kernel and its built-in drivers
- two small Apple security programs that run alongside the kernel (SPTM and TXM)
- the DeviceTree
- the boot arguments
- lists of trusted code

All of these come from the user's macOS download. Stela arranges them; it does not modify them.

## Who builds it, and when
| When | Who |
|---|---|
| While preparing the USB | `stela setup` on your computer |
| At boot, for an installed macOS | `stela.efi`, reading the installed system's boot files from disk |

Both use the same code in `stela-loader`. That is why this crate must work without the standard library.

## Helpful files in `stela-loader`
| File | Job |
|---|---|
| `apple_device_tree.rs` | Read and edit the DeviceTree |
| `kernel_collection.rs` | Read the kernel package |
| `im4p.rs`, `im4m.rs` | Read Apple's signed file format |
| `boot_args.rs` | Build the boot arguments |
| `stela_gpu.rs` | Add Stela's graphics device to the DeviceTree |

Related: [[Setup and USB]], [[Machine Model]], [[Glossary - Apple]].
