<h1 align="center">
  <img src="assets/stela-icon.png" alt="" width="96"><br>
  Stela
</h1>

A translation-based Apple Silicon layer for x86-64 PCs. It is a UEFI boot
program with an arm64-to-x86-64 translator, a Metal driver for NVIDIA GPUs and
a model of Apple silicon hardware. Stela currently presents itself as an M2 Mac mini.

> [!WARNING]
Stela is still in development. It is slow and still may not boot on supported hardware. Fixes are on the way, please submit a GitHub Issue with your boot logs if you encounter an error!

## Translator

An ahead-of-time and just-in-time compiler translates arm64e code to x86-64.
It covers more than 3,000 AArch64 instruction encodings, maps guest registers
to host registers and caches translated code.

## NVIDIA driver

The Metal driver compiles AIR, Metal's shader IR, directly to NIR, Mesa's IR,
and supports the Metal3 and Apple7 GPU families.

## Hardware requirements

| Part | Requirement |
|---|---|
| CPU | Intel or AMD x86-64-v2 (Intel Nehalem, AMD Bulldozer or newer). AVX2 and AVX-512 are used when present. |
| Memory | 8 GB or more. |
| Firmware | 64-bit UEFI with Secure Boot off. |
| GPU | NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series. |
| Disk | NVMe, or SATA in AHCI mode. 60 GB or more for macOS: a GPT partition or a whole disk to erase. |
| USB stick | 8 GB for the Online installer, 32 GB for the Offline installer. |
| Ethernet | Intel I225, I226, I210, I211, I350, 82574L, 82575, 82576, 82579, 82580, I217, I218, I219.<br>Realtek RTL8111, RTL8168, RTL8169, RTL8411, RTL8125, RTL8126, Killer E2600, E3000.<br>Broadcom BCM5720, BCM5761, BCM5762.<br>Aquantia AQC100, AQC107, AQC108, AQC109, AQC111, AQC112, AQC113, AQC114, AQC115C, AQC116C.<br>Qualcomm Atheros AR8131, AR8132, AR8151, AR8152, AR8161, AR8162, AR8171, AR8172, Killer E2200, E2400, E2500.<br>USB: Realtek RTL8152, RTL8153, RTL8156, ASIX AX88179, AX88178A, AX88772, CDC ECM and NCM. |
| Wi-Fi | None currently supported. |
| Bluetooth | USB and Intel PCIe controllers. |
| Audio | None currently supported. |
| USB | xHCI (USB 3) and EHCI controllers, hubs, USB disks. |
| Input | USB keyboards and mice. PS/2 keyboards, mice and Synaptics or Elantech touchpads. |

## Install

1. Download `stela` from the
   [releases page](https://github.com/tylertms/Stela/releases).
2. Run `sudo ./stela setup` (Linux, macOS) or double-click `stela.exe`
   (Windows).
3. Choose the USB stick, the installer, the macOS version and the Mac model,
   then confirm. Setup erases the USB stick.
4. Boot the PC from the USB stick.

## Update

Download the new `stela`, plug the USB stick in and run:

```sh
sudo ./stela setup update stela
```

On Windows, open Terminal with Run as administrator and run:

```
.\stela.exe setup update stela
```

## Logs

If Stela does not work, plug the USB stick into the computer that made it and
save the boot log. Include `log.txt` in your GitHub Issue.

```sh
sudo ./stela setup boot-log --output log.txt
```

On Windows, open Terminal with Run as administrator and run:

```
.\stela.exe setup boot-log --output log.txt
```

## Build

You need [Rust](https://rustup.rs), git, Python 3.10 or newer, and LLVM 21,
22 or 23 with Clang, LLD and libclang. On macOS, Xcode's Apple clang 21 or
newer also works.

```sh
git clone https://github.com/tylertms/Stela
cd Stela
cargo xtask dist
```

This writes `target/release/stela`. The first build fetches and compiles
Linux, Mesa and LLVM sources and takes a long time.

Set `CLANG` to choose the host compiler. If cross-compilation needs a different
Clang, set `STELA_TARGET_CLANG` to its executable (for example, an unwrapped
Nix Clang); it defaults to the host compiler. xtask finds the target C++ driver
and LLVM tools alongside that compiler or via its usual fallbacks. Both
compilers must be supported, and `LIBCLANG_PATH`, when set, must match their
LLVM major version.

## Legal

Stela contains no Apple software. You are responsible for following Apple's
software license agreement and terms of use.

Stela's code is GPL-2.0-or-later. See [LICENSE](LICENSE) and [NOTICE](NOTICE).

## Community

[Discord Server](https://discord.gg/aSkwrJEMYc)
