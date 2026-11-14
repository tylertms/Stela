---
tags: [stela, tooling, cli]
---
# Setup and USB

The `stela` command is the part of the project that normal users touch. It runs on their everyday computer (Windows, macOS or Linux) and prepares the USB stick. Code: `crates/stela-cli`.

## What users type
```
sudo ./stela setup                            # guided setup, step by step
sudo ./stela setup update stela               # put a newer Stela on the USB
sudo ./stela setup boot-log --output log.txt  # copy the log off the USB
```
On Windows, users double-click `stela.exe`.

## What setup does
Described at the top of `crates/stela-cli/src/setup.rs`:
1. **Checks first.** It inspects the USB and the macOS download without writing anything. `--dry-run` stops here.
2. **Downloads macOS** from Apple, as a file called an IPSW (`restore.rs`).
3. **Prepares the boot files.** See [[Boot Bundle]].
4. **Gets approvals from Apple's servers** that a macOS install needs, ahead of time.
5. **Writes the USB.**
6. **Keeps a record** of everything written, so `uninstall` can remove exactly that.

## What ends up on the USB
| Where | What |
|---|---|
| `\EFI\BOOT\BOOTX64.EFI` | Stela itself (`stela.efi`) |
| `\EFI\stela\stela.cfg` | Settings. See [[Boot Modes]] |
| `\EFI\stela\boot.log` | The log from the last boot |
| `stela/boot` | Boot bundles |
| `stela/install` | The macOS installer data |
| `stela/store` | Saved translations. See [[Translation Store]] |

## Two installer sizes
- **Offline** — all of macOS on the USB. Needs a 32 GB stick. No network needed while installing.
- **Online** — only a list of what is needed; Stela downloads macOS during install. Needs 8 GB.

## Keeping `stela` and `stela.efi` in step
The `stela` command writes files that `stela.efi` reads. If the two come from different versions, they could misunderstand each other. So `stela.efi` carries a small version stamp, and setup refuses to install one that doesn't match (`crates/stela-loader/src/mount_interface.rs`).

> [!info] Boot logs matter
> When something fails on a user's PC, the boot log is how developers find out why. That is why log messages in Stela are so detailed.

Related: [[Restore and Install]], [[Build and Check]].
