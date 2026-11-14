---
tags: [stela, boot]
---
# Boot Modes

`stela.efi` is the program the PC starts from the USB. Its code is in `crates/stela-boot`.

> [!info] A name you will see
> The code calls `stela.efi` **"the mount"**, because it *mounts* (attaches) a pretend Apple board onto a PC.

## How it decides what to do
When it starts, `main` in `crates/stela-boot/src/main.rs` reads a one-line settings file from the USB: `\EFI\stela\stela.cfg`. The first word of that line picks the mode.

| The line starts with | What happens |
|---|---|
| `--system` | Run macOS. This is the normal mode |
| `--host-check` | Don't run macOS. Just test the PC and print a report (special test builds only) |
| anything else | Install-related work or a disk check, with no Mac code running |

## The startup menu
If a second file `choices.cfg` exists, Stela first shows a menu on screen, for example "macOS" and "Install macOS". Each menu entry is just a name plus a full settings line. The menu code is in `choice.rs`.

## One definition of the settings
The settings line is written by `stela setup` on your computer and read by `stela.efi` on the PC. Both use **the same code** for it: `crates/stela-boot-core/src/options.rs`. That way the writer and the reader can never disagree about spelling or meaning.

> [!warning] Settings are a promise between two programs
> If you rename or add an option, the USB written by an old `stela` no longer matches a new `stela.efi`. There is a version number for this (`OPTIONS_REVISION` in `crates/stela-loader/src/mount_interface.rs`). Setup refuses to install a `stela.efi` with a different version. Bump it when you change options.

Next: [[Loader Phase]]. Related: [[Setup and USB]].
