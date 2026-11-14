---
tags: [stela, machine, runtime]
---
# Board Mount

A pretend device is only useful if something real stands behind it. When macOS writes to the pretend SSD, the data must land on a real disk. **Mounting the board** means creating all the pretend devices and connecting each to real PC hardware.

Source: `crates/stela-runtime/src/machine/mount.rs`.

## What connects to what
| What macOS sees | What is really behind it |
|---|---|
| Apple SSD | One partition of a PC disk |
| Display | The PC's screen |
| USB devices: network adapter, keyboard, mouse, USB disk, Bluetooth | The PC's network card, keyboard, mouse, USB disks, Bluetooth |
| Small flash chip, security storage, clock | A reserved area on the PC's disk |
| Random numbers | Randomness collected from the PC's firmware at boot |

So the PC's network card is shown to macOS as a USB network adapter. macOS uses its normal USB network driver and never knows the difference.

## Machine state
A real Mac stores some things outside its main SSD: startup settings (NVRAM), security data, and the clock. Stela keeps them in a **hidden area at the end of the chosen partition**. macOS is shown a slightly smaller disk, so it can never overwrite that area.

Code: `crates/stela-runtime/src/platform/pc/machine_state.rs`.

## Choosing the right disk, safely
Stela never guesses which disk to use. The settings file names the disk by its **serial number** and the partition by its **unique ID**. Stela then hands macOS an object that can only reach that one partition.

Code: `crates/stela-runtime/src/platform/pc/storage.rs`.

> [!warning] Be extra careful here
> A bug in this area could overwrite a user's Windows or Linux partition. Review changes near disk selection twice.

## Also in this crate
- `platform/net/` — networking for Stela itself (to download macOS during install)
- `restore/` — see [[Restore and Install]]

Related: [[Device Models]], [[Host Drivers]], [[Engine Phase]].
