---
tags: [stela, drivers]
---
# Host Drivers

A **driver** is code that knows how to talk to one kind of hardware. Once Stela takes over the PC, the firmware's drivers are gone, and there is no Windows or Linux. So Stela brings its own. Entry point: `crates/stela-drivers/src/lib.rs`.

## Two sets of drivers — don't mix them up
- **macOS's drivers** run inside macOS and talk to the *pretend* Apple devices.
- **Stela's drivers** talk to the *real* PC hardware.

The pretend devices sit between them. See [[Board Mount]].

## How the driver code is organized
| Layer | Crate | Contains |
|---|---|---|
| Basics | `stela-driver-base` | Finding devices on the PCI bus, shared helpers, clocks |
| One crate per kind | `stela-driver-storage`, `-net`, `-audio`, `-gpu` | Disk, network, sound and graphics drivers |
| Top level | `stela-drivers` | USB, Bluetooth, keyboard and mouse; plus the startup scan |

## The startup scan
`crates/stela-drivers/src/probe.rs` walks through every device plugged into the PC, looks at what kind it is, and hands it to the matching driver. It collects a list of what worked and what was refused.

## Common interfaces
Drivers for similar hardware share one interface, so the rest of Stela does not care which brand is installed:
- `BlockDevice` — any disk: "read these blocks, write those blocks"
- `Link` — any network card: "send this packet, receive a packet"

## Where the knowledge comes from
Hardware makers rarely publish full manuals. The drivers follow what Linux's open-source drivers do. You will see "as Linux does" in many comments.

> [!warning] Code existing is not the same as "supported"
> The tree contains Wi-Fi and audio driver code, but the README says Wi-Fi and audio are not supported yet. The README's hardware table is the official list. See [[Crates - PC]].

Related: [[Linux Environment]], [[Engine Phase]].
