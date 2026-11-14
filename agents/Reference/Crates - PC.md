---
tags: [stela, reference, crates, drivers]
---
# Crates - PC

These crates talk to the **real PC hardware**. A *driver* is code that knows how to operate one kind of device.

| Crate | Lines | In plain words |
|---|---|---|
| `stela-driver-core` | 2k | Small shared pieces for drivers |
| `stela-driver-base` | 136k | The basics every driver needs: finding devices, sharing memory with them, clocks |
| `stela-driver-storage` | 35k | Disk drivers: NVMe and SATA SSDs, SD cards |
| `stela-driver-net` | 278k | Network card drivers: Intel, Realtek, Broadcom and others |
| `stela-driver-audio` | 48k | Sound drivers |
| `stela-driver-gpu` | 94k | Graphics: finding the card, and Stela's Metal implementation |
| `stela-drivers` | 189k | Ties them together. Also holds USB, Bluetooth, keyboard and mouse |
| `stela-linux` | 21k | Lets Linux driver code run inside Stela. See [[Linux Environment]] |
| `stela-mesa` | 0.5k | Brings in the open-source NVIDIA graphics driver |

## Why is the network crate the biggest?
Network cards come in many models from many makers, each needing its own code. Hardware variety, not complexity of ideas, is what makes driver crates large.

## Where to start
`crates/stela-drivers/src/lib.rs`, then `probe.rs` in the same folder — the code that finds all hardware at startup. See [[Host Drivers]].

> [!warning] "The code exists" does not mean "it works for users"
> Some drivers in the tree are unfinished. The README says Wi-Fi and audio are not supported yet, even though driver code for them is present. Always treat the README's hardware table as the official list.

> [!info] `stela-mesa` is outside the workspace
> Like `stela-boot`, it is not built by a plain `cargo build`. See [[Repo Map]].

Related: [[GPU and Metal]], [[Crates - Programs]].
