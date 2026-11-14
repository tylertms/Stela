---
tags: [stela, engine, smp]
---
# Parallel Execution

A Mac has several CPU cores, and so does a PC. Stela runs **each pretend Mac core on its own real PC core**, all at the same time. Source: `crates/stela-engine/src/machine/engine/parallel.rs`.

## What the cores share
- **Memory.** All cores see the same macOS memory.
- **Devices.** There is one set of pretend devices, protected by a lock so two cores don't change one device at the same moment.
- **A message hub.** Cores use it to tell each other things.

Each core has its own engine instance (`Machine`) and its own registers.

## Messages between cores
Sometimes one core must tell the others something. The main example: macOS changes its page tables on one core, and every core must forget its old copy of that mapping.

The sending core waits until all *running* cores confirm. A sleeping core applies the change when it wakes. This matches real hardware.

## Atomic operations
Programs with several threads use special instructions to update shared memory safely. Arm does it as a pair: "read and remember" then "write only if nobody changed it". PCs have one instruction, compare-and-swap, that does both. Stela translates the pair into that.

> [!warning] One small known difference
> If another core writes the *same value* back in between, Stela does not notice. The code comment says this is accepted, because real software already tolerates it.

## Memory ordering
When cores share memory, chips differ in how strictly they keep reads and writes in order. PCs are stricter than Macs. That is good news: code written for the looser Mac rules runs correctly under stricter PC rules.

## Starting the PC's cores
`crates/stela-boot/src/native_smp.rs` wakes the PC's other cores and gives each its own stack and data.

Related: [[Dispatch Loop]], [[Host and Devices Traits]], [[Engine Phase]].
