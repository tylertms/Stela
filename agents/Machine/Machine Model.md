---
tags: [stela, machine]
---
# Machine Model

macOS expects to run on Apple hardware. Stela builds a convincing imitation of an **M2 Mac mini** in software. This note explains how the pieces fit.

## The crates involved
| Crate | Role | Note |
|---|---|---|
| `stela-soc-data` | Facts about each device: its address, the layout of its settings | [[Device Models]] |
| `stela-soc` | Behavior of each device: what happens on a read or write | [[Device Models]] |
| `stela-runtime` | Puts all devices together into one machine and connects them to the PC | [[Board Mount]] |
| `stela-loader` | Prepares the files macOS starts from | [[Boot Bundle]] |
| `stela-contracts` | Shared basic types | [[Crates - Foundations]] |

## How software talks to hardware
On most chips, each device appears as a small range of **memory addresses**. To control a device, software reads or writes those addresses. This is *memory-mapped I/O* (MMIO). The individual slots are called *registers* (not the same as CPU registers).

## How macOS finds its devices
Apple's boot software hands macOS a list of hardware called the **DeviceTree**. Each entry says "this device lives at this address". Stela places each pretend device at exactly the listed address, so macOS's own drivers find them.

## What happens on a device access
1. macOS code reads or writes a device address.
2. That address is deliberately missing from the shadow page tables, so the access is caught. See [[Shadow Paging]].
3. The engine works out which address and how many bytes, and asks the device code.
4. The pretend device answers, and macOS continues.

## The chip being imitated
`crates/stela-contracts/src/chip.rs` describes which Apple chip is imitated and what it supports. Today that is the M2 (Apple's internal name: T8112) on the Mac mini board (J473).

> [!info] Devices with their own little computer
> Many Apple devices contain a small processor running Apple firmware. Stela does not run that firmware. It imitates the *conversation* macOS has with it.

Related: [[Host and Devices Traits]], [[Glossary - Apple]].
