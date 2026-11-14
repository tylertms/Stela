---
tags: [stela, machine, devices]
---
# Device Models

A **device model** is Rust code that behaves like one piece of Apple hardware. They are in `crates/stela-soc/src/`, one file or folder per device.

## What a model looks like
At its core, each model answers two questions:
- "macOS **read** the register at this offset. What value comes back?"
- "macOS **wrote** this value to that offset. What should change?"

A model also keeps internal state, and can raise an **interrupt** (a signal that tells the CPU "I need attention").

## The rules every model follows
1. It sits at the address the DeviceTree gives it.
2. **An unimplemented register is an error, never a silent zero.** See [[Refusals]].
3. Comments say which behavior comes from documentation and which is an educated guess.

## Some models
| File | Imitates |
|---|---|
| `aic` | The interrupt controller: collects interrupt signals and passes them to CPU cores |
| `pmgr` | The power manager: switches parts of the chip on and off |
| `dart` | Memory protection for devices |
| `ans` | The storage (SSD) controller |
| `smc` | The controller for fans, temperature and power buttons |
| `sep` | The Secure Enclave, Apple's security processor |
| `dcp`, `display` | The display controller |
| `dwc3`, `usb/` | USB |
| `gpio`, `i2c`, `spi`, `nor` | Simple pins, small buses and a tiny flash chip |
| `gpu_gate` | Stela's own graphics doorway. See [[Metal Call Path]] |

## Facts are kept separately
For each model, `crates/stela-soc-data/src/<device>.rs` holds the plain facts (register positions and sizes) with no behavior. This lets other tools use the facts without pulling in the behavior.

## Putting them on one bus
`crates/stela-soc/src/machine.rs` owns all the models. Given an address, it finds the right model and forwards the access. It makes no decisions itself.

> [!tip] How to read a model
> 1. Read the comment at the top of the file.
> 2. Find its `read` and `write` functions.
> 3. Read its tests. They show the sequence macOS really performs.

Related: [[Machine Model]], [[Board Mount]].
