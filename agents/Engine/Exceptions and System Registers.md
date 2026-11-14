---
tags: [stela, engine, arm]
---
# Exceptions and System Registers

Two CPU features that the engine implements in Rust.

## Privilege levels
A CPU runs code at different trust levels, so an app cannot break the system. Arm calls them **exception levels**:

| Level | Who runs there |
|---|---|
| EL0 | Apps |
| EL1 | Normally the kernel |
| EL2 | A level above; on Apple chips the macOS kernel runs here |

Stela implements these three (`crates/stela-engine/src/machine/cpu.rs`).

## Exceptions
An **exception** is when the CPU stops what it is doing and jumps to the kernel. Causes include: an app asks for a service, code touches invalid memory, a timer fires.

The CPU saves where it was and why, switches to a higher level, and jumps to a kernel handler. Stela does exactly the same steps, following the Arm manual (`machine/exception.rs`).

## System registers
**System registers** are the CPU's settings and status values: where the page tables are, which interrupts are allowed, and hundreds more. Code reads them with `MRS` and writes them with `MSR`.

There are too many to write by hand, so the code for the standard ones is **generated** from Arm's official machine-readable specification, by `cargo xtask sysregs`. The output is `machine/system/generated.rs`.

> [!warning] Never edit `generated.rs` by hand
> Your change would be lost the next time it is generated. See [[Dev Commands]].

## Apple's extra registers
Apple added many of its own. No public manual covers them, so each was worked out by studying what macOS does with it.
- `registers.rs`, `impdef.rs` — the list, with a note on what uses each one
- An unknown register is refused, not guessed. See [[Refusals]].

## "What CPU are you?"
macOS asks the CPU which features it supports. `identification.rs` answers honestly: only features Stela really implements, and never more than a real M2 has.

Related: [[Glossary - Arm]], [[Glossary - Apple]], [[Engine Overview]].
