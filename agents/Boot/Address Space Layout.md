---
tags: [stela, boot, engine, memory]
---
# Address Space Layout

This is the most important design decision in the project. Take your time with it.

## Background: virtual addresses
Programs do not use real memory locations. They use *virtual addresses*, and the CPU translates them through page tables. Every program can use the same virtual addresses without clashing.

## Stela's trick
When macOS code reads address `0x1000`, the translated PC code also reads address `0x1000`. Stela does not convert addresses in software. It arranges the PC's page tables so macOS's addresses point at the right memory.

This is fast: one Mac memory read becomes one PC memory read.

## The catch
Stela's own code and data also need addresses. So macOS and Stela must **share one address space** without overlapping. Think of one long street where macOS owns most house numbers and Stela needs a few for itself.

## How the street is divided
On CPUs with LA57 (a very large address space):

| Region | Owner |
|---|---|
| Bottom | macOS apps (the "lower half") |
| A slice in the middle | **The Stela window**: Stela's code, stack, translated code, and a view of all real memory |
| Top | The macOS kernel (the "upper half") |

There is plenty of room, so nothing collides.

On older CPUs with four-level paging the space is smaller. Stela must take a piece of the area the macOS kernel might use. Code that touches those addresses uses a slower fallback that looks addresses up in software (`mmu/translated.rs`).

> [!warning] The safety rule
> macOS must never read or write the Stela window. So every translated memory access first checks "is this address in macOS's range?". If not, the access is forced to fail safely.

Sources: `crates/stela-engine/src/machine/mmu/placement.rs`, `crates/stela-boot/src/layout.rs`.

Related: [[Shadow Paging]], [[Loader Phase]], [[Glossary - PC]].
