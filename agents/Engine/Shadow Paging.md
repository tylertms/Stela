---
tags: [stela, engine, memory]
---
# Shadow Paging

How Stela makes memory access fast. Read [[Address Space Layout]] first.

## The problem
macOS manages memory with **page tables**: lookup tables that say "virtual address X is stored at real location Y". macOS writes them in the Mac chip's format.

The PC's CPU also uses page tables, but in its own format. It cannot read the Mac ones.

## The solution
Stela keeps a **second set of page tables in PC format** that says the same thing as macOS's tables. These are the *shadow* tables. The PC's CPU uses the shadow tables, so translated code can use macOS's addresses directly.

Think of it as keeping a translated copy of a phone book, updated whenever the original changes.

## Who does what
| Job | Where |
|---|---|
| Read macOS's (Arm-format) tables | `crates/stela-engine/src/machine/mmu.rs` |
| Track what has been copied | `crates/stela-engine/src/machine/mmu/shadow.rs` |
| Build the PC-format tables | `crates/stela-boot/src/engine/host.rs` |

## Filled only when needed
The shadow tables start empty. The first time code touches an address, the PC's CPU raises a *page fault* ("I have no entry for this"). Stela catches it, looks the address up in macOS's tables, adds the entry, and retries. Later accesses are instant.

## Several copies for different permissions
An app must not read kernel memory, but the kernel can read almost everything. So Stela keeps separate shadow tables for "running an app" and "running the kernel", and switches between them.

## Staying in sync
- When macOS changes its tables, it tells the CPU with a special instruction (TLBI). Stela sees it and removes the affected shadow entries.
- When macOS switches from one app to another, Stela swaps the app half of the tables and keeps the kernel half.
- **Devices are never added** to the shadow tables. That way every device access faults and reaches Stela's pretend devices.

Related: [[Life of an Instruction]], [[Dispatch Loop]], [[Glossary - Arm]].
