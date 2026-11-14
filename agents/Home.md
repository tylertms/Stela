---
tags: [stela, moc]
aliases: [Stela MOC, Index]
---
# Stela learning vault

A guide to the Stela codebase (version 0.3.1, commit `124ec854b`), written for someone new to it. You do not need to know emulators, Arm or operating-system internals to start. Each note explains its terms or links to a glossary.

> [!info] How to read
> - Source paths look like `this`, relative to the repository root.
> - Almost every source file starts with a `//!` comment that explains it. These notes point you to those comments.
> - If a note and the code disagree, the code is right.

## Start here, in order
1. [[Big Picture]] — what Stela is
2. [[Life of a Boot]] — what happens when you turn the PC on
3. [[Life of an Instruction]] — how one line of Mac code runs on a PC
4. [[Design Values]] — the habits behind the code
5. [[Repo Map]] — what is in each folder

## By area
- **Boot** — [[Boot Modes]], [[Loader Phase]], [[Address Space Layout]], [[Engine Phase]]
- **Translator** — [[Translator Overview]], [[Decode and Emit]], [[Block Profiles and Exits]], [[GuestState]], [[Translator Fast Paths]]
- **Engine** — [[Engine Overview]], [[Dispatch Loop]], [[Shadow Paging]], [[Exceptions and System Registers]], [[Pointer Authentication]], [[Parallel Execution]], [[Translation Store]], [[Host and Devices Traits]]
- **The Mac** — [[Machine Model]], [[Device Models]], [[Board Mount]], [[Boot Bundle]]
- **The PC** — [[Host Drivers]], [[Linux Environment]]
- **Graphics** — [[GPU and Metal]], [[Metal Call Path]], [[Shader Compilation]]
- **User tooling** — [[Setup and USB]], [[Restore and Install]]
- **Developer tooling** — [[Build and Check]], [[QEMU Runs]], [[Dev Commands]], [[Oracles]]

## Reference
- [[Crates - Foundations]], [[Crates - CPU and Mac]], [[Crates - PC]], [[Crates - Programs]], [[Dependency Shape]]
- [[Conventions]], [[Refusals]], [[Commit Style]]
- [[Glossary - Arm]], [[Glossary - Arm Features]], [[Glossary - Apple]], [[Glossary - Apple Hardware]], [[Glossary - Stela]], [[Glossary - PC]]
- [[Where to Look]]

> [!tip] Stuck on a word?
> Check the glossaries first. Most unfamiliar terms are there.

> [!warning] Limits of these notes
> Nothing was built or run while writing them. Commands are copied from the source that defines them.
