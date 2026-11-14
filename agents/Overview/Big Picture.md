---
tags: [stela, overview]
---
# Big Picture

**In one sentence:** Stela lets a normal PC start and run macOS that was built for Apple's own chips, without changing macOS.

## Why that is hard
A modern Mac uses an Apple chip (M1, M2, ...). A PC uses an Intel or AMD chip. They differ in two big ways:

1. **They speak different machine languages.** Mac programs are *arm64* instructions. A PC only understands *x86-64* instructions.
2. **They have different hardware.** macOS expects Apple's storage chip, Apple's interrupt controller, Apple's GPU. A PC has none of them.

Stela solves both.

## What Stela actually is
Stela is a small program called `stela.efi`. The PC starts it from a USB stick, *instead of* Windows or Linux. From then on Stela controls the whole computer. It is not an app running inside another operating system.

It plays three roles at once:

| Role | In plain words | Main crates |
|---|---|---|
| A CPU | Rewrites Mac machine code into PC machine code, then runs it | `stela-compiler`, `stela-engine` |
| A Mac | Pretends to be the hardware of an M2 Mac mini, so macOS finds what it expects | `stela-soc`, `stela-soc-data`, `stela-runtime`, `stela-loader` |
| A mini operating system | Talks to the PC's real disk, network card, USB and graphics card | `stela-drivers`, `stela-driver-*`, `stela-linux` |

A fourth piece runs on *your* everyday computer: the `stela` command, which prepares the USB stick.

> [!info] Two words you will see everywhere
> **Guest** = macOS and everything inside it. **Host** = the PC and Stela itself.

> [!info] No Apple software inside
> The repository has no Apple code. The user downloads macOS from Apple, and `stela setup` helps with that.

Next: [[Life of a Boot]].
