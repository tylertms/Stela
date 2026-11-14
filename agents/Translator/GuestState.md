---
tags: [stela, translator, engine]
---
# GuestState

Source: `crates/stela-compiler/src/state.rs`.

## The problem
A CPU has *registers*: a small set of very fast storage slots it calculates with. A Mac chip has registers named `x0` to `x30`, `sp`, and others. A PC chip has different registers with different names.

So where do the Mac's register values live while PC code is running?

## The answer
In a struct called `GuestState`. It is a plain block of memory with one field per Mac register. Translated code receives a pointer to it and reads and writes the fields.

| Field | What it holds |
|---|---|
| `x` | The 31 general registers, `x0` to `x30` |
| `sp` | The stack pointer |
| `nzcv` | Four flags set by comparisons (negative, zero, carry, overflow) |
| `v` | The 32 vector registers, used for floating point and bulk math |
| `fpcr`, `fpsr` | Floating-point settings and status |
| others | Bookkeeping that helps translated code run safely |

So `add x0, x1, x2` becomes roughly: load field `x[1]`, add field `x[2]`, store into field `x[0]`.

## The layout must never change by accident
Translated code contains fixed positions like "the value 8 bytes from the start". If a field moved, all previously translated code would read the wrong thing.

To prevent this, a test checks the exact size (904 bytes) and the position of each field.

> [!warning] If you change this struct
> That test will fail on purpose. Changing the layout also makes every saved translation useless, so it is a big decision. See [[Translation Store]].

## What is *not* in here
The program counter and most CPU settings are kept by the engine, in `crates/stela-engine/src/machine/cpu.rs`.

> [!info] Anchors
> One translated block can be run by several CPU cores. So it must not contain addresses that belong to one core. Instead it looks them up through a small table in `GuestState` called `anchors`.

Related: [[Block Profiles and Exits]], [[Translator Fast Paths]].
