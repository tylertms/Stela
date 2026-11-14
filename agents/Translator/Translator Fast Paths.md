---
tags: [stela, translator, performance]
---
# Translator Fast Paths

A simple translator gives correct but slow code. These are the main tricks that make it fast. All are in `crates/stela-compiler/src/x86_64/`, and each file starts with a comment explaining its idea.

## Keep hot registers in real registers — `pinned.rs`
Reading and writing [[GuestState]] in memory for every instruction is slow. So the most-used Mac registers are kept in real PC registers while translated code runs. They are only written back to `GuestState` when control returns to the engine.

## Cheaper comparison flags — `flag_word.rs`
Mac and PC chips both keep "result of the last comparison" flags, but in different formats. Converting on every comparison is costly. Stela stores them in the PC's own format while running and converts only when needed.

## Link blocks together — `chain.rs`
Normally each block returns to the engine, which finds the next block. If block A always goes to block B, Stela patches A to jump straight to B. This is **chaining**. A counter makes sure control still returns to the engine regularly, so things like timers get handled.

## Handle common special cases without the engine
Some instructions normally need the engine's help. For the most common ones, Stela installs a tiny piece of code (a **thunk**) that does the job on the spot:
- reading certain CPU settings (`register_read.rs`, `mirrored_read.rs`)
- pointer security checks (`pointer.rs`)

## Skip repeated safety checks
If a block already checked that an address is safe, nearby accesses through the same register don't need the check again (`guest_range.rs`, `stack_alignment.rs`).

## Vector registers too — `vector_cache.rs`
The same idea as pinning, for vector registers, within a single block.

> [!info] The rule for every trick
> A fast path must give exactly the same result as the slow, simple path. When it can't handle a case, it falls back to the slow path. See [[Design Values]].

Related: [[Dispatch Loop]], [[GuestState]].
