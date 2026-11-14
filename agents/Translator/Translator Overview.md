---
tags: [stela, translator]
---
# Translator Overview

**In one sentence:** `crates/stela-compiler` takes Mac machine code (arm64) as bytes and returns PC machine code (x86-64) as bytes.

## A translator, not an interpreter
There are two common ways to run code built for another chip:

- An **interpreter** reads one instruction, acts it out, reads the next. Simple, but slow.
- A **translator** rewrites the code into the other chip's language once, then runs the result directly. Harder, but fast.

Stela only translates. If it meets an instruction it cannot translate, it reports an error. There is no slow backup.

## It only produces bytes
The compiler does not run anything and does not touch memory. It is like a function `translate(bytes) -> bytes`. Running the result is the engine's job ([[Engine Overview]]). This makes the compiler easy to test.

## The main function
```rust
pub fn compile(source: &[u8], base: GuestAddress, options: &Options)
    -> Result<block::CompiledBlock, CompileError>
```
- `source` — the Mac code bytes
- `base` — the address those bytes live at inside macOS
- `options` — which kind of block to make, and which PC CPU features may be used
- result — the PC code, plus a table saying which Mac instruction each part came from

## Folder guide
| Folder | Job |
|---|---|
| `aarch64/` | Understand a Mac instruction. See [[Decode and Emit]] |
| `x86_64/` | Write the PC instructions |
| `block/` | Decide where a block starts and ends. See [[Block Profiles and Exits]] |
| `state.rs` | Where Mac register values are kept. See [[GuestState]] |
| `target.rs` | Which PC CPU features are available |
| `amx/`, `wkdm/` | Two special instruction sets only Apple chips have |

> [!info] Older and newer PCs
> A newer PC CPU has extra instructions (AVX2, AVX-512) that can do some jobs faster. `TargetCpu` tells the compiler what it may use. The minimum is a level called x86-64-v2 (roughly CPUs from 2009 on).

Related: [[Translator Fast Paths]].
