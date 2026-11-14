---
tags: [stela, translator]
---
# Block Profiles and Exits

Source: `crates/stela-compiler/src/block.rs` and `block/features.rs`.

## What is a block?
Stela does not translate one instruction at a time, and not a whole program either. It translates a **block**: a straight run of instructions with no jumps in the middle.

A block ends when the code:
- jumps or calls somewhere,
- asks the operating system for something, or
- does something special that Stela's engine must handle.

A block has at most 256 instructions.

## Blocks always come back
Translated code never jumps to another place by itself. When a block ends, it **returns to Stela's engine** and says "here is why I stopped, and here is where macOS wants to go next". The engine decides what to run next. (The engine can later link blocks together for speed; see [[Translator Fast Paths]].)

## Exit kinds: why a block stopped
| Kind | Meaning |
|---|---|
| `Fallthrough`, `Direct`, `Indirect`, `Call`, `Return` | Ordinary jumps, calls and returns |
| `Trap` | The code asked the operating system for help |
| `System` | A privileged instruction, such as changing a CPU setting. The engine does it |
| `PointerAuthentication` | A security check on a pointer. See [[Pointer Authentication]] |
| `FloatingPoint`, `ScalableNumerical` | Math the engine finishes in software |
| `Clock`, `ThreadRegister`, `Amx` | Other things the engine handles |

## Profiles: different kinds of blocks
The same translator is used in different situations. A **profile** says which situation.

| Profile | Used for |
|---|---|
| `SYSTEM_MODE` | Running all of macOS, kernel included. **This is the one that matters** |
| `ORDINARY` | Simple blocks for tests and tools |
| `LEAF` | One whole small function, for tests and tools |
| `CONTEXT` | Blocks using advanced vector math (SVE/SME) |

When reading the compiler, assume `SYSTEM_MODE` unless the code says otherwise.

Related: [[GuestState]], [[Dispatch Loop]].
