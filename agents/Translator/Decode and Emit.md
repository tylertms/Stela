---
tags: [stela, translator]
---
# Decode and Emit

Translating one instruction takes two steps. Each kind of instruction has **two files with the same name**, one per step.

| Step | Meaning | Example file |
|---|---|---|
| **Decode** | Figure out what the Mac instruction says | `crates/stela-compiler/src/aarch64/conditional_select.rs` |
| **Emit** | Write PC instructions that do the same thing | `crates/stela-compiler/src/x86_64/conditional_select.rs` |

## Step 1: decode
Every Mac instruction is one 32-bit number. Some of its bits say *which* instruction it is; the rest are details, like which registers to use.

```rust
pub fn decode(word: u32) -> Option<Operand> {
    if word & 0x3fe00800 != 0x1a800000 {
        return None;   // not this kind of instruction
    }
    Some(Operand { width, destination, left, right, condition, otherwise })
}
```

Reading it:
- `word & mask` keeps only the bits that identify the instruction.
- If they don't match, return `None`, meaning "not mine, ask another decoder".
- Otherwise pull out the details into a normal Rust struct with named fields.

After decoding, the rest of the compiler works with clear structs, not raw bits.

## Step 2: emit
The emitter takes that struct and writes PC machine code into a buffer. It uses a helper in `x86_64/asm.rs` that knows how to build the bytes of each PC instruction, so emitters say things like "move this register to that one" and never hand-write bytes.

## How it is tested
- **Decoders:** tests loop over all possible bit patterns and check the result.
- **Emitters:** tests actually run the generated code on the developer's PC and compare the result with what a Mac chip would produce.

> [!tip] Adding a new instruction
> 1. Write or extend the decoder.
> 2. Write the emitter.
> 3. Hook both into the main dispatch code.
> 4. Add tests for both.
> 5. In a comment, note where in the Arm manual the instruction is defined.

Related: [[Translator Overview]], [[Block Profiles and Exits]].
