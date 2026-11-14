---
tags: [stela, overview, engine, translator]
---
# Life of an Instruction

Let's follow one tiny piece of macOS code: `ldr x0, [x1, #8]`.

In plain words it means: "take the number in `x1`, add 8, treat that as a memory address, read what is stored there, and put it in `x0`."

## What Stela does
1. **Find the code.** Stela looks at where macOS currently is in its program (the *program counter*, or PC) and reads the instructions there.
2. **Translate, if not done before.** Stela translates a short run of instructions, called a *block*, into PC machine code. It saves the result so the same block is never translated twice. See [[Decode and Emit]].
3. **Run it.** Our instruction became a few PC instructions: add 8 to `x1`, check the address is allowed, read memory.
4. **Handle surprises.** The read may fail the first time, because Stela sets up memory lazily (only when needed). Then one of three things happens:
   - the address is normal memory → Stela connects it and retries
   - the address is a **device** (like the disk controller) → Stela asks its pretend device for the answer
   - the address is invalid → Stela reports the error to macOS, as a real Mac chip would
5. **Move on.** The block ends and tells Stela where to go next.

## Why this is fast enough
- Translating happens once per block; running happens many times.
- Blocks that follow each other get linked directly, so Stela's own code is skipped. This is called *chaining*.
- Translations are saved to the USB and reused on the next boot.

> [!info] The careful part
> If something goes wrong in the middle of a block, macOS must see every register exactly as a real chip would show it. Much of the code's complexity exists to guarantee that.

Related: [[Shadow Paging]], [[Dispatch Loop]], [[GuestState]], [[Translator Fast Paths]].
