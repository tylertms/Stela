---
tags: [stela, engine, arm]
---
# Pointer Authentication

A security feature of Apple chips that Stela must imitate. Source: `crates/stela-engine/src/pauth/`.

## What it is
A *pointer* is a memory address stored in a variable. A common attack is to overwrite a pointer so the program jumps somewhere harmful.

Apple chips defend against this. Before storing an important pointer, the CPU adds a small **signature** into its unused upper bits. Before using the pointer, the CPU checks the signature. If someone changed the pointer, the check fails and the program stops.

The signature is called a **PAC** (pointer authentication code). It is calculated from the pointer, a secret key, and some context. macOS uses it everywhere, for example on every function return.

## Why Stela has work to do
PC chips have no such feature. So Stela calculates and checks the signatures in software.

## Which math is used
| Method | File | When |
|---|---|---|
| AES-based | `keyed.rs` | On PCs with AES instructions (nearly all) — fast |
| QARMA | `qarma.rs` | The algorithm from the Arm manual; used as a fallback |

Is it OK to use a different formula than a real Mac? Yes. Arm lets each chip maker choose, and software never depends on the exact formula. It only needs "sign, then check" to be consistent.

## When a check fails
The pointer is deliberately turned into an invalid address, so using it crashes — the same as on a real chip.

## Making it fast
Signing happens at the start and end of almost every function, so it must be cheap. Stela:
- **remembers recent results** in a small table (a *memo*)
- uses a small piece of code (a *thunk*) to look results up without calling the engine

Code: `crates/stela-compiler/src/x86_64/pointer.rs`.

> [!info] Keys are not baked in
> The secret keys are chosen while running. Saved translations contain no keys, so they stay valid across boots.

Related: [[Translator Fast Paths]], [[Glossary - Arm]].
