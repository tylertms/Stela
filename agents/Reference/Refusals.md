---
tags: [stela, conventions, errors]
---
# Refusals

A **refusal** is Stela's name for an error. The type is `Refusal`, in `crates/stela-contracts/src/refusal.rs`.

## What makes it different from a plain error string
A refusal always carries:
1. a short description of **what** was refused,
2. the **named values** that caused it,
3. optionally, the lower-level refusal that **led** to it.

Compare:
- Unhelpful: `invalid register`
- A refusal: `register 0x48 is not implemented (device=gpio, width=4)`

The second one tells a developer exactly where to look.

## How to create one
```rust
use stela_contracts::{refuse, require};

// Return an error:
return Err(refuse!("register {offset:#x} is not implemented", offset));

// Check a condition, and return an error if it is false:
require!(bytes <= LIMIT, "transfer exceeds the limit", bytes, limit = LIMIT);
```

What is going on:
- `refuse!` builds the error. Variables you list are attached with their names.
- `{offset:#x}` puts a value into the message, in hexadecimal.
- Values not mentioned in the message are added at the end, like `(bytes=0x2000, limit=0x1000)`.
- `require!` is a shortcut for "if not this, return a refusal".
- `.because(other)` wraps another refusal as the cause.

## A common shortcut
Many crates define their own `Result` so functions don't have to spell out the error type:
```rust
pub type Result<T, E = Refusal> = core::result::Result<T, E>;
```

## Why the project cares so much
Stela runs on users' PCs, where no developer is watching. When it fails, the user sends a **boot log**. A good refusal in that log can pinpoint the problem without anyone needing the user's exact hardware.

> [!warning] Things to avoid
> - Returning 0 or a default for something unknown
> - Using `unwrap()` or `panic!` on data that comes from macOS. A panic stops the whole machine
> - Writing an error message without the values that explain it

Related: [[Design Values]], [[Conventions]], [[Device Models]].
