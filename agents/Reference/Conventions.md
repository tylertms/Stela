---
tags: [stela, conventions]
---
# Conventions

The project's coding habits. Following them makes your changes fit in.

## Comments are the documentation
There is no separate docs folder. Instead, every source file starts with a `//!` comment that explains what the file does and why. When you change what a file does, update that comment too.

## Say where a fact came from
Hardware facts are easy to get wrong, so comments name their source. Examples:
- the Arm manual, with a section number
- the Intel manual
- Apple's open-source kernel (XNU), with file and line
- a specific macOS version where a behavior was seen
- "as Linux does"

If something is a guess, the comment says so.

## Errors: stop and explain
Never invent a value to keep going. Return an error that names what went wrong. See [[Refusals]].

## `unsafe` needs a reason
Rust's `unsafe` marks code the compiler cannot check. Here, every `unsafe` block needs a comment starting with `// SAFETY:` explaining why it is correct. The linter enforces this.

## No standard library in core code
Code that ends up in `stela.efi` cannot use `std`, because there is no operating system there. Use `core` and `alloc` instead (they provide `Vec`, `String`, `Box`). Files start with `#![no_std]`.

## Tests
- Tests live in the same file as the code, in a `mod tests` at the bottom.
- Test names are whole sentences, like `options_without_an_emitter_are_refused`. You can tell what broke just from the name.
- Where possible, tests try **every** possible input, not just a few.

## Words with a special meaning here
| Word | Means |
|---|---|
| refuse | Return an error with reasons |
| admit | Accept something after checking it |
| owner | The one piece of code responsible for a resource |
| mount | Attach the pretend Mac to the PC |

More in [[Glossary - Stela]].

> [!tip] Before sharing your change
> Run `cargo xtask check` ([[Build and Check]]) and write the commit message as in [[Commit Style]].

Related: [[Design Values]].
