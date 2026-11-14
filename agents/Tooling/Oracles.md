---
tags: [stela, tooling, testing]
---
# Oracles

## The idea
A normal test checks "does the code do what I *think* it should?". But what if your understanding is wrong? An emulator can pass all its own tests and still behave differently from a real Mac.

An **oracle** is a trusted outside source of the right answer. Oracle tests compare Stela against something independent. Stela has several.

## Compare against a real Mac
`cargo xtask cpu-reference` runs the same small pieces of Arm code twice:
1. on a **real Apple-chip Mac** (over the network),
2. through Stela.

Then it compares every register. Any difference is a bug in Stela.

The Mac's answers can be saved to a file, so later runs don't need a Mac (`xtask/src/cpu_reference/apple-m1-pro.tsv`).

## Compare against a trusted emulator
Apple chips have secret matrix-math instructions called **AMX**, with no documentation. A researcher built an emulator of them and verified it on real hardware. `cargo xtask amx-reference` checks Stela's version against that one, step by step.

The chain of trust: Stela's fast code → Stela's simple model → the researcher's emulator → real hardware.

## Tests that need Apple files
Some tests need files that cannot be in the repository. They are marked `#[ignore]` so a normal test run skips them.

`cargo xtask oracles` finds which of those files you have, runs the matching tests, and writes a report. It can also just list what is missing.

## Replay the installer
`cargo xtask restore-dry-run` checks the install helper against a real macOS download. See [[Restore and Install]].

## Smaller cross-checks inside the code
- Stela's fast crypto code is tested against a well-known Rust crypto library.
- Saved translations are re-checked by translating the same code at a different address.

> [!tip] When you add a fast path
> Add a test that compares it with the slow, simple version on many inputs. That is the oracle habit in miniature. See [[Design Values]].

Related: [[Build and Check]], [[Dev Commands]].
