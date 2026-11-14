---
tags: [stela, tooling, build]
---
# Build and Check

## What you need installed
- **Rust.** The exact version (1.99.1) is chosen automatically from `rust-toolchain.toml`.
- **git**
- **Python 3.10 or newer**
- **LLVM 21, 22 or 23** with Clang, LLD and libclang. Some parts of Stela are C code.

A missing tool is no mystery: the build first runs a check that lists everything missing in one message (`xtask/src/preflight.rs`).

## What is `cargo xtask`?
A common Rust pattern. Instead of shell scripts, helper tasks are written in Rust in the `xtask/` folder and run with `cargo xtask <task>`.

## The main commands
| Command | What it does |
|---|---|
| `cargo xtask check` | Runs **all** checks: formatting, lints, tests. Run before sharing work |
| `cargo xtask check --fix` | Fixes formatting first, then checks |
| `cargo xtask check --low-memory` | Same, one step at a time, for machines with little RAM |
| `cargo xtask dist` | Builds the final `stela` program, at `target/release/stela` |
| `cargo xtask boot build` | Builds only `stela.efi` |
| `cargo xtask gpu fetch` | Downloads the third-party sources |

> [!warning] The first build takes a long time
> It downloads and compiles Linux, Mesa and LLVM. Later builds are much faster.

## Faster loops while you work
Check just the crate you are changing:
```
cargo test -p stela-compiler
cargo test -p stela-boot-core
cargo clippy -p stela-soc -- -D warnings
```

`stela-boot` is special (see [[Repo Map]]); build it with `cargo xtask boot build`.

## What "check" includes
- **Formatting** with rustfmt
- **Clippy**, Rust's linter. Warnings count as errors.
- Lints for several systems, since Stela ships for Linux, Windows and macOS
- **Tests**

## Two rules the compiler enforces
1. Every `unsafe` block needs a `// SAFETY:` comment explaining why it is correct.
2. Inside an `unsafe fn`, unsafe operations still need their own `unsafe` block.

## Automatic checks on GitHub
`.github/workflows/ci.yml` runs a lighter set: formatting everywhere, plus lints and tests for the `stela` command on three systems. The full check is run locally.

Related: [[QEMU Runs]], [[Dev Commands]], [[Conventions]].
