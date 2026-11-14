---
tags: [stela, tooling]
---
# Dev Commands

Two sets of developer commands: `stela-dev` for experiments, and `cargo xtask` for project chores.

## `stela-dev`
A tool for poking at individual parts of Stela. Run it with:
```
cargo run -p stela-dev -- <command>
```
Code: `crates/stela-dev/src/main.rs`.

| Command | What it does |
|---|---|
| `cpu-info` | Shows which CPU features your computer has |
| `compile-block INPUT` | Translates a file of Mac machine code and shows the result |
| `run-blocks INPUT` | Translates **and runs** a file of Mac machine code |
| `compile-throughput` | Measures how fast the translator is |
| `plan-executable` | Inspects a Mac program file |
| `images`, `image-manifest`, `extract-image` | Find and unpack macOS downloads |
| `decode-im4p`, `decrypt-aea` | Open Apple's special file formats |
| `apfs` | Look inside an Apple disk image |
| `boot` | Build a boot bundle by hand |

> [!tip] The best first experiment
> Write three or four Arm instructions, assemble them into a raw file, and run `compile-block` on it. Seeing real input and output makes [[Decode and Emit]] click.

## More `cargo xtask` tasks
Besides the ones in [[Build and Check]] and [[QEMU Runs]]:

| Task | What it does |
|---|---|
| `sysregs --aarchmrs DIR` | Regenerates the CPU-settings code from Arm's official data files. See [[Exceptions and System Registers]] |
| `guest` | Builds the graphics parts that run inside macOS. Works without a Mac |
| `app` | Builds Stela.app. This one does need a Mac |
| `gpu compilers` | Builds and tests the shader translator. See [[Shader Compilation]] |
| `gpu nvk` | Builds the NVIDIA driver parts |
| `oracles`, `cpu-reference`, `amx-reference`, `restore-dry-run` | Compare Stela against trusted references. See [[Oracles]] |

## Finding a command's options
Every command explains itself:
```
cargo xtask --help
cargo xtask boot run --help
cargo run -p stela-dev -- --help
```

Related: [[Build and Check]].
