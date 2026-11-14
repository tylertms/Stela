---
tags: [stela, reference, crates]
---
# Crates - CPU and Mac

## The CPU: two crates
| Crate | Lines | In plain words |
|---|---|---|
| `stela-compiler` | 206k | The translator. Mac machine code in, PC machine code out. It runs nothing |
| `stela-engine` | 240k | Runs the translated code and does everything else a Mac chip does: memory mapping, errors, timers, settings |

A useful way to remember the split: the **compiler writes** code, the **engine runs** it.

Neither crate knows anything about Apple devices or PC hardware. They are only about the CPU.

Start with [[Translator Overview]] and [[Engine Overview]].

## The Mac: four crates
| Crate | Lines | In plain words |
|---|---|---|
| `stela-soc-data` | 34k | **Facts** about Apple's devices: addresses and layouts. No behavior |
| `stela-soc` | 179k | **Behavior** of Apple's devices: what happens when macOS reads or writes them |
| `stela-loader` | 43k | Reads macOS's boot files and arranges them for startup |
| `stela-runtime` | 30k | Builds the whole pretend Mac and connects it to the real PC. Also helps macOS install itself |

"SoC" means *system on a chip*: Apple puts the CPU and most devices onto one chip.

Start with [[Machine Model]], then [[Device Models]], [[Board Mount]] and [[Boot Bundle]].

> [!info] Why split facts from behavior?
> Several tools need to know *where* a device is without needing to imitate it. For example, the code that prepares boot files must know device addresses. Keeping the facts in their own crate lets those tools stay small.

## How they depend on each other
`stela-soc` uses `stela-engine`, because device models plug into the engine. `stela-engine` uses `stela-compiler`. See [[Dependency Shape]].

Related: [[Crates - Foundations]], [[Crates - PC]].
