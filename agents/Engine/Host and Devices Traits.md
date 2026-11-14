---
tags: [stela, engine, architecture]
---
# Host and Devices Traits

The engine does not know what computer it runs on, or what devices macOS expects. It asks for those through two Rust *traits* (interfaces). Both are in `crates/stela-engine/src/machine/engine/boundary.rs`.

```rust
pub struct Machine<H: Host, D: Devices> { pub cpu: Cpu, pub host: H, pub devices: D, /* ... */ }
```

Read this as: "a `Machine` needs *some* `Host` and *some* `Devices`; it doesn't care which".

## `Host`: the real computer underneath
Things the engine needs from the PC:

| Need | Example methods |
|---|---|
| Read and write macOS's memory | `read_ram`, `write_ram` |
| A place to put translated code | `install`, `patch` |
| Run a translated block | `execute` |
| Keep the shadow page tables right | `set_regime` |

The real implementation is `crates/stela-boot/src/engine/host.rs`. Tests use simpler ones that run inside a normal program.

## `Devices`: the pretend Mac hardware
When macOS reads or writes a device, the engine calls:

```rust
fn read(&mut self, cpu: usize, physical: u64, bytes: u8, now: u64) -> Result<u64, DeviceError>;
```

In words: "core number `cpu` reads `bytes` bytes at address `physical`, at time `now`. What is the answer?"

The real implementation is `crates/stela-soc/src/machine.rs`, which forwards to the right device. Tests can use `NoDevices`.

## Why design it this way
- **Testing.** The engine can be tested on any laptop, with no special hardware.
- **Clean separation.** The engine is about the CPU. Apple device knowledge lives in `stela-soc`. PC knowledge lives in `stela-boot`.

> [!tip] A rule of thumb
> If you are about to add PC-specific or Apple-device-specific code inside `stela-engine`, stop. It probably belongs behind one of these traits.

Related: [[Dependency Shape]], [[Device Models]], [[Engine Phase]].
