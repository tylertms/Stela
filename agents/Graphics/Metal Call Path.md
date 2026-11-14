---
tags: [stela, gpu, metal]
---
# Metal Call Path

How one graphics request travels from a Mac app to Stela.

## The goal: be quick
Apps make thousands of graphics calls per second. Normally, getting from an app to hardware goes through the operating system kernel each time, which is slow. Stela gives each app a **direct doorway**, called the **gate**.

## Step by step
**1. Setup, once per app.** Stela adds a device named `stela-gpu` to macOS's hardware list. Stela's kernel part (the kext) finds it and gives each graphics-using app its own small window of memory connected to it. Each app gets a separate window, so apps cannot interfere with each other.

**2. The app makes a request.** Stela's plugin (`guest/stela-gpu/bundle/Gate.m`) writes down the request in the app's own memory: what to do and with what data. Then it writes the address of that note to a special spot in its window called `CALL`.

**3. Stela notices.** That write is a device access, so it is caught and passed to the pretend device in `crates/stela-soc/src/gpu_gate.rs`. See [[Machine Model]].

**4. Stela does the work.** The request is handled by `crates/stela-driver-gpu/src/metal/service.rs`, which fills in the answer in the app's note. Then the app continues.

## Safety rules
- **Handles, not pointers.** The app refers to its textures and buffers by ID numbers. A wrong, old or foreign ID is refused.
- **Stela never trusts app addresses.** Data is copied in and out through a checked helper.
- **Cleanup.** When an app closes, everything it created is released.
- **No blocking.** A slow job, like preparing a shader, does not freeze other CPU cores.
- **No waiting on the card.** When work finishes, the device raises an interrupt and the waiting app wakes up.

> [!warning] Two sides of one agreement
> The request format is defined on the macOS side in `guest/stela-gpu/include/StelaGPU/Interface.h` and on the Stela side in the service's `wire` module. Change both together.

Related: [[GPU and Metal]], [[Shader Compilation]], [[Device Models]].
