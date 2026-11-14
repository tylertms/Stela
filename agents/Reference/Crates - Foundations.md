---
tags: [stela, reference, crates]
---
# Crates - Foundations

A *crate* is a Rust package. These six are the bottom layer: small building blocks that the bigger crates use. None of them runs macOS code or talks to hardware.

| Crate | Lines | What it gives you |
|---|---|---|
| `stela-contracts` | 13k | Basic shared types used everywhere: the error type (`Refusal`), the description of the Apple chip, the "device bus" interface |
| `stela-formats` | 15k | Code to read Apple's file formats: disk images (APFS), signed files (Image4), compressed data (LZFSE) |
| `stela-crypto` | 2k | Hashing (SHA-1, SHA-256) and encryption (AES) |
| `stela-fingerprint` | 0.5k | Computes a hash of a crate's own source code |
| `stela-c-build` | 8k | Helps build the C code in the project |
| `stela-tls` | 3k | Secure network connections (HTTPS) without an operating system |

## Where to start
`stela-contracts` is used by almost every other crate. Three files are worth reading early:
- `crates/stela-contracts/src/refusal.rs` — the error type. See [[Refusals]].
- `crates/stela-contracts/src/chip.rs` — which Apple chip is being imitated.
- `crates/stela-contracts/src/bus.rs` — how device reads and writes are routed.

## Why does `stela-fingerprint` exist?
Stela saves translated code to disk ([[Translation Store]]). If the translator changes, old saved code may be wrong. So each saved translation is tagged with a hash of the translator's source. This crate computes that hash at build time.

## Why does Stela have its own crypto crate?
`stela.efi` is built in a restricted mode where the usual fast crypto libraries don't compile. This crate provides versions that do.

> [!info] "Without an operating system"
> Much of Stela runs with no OS underneath, so it cannot use Rust's standard library (`std`). You will see `#![no_std]` at the top of many files. See [[Conventions]].

Related: [[Crates - CPU and Mac]].
