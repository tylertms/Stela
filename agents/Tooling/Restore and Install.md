---
tags: [stela, tooling, runtime]
---
# Restore and Install

## How a real Mac gets macOS installed
When a Mac is wiped and reinstalled, it is connected to **another computer** by cable. The Mac starts a tiny temporary system, and a program inside it, `restored`, asks the other computer for each piece it needs: "send me the system image", "send me this firmware", and so on. Apple calls this a **restore**.

## What Stela does
Stela keeps that process exactly as it is. macOS's installer runs unchanged inside Stela. The difference: **Stela itself plays the other computer.**

The installer thinks it is talking over a USB cable to a helper computer. On the other end of that pretend cable is Stela's own code, answering each request from the macOS download.

Code: `crates/stela-runtime/src/restore.rs` and the `restore/` folder.

## The main files
| File | Job |
|---|---|
| `session.rs` | The conversation with `restored` |
| `stream.rs` | How messages are framed on the wire |
| `plist.rs` | Read and write Apple's "property list" data format |
| `asr.rs` | Send the big system image |
| `proxy.rs` | Let the installer reach the internet through Stela |
| `store.rs` | Read files from the macOS download and check them against known hashes |
| `answers.rs` | Decide the reply to each request |
| `ticket.rs`, `tss.rs` | Handle Apple's signed approvals |

The protocol was learned from open-source tools that restore iPhones (libimobiledevice, idevicerestore).

## Who decides what to trust
Stela only **delivers** files. Apple's own code inside macOS still checks every signature and decides whether to accept them. Stela does not bypass that.

## Testing it without a PC
```
cargo xtask restore-dry-run
```
plays the installer's side of the conversation against Stela's code using a real macOS download, and checks every answer.

> [!warning] Never commit macOS files
> macOS downloads are large and belong to Apple. `.gitignore` already blocks `*.ipsw` and `*.dmg`.

Related: [[Setup and USB]], [[Board Mount]], [[Oracles]].
