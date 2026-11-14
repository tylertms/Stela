---
tags: [stela, engine, performance]
---
# Translation Store

**In one sentence:** Stela saves its translations to the USB so the next boot does not have to translate the same code again.

Source: `crates/stela-engine/src/translations.rs` and the `translations/` folder.

## Why
Translating takes time, and macOS runs mostly the same code on every boot. Saving the results is a big speed-up after the first boot.

## How a saved translation is found
The obvious key would be the code's address. But the same code can be loaded at a different address next time.

So Stela uses the **content**: it computes a fingerprint (a SHA-256 hash) of each page of Mac code. Same code means same fingerprint, wherever it sits.

## Making translations movable
Translated code sometimes contains addresses, like "jump to 0x5000". If the code is now somewhere else, those are wrong.

So the translator records where every such address sits in its output. When a saved translation is loaded, Stela adjusts just those spots. This is called **rebasing**.

## The file
The saved translations live in one file called the **pack**:
- an index, sorted for fast lookup
- the saved translations
- checksums, so damaged data is detected and ignored

Writing is done in small steps while macOS is idle, and finished cleanly when macOS shuts down (`translations/writer.rs`).

## When old translations become invalid
If the translator's own source code changes, old translations might be wrong. So each one is tagged with a fingerprint of the translator's source. A different translator means the old entries are ignored. The `stela-fingerprint` crate computes it.

> [!tip] A lot of recent work is here
> Many recent commits speed up this store. `git log` shows what is being improved right now.

Related: [[Dispatch Loop]], [[Crates - Foundations]], [[Setup and USB]].
