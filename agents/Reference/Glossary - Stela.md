---
tags: [stela, glossary]
---
# Glossary - Stela

Words this project uses in its own way.

| Term | Meaning |
|---|---|
| **The mount** | `stela.efi`, which mounts an Apple board on a PC |
| **Guest / host** | The Arm macOS side / the x86 PC side |
| **Block** | A translated run of guest instructions. See [[Block Profiles and Exits]] |
| **Leaf** | A whole function translated to return at `RET` |
| **System mode** | Full-system execution with MMU and exception levels |
| **Chaining** | Patching a block's exit to jump into its successor |
| **Thunk** | Small native code installed over an exit to complete it without the engine |
| **Pinned** | A guest register kept in a host register across blocks |
| **Anchor / Place** | A per-PE pointer slot, and an offset from it |
| **Placement** | How guest and Stela share the x86 address space |
| **Stela window** | The part of the address space Stela keeps for itself |
| **Physmap** | A direct mapping of host physical memory in the window |
| **Shadow** | x86 page tables mirroring the guest's Arm tables |
| **Translated variant** | Slower code using a software TLB |
| **Pack / store** | The persistent translation cache |
| **Boot bundle** | Pre-laid-out boot images. See [[Boot Bundle]] |
| **Handoff** | Plain data the loader passes to the engine or drivers |
| **Machine state** | NVRAM, SEP storage and log kept outside the guest's view |
| **Refuse / admit** | Reject with reasons / accept after checks. See [[Refusals]] |
| **Plain read** | A register read with no side effect |
| **Gate** | StelaGPU's MMIO call window. See [[Metal Call Path]] |
| **Oracle** | An independent reference to test against. See [[Oracles]] |

Related: [[Glossary - PC]], [[Glossary - Arm]], [[Glossary - Apple]].
