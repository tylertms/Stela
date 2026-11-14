---
tags: [stela, glossary, apple]
---
# Glossary - Apple

Apple-specific names. Many are abbreviations you will not find in general programming books.

| Term | Meaning |
|---|---|
| **T8112** | Apple's chip identifier for the M2 |
| **J473** | Board identifier of the M2 Mac mini |
| **26A434** | A macOS build number; evidence in the source is cited against it |
| **XNU** | The macOS kernel. Partly open source; headers are in `third_party/xnu` |
| **iBoot / LLB** | Apple's bootloader stages |
| **SPTM** | A small Apple security program that guards the page tables |
| **TXM** | A small Apple security program that decides which code may run |
| **GXF, GL1, GENTER/GEXIT** | Apple's extra protected mode, and the instructions to enter and leave it |
| **SPRR** | An Apple feature for quickly changing memory permissions |
| **CTRR** | Locks the kernel's code so it cannot be modified |
| **DeviceTree (ADT)** | The hardware description iBoot hands the kernel |
| **Kernel collection** | The kernel and its drivers packed into one file |
| **Image4, IM4P, IM4M** | Apple's signed file format: the content (P) and its signature list (M) |
| **Trust cache** | List of code hashes the kernel trusts |
| **IPSW** | A macOS restore image |
| **TSS** | Apple's signing server, which personalizes tickets |
| **ECID** | A device's unique chip ID |
| **LocalPolicy** | Per-machine signed boot policy |
| **Preboot volume** | APFS volume holding boot files of an installed system |
| **recoveryOS** | The recovery system paired with an install |
| **restored / ASR** | The restore daemon and the system-image transfer |

Related: [[Glossary - Arm]], [[Glossary - Stela]], [[Machine Model]], [[Glossary - Apple Hardware]].
