---
tags: [stela, glossary, arm]
---
# Glossary - Arm

Arm is the chip design Apple's processors are based on. These are the Arm words you will meet in the code.

| Term | Meaning |
|---|---|
| **A64** | The 64-bit Arm instruction set. Every instruction is one 32-bit word |
| **arm64e** | Apple's ABI with pointer authentication |
| **PE** | "Processing element": one CPU core |
| **EL0 / EL1 / EL2** | Exception levels: user, kernel, hypervisor |
| **VHE** | A mode that lets a normal kernel run at EL2 |
| **PSTATE** | The CPU's current status: flags, which interrupts are blocked, current level |
| **NZCV** | The four condition flags |
| **DAIF** | Four switches that block interrupts |
| **PAN** | Privileged Access Never: the kernel cannot touch user memory unless it clears this |
| **System register** | A control register accessed with `MRS` and `MSR` |
| **TTBR0 / TTBR1** | Page-table base registers for the lower and upper address halves |
| **TLBI** | Instruction that tells the CPU "forget your cached address mappings" |
| **VMSA** | Arm's name for its virtual memory system |
| **Granule** | Page size. Apple uses 16 KiB |
| **ESR / FAR / ELR / SPSR** | Saved on an exception: the reason, the bad address, where to return, the old status |
| **Data Abort** | Arm's page fault for data |
| **SVC / BRK** | System call and breakpoint instructions |
| **WFI / WFE** | Wait for interrupt / event |
| **IRQ / FIQ** | The two interrupt classes. Apple delivers timers as FIQ |

Related: [[Glossary - Apple]], [[Glossary - Stela]], [[Exceptions and System Registers]], [[Glossary - Arm Features]].
