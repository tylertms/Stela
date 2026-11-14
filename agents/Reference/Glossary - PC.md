---
tags: [stela, glossary, x86]
---
# Glossary - PC

PC-side words: the x86 CPU, the PC's firmware, and the graphics software.

| Term | Meaning |
|---|---|
| **UEFI, ExitBootServices (EBS)** | The PC's built-in startup software, and the call that tells it "I take over now" |
| **LA57** | Five-level paging, 57-bit addresses |
| **PML4 / PML5** | Top-level page tables |
| **SMAP, `RFLAGS.AC`** | A protection that stops kernel code touching app memory, and the flag that pauses it |
| **TSC, APIC, MSI, HPET** | A CPU tick counter, the PC's interrupt controller, a way devices send interrupts, a timer chip |
| **PCI, ECAM, ACPI** | The bus PC devices plug into, how to read their settings, and the firmware's hardware description tables |
| **DMA, IOMMU** | Devices reading/writing memory directly, and the protection that limits it |
| **x86-64-v2** | The CPU feature floor Stela needs |
| **NVK, NIR, NAK, nouveau, GSP** | Mesa's NVIDIA Vulkan driver, IR, shader backend; the Linux kernel driver; NVIDIA's firmware |
| **QEMU, TCG, KVM** | A PC simulator, its slow pure-software mode, and its fast mode using the real CPU |

Related: [[Glossary - Stela]], [[Glossary - Arm]], [[Address Space Layout]], [[Host Drivers]].
