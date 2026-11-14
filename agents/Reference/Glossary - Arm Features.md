---
tags: [stela, glossary, arm]
---
# Glossary - Arm Features

Optional Arm features and manual terms: security protections, multi-core helpers and bulk-math instructions.

| Term | Meaning |
|---|---|
| **PAC / PAuth** | Pointer authentication. See [[Pointer Authentication]] |
| **QARMA** | The architected PAC cipher |
| **TBI** | "Top Byte Ignore": the top 8 bits of an address are free for other uses |
| **MTE** | Memory tagging: small labels on memory that catch misuse bugs |
| **BTI** | A protection that limits where jumps may land |
| **LSE** | Instructions for safely updating memory shared between cores |
| **Exclusive** | An older two-step way to do the same (`LDXR` then `STXR`) |
| **AdvSIMD / NEON** | 128-bit vector instructions |
| **SVE / SME** | Newer instructions for bulk math on vectors and matrices |
| **DIT** | A mode where instructions take the same time whatever the data, for security |
| **AARCHMRS** | Arm's specification as data files that programs can read |
| **UNDEFINED / UNPREDICTABLE / RES1** | Manual terms: "not a valid instruction", "result not specified", "this bit is always 1" |

Related: [[Glossary - Arm]], [[Pointer Authentication]], [[Parallel Execution]].
