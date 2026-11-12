//! The Apple DART of T8110 layout (`dart,t8110`; ADT `/arm-io/dart-*`), the
//! IOMMU through which a device's stream (SID) reaches RAM.
//!
//! Documented by Linux `drivers/iommu/apple-dart.c` and `io-pgtable-dart.c`
//! (Asahi), registers 32-bit:
//! - PARAMS1 (0x00): access region protection 31 (below) and the page
//!   size's shift in [27:24]; PARAMS2 (0x04):
//!   BYPASS_SUPPORT 0; PARAMS3 (0x08): PA_WIDTH [29:24], VA_WIDTH [21:16]
//!   and the hardware version, major [15:8] and minor [7:0]; PARAMS4
//!   (0x0c): NUM_SIDS [8:0]. The 26A434 J473AP `AppleT8110DART::
//!   _dartValidateConfig` panics unless the shift is 14 (16 KiB pages) and,
//!   for a stream its DeviceTree has bypass, unless bypass is supported.
//!   Its `_hwVersion` accepts versions 1.0, 1.1, 2.0 to 2.4 and 3.0, and
//!   SPTM's `t8110dart_init_instance` also 3.2; SPTM requires a VA space
//!   above 40 bits from version 2 on. The ordinary 36-bit profile reports
//!   1.0; T6020 nodes that request wide DVAs and ranged flushes in the same
//!   16 KiB register window report 2.0, without the 2.2 fault extension.
//! - TLB_CMD (0x80): BUSY 31, OP [10:8] (0 flush all, 1 flush one stream;
//!   SPTM also issues 2 and 4, see `TLB_OPS`),
//!   STREAM [7:0]; SPTM's completion barrier at 0x600/0x604 (`TLB_SYNC`).
//! - ERROR (0x100): FLAG 31, STREAM [27:20], CODE [14:0] (READ_FAULT 5,
//!   WRITE_FAULT 4, NO_PTE 3, NO_PMD 2, NO_PGD 1, NO_TTBR 0) and region
//!   protect 29 (below); software
//!   writes back what it read to clear it; SPTM also writes only the code.
//!   ERROR_MASK (0x104), ERROR_ADDR_LO
//!   and _HI (0x170, 0x174) and the per-stream ERROR_STREAMS bitmap (0x1c0),
//!   which software clears by writing ones.
//! - PROTECT, UNPROTECT and PROTECT_LOCK (0x200, 0x204, 0x208), whose
//!   TTBR_TCR bit 0 locks every stream's TCR and TTBR.
//! - ENABLE_STREAMS and DISABLE_STREAMS (0xc00, 0xc20): bitmaps setting and
//!   clearing which streams may access memory at all.
//! - Per stream, TCR (0x1000 + 4 × SID: TRANSLATE_ENABLE 0, BYPASS_DART 1,
//!   BYPASS_DAPF 2, FOUR_LEVEL 3, REMAP_EN 7, REMAP [15:8]) and one TTBR
//!   (0x1400 + 4 × SID: VALID 0, the table's address >> 14 from bit 2).
//! - Tables of 2048 eight-byte entries in 16 KiB pages: a TTBR's table maps
//!   IOVA [35:25] to tables that map [24:14] to pages, unless FOUR_LEVEL adds
//!   a level. An entry is VALID 0, with its target's address >> 4 in
//!   [37:10]; a page entry adds NO_WRITE 2, NO_READ 3 and the first and last
//!   accessible 4-byte unit of the page in [63:52] and [51:40].
//!
//! A DART may pair with an address permission filter (`instance` `DAPF`,
//! the node's second `reg`): `n-apfs-instance-0` slices of 0x40 bytes, each a
//! 32-bit control (`0x333` meaningful), the first and last address as two
//! 32-bit halves at 0x08 and 0x10, and a 256-bit bitmap of the streams it
//! applies to at 0x20. The 26A434 J473AP SPTM programs every slice from
//! `dapf-instance-0` while PROTECT bit 2 (APF) is clear, then writes 4 to
//! PROTECT and PROTECT_LOCK, and verifies instead of writing once both read
//! bit 2 set (`t8110dart` power-up and `t8110dart_write_dapf_reg`). PROTECT
//! and PROTECT_LOCK hold one bit per protection domain: 0 TTBR and TCR, 2
//! APF; a write sets bits, UNPROTECT clears those not locked.
//!
//! SPTM's `t8110dart_powerup` (26A434 file offset 0xcbb90, store at
//! 0xcc014) restores the protected-write word at +0x210. Stela's handoff
//! disables that policy and has no protected-write violation, so a full-word
//! zero write/read succeeds. Nonzero policy encodings remain unsupported;
//! the status test of bit 1 on newer revisions alone does not describe how
//! to enable or acknowledge it. This register is separate from PROTECT.
//! The transaction-limit configuration at +0x228/+0x22c is saved on resume
//! (0xcc678..0xcc684), restored through the write-or-verify helper
//! (0xcc9cc..0xcc9f0), and clamped to zero by `clamp_tlimits`
//! (0xc8bd0..0xc8be0). Each full-word value keeps writes, initially zero.
//! DMA is synchronous with no outstanding transaction queue, so the clamp
//! has no queued transaction to drain; no physical credit rate is invented.
//!
//! Power-cycle evidence: SPTM saves enabled-stream words and optional
//! performance state before power-down (file 0xcd47c..0xcd7c4) and restores
//! TCR/TTBR, APF, interrupt masks and enabled words on power-up
//! (0xcc3d0..0xcc9bc). Linux `apple_dart_suspend`/`apple_dart_resume` also
//! save and restore stream tables around reset. Inferred: ordinary state is
//! lost on a PMGR power gate unless the DT declares `retention`; clock gating
//! preserves it. MMIO and DMA to an unpowered DART refuse. Secure TZ words
//! survive: SPTM checks PROTECT/LOCK bits 3 and 4, +504 select bit0, +500
//! enable bits 2:0 and nine region words at +508..548, stride8, masked with
//! 0x3ffffffc (0xcc038..0xcc13c), before restoring ordinary registers.
//! Stela establishes disabled zero zones in that boot-owned bank. Active TZ
//! programming remains unsupported, so PARAMS2 does not advertise it. The
//! recovered conditional path does not justify enabling that capability.
//!
//! Inferred, not documented: the model caches no translations, so TLB
//! commands complete at once and BUSY never reads set; a fault latches the
//! first error until software acknowledges its FLAG or code, and the interrupt follows
//! FLAG with an unmasked code; a missing first-level entry is NO_PMD and a
//! missing page entry NO_PTE; an access to a stream not enabled, without
//! translation or out of the 36-bit space faults NO_TTBR; while TTBR_TCR is
//! protected, TCR and TTBR writes have no effect, and PROTECT_LOCK keeps it
//! protected; likewise APF protection makes the filter's slices read-only.
//! The filter's slices are SoC register windows a coprocessor's own firmware
//! reaches; the machine's device models make no such accesses, so a
//! transfer on a stream that does not bypass a present filter refuses
//! rather than pass unfiltered.
//!
//! Access region protection is the page entries' accessible-unit bounds.
//! 26A434 `AppleT8110DART` reads the DeviceTree `dart-options` word
//! (`IODARTFamily` file 0x3b55130..0x3b551c0); its bit 1 asks for access
//! region protection, and the instance validation (file 0x2f1d6a4) panics
//! "access region protection is incorrectly configured" unless PARAMS1 bit
//! 31 reports it (J473AP `dart-usb0` sets the option). With the option the
//! driver maps start/end ranges at its `protection-granularity` within a
//! page (0x2f15774) and names ERROR bit 29 "region protect" (0x2f16fe4),
//! beside bits 4 and 5, write and read protect. Inferred: an access outside
//! a page's bounds reports bit 29 after the entry's NO_READ/NO_WRITE
//! checks, and bit 29 masks and signals like CODE. The identification
//! fields not listed above (PARAMS2 bit 4, PARAMS4 NUM_CLIENTS) read 0;
//! FOUR_LEVEL is refused until its behavior is known.
//!
//! Stream remapping: the 26A434 J473AP SPTM reads the DeviceTree `remap`
//! as (source, target) byte pairs, skipping zero pairs, and gives each
//! source a TCR of REMAP_EN with the target in [15:8], keeping the rest
//! (VA 0xfffffff0270d976c..0xfffffff0270d973c); it requires the target
//! to be a configured stream that is not itself remapped (0xfffffff0270ce044),
//! the source to neither translate nor bypass, and its TTBR invalid
//! (0xfffffff0270ce8a0..0xfffffff0270ce96c). An incoming remapped stream
//! therefore takes the target's context: Stela routes it once, then applies
//! the target's enable, TCR and TTBR, as the legacy T6000 model does. Which
//! SID a remapped stream's fault records is not recovered, so such a fault
//! refuses instead of inventing ERROR's STREAM field.
//!
//! Revision 2.2 (`Profile::T8110Rev22`), the register file of the M4- and
//! M5-generation nodes whose translation windows are 0xc000 or 0x20000
//! bytes, keeps the layout above and reports PARAMS3 version 0x0202 with a
//! 42-bit VA space: SPTM requires a VA space above 40 bits from version 2
//! (`t8110dart_init_instance`) and those nodes' `vm-base` is 1 << 40. Its
//! differences, from the 26A434 T8132 SPTM and Mac16 kernelcache:
//! - Faults are per stream, in the bank `gen3::fault::Capture` implements:
//!   a pending bit per stream at 0x4000 + 4 × word, which software clears
//!   by writing ones (SPTM CLEAR_EXCEPTION, VA 0xfffffff0270ce06c, helper
//!   0xfffffff0270cdf80), and a 0x40-byte record per stream at 0x8000 +
//!   0x40 × SID whose fault address is at +8/+0xc (`_DART_EXCEPTION`
//!   0xfffffe0009f32640). The kernel's `_fatalException`
//!   (0xfffffe0009f300d0..0xfffffe0009f303ec, table 0xfffffe0009f30594)
//!   decodes the status at +0 with the gen3 taxonomy: a code in [3:0] and,
//!   for an invalid entry, its level in [13:12]. As for gen3, the complete
//!   status header, the auxiliary AXI fields (`_DART_ERR_SID_AXI_ID`
//!   0xfffffe0009f319a4), a second fault on a pending stream and a record
//!   read after acknowledgement are unchecked and refuse. ERROR,
//!   ERROR_ADDRESS and ERROR_STREAMS are unused: SPTM's CLEAR_ERR raises a
//!   violation from 2.2 (0xfffffff0270cfdf8), so they refuse. ERROR_MASK
//!   moves to 0x4060 (SPTM table file 0x159d0); SPTM only writes it zero,
//!   and a nonzero mask, whose encoding is unrecovered, refuses. Inferred:
//!   the interrupt follows any pending bit. The bitmaps at 0x4020 and
//!   0x4040, which only the kernel's register capture reads, refuse.
//! - FOUR_LEVEL (TCR bit 3) walks three tables from the TTBR: DVA [46:36],
//!   [35:25] and [24:14] (the kernel's `t8110dart_vo_tte`, VA
//!   0xfffffe000be2b714, level masks 0xfffffe0007013cc0 and shifts ..ce0,
//!   whose root is the TTBR at level 0 with FOUR_LEVEL and level 1 without).
//!   A missing first-level entry is NO_PGD (code 0, level 1). Without
//!   FOUR_LEVEL the TTBR's table maps [35:25] as in revision 1; the kernel
//!   expects DVA [46:36] to be 0x10 (or 0 for a node it flags), so both
//!   reach those tables and any other value refuses as unrecovered.
//! - SPTM reads the capture count from TRAD +0x18 from version 2.1
//!   (0xfffffff0270d11a0), which reads zero here; the paired UMMS view's
//!   revision 2 interface is unrecovered and refuses.
//! - PROTECT domains 5 to 8 exist: SPTM requires them with TTBR_TCR locked
//!   below version 2.4 (0xfffffff0270d1e7c). What they guard is unrecovered;
//!   every register they could guard is unmodelled and refuses anyway.
//! - The window between the 0xc000-byte register file and a 0x20000-byte
//!   window's end has no recovered register (iBoot's DART driver maps only
//!   0x4000 or 0xc000 bytes, J604 iBoot VA 0x100c1b90ef0), so it refuses.
//!
//! The paired SMMU view exposes the capture
//! count of this cacheless implementation, not a physical chip's dump.
use super::descriptor::Format;
use super::gen3::fault::{Capture, Code, Level, Record};
use super::{DAPF_BYTES, DAPF_OFFSET, Fault, Profile, SMMU_BYTES, SMMU_OFFSET, SmmuBlock};
use crate::Result;
#[cfg(not(feature = "std"))]
use alloc::vec;
use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;
use stela_contracts::Refusal;
use stela_contracts::refuse;
use {
    stela_contracts::bus::Access, stela_contracts::bus::Direction,
    stela_contracts::bus::MmioDevice, stela_contracts::bus::MmioError, stela_contracts::bus::Width,
    stela_contracts::bus::refuse,
};
use {stela_contracts::dma::DmaError, stela_contracts::dma::Iommu, stela_contracts::dma::Memory};
use {stela_contracts::interrupt::AicInput, stela_contracts::interrupt::Wire};

const NAME: &str = "dart-t8110";

const PARAMS1: u64 = 0x00;
const PARAMS2: u64 = 0x04;
/// PARAMS1: access region protection (page entries' accessible-unit
/// bounds) and the page shift, 16 KiB pages.
const PARAMS1_VALUE: u32 = REGION_PROTECTION | PAGE_SHIFT << 24;
const REGION_PROTECTION: u32 = 1 << 31;
/// PARAMS2: streams can bypass translation (TCR BYPASS_DART).
const BYPASS_SUPPORT: u32 = 1;
const PARAMS3: u64 = 0x08;
const PARAMS4: u64 = 0x0c;
/// PARAMS3 versions of the two register files.
const VERSION_1_0: u16 = 0x0100;
const VERSION_2_0: u16 = 0x0200;
const VERSION_2_2: u16 = 0x0202;
/// Revision 2.2: the TRAD capture count (low 16 bits), the per-stream
/// pending words and records, and the moved ERROR_MASK.
const CAPTURE_COUNT: u64 = 0x18;
const PENDING: u64 = 0x4000;
const ERROR_MASK_2_2: u64 = 0x4060;
use stela_soc_data::dart::T8110_RECORD as RECORD;
use stela_soc_data::dart::T8110_RECORDS as RECORDS;
/// The revision 2.2 register file, ending with the last stream's record.
use stela_soc_data::dart::T8110_REGISTER_BYTES_2_2 as REGISTER_BYTES_2_2;
/// Each walk fault's record class: an invalid entry at its table level, or a
/// permission code, whose level field the decoder ignores.
const CLASSES: [(Fault, Code, Level); 7] = [
    (Fault::NoTtbr, Code::Invalid, Level::Ttbr),
    (Fault::NoPgd, Code::Invalid, Level::Cte),
    (Fault::NoPmd, Code::Invalid, Level::Ste),
    (Fault::NoPte, Code::Invalid, Level::Pte),
    (Fault::Write, Code::Write, Level::Pte),
    (Fault::Read, Code::Read, Level::Pte),
    (Fault::Region, Code::Region, Level::Pte),
];
const REVISION_2_2_UMMS: &str =
    "DART revision 2.2 UMMS capture interface is unrecovered; its count is TRAD +0x18";
const REVISION_2_2_ERRORS: &str =
    "DART revision 2.2 reports faults in per-stream exception records, not ERROR";
const REVISION_2_2_TAIL: &str = "DART window beyond the revision 2.2 register file is unrecovered";
const TLB_CMD: u64 = 0x80;
const TLB_OP: u32 = 0x7 << 8;
const TLB_OP_FLUSH_ALL: u32 = 0;
const TLB_OP_FLUSH_STREAM: u32 = 1 << 8;
const TLB_STREAM: u32 = 0xff;
/// SPTM's invalidation requests (file 0xc8618..0xc8718): OP 0 and 4 on
/// every instance, OP 1 for one stream, optionally (bit 14) bounded by the
/// range words at +0x98/+0xa0, and OP 2 with its parameter at +0x84. Bits
/// 13 and 15 check a request further: SPTM's whole-DART flush is OP 0
/// with bit 13, and bit 15 too from version 2.2 on (file 0xcd7cc..0xcd7dc).
/// A ranged stream flush is OP 1 with bits 13 and 14, adding bit 12 unless
/// its caller passes a nonzero flag (T8112 VA 0xfffffff0270d2c20..
/// 0xfffffff0270d2c2c, T8132 VA 0xfffffff0270d5940). With no translation
/// cached, each completes at once whatever subset it names; the words are
/// kept.
const TLB_OPS: [u32; 4] = [TLB_OP_FLUSH_ALL, TLB_OP_FLUSH_STREAM, 2 << 8, 4 << 8];
const TLB_QUALIFIERS: u32 = 1 << 12 | 1 << 13 | 1 << 14 | 1 << 15;
const TLB_PARAMETER: u64 = 0x84;
const TLB_RANGE: [u64; 2] = [0x98, 0xa0];
/// After a bit-15 request of OP 0 or 1, SPTM writes zero to +0x600 on each
/// instance whose descriptor asks for it (flag +0xc06, file 0xc8718..0xc8758)
/// and waits for bit 0 of +0x604 to clear (file 0xc8884..0xc8894): a
/// completion barrier for the invalidation. It completes with the request,
/// so the trigger reads zero and the status is never busy.
const TLB_SYNC: u64 = 0x600;
const TLB_SYNC_STATUS: u64 = 0x604;
const ERROR: u64 = 0x100;
const ERROR_MASK: u64 = 0x104;
const ERROR_ADDRESS_LOW: u64 = 0x170;
const ERROR_ADDRESS_HIGH: u64 = 0x174;
const ERROR_STREAMS: u64 = 0x1c0;
const PROTECT: u64 = 0x200;
const UNPROTECT: u64 = 0x204;
const PROTECT_LOCK: u64 = 0x208;
/// Protected-write control/status, restored by SPTM at file offset 0xcc014.
const PROTECTED_WRITE: u64 = 0x210;
const TLIMITS: [u64; 2] = [0x228, 0x22c];
/// Interrupt words of the version 2.1-and-earlier layout. SPTM's table
/// (file 0x159e0) gives ERROR_MASK 0x104 and these three; later versions
/// move them to 0x4060 and 0x4200..0x4220. 0x700 enables and 0x704
/// reports the performance counters' interrupts, bits 0x777 (SPTM's
/// clear-performance-interrupts call writes that mask at file 0xc9840;
/// power-up enables all and clears all, file 0xcc8f8..0xcc930; power-down
/// disables, 0xcd624). The counters (+0x760..0x788) exist only for a node
/// with `enable-perf-counters`, which no J473AP DART has: the counters are
/// absent here (their words refuse), so no counter interrupt is ever
/// pending. At 0x720 power-up writes one bit per stream from SPTM's set of
/// streams in use (file 0xcc948); whether these gate the error interrupt
/// per stream is unrecovered, so they are held and the error
/// interrupt follows ERROR_MASK alone.
const PERF_INTERRUPT_ENABLE: u64 = 0x700;
const PERF_INTERRUPT_STATUS: u64 = 0x704;
const PERF_INTERRUPTS: u32 = 0x777;
const STREAM_INTERRUPTS: u64 = 0x720;
const ENABLE_STREAMS: u64 = 0xc00;
const DISABLE_STREAMS: u64 = 0xc20;
const TCR: u64 = 0x1000;
const TTBR: u64 = 0x1400;
/// Bitmap words of the 256 streams the layout addresses.
const STREAM_WORDS: u64 = 8;

const ERROR_FLAG: u32 = 1 << 31;
const ERROR_WRITE: u32 = 1 << 28;
const ERROR_STREAM_SHIFT: u32 = 20;
const ERROR_CODE: u32 = 0x7fff | Fault::Region as u32;
const PROTECT_TTBR_TCR: u32 = 1;
const PROTECT_APF: u32 = 1 << 2;
/// Secure translation-zone configuration is outside the switched DART state.
const PROTECT_TZ: u32 = (1 << 3) | (1 << 4);
/// Revision 2 protection domains whose guarded registers are unrecovered.
const PROTECT_UNRECOVERED: u32 = 0x1e0;
const TZ_CONFIG: u64 = 0x500;
const TZ_SELECT: u64 = 0x504;
const TZ_FIRST: u64 = 0x508;
const TZ_END: u64 = 0x550;

const SLICE: u64 = 0x40;
const SLICE_CONTROL: u64 = 0x00;
const SLICE_START: u64 = 0x08;
const SLICE_END: u64 = 0x10;
const SLICE_STREAMS: u64 = 0x20;
const SLICE_CONTROL_KNOWN: u32 = 0x333;

const TCR_TRANSLATE: u32 = 1;
const TCR_BYPASS_DART: u32 = 1 << 1;
const TCR_BYPASS_DAPF: u32 = 1 << 2;
const TCR_FOUR_LEVEL: u32 = 1 << 3;
const TCR_REMAP_ENABLE: u32 = 1 << 7;
const TCR_REMAP_SHIFT: u32 = 8;
const TCR_REMAP: u32 = 0xff << TCR_REMAP_SHIFT;
const TCR_KNOWN: u32 = TCR_TRANSLATE
    | TCR_BYPASS_DART
    | TCR_BYPASS_DAPF
    | TCR_FOUR_LEVEL
    | TCR_REMAP_ENABLE
    | TCR_REMAP;
const TTBR_VALID: u32 = 1;
const TTBR_ADDRESS_SHIFT: u32 = 2;
/// The 28-bit page-number field, independently defined by m1n1's R_TTBR.
const TTBR_ADDRESS: u32 = 0x3fff_fffc;

const PAGE_SHIFT: u32 = 14;
const PAGE: u64 = 1 << PAGE_SHIFT;
const LEVEL_BITS: u32 = 11;
const IOVA_BITS: u32 = PAGE_SHIFT + 2 * LEVEL_BITS;
const IOVA_BITS_2_2: u32 = 42;
/// Revision 2.2 two-level walks: the DVA [46:36] values the kernel uses.
const TWO_LEVEL_WINDOWS: [u64; 2] = [0, 0x10];
/// The PAs an entry's address field reaches.
const PA_BITS: u32 = (ENTRY_ADDRESS << ENTRY_ADDRESS_SHIFT).ilog2() + 1;
const ENTRY_VALID: u64 = 1;
const ENTRY_NO_WRITE: u64 = 1 << 2;
const ENTRY_NO_READ: u64 = 1 << 3;
const ENTRY_ADDRESS: u64 = 0x3f_ffff_fc00;
const ENTRY_ADDRESS_SHIFT: u32 = 4;
const SUBPAGE_UNIT: u64 = 4;

pub struct Dart {
    /// PARAMS3's version, which selects the register file.
    version: u16,
    streams: usize,
    powered: bool,
    clock_enabled: bool,
    quiesced: bool,
    reset: bool,
    tlimits: [u32; 2],
    tlb_words: [u32; 3],
    tcr: Vec<u32>,
    ttbr: Vec<u32>,
    enabled: [u32; STREAM_WORDS as usize],
    perf_interrupts: u32,
    stream_interrupts: [u32; STREAM_WORDS as usize],
    error: u32,
    error_mask: u32,
    error_address: u64,
    error_streams: [u32; STREAM_WORDS as usize],
    /// Revision 2.2's per-stream fault bank.
    faults: Option<Capture>,
    protect: u32,
    locked: u32,
    /// The address permission filter's slices, when the DART has one.
    apf: Option<Vec<Slice>>,
    smmu: Option<SmmuBlock>,
    irq: Option<AicInput>,
    irq_level: bool,
    shared_fault: Option<(Rc<RefCell<Vec<bool>>>, usize)>,
}

/// One address permission filter slice, as its 32-bit registers hold it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Slice {
    control: u32,
    start: [u32; 2],
    end: [u32; 2],
    streams: [u32; STREAM_WORDS as usize],
}

impl Slice {
    fn register(&mut self, offset: u64) -> Option<(&mut u32, u32)> {
        let word = |base: u64| usize::try_from((offset - base) / 4).ok();
        match offset {
            SLICE_CONTROL => Some((&mut self.control, SLICE_CONTROL_KNOWN)),
            SLICE_START | 0x0c => Some((&mut self.start[word(SLICE_START)?], u32::MAX)),
            SLICE_END | 0x14 => Some((&mut self.end[word(SLICE_END)?], u32::MAX)),
            _ if (SLICE_STREAMS..SLICE).contains(&offset) => {
                Some((&mut self.streams[word(SLICE_STREAMS)?], u32::MAX))
            }
            _ => None,
        }
    }
}

impl Dart {
    pub(super) fn bind_shared_irq(&mut self, irq: &super::SharedIrq) -> Result<()> {
        if self.irq != Some(irq.input) {
            return Err(refuse!("DART shared IRQ does not match its physical input"));
        }
        if self.irq_level
            || self
                .shared_fault
                .as_ref()
                .is_some_and(|(levels, _)| levels.borrow().iter().any(|&level| level))
        {
            return Err(refuse!("an asserted DART IRQ source cannot be rebound"));
        }
        self.shared_fault = Some(irq.source());
        Ok(())
    }

    pub(super) const fn stream_capacity(&self) -> usize {
        self.streams
    }

    /// A revision-1.0 DART of `streams` streams (`sid-count`, at most 256),
    /// every stream disabled and untranslated, signalling faults on `irq`.
    pub fn new(streams: usize, irq: Option<AicInput>) -> Self {
        Self::for_version(VERSION_1_0, streams, irq)
    }

    /// A DART of the register file PARAMS3 `version` names.
    pub(super) fn for_version(version: u16, streams: usize, irq: Option<AicInput>) -> Self {
        assert!(
            (1..=256).contains(&streams),
            "the T8110 layout addresses 1..256 streams"
        );
        assert!(
            [VERSION_1_0, VERSION_2_0, VERSION_2_2].contains(&version),
            "a modelled T8110 register file"
        );
        Self {
            version,
            streams,
            powered: true,
            clock_enabled: true,
            quiesced: false,
            reset: false,
            tlimits: [0; 2],
            tlb_words: [0; 3],
            tcr: vec![0; streams],
            ttbr: vec![0; streams],
            enabled: [0; STREAM_WORDS as usize],
            perf_interrupts: 0,
            stream_interrupts: [0; STREAM_WORDS as usize],
            error: 0,
            error_mask: 0,
            error_address: 0,
            error_streams: [0; STREAM_WORDS as usize],
            faults: (version >= VERSION_2_2)
                .then(|| Capture::new(streams).expect("validated stream count")),
            protect: 0,
            locked: 0,
            apf: None,
            smmu: None,
            irq,
            irq_level: false,
            shared_fault: None,
        }
    }

    /// Independent TRAD instances sharing one level-sensitive fault output.
    /// The J473AP `dart-usb0` DT has two ranges and one `interrupts` cell.
    /// Clearing one instance cannot deassert another instance's pending fault.
    pub fn group(streams: usize, irq: Option<AicInput>, count: usize) -> Vec<Self> {
        Self::group_for_version(VERSION_1_0, streams, irq, count)
    }

    pub(super) fn group_for_version(
        version: u16,
        streams: usize,
        irq: Option<AicInput>,
        count: usize,
    ) -> Vec<Self> {
        let levels = Rc::new(RefCell::new(vec![false; count]));
        (0..count)
            .map(|index| {
                let mut dart = Self::for_version(version, streams, irq);
                dart.shared_fault = Some((levels.clone(), index));
                dart
            })
            .collect()
    }

    /// The DART with an address permission filter of `slices` slices.
    pub fn with_apf(mut self, slices: usize) -> Self {
        assert!(
            slices as u64 * SLICE <= DAPF_BYTES,
            "APF slices exceed the filter's registers"
        );
        self.apf = Some(vec![Slice::default(); slices]);
        self
    }

    /// Attach the view to an already placed TRAD. Returns false if already
    /// attached, so the machine can reject duplicate physical views. The
    /// device group's power, retention, quiesce and reset apply to both views.
    pub fn enable_smmu(&mut self) -> bool {
        if self.smmu.is_some() {
            return false;
        }
        self.smmu = Some(SmmuBlock);
        true
    }

    pub fn has_smmu(&self) -> bool {
        self.smmu.is_some()
    }

    /// Apply a PMGR supply transition. Clock gating keeps this state; a
    /// power gate resets ordinary registers unless the DT declares retention.
    /// Secure translation-zone configuration/protection remains outside the
    /// switched supply: SPTM verifies it before restoring ordinary state.
    pub fn set_power(&mut self, powered: bool, retention: bool, access: &mut Access<'_>) {
        if self.powered && !powered && !retention {
            self.clear_ordinary(access);
        }
        self.powered = powered;
    }

    /// Quiesce a bus master before resetting its domain. DMA is synchronous,
    /// so there are no outstanding transactions to drain. Configuration stays
    /// accessible until reset is asserted; releasing reset does not undo this
    /// separate inhibit. The machine applies PMGR transitions in order.
    pub fn set_quiesced(&mut self, quiesced: bool) {
        self.quiesced = quiesced;
    }

    pub fn set_clock(&mut self, enabled: bool) {
        self.clock_enabled = enabled;
    }

    /// Apply the ordinary register bank's reset line independently of power
    /// retention. Assertion clears ordinary state and faults; while held, MMIO
    /// and DMA refuse. The disabled secure-zone bank is boot-owned and outside
    /// this reset, as it is outside the switched ordinary power domain.
    pub fn set_reset(&mut self, asserted: bool, access: &mut Access<'_>) {
        if asserted && !self.reset {
            self.clear_ordinary(access);
        }
        self.reset = asserted;
    }

    fn clear_ordinary(&mut self, access: &mut Access<'_>) {
        self.tlimits.fill(0);
        self.tlb_words.fill(0);
        self.tcr.fill(0);
        self.ttbr.fill(0);
        self.enabled.fill(0);
        self.perf_interrupts = 0;
        self.stream_interrupts.fill(0);
        self.error = 0;
        self.error_mask = 0;
        self.error_address = 0;
        self.error_streams.fill(0);
        if let Some(faults) = &mut self.faults {
            faults.clear();
        }
        self.protect &= PROTECT_TZ;
        self.locked &= PROTECT_TZ;
        if let Some(apf) = &mut self.apf {
            apf.fill(Slice::default());
        }
        self.signal(access);
    }

    /// The protection domains this DART has.
    fn domains(&self) -> u32 {
        let unrecovered = if self.revision_2_2() {
            PROTECT_UNRECOVERED
        } else {
            0
        };
        PROTECT_TTBR_TCR
            | PROTECT_TZ
            | unrecovered
            | if self.apf.is_some() { PROTECT_APF } else { 0 }
    }

    const fn revision_2_2(&self) -> bool {
        self.version >= VERSION_2_2
    }

    const fn iova_bits(&self) -> u32 {
        if self.version >= VERSION_2_0 {
            IOVA_BITS_2_2
        } else {
            IOVA_BITS
        }
    }

    /// Whether `offset` is one of revision 2.2's per-stream fault registers.
    fn fault_register(&self, offset: u64) -> bool {
        let pending = PENDING..PENDING + 4 * self.streams.div_ceil(32) as u64;
        let records = RECORDS..RECORDS + RECORD * self.streams as u64;
        pending.contains(&offset) || records.contains(&offset)
    }

    /// A revision 2.2 register read, or None for the shared layout.
    fn revision_2_2_read(
        &mut self,
        offset: u64,
        width: Width,
        access: &mut Access<'_>,
    ) -> Option<Result<u64, MmioError>> {
        let fail = |reason: Refusal| refuse(NAME, offset, width, Direction::Read, reason);
        let registers = self.fault_register(offset);
        let faults = self.faults.as_mut()?;
        Some(match offset {
            ERROR | ERROR_MASK | ERROR_ADDRESS_LOW | ERROR_ADDRESS_HIGH => {
                Err(fail(REVISION_2_2_ERRORS.into()))
            }
            _ if bitmap_word(offset, ERROR_STREAMS).is_some() => {
                Err(fail(REVISION_2_2_ERRORS.into()))
            }
            // The cacheless model captures no state; only a zero mask is
            // accepted.
            CAPTURE_COUNT | ERROR_MASK_2_2 => Ok(0),
            _ if registers => faults.read(offset, width, access),
            _ if (REGISTER_BYTES_2_2..DAPF_OFFSET).contains(&offset) => {
                Err(fail(REVISION_2_2_TAIL.into()))
            }
            _ => return None,
        })
    }

    /// A revision 2.2 register write, or None for the shared layout.
    fn revision_2_2_write(
        &mut self,
        offset: u64,
        width: Width,
        value: u64,
        access: &mut Access<'_>,
    ) -> Option<Result<(), MmioError>> {
        let fail = |reason: Refusal| refuse(NAME, offset, width, Direction::Write, reason);
        let registers = self.fault_register(offset);
        let faults = self.faults.as_mut()?;
        Some(match offset {
            ERROR | ERROR_MASK | ERROR_ADDRESS_LOW | ERROR_ADDRESS_HIGH => {
                Err(fail(REVISION_2_2_ERRORS.into()))
            }
            _ if bitmap_word(offset, ERROR_STREAMS).is_some() => {
                Err(fail(REVISION_2_2_ERRORS.into()))
            }
            CAPTURE_COUNT => Err(fail(refuse!("DART capture count is read-only"))),
            ERROR_MASK_2_2 if value == 0 => Ok(()),
            ERROR_MASK_2_2 => Err(fail(refuse!(
                "DART revision 2.2 error-mask encoding is unrecovered"
            ))),
            _ if registers => faults.write(offset, width, value, access),
            _ if (REGISTER_BYTES_2_2..DAPF_OFFSET).contains(&offset) => {
                Err(fail(REVISION_2_2_TAIL.into()))
            }
            _ => return None,
        })
    }

    /// The filter register at `offset` from the filter's start and the bits
    /// it holds.
    fn apf_register(&mut self, offset: u64) -> Option<(&mut u32, u32)> {
        let slices = self.apf.as_mut()?;
        let slice = slices.get_mut(usize::try_from(offset / SLICE).ok()?)?;
        if !offset.is_multiple_of(4) {
            return None;
        }
        slice.register(offset % SLICE)
    }

    /// The context an incoming stream takes: its remap target, if enabled.
    fn context(&self, sid: usize) -> Result<usize, DmaError> {
        let tcr = self.tcr[sid];
        if tcr & TCR_REMAP_ENABLE == 0 {
            return Ok(sid);
        }
        if tcr & (TCR_TRANSLATE | TCR_BYPASS_DART) != 0 || self.ttbr[sid] & TTBR_VALID != 0 {
            return Err(DmaError::Unsupported(refuse!(
                "DART remapped stream with its own translation, bypass or table"
            )));
        }
        let target = ((tcr & TCR_REMAP) >> TCR_REMAP_SHIFT) as usize;
        if target >= self.streams || self.tcr[target] & TCR_REMAP_ENABLE != 0 {
            return Err(DmaError::Unsupported(refuse!(
                "DART remap target is outside the profile or itself remapped"
            )));
        }
        Ok(target)
    }

    fn stream_enabled(&self, sid: usize) -> bool {
        self.enabled[sid / 32] >> (sid % 32) & 1 != 0
    }

    pub(super) fn walk(
        &self,
        memory: &mut dyn Memory,
        sid: usize,
        iova: u64,
        last: u64,
        write: bool,
    ) -> Result<u64, DmaError> {
        if !self.powered {
            return Err(DmaError::Unsupported(refuse!("DART power domain is off")));
        }
        if !self.clock_enabled {
            return Err(DmaError::Unsupported(refuse!(
                "DART functional clock is gated"
            )));
        }
        if self.reset {
            return Err(DmaError::Unsupported(refuse!("DART reset is asserted")));
        }
        if self.quiesced {
            return Err(DmaError::Unsupported(refuse!("DART DMA is quiesced")));
        }
        if sid >= self.streams {
            return Err(DmaError::Unsupported(refuse!(
                "DART incoming stream is outside the native profile"
            )));
        }
        let sid = self.context(sid)?;
        if !self.stream_enabled(sid) {
            return Err(fault(Fault::NoTtbr));
        }
        let tcr = self.tcr[sid];
        if tcr & (TCR_BYPASS_DART | TCR_TRANSLATE) == TCR_BYPASS_DART | TCR_TRANSLATE {
            return Err(DmaError::Unsupported(refuse!(
                "DART simultaneous translation and bypass"
            )));
        }
        if self.apf.is_some() && tcr & TCR_BYPASS_DAPF == 0 {
            return Err(DmaError::Unsupported(refuse!(
                "DART address permission filtering"
            )));
        }
        if tcr & TCR_BYPASS_DART != 0 {
            return Ok(iova);
        }
        if tcr & TCR_TRANSLATE == 0 || iova >> self.iova_bits() != 0 {
            return Err(fault(Fault::NoTtbr));
        }
        let four_level = tcr & TCR_FOUR_LEVEL != 0;
        if four_level && self.version < VERSION_2_0 {
            return Err(DmaError::Unsupported(refuse!("four-level DART tables")));
        }
        if !four_level && !TWO_LEVEL_WINDOWS.contains(&(iova >> IOVA_BITS)) {
            return Err(DmaError::Unsupported(refuse!(
                "DART two-level DVA bits [46:36] outside the recovered 0 and 0x10 windows"
            )));
        }
        let ttbr = self.ttbr[sid];
        if ttbr & TTBR_VALID == 0 {
            return Err(fault(Fault::NoTtbr));
        }
        let mut table = u64::from((ttbr & TTBR_ADDRESS) >> TTBR_ADDRESS_SHIFT) << PAGE_SHIFT;
        if four_level {
            let upper = entry(memory, table, iova >> IOVA_BITS)?;
            if upper & ENTRY_VALID == 0 {
                return Err(fault(Fault::NoPgd));
            }
            table = target(upper, false)?;
        }
        let first = entry(memory, table, iova >> (PAGE_SHIFT + LEVEL_BITS))?;
        if first & ENTRY_VALID == 0 {
            return Err(fault(Fault::NoPmd));
        }
        let page = entry(memory, target(first, false)?, iova >> PAGE_SHIFT)?;
        if page & ENTRY_VALID == 0 {
            return Err(fault(Fault::NoPte));
        }
        let target = target(page, true)?;
        let offset = iova & (PAGE - 1);
        let (first_unit, last_unit) = (page >> 52 & 0xfff, page >> 40 & 0xfff);
        let inside =
            offset / SUBPAGE_UNIT >= first_unit && (last & (PAGE - 1)) / SUBPAGE_UNIT <= last_unit;
        match write {
            true if page & ENTRY_NO_WRITE != 0 => Err(fault(Fault::Write)),
            false if page & ENTRY_NO_READ != 0 => Err(fault(Fault::Read)),
            _ if !inside => Err(fault(Fault::Region)),
            _ => Ok(target + offset),
        }
    }

    /// Latch a fault: revision 1.0's first ERROR, or revision 2.2's
    /// stream record, whose second capture is unchecked and refuses.
    fn report(
        &mut self,
        sid: usize,
        iova: u64,
        code: u32,
        write: bool,
        access: &mut Access<'_>,
    ) -> Result<(), DmaError> {
        if let Some(faults) = &mut self.faults {
            let &(_, code, level) = CLASSES
                .iter()
                .find(|(fault, ..)| *fault as u32 == code)
                .expect("every walk fault has a record class");
            let record = Record {
                code,
                level,
                write,
                address: iova,
            };
            faults.capture(sid, record).map_err(DmaError::Unsupported)?;
            self.signal(access);
            return Ok(());
        }
        if self.error & ERROR_FLAG == 0 {
            self.error = ERROR_FLAG
                | (sid as u32 & 0xff) << ERROR_STREAM_SHIFT
                | if write { ERROR_WRITE } else { 0 }
                | code;
            self.error_address = iova;
        }
        if sid < 256 {
            self.error_streams[sid / 32] |= 1 << (sid % 32);
        }
        self.signal(access);
        Ok(())
    }

    fn signal(&mut self, access: &mut Access<'_>) {
        let level = if let Some(faults) = &self.faults {
            (0..self.streams.div_ceil(32)).any(|word| faults.pending(word) != Ok(0))
        } else {
            self.error & ERROR_FLAG != 0 && self.error & ERROR_CODE & !self.error_mask != 0
        };
        if level != self.irq_level {
            self.irq_level = level;
            let level = if let Some((levels, index)) = &self.shared_fault {
                let mut levels = levels.borrow_mut();
                levels[*index] = level;
                levels.iter().any(|&level| level)
            } else {
                level
            };
            if let Some(irq) = self.irq {
                access.interrupts.set_level(Wire::Aic(irq), level);
            }
        }
    }

    /// The stream of a per-stream register at `offset` from `base`.
    fn stream(&self, offset: u64, base: u64) -> Option<usize> {
        if !offset.is_multiple_of(4) {
            return None;
        }
        let index = usize::try_from(offset.checked_sub(base)? / 4).ok()?;
        (index < self.streams).then_some(index)
    }
}

/// The PARAMS3 version of a `dart,t8110` profile.
pub(super) const fn version(profile: Profile) -> u16 {
    match profile {
        Profile::T8110Rev20 => VERSION_2_0,
        Profile::T8110Rev22 => VERSION_2_2,
        _ => VERSION_1_0,
    }
}

fn fault(fault: Fault) -> DmaError {
    DmaError::Fault(fault as u32)
}

/// The refusal of fault `code` on a remapped stream, whose ERROR record
/// needs the unrecovered source-versus-context SID provenance.
fn remapped_fault(code: u32) -> &'static str {
    const FAULTS: [(Fault, &str); 7] = [
        (
            Fault::NoTtbr,
            "DART remapped no-TTBR fault: source-versus-context SID provenance",
        ),
        (
            Fault::NoPgd,
            "DART remapped no-PGD fault: source-versus-context SID provenance",
        ),
        (
            Fault::NoPmd,
            "DART remapped no-PMD fault: source-versus-context SID provenance",
        ),
        (
            Fault::NoPte,
            "DART remapped no-PTE fault: source-versus-context SID provenance",
        ),
        (
            Fault::Write,
            "DART remapped write fault: source-versus-context SID provenance",
        ),
        (
            Fault::Read,
            "DART remapped read fault: source-versus-context SID provenance",
        ),
        (
            Fault::Region,
            "DART remapped subpage fault: source-versus-context SID provenance",
        ),
    ];
    FAULTS
        .iter()
        .find(|(fault, _)| *fault as u32 == code)
        .map_or(
            "DART remapped fault: source-versus-context SID provenance",
            |(_, why)| why,
        )
}

impl Iommu for Dart {
    fn page(&self) -> u64 {
        PAGE
    }

    /// A fault is latched in ERROR and signalled.
    fn translate(
        &mut self,
        memory: &mut dyn Memory,
        sid: usize,
        iova: u64,
        last: u64,
        write: bool,
        access: &mut Access<'_>,
    ) -> Result<u64, DmaError> {
        debug_assert!(last >= iova && last >> PAGE_SHIFT == iova >> PAGE_SHIFT);
        let result = self.walk(memory, sid, iova, last, write);
        if let Err(DmaError::Fault(code)) = result {
            if self.tcr[sid] & TCR_REMAP_ENABLE != 0 {
                return Err(DmaError::Unsupported(remapped_fault(code).into()));
            }
            self.report(sid, iova, code, write, access)?;
        }
        result
    }
}

/// The entry `index` (masked to a table) of the table at `table`.
fn entry(memory: &mut dyn Memory, table: u64, index: u64) -> Result<u64, DmaError> {
    let mut bytes = [0; 8];
    let address = table + (index & ((1 << LEVEL_BITS) - 1)) * 8;
    if !memory.read(address, &mut bytes) {
        return Err(DmaError::Memory);
    }
    Ok(u64::from_le_bytes(bytes))
}

/// The address an entry points at.
fn target(entry: u64, leaf: bool) -> Result<u64, DmaError> {
    Format::Dart2.target(entry, PA_BITS, PAGE, leaf)
}

fn bitmap_word(offset: u64, base: u64) -> Option<usize> {
    let word = offset.checked_sub(base)? / 4;
    (word < STREAM_WORDS && offset.is_multiple_of(4)).then_some(word as usize)
}

impl MmioDevice for Dart {
    fn name(&self) -> &'static str {
        NAME
    }

    fn read(
        &mut self,
        offset: u64,
        width: Width,
        access: &mut Access<'_>,
    ) -> Result<u64, MmioError> {
        let fail = |reason: Refusal| refuse(NAME, offset, width, Direction::Read, reason);
        if width != Width::Word {
            return Err(fail(refuse!("DART registers are 32-bit")));
        }
        if !self.powered {
            return Err(fail(refuse!("DART power domain is off")));
        }
        if !self.clock_enabled {
            return Err(fail(refuse!("DART functional clock is gated")));
        }
        if self.reset {
            return Err(fail(refuse!("DART reset is asserted")));
        }
        if (SMMU_OFFSET..SMMU_OFFSET + SMMU_BYTES).contains(&offset) {
            if self.revision_2_2() && self.smmu.is_some() {
                return Err(fail(REVISION_2_2_UMMS.into()));
            }
            return self
                .smmu
                .as_mut()
                .ok_or_else(|| fail(refuse!("DART has no paired SMMU instance")))?
                .read(offset - SMMU_OFFSET, width, access);
        }
        if let Some(apf) = offset.checked_sub(DAPF_OFFSET) {
            return self
                .apf_register(apf)
                .map(|(value, _)| u64::from(*value))
                .ok_or_else(|| fail(refuse!("unknown DART APF register")));
        }
        if let Some(result) = self.revision_2_2_read(offset, width, access) {
            return result;
        }
        let value = match offset {
            PARAMS1 => PARAMS1_VALUE,
            PARAMS2 => BYPASS_SUPPORT,
            PARAMS3 => PA_BITS << 24 | self.iova_bits() << 16 | u32::from(self.version),
            PARAMS4 => self.streams as u32,
            TLB_CMD | TLB_SYNC | TLB_SYNC_STATUS => 0,
            TLB_PARAMETER => self.tlb_words[0],
            _ if TLB_RANGE.contains(&offset) => {
                self.tlb_words[1 + usize::from(offset == TLB_RANGE[1])]
            }
            ERROR => self.error,
            ERROR_MASK => self.error_mask,
            ERROR_ADDRESS_LOW => self.error_address as u32,
            ERROR_ADDRESS_HIGH => (self.error_address >> 32) as u32,
            // UNPROTECT and DISABLE_STREAMS are clear-side views of the
            // protection and enable state. SPTM reads each register back
            // after writing it to order the write (file 0xcd75c), ignoring
            // the value; inferred: the view reads the state it clears.
            PROTECT | UNPROTECT => self.protect,
            PROTECT_LOCK => self.locked,
            // The boot stage leaves protected-write reporting disabled and
            // there is no pending protected-write violation.
            PROTECTED_WRITE => 0,
            PERF_INTERRUPT_ENABLE => self.perf_interrupts,
            PERF_INTERRUPT_STATUS => 0,
            _ if TLIMITS.contains(&offset) => self.tlimits[((offset - TLIMITS[0]) / 4) as usize],
            TZ_CONFIG | TZ_SELECT => 0,
            _ if (TZ_FIRST..TZ_END).contains(&offset) && (offset - TZ_FIRST).is_multiple_of(8) => 0,
            _ => {
                if let Some(word) = bitmap_word(offset, ERROR_STREAMS) {
                    self.error_streams[word]
                } else if let Some(word) = bitmap_word(offset, ENABLE_STREAMS)
                    .or_else(|| bitmap_word(offset, DISABLE_STREAMS))
                {
                    self.enabled[word]
                } else if let Some(word) = bitmap_word(offset, STREAM_INTERRUPTS) {
                    self.stream_interrupts[word]
                } else if let Some(sid) = self.stream(offset, TCR).filter(|_| offset < TTBR) {
                    self.tcr[sid]
                } else if let Some(sid) = self.stream(offset, TTBR) {
                    self.ttbr[sid]
                } else {
                    return Err(fail(refuse!("unknown DART register")));
                }
            }
        };
        Ok(u64::from(value))
    }

    fn write(
        &mut self,
        offset: u64,
        width: Width,
        value: u64,
        access: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        let fail = |reason: Refusal| refuse(NAME, offset, width, Direction::Write, reason);
        if width != Width::Word {
            return Err(fail(refuse!("DART registers are 32-bit")));
        }
        if !self.powered {
            return Err(fail(refuse!("DART power domain is off")));
        }
        if !self.clock_enabled {
            return Err(fail(refuse!("DART functional clock is gated")));
        }
        if self.reset {
            return Err(fail(refuse!("DART reset is asserted")));
        }
        if (SMMU_OFFSET..SMMU_OFFSET + SMMU_BYTES).contains(&offset) {
            if self.revision_2_2() && self.smmu.is_some() {
                return Err(fail(REVISION_2_2_UMMS.into()));
            }
            return self
                .smmu
                .as_mut()
                .ok_or_else(|| fail(refuse!("DART has no paired SMMU instance")))?
                .write(offset - SMMU_OFFSET, width, value, access);
        }
        let value = value as u32;
        if let Some(apf) = offset.checked_sub(DAPF_OFFSET) {
            let locked = self.protect & PROTECT_APF != 0;
            let (register, known) = self
                .apf_register(apf)
                .ok_or_else(|| fail(refuse!("unknown DART APF register")))?;
            if value & !known != 0 {
                return Err(fail(refuse!("DART APF slice control bits outside 0x333")));
            }
            if !locked {
                *register = value;
            }
            return Ok(());
        }
        if let Some(result) = self.revision_2_2_write(offset, width, u64::from(value), access) {
            result?;
            self.signal(access);
            return Ok(());
        }
        let protected = self.protect & PROTECT_TTBR_TCR != 0;
        if matches!(offset, PROTECT | UNPROTECT | PROTECT_LOCK) && value & !self.domains() != 0 {
            return Err(fail(refuse!("DART protection domain without a model")));
        }
        match offset {
            TLB_CMD => {
                if value & !(TLB_STREAM | TLB_OP | TLB_QUALIFIERS) != 0
                    || !TLB_OPS.contains(&(value & TLB_OP))
                {
                    return Err(fail(refuse!("unknown DART TLB command")));
                }
            }
            TLB_SYNC if value == 0 => {}
            TLB_SYNC => return Err(fail(refuse!("DART TLB barrier trigger takes zero"))),
            TLB_PARAMETER => self.tlb_words[0] = value,
            _ if TLB_RANGE.contains(&offset) => {
                self.tlb_words[1 + usize::from(offset == TLB_RANGE[1])] = value
            }
            ERROR => {
                // Linux writes the captured word. SPTM also acknowledges
                // one code without FLAG (26A434 file 0xc99e8).
                if value & ERROR_FLAG != 0 || value & self.error & ERROR_CODE != 0 {
                    self.error = 0;
                }
            }
            ERROR_MASK => self.error_mask = value & ERROR_CODE,
            TZ_CONFIG | TZ_SELECT => {
                return Err(fail(refuse!(
                    "translation zones are owned by the boot stage; active-zone programming is unsupported"
                )));
            }
            _ if (TZ_FIRST..TZ_END).contains(&offset) && (offset - TZ_FIRST).is_multiple_of(8) => {
                return Err(fail(refuse!(
                    "translation-zone bounds are owned by the boot stage"
                )));
            }
            _ if TLIMITS.contains(&offset) => {
                self.tlimits[((offset - TLIMITS[0]) / 4) as usize] = value
            }
            PERF_INTERRUPT_ENABLE => self.perf_interrupts = value & PERF_INTERRUPTS,
            // Write-one-to-clear; nothing is pending.
            PERF_INTERRUPT_STATUS => {}
            PROTECTED_WRITE if value == 0 => {}
            PROTECTED_WRITE => {
                return Err(fail(refuse!(
                    "nonzero protected-write policy is not described"
                )));
            }
            PROTECT => self.protect |= value,
            UNPROTECT => self.protect &= !(value & !self.locked),
            PROTECT_LOCK => self.locked |= value,
            _ => {
                if let Some(word) = bitmap_word(offset, ERROR_STREAMS) {
                    self.error_streams[word] &= !value;
                } else if let Some(word) = bitmap_word(offset, ENABLE_STREAMS) {
                    self.enabled[word] |= value;
                } else if let Some(word) = bitmap_word(offset, DISABLE_STREAMS) {
                    self.enabled[word] &= !value;
                } else if let Some(word) = bitmap_word(offset, STREAM_INTERRUPTS) {
                    self.stream_interrupts[word] = value;
                } else if let Some(sid) = self.stream(offset, TCR).filter(|_| offset < TTBR) {
                    if value & !TCR_KNOWN != 0 {
                        return Err(fail(refuse!("DART TCR reserved bits")));
                    }
                    if !protected {
                        self.tcr[sid] = value;
                    }
                } else if let Some(sid) = self.stream(offset, TTBR) {
                    if !protected {
                        self.ttbr[sid] = value;
                    }
                } else {
                    return Err(fail(refuse!("unknown DART register")));
                }
            }
        }
        self.signal(access);
        Ok(())
    }
}

#[cfg(test)]
mod tests;
