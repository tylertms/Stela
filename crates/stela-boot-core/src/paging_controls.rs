//! Controls Stela must establish rather than inherit from UEFI.

const CR0_MONITOR_FPU: u64 = 1 << 1;
const CR0_EMULATE_FPU: u64 = 1 << 2;
const CR0_TASK_SWITCHED: u64 = 1 << 3;
const CR0_NATIVE_EXCEPTIONS: u64 = 1 << 5;
const CR0_WRITE_PROTECT: u64 = 1 << 16;

pub const fn native_cr0(firmware: u64) -> u64 {
    // Intel SDM Vol. 3A, CR0.NE: deliver native x87 exceptions through #MF,
    // not the legacy external FERR#/IGNNE# interface. The canonical runtime
    // requires this on the BSP just as the owned AP bootstrap does.
    (firmware | CR0_MONITOR_FPU | CR0_NATIVE_EXCEPTIONS | CR0_WRITE_PROTECT)
        & !(CR0_EMULATE_FPU | CR0_TASK_SWITCHED)
}

/// CR3's page-table base (Intel SDM Vol. 3A 4.5, AMD APM Vol. 2 5.3.2).
pub const ROOT_ADDRESS: u64 = 0x000f_ffff_ffff_f000;
/// CR3's low twelve bits: the root's PCID with CR4.PCIDE, else PWT and PCD.
pub const ROOT_CONTEXT: u64 = 0xfff;

/// Whether `cr3` names one of `roots`, page-table bases with no low bits.
/// With `tagged` (CR4.PCIDE set, and each root loaded in its own PCID below
/// `contexts`) the low twelve bits are that PCID and only tag TLB entries,
/// so the base alone decides. Without it they are PWT and PCD, cache
/// controls none of `roots` carries, and any of them set refuses. CR3 never
/// reads back the no-flush bit 63 of a write, and its bits 52 to 62 are
/// reserved, so a value with any of them set refuses too.
pub fn registered_root(cr3: u64, tagged: bool, contexts: u64, roots: &[u64]) -> bool {
    let context = cr3 & ROOT_CONTEXT;
    let admitted = if tagged {
        context < contexts
    } else {
        context == 0
    };
    admitted && cr3 & !(ROOT_ADDRESS | ROOT_CONTEXT) == 0 && roots.contains(&(cr3 & ROOT_ADDRESS))
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROOTS: [u64; 3] = [0x1000, 0x7_6543_2000, 0x000f_ffff_ffff_f000];

    #[test]
    fn untagged_roots_match_exactly() {
        for &root in &ROOTS {
            assert!(registered_root(root, false, 0, &ROOTS));
            // PWT or PCD on the root is not a registered root.
            assert!(!registered_root(root | 0x8, false, 0, &ROOTS));
            assert!(!registered_root(root | 0x10, false, 0, &ROOTS));
        }
        assert!(!registered_root(0x2000, false, 0, &ROOTS));
    }

    #[test]
    fn tagged_roots_match_by_base_under_their_contexts() {
        for &root in &ROOTS {
            for context in 0..18 {
                assert!(registered_root(root | context, true, 18, &ROOTS));
            }
            // A PCID no root is loaded in.
            assert!(!registered_root(root | 18, true, 18, &ROOTS));
            assert!(!registered_root(root | 0xfff, true, 18, &ROOTS));
        }
        assert!(!registered_root(0x2000 | 3, true, 18, &ROOTS));
    }

    #[test]
    fn reserved_and_no_flush_bits_refuse() {
        for tagged in [false, true] {
            assert!(!registered_root(0x1000 | 1 << 63, tagged, 18, &ROOTS));
            assert!(!registered_root(0x1000 | 1 << 52, tagged, 18, &ROOTS));
        }
    }

    #[test]
    fn legacy_firmware_exception_mode_gets_the_same_native_floor_as_aps() {
        // Paging/protected mode are already live. UEFI's incoming exception
        // mode and lazy-FPU bits must not decide canonical RAM admission.
        for firmware_fpu in 0..16 {
            let firmware = 0x8000_0011
                | (firmware_fpu & 1) << 1
                | (firmware_fpu & 2) << 1
                | (firmware_fpu & 4) << 1
                | (firmware_fpu & 8) << 2;
            assert_eq!(native_cr0(firmware), 0x8001_0033);
        }
    }

    #[test]
    fn takeover_preserves_other_controls_and_is_idempotent() {
        let owned = CR0_MONITOR_FPU
            | CR0_EMULATE_FPU
            | CR0_TASK_SWITCHED
            | CR0_NATIVE_EXCEPTIONS
            | CR0_WRITE_PROTECT;
        for bit in 0..64 {
            let firmware = 1u64 << bit;
            let native = native_cr0(firmware);
            assert_eq!(native & !owned, firmware & !owned);
            assert_eq!(native_cr0(native), native);
        }
    }
}
