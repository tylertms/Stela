//! The page below 1 MiB the other processors start from: a SIPI vector
//! names a page there (Intel SDM Vol. 3A 9.4.4). Stela allocates a free one
//! before boot services end. Firmware that marks every usable page below
//! 1 MiB as boot services code or data leaves none; Linux then takes a boot
//! services page once the firmware has exited ("Nasty quirk", in
//! `arch/x86/platform/efi/quirks.c` and `arch/x86/realmode/init.c`), and so
//! does Stela, after it has stopped the firmware's processors.

const PAGE: u64 = 4096;
/// Above the real-mode interrupt vector table.
pub const LOWEST: u64 = PAGE;
/// The first byte above the pages a startup page may take: the EBDA
/// usually starts at 0x9fc00.
pub const LIMIT: u64 = 0x9f000;
/// EfiBootServicesCode.
pub const BOOT_SERVICES_CODE: u32 = 3;
/// EfiBootServicesData.
pub const BOOT_SERVICES_DATA: u32 = 4;
/// EfiLoaderCode, the type of the page Stela allocates.
pub const LOADER_CODE: u32 = 1;
/// EfiConventionalMemory.
pub const CONVENTIONAL: u32 = 7;

fn boot_services(kind: u32) -> bool {
    matches!(kind, BOOT_SERVICES_CODE | BOOT_SERVICES_DATA)
}

/// The highest boot services page in `[LOWEST, LIMIT)` among memory map
/// entries `(type, start, pages)`. Only an entry that ends at or below
/// 1 MiB counts: the guest's RAM takes boot services memory in whole 2 MiB
/// pages alone, which such an entry never holds, so the page is Stela's
/// once boot services end.
pub fn boot_services_page(entries: impl IntoIterator<Item = (u32, u64, u64)>) -> Option<u64> {
    entries
        .into_iter()
        .filter(|&(kind, start, _)| boot_services(kind) && start.is_multiple_of(PAGE))
        .filter_map(|(_, start, pages)| {
            let end = start.checked_add(pages.checked_mul(PAGE)?)?;
            if end > 1 << 20 {
                return None;
            }
            let page = end.min(LIMIT).checked_sub(PAGE)?;
            (page >= start.max(LOWEST)).then_some(page)
        })
        .max()
}

/// Whether the final memory map still gives `page` to Stela: its own
/// LOADER_CODE allocation, or, when it was `taken` from boot services
/// memory, an entry that is still boot services memory, or free memory the
/// firmware gave back, and that ends at or below 1 MiB. Any other type
/// means something allocated the page since.
pub fn kept(entries: impl IntoIterator<Item = (u32, u64, u64)>, page: u64, taken: bool) -> bool {
    entries.into_iter().any(|(kind, start, pages)| {
        let Some(end) = pages
            .checked_mul(PAGE)
            .and_then(|bytes| start.checked_add(bytes))
        else {
            return false;
        };
        let owned = if taken {
            (boot_services(kind) || kind == CONVENTIONAL) && end <= 1 << 20
        } else {
            kind == LOADER_CODE
        };
        owned && start <= page && page + PAGE <= end
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const RESERVED: u32 = 0;

    #[test]
    fn all_low_memory_in_boot_services_still_gives_a_page() {
        let map = [
            (BOOT_SERVICES_DATA, 0, 1),
            (BOOT_SERVICES_CODE, 0x1000, 0x40),
            (BOOT_SERVICES_DATA, 0x41000, 0x5f),
            (RESERVED, 0xa0000, 0x60),
            (CONVENTIONAL, 0x10_0000, 0x1000),
        ];
        let page = boot_services_page(map).unwrap();
        assert_eq!(page, 0x9e000);
        assert!((LOWEST..LIMIT).contains(&page));
        assert!(kept(map, page, true));
        assert!(!kept(map, page, false));
    }

    #[test]
    fn the_vector_table_page_and_entries_past_1_mib_do_not_count() {
        assert_eq!(boot_services_page([(BOOT_SERVICES_DATA, 0, 1)]), None);
        // Its 2 MiB-aligned part could become guest RAM.
        assert_eq!(boot_services_page([(BOOT_SERVICES_DATA, 0, 0x400)]), None);
        assert_eq!(boot_services_page([(CONVENTIONAL, 0x1000, 0x10)]), None);
        assert_eq!(boot_services_page([(BOOT_SERVICES_CODE, 0x9f000, 1)]), None);
        assert_eq!(
            boot_services_page([
                (BOOT_SERVICES_CODE, 0x8000, 2),
                (BOOT_SERVICES_DATA, 0x20000, 1)
            ]),
            Some(0x20000)
        );
    }

    #[test]
    fn a_page_given_to_another_type_since_is_not_kept() {
        assert!(kept([(LOADER_CODE, 0x8000, 1)], 0x8000, false));
        assert!(!kept([(BOOT_SERVICES_DATA, 0x8000, 1)], 0x8000, false));
        // Taken from boot services: Stela's own later allocation there, or
        // the firmware's runtime data, is not Stela's startup page.
        assert!(kept([(CONVENTIONAL, 0x8000, 1)], 0x8000, true));
        assert!(!kept([(LOADER_CODE, 0x8000, 1)], 0x8000, true));
        assert!(!kept([(6, 0x8000, 1)], 0x8000, true));
        assert!(!kept([(CONVENTIONAL, 0, 0x400)], 0x8000, true));
        assert!(!kept([(BOOT_SERVICES_DATA, 0x8000, 1)], 0x9000, true));
        assert!(!kept([], 0x8000, true));
    }
}
