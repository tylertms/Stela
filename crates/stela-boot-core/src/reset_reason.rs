//! Why an AMD PC last reset or powered off, for the boot log.
//!
//! The FCH's power-management block keeps `S5_RESET_STATUS` at offset 0xc0
//! of its MMIO window at 0xfed8_0300 (Linux
//! `include/linux/platform_data/x86/amd-fch.h`). Each set bit names one
//! cause of the last reset or power-off; the bits stay until software clears
//! them, so the boot after a hang, a watchdog reset or a power-button hold
//! still reads what ended the one before. Linux reads it once on every Zen
//! processor (families 0x17 and later) and ignores all ones, an error
//! response, and so does Stela, which never clears it.
//!
//! A boot whose log ends with no stop line, because the stop could not
//! reach the disk or the PC reset first, is told apart by this line at the
//! next start: a power-button hold, a thermal trip, a processor shutdown
//! (triple fault), a watchdog or a sync flood after an uncorrected error.
use alloc::{format, string::String, vec::Vec};

/// `FCH_PM_BASE + FCH_PM_S5_RESET_STATUS`.
pub const ADDRESS: u64 = 0xfed8_0300 + 0xc0;

/// The causes the bits name, as Linux decodes them; bits not listed are
/// reserved or unnamed and reported by number.
const CAUSES: [(u32, &str); 20] = [
    (0, "thermal trip pin"),
    (1, "power button held for 4 s"),
    (2, "shutdown pin"),
    (4, "remote ASF power-off command"),
    (9, "processor thermal limit"),
    (16, "reset pin"),
    (17, "software PCI reset"),
    (18, "software wrote 0x4 to port 0xcf9"),
    (19, "software wrote 0x6 to port 0xcf9"),
    (20, "software wrote 0xe to port 0xcf9"),
    (21, "ACPI sleep or power-off transition"),
    (22, "keyboard reset pin"),
    (23, "processor shutdown event (triple fault)"),
    (24, "failed-boot timer expired"),
    (25, "hardware watchdog expired"),
    (26, "remote ASF reset command"),
    (27, "data fabric sync flood after an uncorrected error"),
    (29, "FCH and MP1 warm reset handshake failed"),
    (30, "parity error"),
    (31, "software sync flood"),
];

/// Whether the processor this vendor string and display family name has
/// the register: AMD's Zen families, as Linux's `X86_FEATURE_ZEN` gates it.
pub fn applies(vendor: &str, family: u32) -> bool {
    vendor == "AuthenticAMD" && family >= 0x17
}

/// The causes `status` records, in bit order; `None` for all ones.
pub fn causes(status: u32) -> Option<Vec<String>> {
    if status == u32::MAX {
        return None;
    }
    Some(
        (0..32)
            .filter(|bit| status & (1 << bit) != 0)
            .map(|bit| {
                CAUSES
                    .iter()
                    .find(|&&(named, _)| named == bit)
                    .map_or_else(|| format!("bit {bit}"), |&(_, cause)| cause.into())
            })
            .collect(),
    )
}

/// `HOST RESET previous status=… causes: …`, or `None` when the read
/// failed (all ones) or recorded no cause.
pub fn line(status: u32) -> Option<String> {
    let causes = causes(status).filter(|causes| !causes.is_empty())?;
    Some(format!(
        "HOST RESET previous status={status:#010x} causes: {}\n",
        causes.join("; ")
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_power_off_and_an_unnamed_bit_are_both_reported() {
        // A PC shut down from its operating system: bit 21 and bit 11, which
        // has no name.
        assert_eq!(
            line(0x0020_0800).as_deref(),
            Some(
                "HOST RESET previous status=0x00200800 causes: bit 11; \
                 ACPI sleep or power-off transition\n"
            )
        );
    }

    #[test]
    fn a_hang_that_ended_in_a_reset_names_its_cause() {
        assert_eq!(
            causes(1 << 1).unwrap(),
            ["power button held for 4 s"],
            "a user's hold of the power button"
        );
        assert_eq!(
            causes((1 << 23) | (1 << 27)).unwrap(),
            [
                "processor shutdown event (triple fault)",
                "data fabric sync flood after an uncorrected error"
            ]
        );
        assert_eq!(causes(1 << 25).unwrap(), ["hardware watchdog expired"]);
    }

    #[test]
    fn an_error_response_or_an_empty_record_prints_nothing() {
        assert_eq!(causes(u32::MAX), None);
        assert_eq!(line(u32::MAX), None);
        assert_eq!(line(0), None);
    }

    #[test]
    fn only_zen_processors_have_the_register() {
        assert!(applies("AuthenticAMD", 0x17));
        assert!(applies("AuthenticAMD", 0x19));
        assert!(applies("AuthenticAMD", 0x1a));
        assert!(!applies("AuthenticAMD", 0x15));
        assert!(!applies("GenuineIntel", 0x6));
        assert!(!applies("HygonGenuine", 0x18));
    }

    #[test]
    fn every_named_bit_is_listed_once_in_order() {
        assert!(CAUSES.windows(2).all(|pair| pair[0].0 < pair[1].0));
        assert!(CAUSES.iter().all(|&(bit, _)| bit < 32));
    }
}
