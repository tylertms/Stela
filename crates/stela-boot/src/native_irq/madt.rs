//! ACPI interrupt routing shared with the hosted boot-core tests.
pub use stela_boot_core::madt::*;

use core::sync::atomic::{AtomicBool, Ordering};

static IGNORED_REPORTED: AtomicBool = AtomicBool::new(false);
static NMIS_REPORTED: AtomicBool = AtomicBool::new(false);
static TOLERATED_REPORTED: AtomicBool = AtomicBool::new(false);

/// Parses the MADT and reports the entry types the routing skips, the
/// defects it carries on past and the NMI source pins it never routes,
/// each once per boot.
pub fn parse(madt: &[u8], physical_bits: u8) -> Result<Routing, RouteError> {
    let routing = Routing::parse(madt, physical_bits)?;
    if !routing.tolerated.is_empty() && !TOLERATED_REPORTED.swap(true, Ordering::Relaxed) {
        for defect in &routing.tolerated {
            crate::output::arguments(format_args!("FIRMWARE MADT {}\n", defect.message()));
        }
    }
    if !routing.ignored.is_empty() && !IGNORED_REPORTED.swap(true, Ordering::Relaxed) {
        crate::output::arguments(format_args!(
            "UNSUPPORTED HOST MADT entry types {:#04x?} skipped\n",
            routing.ignored
        ));
    }
    if !routing.ioapic_nmis().is_empty() && !NMIS_REPORTED.swap(true, Ordering::Relaxed) {
        crate::output::arguments(format_args!(
            "UNSUPPORTED HOST MADT NMI sources at GSIs {:?}: pins stay masked and unrouted\n",
            routing.ioapic_nmis()
        ));
    }
    Ok(routing)
}
