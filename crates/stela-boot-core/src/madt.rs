// SPDX-License-Identifier: GPL-2.0-only
// SCI conforming electrical defaults adapted from Linux arch/x86/kernel/acpi/boot.c,
// Copyright (C) 2001, 2002 Paul Diefenbaugh; (C) 2001 Jun Nakajima.
//! Bounded ACPI line routing; CPU enumeration/bootstrap belongs to its owner.
//! SCI conforming electrical flags mean active-low/level, rather than ISA's
//! active-high/edge defaults. No bridge/PIRQ routing is inferred from a MADT.
use alloc::vec::Vec;
use stela_drivers::bus::irq::{LineRoute, Polarity, Trigger};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RouteError {
    InvalidTable,
    InvalidStructure,
    InvalidAddress,
    Duplicate,
    MissingUnit,
    NmiConflict,
    UnsupportedSci,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Unit {
    pub id: u8,
    pub base: u64,
    pub gsi_base: u32,
}
#[derive(Clone, Copy, Debug)]
struct Override {
    source: u8,
    gsi: u32,
    flags: u16,
}

pub struct Routing {
    pub units: Vec<Unit>,
    overrides: Vec<Override>,
    nmis: Vec<u32>,
    legacy_pic: bool,
    /// Each entry type the routing does not use, once, in table order:
    /// other architectures', the multiprocessor wakeup (ACPI 6.4 5.2.12.19)
    /// and OEM types, which Linux's `acpi_parse_entries_array` skips too.
    pub ignored: Vec<u8>,
    /// Each kind of defect the routing carried on past, once, in table
    /// order, as Linux does.
    pub tolerated: Vec<Tolerated>,
}

/// A MADT defect that Linux carries on past, and the routing too.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Tolerated {
    /// Flags other than PC-AT compatibility are set; they are ignored.
    ReservedFlags,
    /// An entry is longer than its type's structure; the rest is ignored
    /// (`BAD_MADT_ENTRY` refuses only shorter ones).
    LongEntry(u8),
    /// An IOAPIC entry's reserved byte is set.
    IoapicReserved,
    /// Two IOAPICs give the same ID. Linux on x86-64 numbers the second
    /// again; units here are told apart by address and GSI base.
    DuplicateIoapicId(u8),
    /// An IOAPIC address is listed again; the repeat is ignored, as
    /// Linux's `mp_register_ioapic` ignores it.
    RepeatedIoapic(u64),
    /// An interrupt source override names a bus other than ISA's 0, which
    /// Linux does not read.
    OverrideBus,
    /// An override's source is not an ISA IRQ; it is ignored.
    OverrideSource(u8),
    /// An ISA IRQ has a second override; the first is kept.
    RepeatedOverride(u8),
    /// A reserved polarity or trigger mode means active low or level, as
    /// Linux's `irq_polarity` and `irq_trigger` take it.
    ReservedElectrical,
}

impl Tolerated {
    /// A plain sentence for the boot log.
    pub fn message(self) -> alloc::string::String {
        match self {
            Self::ReservedFlags => "has reserved flags set; they are ignored".into(),
            Self::LongEntry(kind) => {
                alloc::format!(
                    "has a type {kind} entry longer than its structure; the rest is ignored"
                )
            }
            Self::IoapicReserved => "has an IOAPIC entry with its reserved byte set".into(),
            Self::DuplicateIoapicId(id) => {
                alloc::format!("gives IOAPIC ID {id} twice; both units are kept by address")
            }
            Self::RepeatedIoapic(base) => {
                alloc::format!("lists the IOAPIC at {base:#x} twice; the repeat is ignored")
            }
            Self::OverrideBus => "has an interrupt override for a bus other than ISA".into(),
            Self::OverrideSource(source) => {
                alloc::format!("overrides IRQ {source}, which is not an ISA IRQ; it is ignored")
            }
            Self::RepeatedOverride(source) => {
                alloc::format!("overrides ISA IRQ {source} twice; the first is kept")
            }
            Self::ReservedElectrical => {
                "gives a reserved polarity or trigger mode; it means active low or level".into()
            }
        }
    }
}
fn word(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes(b[o..o + 2].try_into().unwrap())
}
fn dword(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
fn table<'a>(b: &'a [u8], signature: &[u8; 4], minimum: usize) -> Result<&'a [u8], RouteError> {
    if b.len() < minimum || &b[..4] != signature {
        return Err(RouteError::InvalidTable);
    }
    let length = dword(b, 4) as usize;
    if length < minimum || length > b.len() {
        return Err(RouteError::InvalidTable);
    }
    // A wrong checksum is reported where the table is found
    // (`acpi::on_checksum_fault`) and, as in Linux, the table is used.
    Ok(&b[..length])
}
/// Whether override `flags` use a reserved polarity or trigger mode.
fn reserved_electrical(flags: u16) -> bool {
    flags & 3 == 2 || (flags >> 2) & 3 == 2
}
/// Bits above the trigger mode are reserved and ignored.
fn electrical(flags: u16, sci: bool) -> (Polarity, Trigger) {
    let default = if sci {
        (Polarity::Low, Trigger::Level)
    } else {
        (Polarity::High, Trigger::Edge)
    };
    let polarity = match flags & 3 {
        0 => default.0,
        1 => Polarity::High,
        _ => Polarity::Low,
    };
    let trigger = match (flags >> 2) & 3 {
        0 => default.1,
        1 => Trigger::Edge,
        _ => Trigger::Level,
    };
    (polarity, trigger)
}

impl Routing {
    pub fn parse(madt: &[u8], physical_bits: u8) -> Result<Self, RouteError> {
        if !(32..=52).contains(&physical_bits) {
            return Err(RouteError::InvalidAddress);
        }
        let madt = table(madt, b"APIC", 44)?;
        // The PC/AT compatibility flag is the only currently defined bit.
        let flags = dword(madt, 40);
        let mut routing = Self {
            units: Vec::new(),
            overrides: Vec::new(),
            nmis: Vec::new(),
            legacy_pic: flags & 1 != 0,
            ignored: Vec::new(),
            tolerated: Vec::new(),
        };
        if flags & !1 != 0 {
            routing.tolerate(Tolerated::ReservedFlags);
        }
        let mut lapic_override = false;
        let mut offset = 44;
        while offset < madt.len() {
            if madt.len() - offset < 2 {
                return Err(RouteError::InvalidStructure);
            }
            let length = usize::from(madt[offset + 1]);
            if length < 2 || length > madt.len() - offset {
                return Err(RouteError::InvalidStructure);
            }
            let entry = &madt[offset..offset + length];
            let kind = entry[0];
            // Linux's `BAD_MADT_ENTRY` refuses only an entry shorter than
            // its structure; the rest of a longer one is ignored.
            let base_length = match kind {
                0 | 3 => Some(8),
                1 | 5 | 10 => Some(12),
                2 => Some(10),
                4 => Some(6),
                9 => Some(16),
                _ => None,
            };
            match base_length {
                Some(base) if length < base => return Err(RouteError::InvalidStructure),
                Some(base) if length > base => routing.tolerate(Tolerated::LongEntry(kind)),
                _ => {}
            }
            match kind {
                1 => {
                    let base = u64::from(dword(entry, 4));
                    if base == 0
                        || base & 4095 != 0
                        || base
                            .checked_add(4096)
                            .is_none_or(|end| end > 1u64 << physical_bits)
                    {
                        return Err(RouteError::InvalidAddress);
                    }
                    if entry[3] != 0 {
                        routing.tolerate(Tolerated::IoapicReserved);
                    }
                    let unit = Unit {
                        id: entry[2],
                        base,
                        gsi_base: dword(entry, 8),
                    };
                    if routing.units.iter().any(|u| u.base == base) {
                        routing.tolerate(Tolerated::RepeatedIoapic(base));
                    } else if routing.units.iter().any(|u| u.gsi_base == unit.gsi_base) {
                        // Which unit owns those GSIs is unknown, and a unit
                        // left out would stay uncontained.
                        return Err(RouteError::Duplicate);
                    } else {
                        if routing.units.iter().any(|u| u.id == unit.id) {
                            routing.tolerate(Tolerated::DuplicateIoapicId(unit.id));
                        }
                        routing.units.push(unit);
                    }
                }
                2 => {
                    let flags = word(entry, 8);
                    if reserved_electrical(flags) {
                        routing.tolerate(Tolerated::ReservedElectrical);
                    }
                    if entry[2] != 0 {
                        routing.tolerate(Tolerated::OverrideBus);
                    }
                    let value = Override {
                        source: entry[3],
                        gsi: dword(entry, 4),
                        flags,
                    };
                    if value.source >= 16 {
                        routing.tolerate(Tolerated::OverrideSource(value.source));
                    } else if routing
                        .overrides
                        .iter()
                        .any(|old| old.source == value.source)
                    {
                        routing.tolerate(Tolerated::RepeatedOverride(value.source));
                    } else {
                        // Two ISA IRQs overridden onto one GSI refuse only
                        // those two lines (`Routing::line`).
                        routing.overrides.push(value);
                    }
                }
                // Linux's `acpi_parse_nmi_src` reads nothing from an NMI
                // source; its pin stays masked like any unclaimed one. Here
                // the GSI is kept only so that no line is ever routed to it,
                // so its flags and repeats change nothing.
                3 => {
                    let gsi = dword(entry, 4);
                    if !routing.nmis.contains(&gsi) {
                        routing.nmis.push(gsi);
                    }
                }
                // Validate lengths, but never apply or elect processors here.
                0 | 4 | 9 | 10 => {}
                5 => {
                    let address = u64::from_le_bytes(entry[4..12].try_into().unwrap());
                    if lapic_override
                        || word(entry, 2) != 0
                        || address == 0
                        || address & 4095 != 0
                        || address
                            .checked_add(4096)
                            .is_none_or(|end| end > 1u64 << physical_bits)
                    {
                        return Err(RouteError::InvalidAddress);
                    }
                    lapic_override = true;
                }
                kind => {
                    if !routing.ignored.contains(&kind) {
                        routing.ignored.push(kind);
                    }
                }
            }
            offset += length;
        }
        if routing.units.is_empty() {
            return Err(RouteError::MissingUnit);
        }
        Ok(routing)
    }

    fn tolerate(&mut self, defect: Tolerated) {
        let seen = self
            .tolerated
            .iter()
            .any(|old| core::mem::discriminant(old) == core::mem::discriminant(&defect));
        if !seen {
            self.tolerated.push(defect);
        }
    }

    pub fn isa(&self, irq: u8) -> Result<LineRoute, RouteError> {
        if irq >= 16 {
            return Err(RouteError::InvalidStructure);
        }
        self.line(u32::from(irq), false)
    }
    pub fn sci(&self, fadt: &[u8]) -> Result<LineRoute, RouteError> {
        let fadt = table(fadt, b"FACP", 116)?;
        // HW_REDUCED_ACPI removes the legacy fixed hardware, including SCI;
        // a nonzero legacy SCI_INT field cannot authorize that interface.
        if dword(fadt, 112) & (1 << 20) != 0 {
            return Err(RouteError::UnsupportedSci);
        }
        let irq = u32::from(word(fadt, 46));
        if irq == 0 || irq == u32::from(u16::MAX) {
            return Err(RouteError::UnsupportedSci);
        }
        let line = self.line(irq, true)?;
        if line.trigger != Trigger::Level {
            return Err(RouteError::UnsupportedSci);
        }
        Ok(line)
    }
    fn line(&self, irq: u32, sci: bool) -> Result<LineRoute, RouteError> {
        let override_ = self.overrides.iter().find(|o| u32::from(o.source) == irq);
        let (gsi, flags) = override_.map_or((irq, 0), |o| (o.gsi, o.flags));
        if self.nmis.contains(&gsi) {
            return Err(RouteError::NmiConflict);
        }
        if self
            .overrides
            .iter()
            .any(|o| o.gsi == gsi && u32::from(o.source) != irq)
        {
            return Err(RouteError::Duplicate);
        }
        let (polarity, trigger) = electrical(flags, sci);
        Ok(LineRoute {
            gsi,
            polarity,
            trigger,
        })
    }

    pub fn has_legacy_pic(&self) -> bool {
        self.legacy_pic
    }

    /// The GSIs MADT NMI sources name, each once. No line is routed to them.
    pub fn ioapic_nmis(&self) -> &[u32] {
        &self.nmis
    }

    /// Bind IOAPIC version-reported pin counts, never a guessed 24-pin
    /// span. Every MADT unit must be present exactly once, with disjoint GSIs.
    pub fn validate_units(&self, actual: &[(Unit, u16)]) -> Result<(), RouteError> {
        if actual.len() != self.units.len() {
            return Err(RouteError::MissingUnit);
        }
        for (index, (unit, pins)) in actual.iter().enumerate() {
            if !self.units.contains(unit) || *pins == 0 || *pins > 120 {
                return Err(RouteError::MissingUnit);
            }
            let end = unit
                .gsi_base
                .checked_add(u32::from(*pins))
                .ok_or(RouteError::InvalidAddress)?;
            for (other, other_pins) in &actual[..index] {
                let other_end = other
                    .gsi_base
                    .checked_add(u32::from(*other_pins))
                    .ok_or(RouteError::InvalidAddress)?;
                if unit.gsi_base < other_end && other.gsi_base < end {
                    return Err(RouteError::Duplicate);
                }
            }
        }
        Ok(())
    }

    pub fn owner(&self, line: LineRoute, actual: &[(Unit, u16)]) -> Result<Unit, RouteError> {
        self.validate_units(actual)?;
        if self.nmis.contains(&line.gsi) {
            return Err(RouteError::NmiConflict);
        }
        actual
            .iter()
            .find(|(u, pins)| line.gsi >= u.gsi_base && line.gsi - u.gsi_base < u32::from(*pins))
            .map(|(u, _)| *u)
            .ok_or(RouteError::MissingUnit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    fn finish(mut b: Vec<u8>, sig: &[u8; 4]) -> Vec<u8> {
        b[..4].copy_from_slice(sig);
        let length = b.len() as u32;
        b[4..8].copy_from_slice(&length.to_le_bytes());
        b[9] = 0;
        b[9] = 0u8.wrapping_sub(b.iter().fold(0u8, |s, x| s.wrapping_add(*x)));
        b
    }
    fn madt(extra: &[u8]) -> Vec<u8> {
        let mut b = vec![0; 44];
        b.extend_from_slice(&[1, 12, 0, 0, 0, 0, 0xc0, 0xfe, 0, 0, 0, 0]);
        b.extend_from_slice(extra);
        finish(b, b"APIC")
    }
    fn fadt(irq: u16) -> Vec<u8> {
        let mut b = vec![0; 116];
        b[46..48].copy_from_slice(&irq.to_le_bytes());
        finish(b, b"FACP")
    }
    #[test]
    fn wakeup_and_oem_entries_are_skipped_once_each() {
        let plain = Routing::parse(&madt(&[]), 48).unwrap();
        // A multiprocessor wakeup entry (type 0x10, 16 bytes), two OEM
        // entries of one type and one of another.
        let mut extra = vec![0x10, 16];
        extra.extend_from_slice(&[0; 14]);
        extra.extend_from_slice(&[0x80, 4, 1, 2, 0x80, 2, 0xff, 3, 0]);
        let skipped = Routing::parse(&madt(&extra), 48).unwrap();
        assert_eq!(skipped.ignored, [0x10, 0x80, 0xff]);
        assert_eq!(skipped.units, plain.units);
        assert_eq!(skipped.isa(9), plain.isa(9));
        // A known type of the wrong length is still malformed.
        assert_eq!(
            Routing::parse(&madt(&[2, 4, 0, 9]), 48).err(),
            Some(RouteError::InvalidStructure)
        );
        // So is an entry running past the table.
        assert_eq!(
            Routing::parse(&madt(&[0x80, 9, 0]), 48).err(),
            Some(RouteError::InvalidStructure)
        );
    }
    #[test]
    fn pc_at_compatibility_reports_the_independent_legacy_pic() {
        let mut table = madt(&[]);
        assert!(!Routing::parse(&table, 48).unwrap().has_legacy_pic());
        table[40] = 1;
        assert!(
            Routing::parse(&finish(table, b"APIC"), 48)
                .unwrap()
                .has_legacy_pic()
        );
    }
    #[test]
    fn sci_override_conform_means_low_level_while_isa_is_high_edge() {
        let r = Routing::parse(&madt(&[2, 10, 0, 9, 11, 0, 0, 0, 0, 0]), 48).unwrap();
        assert_eq!(
            r.sci(&fadt(9)),
            Ok(LineRoute {
                gsi: 11,
                polarity: Polarity::Low,
                trigger: Trigger::Level
            })
        );
        assert_eq!(
            r.isa(9),
            Ok(LineRoute {
                gsi: 11,
                polarity: Polarity::High,
                trigger: Trigger::Edge
            })
        );
        assert_eq!(r.sci(&fadt(20)).unwrap().gsi, 20);
        let explicit = Routing::parse(&madt(&[2, 10, 0, 9, 9, 0, 0, 0, 15, 0]), 48).unwrap();
        assert_eq!(explicit.sci(&fadt(9)).unwrap().trigger, Trigger::Level);
        let high = Routing::parse(&madt(&[2, 10, 0, 9, 9, 0, 0, 0, 13, 0]), 48).unwrap();
        assert_eq!(
            high.sci(&fadt(9)),
            Ok(LineRoute {
                gsi: 9,
                polarity: Polarity::High,
                trigger: Trigger::Level
            })
        );
        let alias = Routing::parse(&madt(&[2, 10, 0, 5, 9, 0, 0, 0, 13, 0]), 48).unwrap();
        assert_eq!(alias.sci(&fadt(9)), Err(RouteError::Duplicate));
        let edge = Routing::parse(&madt(&[2, 10, 0, 9, 9, 0, 0, 0, 5, 0]), 48).unwrap();
        assert_eq!(edge.sci(&fadt(9)), Err(RouteError::UnsupportedSci));
    }
    #[test]
    fn hardware_reduced_fadt_never_admits_legacy_sci_even_with_nonzero_interrupt() {
        let routing = Routing::parse(&madt(&[]), 48).unwrap();
        let legacy = LineRoute {
            gsi: 9,
            polarity: Polarity::Low,
            trigger: Trigger::Level,
        };
        for flags in [0u32, 1 << 19, 1 << 21, 1 << 20, (1 << 20) | (1 << 21)] {
            let mut input = fadt(9);
            input.resize(276, 0);
            input[8] = 6;
            input[112..116].copy_from_slice(&flags.to_le_bytes());
            let input = finish(input, b"FACP");
            assert_eq!(
                input.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)),
                0
            );
            assert_eq!(word(&input, 46), 9);
            let expected = if flags & (1 << 20) != 0 {
                Err(RouteError::UnsupportedSci)
            } else {
                Ok(legacy)
            };
            assert_eq!(routing.sci(&input), expected, "FADT flags {flags:#x}");
        }
    }

    #[test]
    fn short_entries_and_nmi_are_refused() {
        for extra in [&[2, 0][..], &[2, 11, 0, 0][..], &[4, 5, 0, 0, 0][..]] {
            assert!(Routing::parse(&madt(extra), 48).is_err());
        }
        let r = Routing::parse(&madt(&[3, 8, 15, 0, 9, 0, 0, 0]), 48).unwrap();
        assert_eq!(r.sci(&fadt(9)), Err(RouteError::NmiConflict));
        assert_eq!(r.ioapic_nmis(), [9]);
        let line = LineRoute {
            gsi: 9,
            polarity: Polarity::Low,
            trigger: Trigger::Level,
        };
        assert_eq!(
            r.owner(line, &[(r.units[0], 24)]),
            Err(RouteError::NmiConflict)
        );
        assert_eq!(r.isa(4).unwrap().gsi, 4);
    }
    #[test]
    fn defects_linux_carries_on_past_keep_the_routing() {
        let tolerated = |extra: &[u8]| Routing::parse(&madt(extra), 48).unwrap().tolerated;
        // A wrong checksum and reserved table flags.
        let mut odd = madt(&[]);
        odd[9] ^= 1;
        odd[40] = 0x81;
        let r = Routing::parse(&odd, 48).unwrap();
        assert_eq!(r.tolerated, [Tolerated::ReservedFlags]);
        assert!(r.has_legacy_pic());
        // Reserved polarity and trigger mean low and level; reserved bits
        // above them are ignored.
        let r = Routing::parse(&madt(&[2, 10, 0, 9, 9, 0, 0, 0, 0x0a, 0x80]), 48).unwrap();
        assert_eq!(r.tolerated, [Tolerated::ReservedElectrical]);
        let line = r.isa(9).unwrap();
        assert_eq!(
            (line.polarity, line.trigger),
            (Polarity::Low, Trigger::Level)
        );
        // An override for another bus or a non-ISA source, and a repeat.
        let first = [2, 10, 1, 9, 20, 0, 0, 0, 0, 0];
        let mut repeated = first.to_vec();
        repeated.extend_from_slice(&[2, 10, 0, 9, 21, 0, 0, 0, 0, 0]);
        repeated.extend_from_slice(&[2, 10, 0, 40, 22, 0, 0, 0, 0, 0]);
        let r = Routing::parse(&madt(&repeated), 48).unwrap();
        assert_eq!(
            r.tolerated,
            [
                Tolerated::OverrideBus,
                Tolerated::RepeatedOverride(9),
                Tolerated::OverrideSource(40),
            ]
        );
        assert_eq!(r.isa(9).unwrap().gsi, 20);
        // Entries longer than their structure, and an IOAPIC's reserved byte.
        let mut long = vec![2, 12, 0, 4, 4, 0, 0, 0, 0, 0, 0xee, 0xee];
        long.extend_from_slice(&[1, 14, 1, 7, 0, 0, 0xd0, 0xfe, 24, 0, 0, 0, 0, 0]);
        let r = Routing::parse(&madt(&long), 48).unwrap();
        assert_eq!(
            r.tolerated,
            [Tolerated::LongEntry(2), Tolerated::IoapicReserved]
        );
        assert_eq!(r.units.len(), 2);
        // A repeated IOAPIC address, and a second unit with the same ID.
        assert_eq!(
            tolerated(&[1, 12, 0, 0, 0, 0, 0xc0, 0xfe, 0, 0, 0, 0]),
            [Tolerated::RepeatedIoapic(0xfec0_0000)]
        );
        assert_eq!(
            tolerated(&[1, 12, 0, 0, 0, 0, 0xd0, 0xfe, 24, 0, 0, 0]),
            [Tolerated::DuplicateIoapicId(0)]
        );
        // Two units claiming one GSI base still refuse the table: a unit
        // left out would stay uncontained.
        assert_eq!(
            Routing::parse(&madt(&[1, 12, 1, 0, 0, 0, 0xd0, 0xfe, 0, 0, 0, 0]), 48).err(),
            Some(RouteError::Duplicate)
        );
    }
    #[test]
    fn nmi_sources_with_odd_flags_or_repeats_only_withhold_their_pin() {
        // Reserved flag bits, a reserved polarity, and the same GSI twice.
        let extra = [3, 8, 0xf2, 0xff, 9, 0, 0, 0, 3, 8, 0, 0, 9, 0, 0, 0];
        let r = Routing::parse(&madt(&extra), 48).unwrap();
        assert_eq!(r.ioapic_nmis(), [9]);
        assert_eq!(r.sci(&fadt(9)), Err(RouteError::NmiConflict));
        assert_eq!(r.isa(4).unwrap().gsi, 4);
    }
    #[test]
    fn ownership_uses_live_pin_counts_and_refuses_missing_duplicate_or_overlapping_units() {
        let r = Routing::parse(&madt(&[]), 48).unwrap();
        let u = r.units[0];
        let line = r.sci(&fadt(9)).unwrap();
        assert_eq!(r.owner(line, &[(u, 24)]), Ok(u));
        assert_eq!(r.owner(line, &[]), Err(RouteError::MissingUnit));
        assert_eq!(r.owner(line, &[(u, 8)]), Err(RouteError::MissingUnit));
        assert_eq!(r.owner(line, &[(u, 121)]), Err(RouteError::MissingUnit));
        let mut second = [1, 12, 1, 0, 0, 16, 0xc0, 0xfe, 16, 0, 0, 0];
        let r = Routing::parse(&madt(&second), 48).unwrap();
        assert_eq!(
            r.owner(line, &[(r.units[0], 24), (r.units[1], 24)]),
            Err(RouteError::Duplicate)
        );
        second[8] = 24;
        let r = Routing::parse(&madt(&second), 48).unwrap();
        assert_eq!(
            r.owner(line, &[(r.units[0], 24), (r.units[0], 24)]),
            Err(RouteError::Duplicate)
        );
        assert_eq!(
            r.owner(line, &[(r.units[0], 24), (r.units[1], 24)]),
            Ok(r.units[0])
        );
    }
}
