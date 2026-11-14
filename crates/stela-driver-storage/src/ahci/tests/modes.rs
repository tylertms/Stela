// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright 2004-2005 Red Hat, Inc.
//! AHCI modes cases.
use super::*;
#[test]
fn error_history_selects_speed_down_actions() {
    // Reference: libata ata_eh_speed_down_verdict.
    let (mut fixture, _clock) = recovering(identify_page("S1", 1024));
    let disk = &mut fixture.disk;
    // Errors `minutes` old, newest last.
    let mut judge = |errors: &[(u32, u32, u64)]| {
        disk.errors = ErrorRing::default();
        let now = disk.clock.try_start().unwrap();
        for &(eflags, mask, minutes) in errors {
            disk.errors.record(eflags, mask, now - minutes * 60_000_000);
        }
        disk.verdict().unwrap()
    };
    let io = EFLAG_IO;
    let dubious = EFLAG_IO | EFLAG_DUBIOUS;
    let spdn = VERDICT_SPEED_DOWN;
    // Two unverified bus errors or timeouts within five minutes.
    assert_eq!(
        judge(&[(dubious, ERR_ATA_BUS, 4), (dubious, ERR_TIMEOUT, 0)]),
        spdn | VERDICT_FALLBACK_TO_PIO | VERDICT_KEEP_ERRORS
    );
    assert_eq!(
        judge(&[(dubious, ERR_ATA_BUS, 6), (dubious, ERR_TIMEOUT, 0)]),
        0
    );
    // A newer verified error makes older errors verified too.
    assert_eq!(
        judge(&[
            (dubious, ERR_ATA_BUS, 2),
            (dubious, ERR_ATA_BUS, 1),
            (io, ERR_HSM, 0)
        ]),
        0
    );
    // Unverified timeouts or device errors turn NCQ off.
    assert_eq!(
        judge(&[(dubious, ERR_DEVICE, 1), (dubious, ERR_HSM, 0)]),
        VERDICT_NCQ_OFF | VERDICT_KEEP_ERRORS
    );
    // More than three bus errors or timeouts within ten minutes.
    let four = [
        (io, ERR_ATA_BUS, 9),
        (0, ERR_TIMEOUT, 7),
        (io, ERR_HSM, 3),
        (io, ERR_ATA_BUS | ERR_DEVICE, 0),
    ];
    assert_eq!(judge(&four), spdn);
    let mut expired = four;
    expired[0].2 = 11;
    assert_eq!(judge(&expired), 0);
    // More than six unknown device errors; HSM and media errors count only
    // for I/O and unknown device errors.
    let mut seven = [(io, ERR_DEVICE, 0); 7];
    assert_eq!(
        judge(&seven),
        spdn | VERDICT_NCQ_OFF | VERDICT_FALLBACK_TO_PIO
    );
    seven[0] = (io, ERR_DEVICE | ERR_MEDIA, 0);
    assert_eq!(judge(&seven), VERDICT_NCQ_OFF);
    seven[0] = (0, ERR_HSM, 0);
    assert_eq!(judge(&seven), VERDICT_NCQ_OFF);
    // A cleared history stops the scan.
    judge(&four[..3]);
    disk.errors.clear();
    let now = disk.clock.try_start().unwrap();
    disk.errors.record(io, ERR_ATA_BUS, now);
    assert_eq!(disk.verdict().unwrap(), 0);
    // The ring keeps the newest 32.
    for _ in 0..40 {
        disk.errors.record(io, ERR_DEVICE, now);
    }
    assert_eq!(disk.errors.newest().count(), ERROR_RING);
}

#[test]
fn identify_capabilities_select_transfer_modes() {
    // Reference: libata ata_id_xfermask and ata_set_mode.
    fn words(pairs: &[(usize, u16)]) -> impl Fn(usize) -> u16 + '_ {
        move |index| {
            pairs
                .iter()
                .find(|(at, _)| *at == index)
                .map_or(0, |(_, value)| *value)
        }
    }
    let modern = Transfer::from_identify(words(&[(53, 0x06), (64, 0x03), (88, 0x7f)]));
    assert_eq!((modern.pio, modern.udma), (0x1f, 0x7f));
    // Without word 64, word 51 names the fastest PIO mode.
    let old = Transfer::from_identify(words(&[(51, 2 << 8)]));
    assert_eq!((old.pio, old.udma), (0x07, 0));
    assert_eq!(Transfer::from_identify(words(&[(51, 7 << 8)])).pio, 1);
    assert!(Transfer::from_identify(words(&[(0, 0x848a)])).compact_flash);
    assert!(Transfer::from_identify(words(&[(83, 0x4004)])).compact_flash);

    // SET FEATURES carries the mode, then IDENTIFY revalidates.
    let (mut fixture, _clock) = recovering(identify_page("S1", 1024));
    fixture.disk.mode = 0;
    let link = Link::new(&[]);
    fixture.disk.set_mode(&link).unwrap();
    assert_eq!(fixture.disk.mode, 0x46);
    assert!(fixture.disk.dubious);
    assert_eq!(link.issued.get(), 2);
    // A refusal is ignored only at the disk's fastest mode.
    let link = Link::new(&[Fault::TaskFile(0x0451)]);
    assert_eq!(fixture.disk.set_mode(&link), Ok(()));
    fixture.disk.udma_mask = 0x1f;
    let link = Link::new(&[Fault::TaskFile(0x0451)]);
    assert!(fixture.disk.set_mode(&link).is_err());
    assert_eq!(fixture.disk.mode, 0x44);
    // No UDMA: libata would use PIO commands these drivers do not issue.
    fixture.disk.udma_mask = 0;
    assert!(matches!(
        fixture.disk.set_mode(&Link::new(&[])),
        Err(BlockError::Unsupported(_))
    ));

    // ata_down_xfermask_limit.
    fixture.disk.pio_mask = 0x1f;
    fixture.disk.udma_mask = 0x7f;
    assert!(fixture.disk.lower_transfer_limit(StepDown::Dma));
    assert_eq!(fixture.disk.udma_mask, 0x3f);
    assert!(fixture.disk.lower_transfer_limit(StepDown::FortyWire));
    assert_eq!(fixture.disk.udma_mask, 0x07);
    assert!(!fixture.disk.lower_transfer_limit(StepDown::FortyWire));
    fixture.disk.udma_mask = 0x01;
    assert!(!fixture.disk.lower_transfer_limit(StepDown::Dma));
    assert!(fixture.disk.lower_transfer_limit(StepDown::Pio));
    assert_eq!(fixture.disk.pio_mask, 0x0f);
    fixture.disk.pio_mask = 0x01;
    assert!(!fixture.disk.lower_transfer_limit(StepDown::Pio));
}

#[test]
fn device_quirks_adjust_transfer_configuration() {
    // Reference: libata ata_dev_configure.
    // An NCQ disk with auto-activation, by model and firmware revision.
    let page = |model: &str, firmware: &str| {
        let mut page = identify_page("S1", 1024);
        for (words, text) in [(IDENTIFY_MODEL, model), (IDENTIFY_FIRMWARE, firmware)] {
            let mut bytes = text.as_bytes().to_vec();
            bytes.resize(2 * words.len(), b' ');
            page[2 * words.start..2 * words.end].copy_from_slice(&bytes);
            for pair in page[2 * words.start..2 * words.end].chunks_mut(2) {
                pair.swap(0, 1);
            }
        }
        for (word, value) in [(76u16, 1u16 << 8 | 0x6), (78, 0x04)] {
            let at = 2 * usize::from(word);
            page[at..at + 2].copy_from_slice(&value.to_le_bytes());
        }
        identify(&page).unwrap()
    };
    let configure = |found: &Identified| {
        let (mut fixture, _clock) = recovering(identify_page("S1", 1024));
        fixture.disk.ncq.host = true;
        (fixture.disk.configure(&Link::new(&[]), found), fixture)
    };

    // Without a quirk: NCQ and auto-activation.
    let (result, fixture) = configure(&page("Fixture SSD", "1.0"));
    assert_eq!(result, Ok(()));
    assert!(fixture.disk.ncq.enabled());
    assert_eq!(fixture.disk.quirks, 0);
    // Seagate's NCQ and FLUSH CACHE firmware bug: neither.
    let (result, fixture) = configure(&page("ST31500341AS", "SD17"));
    assert_eq!(result, Ok(()));
    assert!(!fixture.disk.ncq.enabled());
    assert_eq!(fixture.disk.table.bytes()[2], 0);
    // Broken LPM forces max_power.
    let (_, fixture) = configure(&page("ADATA SU680", "1.0"));
    assert_eq!(fixture.disk.lpm.target(), LPM_MAX_POWER);
    // A 1.5 Gbps limit on a 3 Gbps link takes another reset.
    let (result, fixture) = configure(&page("WD My Book", "1.0"));
    assert_eq!(result, Err(AGAIN));
    assert_eq!(fixture.disk.speed_limit, 0b1);
}

/// ATA strings hold two characters per word, high byte first, padded with
/// spaces on either side or ended by a NUL. The serial the disk is found by
/// is read as the firmware-side chooser reads it (`ascii`): unswapped and
/// trimmed, so both name one disk alike.
#[test]
fn identify_serials_are_unswapped_and_trimmed_of_padding() {
    let serial = |raw: &[u8]| {
        let mut page = identify_page("", 1024);
        let words = IDENTIFY_SERIAL;
        let mut bytes = raw.to_vec();
        bytes.resize(2 * words.len(), b' ');
        for pair in bytes.chunks_mut(2) {
            pair.swap(0, 1);
        }
        page[2 * words.start..2 * words.end].copy_from_slice(&bytes);
        identify(&page).unwrap().serial
    };
    // Twenty characters, right-justified as many disks report them.
    assert_eq!(serial(b"           WD-ABC123"), "WD-ABC123");
    assert_eq!(serial(b"00000000000042"), "00000000000042");
    // An odd length splits the last word between a character and padding.
    assert_eq!(serial(b"  ABC12"), "ABC12");
    assert_eq!(serial(b"XYZ27\0garbage"), "XYZ27");
    assert_eq!(serial(b""), "");
}

#[test]
fn identify_word_217_reports_whether_the_disk_spins() {
    // Reference: ata_id_rotation_rate and ata_id_is_ssd.
    let mut page = identify_page("S1", 1024);
    assert_eq!(identify(&page).unwrap().rotation, Rotation::Unreported);
    for (word, rotation) in [
        (1, Rotation::None),
        (5400, Rotation::Rpm(5400)),
        (0xffff, Rotation::Unreported),
    ] {
        page[2 * 217..2 * 217 + 2].copy_from_slice(&u16::to_le_bytes(word));
        assert_eq!(identify(&page).unwrap().rotation, rotation, "{word:#x}");
    }
}
