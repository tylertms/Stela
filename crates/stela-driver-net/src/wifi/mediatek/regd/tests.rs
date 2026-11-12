use super::*;
use crate::wifi::regulatory::database::tests::country_of;
use alloc::vec;

/// The United States' rules in regulatory.db's shape, 5.9 GHz indoors.
fn us() -> Country {
    country_of(&[
        (2_402_000, 2_472_000, 40_000, 3000, 0),
        (5_170_000, 5_250_000, 80_000, 2300, 0),
        (5_730_000, 5_850_000, 80_000, 3000, 0),
        (5_850_000, 5_895_000, 40_000, 2700, 2),
        (5_925_000, 7_125_000, 160_000, 2400, 2),
    ])
}

fn planned(configuration: u8, eht_control: Option<&[u8]>, sar: &Sar) -> Plan {
    Plan::new(Inputs {
        chip: Chip::Mt7925,
        country: &us(),
        bands: [true; 3],
        channel_configuration: configuration,
        eht_control,
        sar,
    })
    .unwrap()
}

fn values(channels: &[domain::Channel]) -> Vec<u16> {
    channels.iter().map(|c| c.hardware_value).collect()
}

fn power(rows: Option<&[power::Channel]>, value: u8) -> i8 {
    rows.unwrap()
        .iter()
        .find(|row| row.hardware_value == value)
        .unwrap()
        .limits
        .cck[0]
}

#[test]
fn the_domain_holds_each_band_s_regulated_channels() {
    let plan = planned(0x1f, None, &Sar::default());
    assert_eq!(
        plan.default_channel(),
        Some(Channel {
            band: Band::Ghz2,
            number: 1,
            frequency_mhz: 2412
        })
    );
    let inputs = plan.inputs();
    assert_eq!(inputs.channel_configuration, 0x1f);
    assert_eq!(
        values(inputs.domain.channels_2ghz),
        (1..=11).collect::<Vec<_>>()
    );
    assert_eq!(
        values(inputs.domain.channels_5ghz),
        [36, 40, 44, 48, 149, 153, 157, 161, 165, 169, 173, 177]
    );
    assert_eq!(inputs.domain.channels_6ghz.len(), 59);
    // Channel 169 straddles the 5.8 and 5.9 GHz rules.
    let c169 = inputs.domain.channels_5ghz[9];
    assert_eq!(
        c169.flags,
        channel::NO_80MHZ | channel::NO_160MHZ | channel::NO_320MHZ | channel::INDOOR_ONLY
    );
    assert_eq!(
        inputs.domain.channels_2ghz[0].flags,
        channel::NO_HT40MINUS | channel::NO_80MHZ | channel::NO_160MHZ | channel::NO_320MHZ
    );
}

#[test]
fn rate_power_takes_the_regulatory_limit_of_enabled_channels() {
    let plan = planned(0x1f, None, &Sar::default());
    let inputs = plan.inputs();
    for (value, expected) in [(36, 46), (38, 127), (52, 127), (165, 60)] {
        assert_eq!(power(inputs.power.ghz5, value), expected, "channel {value}");
    }
    assert_eq!(power(inputs.power.ghz2, 1), 60);
    assert_eq!(power(inputs.power.ghz2, 14), 127);
    assert_eq!(power(inputs.power.ghz6, 1), 48);
}

#[test]
fn the_clc_channel_configuration_disables_unii_bands() {
    let plan = planned(0x0f, None, &Sar::default());
    let inputs = plan.inputs();
    assert_eq!(*values(inputs.domain.channels_5ghz).last().unwrap(), 177);
    // UNII-8 from 6875 MHz, channel 185, is off.
    assert_eq!(*values(inputs.domain.channels_6ghz).last().unwrap(), 181);
    let plan = planned(0, None, &Sar::default());
    let inputs = plan.inputs();
    assert_eq!(*values(inputs.domain.channels_5ghz).last().unwrap(), 165);
    assert!(inputs.domain.channels_6ghz.is_empty());
    assert!(inputs.power.ghz6.is_none());
    assert!(inputs.power.ghz5.is_some());
}

#[test]
fn mtcl_masks_the_configuration_and_eht() {
    // An MTCL v2 naming no country.
    let mut mtcl = vec![0; 19];
    mtcl[4] = 2;
    let sar = Sar::new(Some(mtcl), None, None);
    let plan = planned(0x1f, None, &sar);
    let inputs = plan.inputs();
    assert_eq!(inputs.channel_configuration, 0);
    assert!(inputs.domain.channels_6ghz.is_empty());
    assert!(inputs.power.ghz6.is_none());
    assert!(
        inputs
            .domain
            .channels_2ghz
            .iter()
            .all(|c| c.flags & channel::NO_EHT == 0),
        "v2 tables leave EHT on"
    );
}

#[test]
fn the_be_ctrl_record_turns_eht_off_for_its_countries() {
    let mut rules = [0; 32];
    rules[4..6].copy_from_slice(b"CA");
    rules[16..20].copy_from_slice(&1u32.to_le_bytes());
    rules[20..22].copy_from_slice(b"US");
    for (control, eht) in [
        (None, true),
        (Some(&rules[..]), false),
        (Some(&rules[..16]), true),
    ] {
        let plan = planned(0x1f, control, &Sar::default());
        let inputs = plan.inputs();
        for band in [
            inputs.domain.channels_2ghz,
            inputs.domain.channels_5ghz,
            inputs.domain.channels_6ghz,
        ] {
            assert!(band.iter().all(|c| (c.flags & channel::NO_EHT == 0) == eht));
        }
    }
}

#[test]
fn acpi_sar_caps_each_range() {
    // An enabled MTDS v1 limiting 2.4 GHz to 10 dBm.
    let mut mtds = vec![0; 12];
    mtds[4] = 1;
    mtds[5] = 1;
    mtds[7..12].copy_from_slice(&[20, 0xff, 0xff, 0xff, 0xff]);
    let plan = planned(0x1f, None, &Sar::new(None, Some(mtds), None));
    let inputs = plan.inputs();
    assert_eq!(power(inputs.power.ghz2, 1), 20);
    assert_eq!(power(inputs.power.ghz5, 36), 46);
}

#[test]
fn a_missing_band_stops_the_channel_update_as_the_source_does() {
    let plan = Plan::new(Inputs {
        chip: Chip::Mt7925,
        country: &us(),
        bands: [false, true, true],
        channel_configuration: 0,
        eht_control: None,
        sar: &Sar::default(),
    })
    .unwrap();
    assert_eq!(
        plan.default_channel(),
        Some(Channel {
            band: Band::Ghz5,
            number: 36,
            frequency_mhz: 5180
        })
    );
    let inputs = plan.inputs();
    assert!(inputs.domain.channels_2ghz.is_empty());
    assert_eq!(*values(inputs.domain.channels_5ghz).last().unwrap(), 177);
    assert_eq!(inputs.domain.channels_6ghz.len(), 59);
    assert!(inputs.power.ghz2.is_none());
}

#[test]
fn connac2_follows_mt7921_regd_update() {
    let mut rules = [0; 16];
    rules[..4].copy_from_slice(&1u32.to_le_bytes());
    rules[4..6].copy_from_slice(b"US");
    for chip in [Chip::Mt7921, Chip::Mt7922] {
        // No 2.4 GHz band: mt7921_regd_channel_update still drops UNII-4
        // and UNII-5 to 8 without their CLC bits; a BE_CTRL rule never
        // marks NO_EHT.
        let plan = Plan::new(Inputs {
            chip,
            country: &us(),
            bands: [false, true, true],
            channel_configuration: 0,
            eht_control: Some(&rules),
            sar: &Sar::default(),
        })
        .unwrap();
        let inputs = plan.inputs();
        assert_eq!(*values(inputs.domain.channels_5ghz).last().unwrap(), 165);
        assert!(inputs.domain.channels_6ghz.is_empty());
        assert!(
            inputs
                .domain
                .channels_5ghz
                .iter()
                .all(|c| c.flags & channel::NO_EHT == 0)
        );
        // Every band's rate power, 6 GHz without a CLC configuration too,
        // over connac2's channel lists.
        assert!(inputs.power.ghz2.is_none());
        for (rows, band) in [
            (inputs.power.ghz5, Band::Ghz5),
            (inputs.power.ghz6, Band::Ghz6),
        ] {
            let values: Vec<u8> = rows.unwrap().iter().map(|r| r.hardware_value).collect();
            assert_eq!(values, power::channels(chip, band));
        }
        assert_eq!(power(inputs.power.ghz5, 36), 46);
        // Disabled 6 GHz channels keep the unregulated start.
        assert_eq!(power(inputs.power.ghz6, 1), 127);
    }
}
