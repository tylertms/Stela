// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Exact initial source command bodies and effective response semantics.
//! Generic completion has no invented status. C2 raw-peer updates use the
//! station-result decoder; C3 WMWA updates keep generic facts and
//! cannot validate per-peer header format or imply association.
use super::{Encoder, Event, runtime};
use crate::wifi::{
    Band,
    mediatek::{
        identity::Chip,
        mcu::device::{self, Context},
    },
};
use alloc::vec::Vec;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Kind {
    Restart,
    FirmwareLog,
    Scan,
    CancelScan,
    ClientFilter,
    Roc,
    EfuseMode,
    RtsThreshold,
    MacEnable(bool),
    RxPath,
    RadioLed(u8),
    RawTranslation {
        wcid: u16,
    },
    ChipConfig,
    /// connac2's CE SET_EDCA_PARMS.
    Edca,
    /// connac2's CE SET_BSS_ABORT.
    BssAbort,
    /// connac2's CE RSSI_MONITOR.
    RssiMonitor,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    WrongChip,
    Context(device::Error),
    InvalidWcid,
    InvalidChannel,
    InvalidAntenna,
    OneWay,
    Event(super::Error),
    Runtime(runtime::Error),
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek initial command: {self:?}")
    }
}
impl core::error::Error for Error {}
pub(in crate::wifi::mediatek) fn validate_channel(
    band: Band,
    channel: u8,
    frequency_mhz: u16,
) -> Result<(), Error> {
    let actual = match band {
        Band::Ghz2 => {
            if channel == 14 {
                2484
            } else if (1..14).contains(&channel) {
                2407 + u16::from(channel) * 5
            } else {
                return Err(Error::InvalidChannel);
            }
        }
        Band::Ghz5 => 5000 + u16::from(channel) * 5,
        Band::Ghz6 => {
            if channel == 2 {
                5935
            } else {
                5950 + u16::from(channel) * 5
            }
        }
    };
    if channel == 0 || actual != frequency_mhz {
        return Err(Error::InvalidChannel);
    }
    Ok(())
}
pub struct Request {
    chip: Chip,
    kind: Kind,
    command: u32,
    body: Vec<u8>,
    wait: bool,
}
impl Request {
    fn new(chip: Chip, kind: Kind, command: u32, body: Vec<u8>, wait: bool) -> Self {
        Self {
            chip,
            kind,
            command,
            body,
            wait,
        }
    }
    pub fn restart(chip: Chip) -> Self {
        Self::new(chip, Kind::Restart, 4, vec![1, 0, 0, 0], false)
    }
    pub fn firmware_log(chip: Chip) -> Self {
        if chip == Chip::Mt7925 {
            let mut body = vec![0; 12];
            body[6] = 8;
            body[8] = 1;
            Self::new(chip, Kind::FirmwareLog, (1 << 17) | 0xb, body, true)
        } else {
            Self::new(
                chip,
                Kind::FirmwareLog,
                (1 << 18) | 0xc5,
                vec![1, 0, 0, 0],
                false,
            )
        }
    }
    pub fn efuse_mode(chip: Chip) -> Self {
        if chip == Chip::Mt7925 {
            let mut body = vec![0; 12];
            body[4] = 2;
            body[6] = 8;
            body[9] = 1;
            Self::new(chip, Kind::EfuseMode, (1 << 17) | 0x2d, body, true)
        } else {
            Self::new(
                chip,
                Kind::EfuseMode,
                0xed | (0x21 << 8),
                vec![0, 1, 0, 0],
                true,
            )
        }
    }
    /// `mt7925_mcu_chip_config`: a firmware configuration string, sent
    /// without an acknowledgement as `mt7925_mcu_fill_message` sends
    /// CHIP_CONFIG. `None` for another chip or a string longer than the
    /// command's 320 bytes with its NUL.
    pub fn chip_config(chip: Chip, text: &str) -> Option<Self> {
        if chip != Chip::Mt7925 || text.len() >= 320 || text.as_bytes().contains(&0) {
            return None;
        }
        // _rsv[4], tag UNI_CHIP_CONFIG_CHIP_CFG, len, then
        // mt76_connac_config: id, type, resp_type, data_size, resv, data[320].
        let mut body = vec![0; 336];
        body[4..6].copy_from_slice(&2u16.to_le_bytes());
        body[6..8].copy_from_slice(&332u16.to_le_bytes());
        body[12..14].copy_from_slice(&(text.len() as u16 + 1).to_le_bytes());
        body[16..16 + text.len()].copy_from_slice(text.as_bytes());
        Some(Self::new(
            chip,
            Kind::ChipConfig,
            (1 << 17) | 0x0e,
            body,
            false,
        ))
    }
    /// `mt7925_mcu_set_thermal_protect`'s two settings, 2.4 GHz then 5 GHz.
    pub fn thermal_protection(chip: Chip) -> Option<[Self; 2]> {
        Some([
            Self::chip_config(chip, "ThermalProtGband 0 100 90 80 30 1 1 115 105 5")?,
            Self::chip_config(chip, "ThermalProtAband 1 100 90 80 30 1 1 115 105 5")?,
        ])
    }
    pub fn rts(chip: Chip) -> Self {
        if chip == Chip::Mt7925 {
            let mut body = vec![0; 16];
            body[4] = 8;
            body[6] = 12;
            body[8..12].copy_from_slice(&0x92bu32.to_le_bytes());
            body[12] = 2;
            Self::new(chip, Kind::RtsThreshold, (1 << 17) | 8, body, true)
        } else {
            let mut body = vec![0; 12];
            body[0] = 1;
            body[4..8].copy_from_slice(&0x92bu32.to_le_bytes());
            body[8] = 2;
            Self::new(chip, Kind::RtsThreshold, 0xed | (0x3e << 8), body, true)
        }
    }
    pub fn mac_enable(chip: Chip, enable: bool) -> Result<Self, Error> {
        if chip == Chip::Mt7925 {
            return Err(Error::WrongChip);
        }
        // The pinned helper's hdr_trans argument is UNUSED. Global
        // and per-WCID disables are independent real operations, not inferred.
        Ok(Self::new(
            chip,
            Kind::MacEnable(enable),
            0xed | (0x46 << 8),
            vec![u8::from(enable), 0, 0, 0],
            true,
        ))
    }
    pub fn radio_led(chip: Chip, value: u8) -> Result<Self, Error> {
        if chip == Chip::Mt7925 {
            return Err(Error::WrongChip);
        }
        Ok(Self::new(
            chip,
            Kind::RadioLed(value),
            0xed | (5 << 8),
            vec![value, 0, 0, 0],
            false,
        ))
    }
    pub fn raw_translation(
        chip: Chip,
        context: Context,
        wcid: u16,
        peer: bool,
    ) -> Result<Self, Error> {
        device::Requests::station(context, true).map_err(Error::Context)?;
        if wcid >= 20 {
            return Err(Error::InvalidWcid);
        }
        let c3 = chip == Chip::Mt7925;
        let mut body = vec![0; if c3 { 16 } else { 28 }];
        body[0] = context.bss_index;
        body[1] = wcid as u8;
        body[2] = 1;
        body[4] = 1;
        body[5] = if peer { context.omac_index } else { 0xe };
        if c3 {
            body[8] = 0x2b;
            body[10] = 8;
            body[13] = 1;
            body[14] = 1;
        } else {
            body[8] = 13;
            body[10] = 20;
            body[12] = wcid as u8;
            body[13] = 2;
            body[14] = 1;
            body[20] = 6;
            body[22] = 8;
            body[24] = 1;
            body[26] = 1;
        }
        Ok(Self::new(
            chip,
            Kind::RawTranslation { wcid },
            (1 << 17) | 3 | if c3 { (1 << 19) | (1 << 20) } else { 0 },
            body,
            true,
        ))
    }
    /// Initial source SET_RX_PATH profile: one non-MLO20MHz channel and the
    /// reported primary PHY antenna mask, with no monitor/scan/DFS
    /// branch substituted. This is wire data, not channel/power permission.
    pub fn rx_path(
        chip: Chip,
        band: Band,
        channel: u8,
        frequency_mhz: u16,
        antenna_mask: u8,
    ) -> Result<Self, Error> {
        if chip == Chip::Mt7925 {
            return Err(Error::WrongChip);
        }
        if antenna_mask == 0 || antenna_mask & !0xf != 0 {
            return Err(Error::InvalidAntenna);
        }
        validate_channel(band, channel, frequency_mhz)?;
        let mut body = vec![0; 76];
        body[0] = channel;
        body[1] = channel;
        body[3] = antenna_mask.count_ones() as u8;
        body[4] = antenna_mask;
        body[10] = match band {
            Band::Ghz2 => 0,
            Band::Ghz5 => 1,
            Band::Ghz6 => 2,
        };
        Ok(Self::new(
            chip,
            Kind::RxPath,
            0xed | (0x4e << 8),
            body,
            true,
        ))
    }
    /// MT7925 scan uses the generic source response path, unlike plain
    /// DEV/BSS/STA commands with a status record. An ACK is not scan done.
    /// A connac2 chip's CE scan request or cancellation, which the firmware
    /// does not acknowledge.
    pub(in crate::wifi::mediatek) fn connac2_scan(chip: Chip, command: u32, body: Vec<u8>) -> Self {
        let kind = if command & 0xff == 0x1b {
            Kind::CancelScan
        } else {
            Kind::Scan
        };
        Self::new(chip, kind, command, body, false)
    }
    pub(in crate::wifi::mediatek) fn scan(body: Vec<u8>, cancel: bool) -> Self {
        Self::new(
            Chip::Mt7925,
            if cancel { Kind::CancelScan } else { Kind::Scan },
            (1 << 17) | 0x16,
            body,
            true,
        )
    }
    pub(in crate::wifi::mediatek) fn client_filter(body: Vec<u8>) -> Self {
        Self::new(Chip::Mt7925, Kind::ClientFilter, (1 << 17) | 8, body, true)
    }
    pub(in crate::wifi::mediatek) fn roc(body: Vec<u8>) -> Self {
        Self::new(Chip::Mt7925, Kind::Roc, (1 << 17) | 0x27, body, true)
    }
    /// A connac2 chip's station command the firmware does not acknowledge:
    /// UNI ROC as `mt7921_mcu_set_roc` sends it, or a CE command.
    pub(in crate::wifi::mediatek) fn connac2_client(
        chip: Chip,
        kind: Kind,
        command: u32,
        body: Vec<u8>,
    ) -> Self {
        Self::new(chip, kind, command, body, false)
    }
    pub fn chip(&self) -> Chip {
        self.chip
    }
    pub fn kind(&self) -> Kind {
        self.kind
    }
    pub fn command(&self) -> u32 {
        self.command
    }
    pub fn body(&self) -> &[u8] {
        &self.body
    }
    pub fn expects_response(&self) -> bool {
        self.wait
    }
}
enum Wire {
    Direct { sequence: u8, bytes: Vec<u8> },
    Station(runtime::Packet),
}
pub struct Packet {
    chip: Chip,
    kind: Kind,
    wait: bool,
    wire: Wire,
}
impl Packet {
    pub fn chip(&self) -> Chip {
        self.chip
    }
    pub fn kind(&self) -> Kind {
        self.kind
    }
    pub fn sequence(&self) -> u8 {
        match &self.wire {
            Wire::Direct { sequence, .. } => *sequence,
            Wire::Station(p) => p.sequence(),
        }
    }
    pub fn bytes(&self) -> &[u8] {
        match &self.wire {
            Wire::Direct { bytes, .. } => bytes,
            Wire::Station(p) => p.bytes(),
        }
    }
    pub fn expects_response(&self) -> bool {
        self.wait
    }
}
impl Encoder {
    pub fn encode_bootstrap(&mut self, request: &Request) -> Packet {
        let wire = if matches!(request.kind, Kind::RawTranslation { .. })
            && request.chip != Chip::Mt7925
        {
            Wire::Station(
                self.encode_uni(request.chip, runtime::Command::StationRecord, &request.body)
                    .expect("bounded private-built raw translation request"),
            )
        } else {
            let uni = request.command & (1 << 17) != 0;
            let header = if uni { 48 } else { 64 };
            let sequence = self.next_sequence();
            let length = header + request.body.len();
            let mut bytes = vec![0; length];
            bytes[..4].copy_from_slice(&(length as u32 | (2 << 23) | (0x20 << 25)).to_le_bytes());
            bytes[4..8].copy_from_slice(
                &(1u32 << 16
                    | if request.chip == Chip::Mt7925 {
                        0
                    } else {
                        1 << 31
                    })
                .to_le_bytes(),
            );
            bytes[32..34].copy_from_slice(&((length - 32) as u16).to_le_bytes());
            bytes[37] = 0xa0;
            bytes[39] = sequence;
            if uni {
                bytes[34..36].copy_from_slice(&((request.command & 0xff) as u16).to_le_bytes());
                // MCU_CMD_UNI_EXT_ACK, less MCU_CMD_ACK for CHIP_CONFIG.
                bytes[43] = if request.kind == Kind::ChipConfig {
                    6
                } else {
                    7
                };
            } else {
                bytes[34..36].copy_from_slice(&0x8000u16.to_le_bytes());
                bytes[36] = request.command as u8;
                bytes[38] = if request.command & (1 << 18) != 0 || request.command & 0xff00 != 0 {
                    1
                } else {
                    3
                };
                bytes[41] = (request.command >> 8) as u8;
                bytes[43] = u8::from(request.command & 0xff00 != 0);
            }
            bytes[header..].copy_from_slice(&request.body);
            Wire::Direct { sequence, bytes }
        };
        Packet {
            chip: request.chip,
            kind: request.kind,
            wait: request.wait,
            wire,
        }
    }
}
impl Event<'_> {
    pub fn bootstrap_result(&self, packet: &Packet) -> Result<Option<u32>, Error> {
        if !packet.wait {
            return Err(Error::OneWay);
        }
        if let Wire::Station(p) = &packet.wire {
            return self
                .uni_result(p)
                .map(|r| Some(r.status))
                .map_err(Error::Runtime);
        }
        if self.unsolicited || self.sequence == 0 {
            return Err(Error::Event(super::Error::Unsolicited));
        }
        if self.sequence != packet.sequence() {
            return Err(Error::Event(super::Error::UnexpectedSequence {
                expected: packet.sequence(),
                actual: self.sequence,
            }));
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    #[test]
    #[ignore = "oracle: STELA_MT792X_BOOTSTRAP_ORACLE"]
    fn complete_unchanged_initial_helpers_and_full_writers_match() {
        let root =
            std::path::PathBuf::from(std::env::var("STELA_MT792X_BOOTSTRAP_ORACLE").unwrap());
        for (chip, id) in [
            (Chip::Mt7921, 7921),
            (Chip::Mt7922, 7922),
            (Chip::Mt7925, 7925),
        ] {
            let bytes = std::fs::read(root.join(alloc::format!("{id}-commands.trace"))).unwrap();
            let mut at = 0;
            let mut e = Encoder::default();
            for i in 0..32 {
                let mut requests = vec![
                    Request::restart(chip),
                    Request::firmware_log(chip),
                    Request::efuse_mode(chip),
                    Request::rts(chip),
                ];
                if chip != Chip::Mt7925 {
                    requests.extend([
                        Request::mac_enable(chip, i % 2 != 0).unwrap(),
                        Request::rx_path(
                            chip,
                            if i % 2 == 0 { Band::Ghz2 } else { Band::Ghz5 },
                            if i % 2 == 0 { 1 } else { 36 },
                            if i % 2 == 0 { 2412 } else { 5180 },
                            1 + (i % 15),
                        )
                        .unwrap(),
                        Request::radio_led(chip, 1).unwrap(),
                        Request::radio_led(chip, 2).unwrap(),
                    ]);
                }
                for r in requests {
                    let command = u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
                    let wait = bytes[at + 4] != 0;
                    let n = u16::from_le_bytes(bytes[at + 5..at + 7].try_into().unwrap()) as usize;
                    at += 7;
                    assert_eq!(r.command(), command);
                    assert_eq!(r.expects_response(), wait);
                    assert_eq!(r.body(), &bytes[at..at + n]);
                    at += n;
                    let n = u16::from_le_bytes(bytes[at..at + 2].try_into().unwrap()) as usize;
                    at += 2;
                    let p = e.encode_bootstrap(&r);
                    assert_eq!(
                        p.bytes(),
                        &bytes[at..at + n],
                        "{chip:?} case{i} {:?}",
                        r.kind()
                    );
                    at += n;
                }
            }
            assert_eq!(at, bytes.len());
        }
    }
    #[test]
    #[ignore = "oracle: STELA_MT792X_BOOTSTRAP_ORACLE"]
    fn raw_commands_and_asymmetric_parser_paths_match_source() {
        let root =
            std::path::PathBuf::from(std::env::var("STELA_MT792X_BOOTSTRAP_ORACLE").unwrap());
        for (chip, id) in [
            (Chip::Mt7921, 7921),
            (Chip::Mt7922, 7922),
            (Chip::Mt7925, 7925),
        ] {
            let bytes = std::fs::read(root.join(alloc::format!("{id}-raw.trace"))).unwrap();
            let mut at = 0;
            let mut e = Encoder::default();
            for i in 0..128 {
                let c = Context {
                    bss_index: i % 4,
                    omac_index: i % 64,
                    band_index: if chip == Chip::Mt7925 { 0xff } else { 0 },
                    wmm_index: 0,
                    link_index: 0,
                    broadcast_wcid: 19,
                    address: crate::Mac([2, 1, 2, 3, 4, 5]),
                };
                let r = Request::raw_translation(chip, c, u16::from(i % 20), i % 2 != 0).unwrap();
                let command = u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
                let wait = bytes[at + 4];
                let n = u16::from_le_bytes(bytes[at + 5..at + 7].try_into().unwrap()) as usize;
                at += 7;
                assert_eq!(r.command(), command);
                assert_eq!(wait, 1);
                assert_eq!(r.body(), &bytes[at..at + n]);
                at += n;
                let n = u16::from_le_bytes(bytes[at..at + 2].try_into().unwrap()) as usize;
                at += 2;
                assert_eq!(e.encode_bootstrap(&r).bytes(), &bytes[at..at + n]);
                at += n;
            }
            assert_eq!(at, bytes.len());
            let trace = std::fs::read(root.join(alloc::format!("{id}-parse.trace"))).unwrap();
            assert_eq!(trace.len(), 64 * 16);
            for row in trace.as_chunks::<16>().0 {
                let command = u32::from_le_bytes(row[..4].try_into().unwrap());
                let status = u32::from_le_bytes(row[4..8].try_into().unwrap());
                let cid = row[8];
                let seq = row[9];
                let result = i32::from_le_bytes(row[10..14].try_into().unwrap());
                let c = Context {
                    bss_index: 0,
                    omac_index: 0,
                    band_index: 0,
                    wmm_index: 0,
                    link_index: 0,
                    broadcast_wcid: 19,
                    address: crate::Mac([2, 1, 2, 3, 4, 5]),
                };
                let r = Request::raw_translation(chip, c, 1, true).unwrap();
                let mut e = Encoder { sequence: 2 };
                let packet = e.encode_bootstrap(&r);
                let mut e = Encoder { sequence: 2 };
                let plain = e
                    .encode_uni(chip, runtime::Command::StationRecord, r.body())
                    .unwrap();
                assert_eq!(
                    packet.bytes(),
                    plain.bytes(),
                    "same on-wire CID/body/header despite actual command/parser distinction"
                );
                let base = if chip == Chip::Mt7925 { 32 } else { 24 };
                let mut response = vec![0; base + 20];
                response[..4].copy_from_slice(&(7u32 << 27).to_le_bytes());
                response[base + 4] = 1;
                response[base + 5] = seq;
                response[base + 6] = 2;
                response[base + 12] = cid;
                response[base + 16..base + 20].copy_from_slice(&status.to_le_bytes());
                let event = Event::parse(chip, &response).unwrap();
                if command == r.command() {
                    let observed = event.bootstrap_result(&packet);
                    if seq != 3 || (chip != Chip::Mt7925 && cid != 3) {
                        assert!(observed.is_err());
                        assert!(
                            result < 0,
                            "source retry errno (host C errno boundary), never a returned firmware status"
                        );
                    } else if chip == Chip::Mt7925 {
                        assert_eq!(observed, Ok(None));
                        assert_eq!(result, 0);
                    } else {
                        assert_eq!(observed, Ok(Some(status)));
                        assert_eq!(result, status as i32);
                    }
                }
                if chip == Chip::Mt7925 && seq == 3 && status != 0 {
                    assert_eq!(event.bootstrap_result(&packet), Ok(None));
                    if cid == 3 {
                        assert_eq!(event.uni_result(&plain).unwrap().status, status);
                    } else {
                        assert!(event.uni_result(&plain).is_err());
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod chip_config_tests {
    use super::*;

    #[test]
    fn thermal_protection_is_two_unacknowledged_chip_config_strings() {
        let [gband, aband] = Request::thermal_protection(Chip::Mt7925).unwrap();
        let text = b"ThermalProtGband 0 100 90 80 30 1 1 115 105 5";
        assert!(!gband.expects_response());
        assert_eq!(gband.body().len(), 336);
        assert_eq!(gband.body()[4..8], [2, 0, 76, 1]);
        assert_eq!(
            gband.body()[12..14],
            ((text.len() + 1) as u16).to_le_bytes()
        );
        assert_eq!(&gband.body()[16..16 + text.len()], text);
        assert!(gband.body()[16 + text.len()..].iter().all(|&b| b == 0));
        assert_eq!(aband.body()[16..32], *b"ThermalProtAband");
        let packet = Encoder::default().encode_bootstrap(&gband);
        let bytes = packet.bytes();
        assert_eq!(bytes.len(), 48 + 336);
        assert_eq!(bytes[34..36], [0x0e, 0]);
        // MCU_CMD_UNI | MCU_CMD_SET, without MCU_CMD_ACK.
        assert_eq!(bytes[43], 6);
        assert!(Request::thermal_protection(Chip::Mt7921).is_none());
        assert!(Request::chip_config(Chip::Mt7925, &"x".repeat(320)).is_none());
    }
}
