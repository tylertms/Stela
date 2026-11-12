// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright (C) 2020-2023 MediaTek Inc.
// Channel flag ABI: Copyright 2006-2010 Johannes Berg <johannes@sipsolutions.net>
// Copyright 2013-2014 Intel Mobile Communications GmbH
// Copyright 2015-2017 Intel Deutschland GmbH
// Copyright (C) 2018-2026 Intel Corporation
//! Source channel-domain wire requests. Raw Linux flags and source bandwidth
//! header constants are not native radio, OS/location or power permission.
use super::{Encoder, Event};
use crate::wifi::mediatek::identity::Chip;
use alloc::vec::Vec;

const DISABLED: u32 = 1; // pinned include/net/cfg80211.h IEEE80211_CHAN_DISABLED
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Channel {
    pub hardware_value: u16,
    /// Exact pinned Linux ieee80211_channel_flags wire ABI. A trusted native
    /// composition must construct/intersect restrictions before use;
    /// caller-provided raw flags alone never satisfy policy admission.
    pub flags: u32,
}
pub struct Inputs<'a> {
    pub alpha2: [u8; 2],
    pub channels_2ghz: &'a [Channel],
    pub channels_5ghz: &'a [Channel],
    pub channels_6ghz: &'a [Channel],
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    TooManyChannels { band: u8, enabled: usize },
    TransferTooLarge { bytes: usize },
    OneWayCommand,
    Event(super::Error),
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek channel domain: {self:?}")
    }
}
impl core::error::Error for Error {}
#[derive(Debug)]
pub struct Request {
    chip: Chip,
    body: Vec<u8>,
    counts: [u8; 3],
}
impl Request {
    /// Source order:2GHz→5GHz→6GHz, preserve each enabled channel's
    /// full raw flags, omit disabled channels. The source fixed BW0/3/3 fields
    /// are protocol bytes only; they never override per-channel policy.
    /// Refuse count truncation/current transport overflow before publication.
    pub fn build(chip: Chip, input: Inputs<'_>) -> Result<Self, Error> {
        let bands = [
            input.channels_2ghz,
            input.channels_5ghz,
            input.channels_6ghz,
        ];
        let mut counts = [0; 3];
        for (index, channels) in bands.iter().enumerate() {
            let enabled = channels.iter().filter(|c| c.flags & DISABLED == 0).count();
            counts[index] = u8::try_from(enabled).map_err(|_| Error::TooManyChannels {
                band: index as u8,
                enabled,
            })?;
        }
        let c3 = chip == Chip::Mt7925;
        let header = if c3 { 16 } else { 12 };
        let n: usize = counts.iter().map(|&n| usize::from(n)).sum();
        let bytes = header + n * 8;
        if bytes + if c3 { 48 } else { 64 } > 4096 {
            return Err(Error::TransferTooLarge { bytes });
        }
        let mut body = vec![0; header];
        body[..2].copy_from_slice(&input.alpha2);
        body[5] = 3;
        body[6] = 3;
        if c3 {
            body[8..10].copy_from_slice(&2u16.to_le_bytes());
            body[10..12].copy_from_slice(&((8 + n * 8) as u16).to_le_bytes());
            body[12..15].copy_from_slice(&counts);
        } else {
            body[8..11].copy_from_slice(&counts);
        }
        for channels in bands {
            for channel in channels.iter().filter(|c| c.flags & DISABLED == 0) {
                body.extend_from_slice(&channel.hardware_value.to_le_bytes());
                body.extend_from_slice(&[0; 2]);
                body.extend_from_slice(&channel.flags.to_le_bytes());
            }
        }
        Ok(Self { chip, body, counts })
    }
    pub fn chip(&self) -> Chip {
        self.chip
    }
    pub fn body(&self) -> &[u8] {
        &self.body
    }
    pub fn alpha2(&self) -> [u8; 2] {
        self.body[..2].try_into().unwrap()
    }
    pub fn counts(&self) -> [u8; 3] {
        self.counts
    }
    /// The rows `build` wrote for band index `band` (2.4, 5, 6 GHz), each a
    /// hardware value, two pad bytes and the raw flags, after this chip's
    /// header.
    pub fn rows(&self, band: usize) -> &[[u8; 8]] {
        let header = if self.chip == Chip::Mt7925 { 16 } else { 12 };
        let first: usize = self.counts[..band].iter().map(|&n| usize::from(n)).sum();
        let rows = self.body[header..].as_chunks::<8>().0;
        &rows[first..first + usize::from(self.counts[band])]
    }
    pub fn expects_response(&self) -> bool {
        self.chip == Chip::Mt7925
    }
}
pub struct Packet {
    chip: Chip,
    sequence: u8,
    bytes: Vec<u8>,
}
impl Packet {
    pub fn chip(&self) -> Chip {
        self.chip
    }
    pub fn sequence(&self) -> u8 {
        self.sequence
    }
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }
    pub fn expects_response(&self) -> bool {
        self.chip == Chip::Mt7925
    }
}
impl Encoder {
    pub fn encode_domain(&mut self, request: &Request) -> Packet {
        let c3 = request.chip == Chip::Mt7925;
        let header = if c3 { 48 } else { 64 };
        let sequence = self.next_sequence();
        let length = header + request.body.len();
        let mut bytes = vec![0; length];
        bytes[..4].copy_from_slice(&(length as u32 | (2 << 23) | (0x20 << 25)).to_le_bytes());
        bytes[4..8].copy_from_slice(&(1u32 << 16 | if c3 { 0 } else { 1 << 31 }).to_le_bytes());
        bytes[32..34].copy_from_slice(&((length - 32) as u16).to_le_bytes());
        bytes[37] = 0xa0;
        bytes[39] = sequence;
        if c3 {
            bytes[34..36].copy_from_slice(&0x15u16.to_le_bytes());
            bytes[43] = 7;
        } else {
            bytes[34..36].copy_from_slice(&0x8000u16.to_le_bytes());
            bytes[36] = 0x0f;
            bytes[38] = 1;
        }
        bytes[header..].copy_from_slice(request.body());
        Packet {
            chip: request.chip,
            sequence,
            bytes,
        }
    }
}
impl Event<'_> {
    /// Connac3 uses the generic source parser, not station status/CID matching.
    /// Preserve the entire raw body; an empty reply is not invented status0.
    pub fn domain_payload<'a>(&'a self, packet: &Packet) -> Result<&'a [u8], Error> {
        if !packet.expects_response() {
            return Err(Error::OneWayCommand);
        }
        if self.unsolicited || self.sequence == 0 {
            return Err(Error::Event(super::Error::Unsolicited));
        }
        if self.sequence != packet.sequence {
            return Err(Error::Event(super::Error::UnexpectedSequence {
                expected: packet.sequence,
                actual: self.sequence,
            }));
        }
        Ok(self.payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    #[test]
    fn exact_band_order_and_raw_restrictions_survive_disabled_filtering() {
        let channels = [
            Channel {
                hardware_value: 0xffff,
                flags: 0x8000_0000,
            },
            Channel {
                hardware_value: 1,
                flags: 1,
            },
            Channel {
                hardware_value: 7,
                flags: 0x4000_0002,
            },
        ];
        for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
            let request = Request::build(
                chip,
                Inputs {
                    alpha2: *b"US",
                    channels_2ghz: &channels,
                    channels_5ghz: &[],
                    channels_6ghz: &channels[1..],
                },
            )
            .unwrap();
            assert_eq!(request.counts(), [2, 0, 1]);
            let start = if chip == Chip::Mt7925 { 16 } else { 12 };
            assert_eq!(&request.body()[..8], &[b'U', b'S', 0, 0, 0, 3, 3, 0]);
            assert_eq!(
                &request.body()[start..],
                &[
                    255, 255, 0, 0, 0, 0, 0, 128, 7, 0, 0, 0, 2, 0, 0, 64, 7, 0, 0, 0, 2, 0, 0, 64
                ]
            );
            let mut encoder = Encoder::default();
            let packet = encoder.encode_domain(&request);
            assert_eq!(packet.expects_response(), chip == Chip::Mt7925);
            let mut event = Event {
                event_id: 0x55,
                extended_event_id: 0,
                packet_type_id: 0,
                declared_length: 12,
                status: 0,
                sequence: packet.sequence(),
                option: 2,
                unsolicited: false,
                payload: &[],
            };
            if chip == Chip::Mt7925 {
                assert_eq!(event.domain_payload(&packet).unwrap(), &[]);
                event.payload = &[0xde, 0xad];
                assert_eq!(event.domain_payload(&packet).unwrap(), &[0xde, 0xad]);
                event.unsolicited = true;
                assert!(matches!(
                    event.domain_payload(&packet),
                    Err(Error::Event(super::super::Error::Unsolicited))
                ));
                event.unsolicited = false;
                event.sequence = 0;
                assert!(event.domain_payload(&packet).is_err());
                event.sequence = 2;
                assert!(matches!(
                    event.domain_payload(&packet),
                    Err(Error::Event(super::super::Error::UnexpectedSequence { .. }))
                ));
            } else {
                assert_eq!(event.domain_payload(&packet), Err(Error::OneWayCommand));
            }
        }
    }
    #[test]
    fn count_truncation_and_current_transport_overflow_refuse_atomically() {
        let enabled = Channel {
            hardware_value: 1,
            flags: 0,
        };
        let channels = vec![enabled; 256];
        for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
            assert!(matches!(
                Request::build(
                    chip,
                    Inputs {
                        alpha2: *b"00",
                        channels_2ghz: &channels,
                        channels_5ghz: &[],
                        channels_6ghz: &[]
                    }
                ),
                Err(Error::TooManyChannels {
                    band: 0,
                    enabled: 256
                })
            ));
            assert!(matches!(
                Request::build(
                    chip,
                    Inputs {
                        alpha2: *b"00",
                        channels_2ghz: &channels[..255],
                        channels_5ghz: &channels[..255],
                        channels_6ghz: &[]
                    }
                ),
                Err(Error::TransferTooLarge { .. })
            ));
            let request = Request::build(
                chip,
                Inputs {
                    alpha2: *b"00",
                    channels_2ghz: &[],
                    channels_5ghz: &[],
                    channels_6ghz: &[],
                },
            )
            .unwrap();
            assert_eq!(request.counts(), [0; 3]);
        }
    }
    #[test]
    #[ignore = "oracle: STELA_MT792X_DOMAIN_ORACLE"]
    fn domain_helpers_match_linux_for_every_channel() {
        let root = std::path::PathBuf::from(std::env::var("STELA_MT792X_DOMAIN_ORACLE").unwrap());
        for (chip, number) in [
            (Chip::Mt7921, 7921),
            (Chip::Mt7922, 7922),
            (Chip::Mt7925, 7925),
        ] {
            let trace = std::fs::read(root.join(alloc::format!("{number}.trace"))).unwrap();
            let mut at = 0;
            let mut encoder = Encoder::default();
            for _ in 0..192 {
                let counts: [usize; 3] = core::array::from_fn(|n| {
                    usize::from(u16::from_le_bytes(
                        trace[at + n * 2..at + n * 2 + 2].try_into().unwrap(),
                    ))
                });
                at += 6;
                let alpha2 = trace[at..at + 2].try_into().unwrap();
                at += 2;
                let channels: [Vec<Channel>; 3] = counts.map(|n| {
                    (0..n)
                        .map(|_| {
                            let channel = Channel {
                                hardware_value: u16::from_le_bytes(
                                    trace[at..at + 2].try_into().unwrap(),
                                ),
                                flags: u32::from_le_bytes(
                                    trace[at + 2..at + 6].try_into().unwrap(),
                                ),
                            };
                            at += 6;
                            channel
                        })
                        .collect()
                });
                let request = Request::build(
                    chip,
                    Inputs {
                        alpha2,
                        channels_2ghz: &channels[0],
                        channels_5ghz: &channels[1],
                        channels_6ghz: &channels[2],
                    },
                )
                .unwrap();
                assert_eq!(
                    u32::from_le_bytes(trace[at..at + 4].try_into().unwrap()),
                    if chip == Chip::Mt7925 {
                        (1 << 17) | 0x15
                    } else {
                        (1 << 18) | 0x0f
                    }
                );
                assert_eq!(trace[at + 4] != 0, request.expects_response());
                let n = usize::from(u16::from_le_bytes(
                    trace[at + 5..at + 7].try_into().unwrap(),
                ));
                at += 7;
                assert_eq!(request.body(), &trace[at..at + n]);
                at += n;
                let n = usize::from(u16::from_le_bytes(trace[at..at + 2].try_into().unwrap()));
                at += 2;
                let packet = encoder.encode_domain(&request);
                assert_eq!(packet.chip(), chip);
                assert_eq!(packet.bytes(), &trace[at..at + n]);
                at += n;
            }
            assert_eq!(at, trace.len());
        }
    }
}
