// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Bounded ordinary PCIe firmware TX-free notification decoding. This extracts
//! evidence only: it neither releases DMA leases nor reports air ACKs. Token
//! ownership, exact-once processing and firmware-epoch checks precede release.
use super::identity::Chip;
use crate::bytes::get_le32;
use alloc::{collections::BTreeSet, vec::Vec};

const MAX_BYTES: usize = 16 * 1024;
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Truncated,
    TooLarge,
    WrongPacketType(u8),
    UnsupportedVersion(u8),
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek TX-free: {self:?}")
    }
}
impl core::error::Error for Error {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Accounting {
    /// Raw firmware attempt count, including zero. This is not an ACK receipt.
    pub attempts: u16,
    /// Zero is donor success, other values are firmware drops. Air
    /// outcome remains separately associated TXS/PID evidence.
    pub status: u8,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Event {
    Pair {
        wcid: u16,
        raw: u32,
    },
    Accounting {
        wcid: Option<u16>,
        value: Accounting,
        raw: u32,
    },
    Release {
        wcid: Option<u16>,
        token: u16,
        accounting: Option<Accounting>,
        raw: u32,
    },
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Notification<'a> {
    pub header: [u32; 2],
    pub declared_tokens: u16,
    /// Connac3 accepts version >=4 under the pinned donor's compatibility
    /// contract; Connac2's second header word is held without guessing it.
    pub version: Option<u8>,
    pub events: Vec<Event>,
    pub consumed_bytes: usize,
    /// DMA packet bytes, including any unconsumed transport tail.
    pub raw: &'a [u8],
}
fn word(bytes: &[u8], offset: usize) -> Result<u32, Error> {
    get_le32(bytes, offset).ok_or(Error::Truncated)
}

/// Parse the complete bounded notification before any token-table mutation.
/// Headers are selected by observed chip, not guessed from packet bytes.
/// The encoded byte count is held; as in the donor, the completed DMA frame
/// length bounds parsing. Unknown/duplicate/old tokens must be checked against
/// the owning live table and real firmware epoch before applying these events.
pub fn parse(chip: Chip, bytes: &[u8]) -> Result<Notification<'_>, Error> {
    if bytes.len() > MAX_BYTES {
        return Err(Error::TooLarge);
    }
    let header = [word(bytes, 0)?, word(bytes, 4)?];
    let packet_type = (header[0] >> 27) as u8;
    if packet_type != 6 {
        return Err(Error::WrongPacketType(packet_type));
    }
    let total = ((header[0] >> 16) & 0x3ff) as u16;
    let connac3 = chip == Chip::Mt7925;
    let version = connac3.then_some(((header[1] >> 16) & 15) as u8);
    if let Some(v) = version
        && v < 4
    {
        return Err(Error::UnsupportedVersion(v));
    }
    let mut result = Notification {
        header,
        declared_tokens: total,
        version,
        events: Vec::new(),
        consumed_bytes: 8,
        raw: bytes,
    };
    // mt7925_mac_tx_free counts every MSDU ID against the total and lets
    // mt76_token_release skip one it no longer holds, so a repeated ID is
    // counted and released once.
    let mut released = BTreeSet::new();
    let mut count = 0usize;
    let mut wcid = None;
    while count < usize::from(total) {
        let raw = word(bytes, result.consumed_bytes)?;
        result.consumed_bytes += 4;
        if raw & (1 << 31) != 0 {
            let index = if connac3 {
                (raw >> 12) & 0xfff
            } else {
                (raw >> 14) & 0x3ff
            } as u16;
            wcid = Some(index);
            result.events.push(Event::Pair { wcid: index, raw });
            continue;
        }
        if connac3 && raw & (1 << 30) != 0 {
            result.events.push(Event::Accounting {
                wcid,
                value: Accounting {
                    attempts: ((raw >> 24) & 15) as u16,
                    status: ((raw >> 28) & 3) as u8,
                },
                raw,
            });
            continue;
        }
        let tokens = if connac3 {
            [
                Some((raw & 0x7fff) as u16),
                Some(((raw >> 15) & 0x7fff) as u16),
            ]
        } else {
            [Some(((raw >> 16) & 0x7fff) as u16), None]
        };
        for token in tokens.into_iter().flatten() {
            if connac3 && token == 0x7fff {
                continue;
            }
            count += 1;
            if !released.insert(token) {
                continue;
            }
            result.events.push(Event::Release {
                wcid,
                token,
                accounting: (!connac3).then_some(Accounting {
                    attempts: (raw & 0x1fff) as u16,
                    status: ((raw >> 13) & 3) as u8,
                }),
                raw,
            });
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    fn packet(chip: Chip, total: u16, words: &[u32]) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(
            &((6u32 << 27) | (u32::from(total) << 16) | (8 + 4 * words.len()) as u32).to_le_bytes(),
        );
        bytes.extend_from_slice(
            &(if chip == Chip::Mt7925 {
                4u32 << 16
            } else {
                0xdec0beef
            })
            .to_le_bytes(),
        );
        for word in words {
            bytes.extend_from_slice(&word.to_le_bytes());
        }
        bytes
    }
    #[test]
    fn connac2_pairs_per_msdu_status_and_complete_tail_are_kept() {
        for chip in [Chip::Mt7921, Chip::Mt7922] {
            let pair = (1 << 31) | (19 << 14);
            let words = [7 << 16, pair, (8 << 16) | (2 << 13) | 5, 0xabcdef01];
            let bytes = packet(chip, 2, &words);
            let n = parse(chip, &bytes).unwrap();
            assert_eq!(n.version, None);
            assert_eq!(n.consumed_bytes, 20);
            assert_eq!(n.raw, &bytes);
            assert_eq!(n.header[1], 0xdec0beef);
            assert_eq!(
                n.events,
                [
                    Event::Release {
                        wcid: None,
                        token: 7,
                        accounting: Some(Accounting {
                            attempts: 0,
                            status: 0
                        }),
                        raw: words[0]
                    },
                    Event::Pair {
                        wcid: 19,
                        raw: pair
                    },
                    Event::Release {
                        wcid: Some(19),
                        token: 8,
                        accounting: Some(Accounting {
                            attempts: 5,
                            status: 2
                        }),
                        raw: words[2]
                    }
                ]
            );
        }
    }
    #[test]
    fn connac3_packed_tokens_accounting_headers_and_odd_sentinel_are_exact() {
        let words = [
            (1 << 31) | (7 << 12),
            (1 << 30) | (3 << 28) | (15 << 24),
            12 | (13 << 15),
            14 | (0x7fff << 15),
        ];
        let bytes = packet(Chip::Mt7925, 3, &words);
        let n = parse(Chip::Mt7925, &bytes).unwrap();
        assert_eq!(n.version, Some(4));
        assert_eq!(n.consumed_bytes, 24);
        assert_eq!(n.events.len(), 5);
        assert_eq!(
            n.events[1],
            Event::Accounting {
                wcid: Some(7),
                value: Accounting {
                    attempts: 15,
                    status: 3
                },
                raw: words[1]
            }
        );
        let tokens: Vec<_> = n
            .events
            .iter()
            .filter_map(|e| {
                if let Event::Release {
                    token,
                    accounting,
                    wcid,
                    ..
                } = e
                {
                    assert_eq!(*accounting, None);
                    assert_eq!(*wcid, Some(7));
                    Some(*token)
                } else {
                    None
                }
            })
            .collect();
        assert_eq!(tokens, [12, 13, 14]);
    }
    #[test]
    fn malformed_lengths_versions_duplicate_and_extra_tokens_reject_atomically() {
        for chip in [Chip::Mt7921, Chip::Mt7925] {
            for len in 0..8 {
                assert_eq!(parse(chip, &[0; 8][..len]), Err(Error::Truncated));
            }
            let bytes = packet(chip, 2, &[]);
            assert_eq!(parse(chip, &bytes), Err(Error::Truncated));
            let mut wrong = packet(chip, 0, &[]);
            wrong[3] = 0;
            assert_eq!(parse(chip, &wrong), Err(Error::WrongPacketType(0)));
            assert_eq!(parse(chip, &vec![0; MAX_BYTES + 1]), Err(Error::TooLarge));
        }
        for version in 0..4 {
            let mut bytes = packet(Chip::Mt7925, 0, &[]);
            bytes[6] = version;
            assert_eq!(
                parse(Chip::Mt7925, &bytes),
                Err(Error::UnsupportedVersion(version))
            );
        }
        // As mt7925_mac_tx_free: both IDs of a word are taken even past the
        // total, and a repeated ID is counted but released once.
        let releases = |chip, total, words: &[u32]| {
            parse(chip, &packet(chip, total, words))
                .unwrap()
                .events
                .iter()
                .filter_map(|event| match event {
                    Event::Release { token, .. } => Some(*token),
                    _ => None,
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(releases(Chip::Mt7925, 1, &[1 | (2 << 15)]), [1, 2]);
        assert_eq!(releases(Chip::Mt7925, 2, &[1 | (1 << 15)]), [1]);
        assert_eq!(releases(Chip::Mt7921, 2, &[1 << 16, 1 << 16]), [1]);
        let mut words = vec![(1 << 31) | (2 << 14); 1000];
        words.push(1 << 16);
        assert_eq!(
            parse(Chip::Mt7921, &packet(Chip::Mt7921, 1, &words))
                .unwrap()
                .events
                .len(),
            1001
        );
        assert_eq!(
            parse(Chip::Mt7925, &packet(Chip::Mt7925, 1, &[0x3fffffff])),
            Err(Error::Truncated)
        );
    }
    #[test]
    fn full_ten_bit_count_does_not_wrap_and_future_compatible_version_is_kept() {
        let words: Vec<_> = (0..1023).map(|t| t << 16).collect();
        assert_eq!(
            parse(Chip::Mt7921, &packet(Chip::Mt7921, 1023, &words))
                .unwrap()
                .events
                .len(),
            1023
        );
        let mut bytes = packet(Chip::Mt7925, 0, &[]);
        bytes[6] = 15;
        assert_eq!(parse(Chip::Mt7925, &bytes).unwrap().version, Some(15));
    }
    #[test]
    #[ignore = "oracle: STELA_MT792X_TX_FREE_ORACLE"]
    fn unchanged_pinned_free_helpers_match_token_order_wcid_and_accounting() {
        let root =
            std::env::var_os("STELA_MT792X_TX_FREE_ORACLE").expect("reference trace directory");
        for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
            let name = match chip {
                Chip::Mt7921 => "7921",
                Chip::Mt7922 => "7922",
                Chip::Mt7925 => "7925",
            };
            let bytes =
                std::fs::read(std::path::Path::new(&root).join(format!("mt{name}.bin"))).unwrap();
            let mut at = 0;
            let word = |at: &mut usize| {
                let value = u32::from_le_bytes(bytes[*at..*at + 4].try_into().unwrap());
                *at += 4;
                value
            };
            for _ in 0..100 {
                let len = word(&mut at) as usize;
                let notification = parse(chip, &bytes[at..at + len]).unwrap();
                at += len;
                assert_eq!(notification.consumed_bytes, len);
                let effective =
                    |wcid: Option<u16>| wcid.filter(|&id| id < 20).map_or(u32::MAX, u32::from);
                let mut released = Vec::new();
                let mut stats = [[0u32; 2]; 20];
                let mut account = |wcid: Option<u16>, a: Accounting| {
                    if let Some(id) = wcid.filter(|&id| id < 20) {
                        stats[id as usize][0] += u32::from(a.attempts.saturating_sub(1));
                        stats[id as usize][1] += u32::from(a.status != 0);
                    }
                };
                for event in notification.events {
                    match event {
                        Event::Pair { .. } => {}
                        Event::Accounting { wcid, value, .. } => account(wcid, value),
                        Event::Release {
                            wcid,
                            token,
                            accounting,
                            ..
                        } => {
                            released.push((u32::from(token), effective(wcid)));
                            if let Some(value) = accounting {
                                account(wcid, value);
                            }
                        }
                    }
                }
                assert_eq!(released.len(), word(&mut at) as usize);
                for actual in released {
                    assert_eq!(actual, (word(&mut at), word(&mut at)));
                }
                for actual in stats {
                    assert_eq!(actual, [word(&mut at), word(&mut at)]);
                }
            }
            assert_eq!(at, bytes.len());
        }
    }
}
