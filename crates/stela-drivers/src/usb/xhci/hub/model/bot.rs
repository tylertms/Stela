// SPDX-License-Identifier: GPL-2.0-or-later
//! Opt-in device-side BOT peer for complete native topology/class rebinding.
//! Decodes wire CBWs independently and serves RAM media through TDs.
use alloc::vec::Vec;

const BLOCK_BYTES: usize = 512;
const BLOCKS: usize = 16;

#[derive(Clone)]
enum Phase {
    Command,
    Data {
        tag: u32,
        input: bool,
        bytes: Vec<u8>,
        write_offset: Option<usize>,
    },
    Status(u32),
}

#[derive(Clone)]
pub(super) struct Disk {
    media: Vec<u8>,
    phase: Phase,
    pub(super) commands: Vec<(u32, u8)>,
    /// Hung after its configuration, as a stick may be: it takes no bulk
    /// transfer and stalls the class reset until a port reset readdresses it.
    pub(super) wedged: bool,
}

impl Disk {
    pub(super) fn new() -> Self {
        Self {
            media: alloc::vec![0; BLOCK_BYTES * BLOCKS],
            phase: Phase::Command,
            commands: Vec::new(),
            wedged: false,
        }
    }

    pub(super) fn reset(&mut self) {
        self.phase = Phase::Command;
    }

    pub(super) fn transfer(&mut self, input: bool, address: u64, length: usize) {
        // SAFETY: the caller supplies the TD's device-side RAM.
        let wire = unsafe { core::slice::from_raw_parts_mut(address as *mut u8, length) };
        match core::mem::replace(&mut self.phase, Phase::Command) {
            Phase::Command => {
                assert!(!input);
                assert_eq!(length, 31);
                assert_eq!(&wire[..4], b"USBC");
                let tag = u32::from_le_bytes(wire[4..8].try_into().unwrap());
                let bytes = u32::from_le_bytes(wire[8..12].try_into().unwrap()) as usize;
                assert_eq!(wire[12] & 0x7f, 0);
                assert_eq!(wire[13], 0, "only physical LUN0 exists");
                assert!((1..=16).contains(&wire[14]));
                let cdb = &wire[15..15 + usize::from(wire[14])];
                self.commands.push((tag, cdb[0]));
                let mut data = alloc::vec![0; bytes];
                let mut write_offset = None;
                match cdb[0] {
                    0x00 | 0x1b | 0x35 => {
                        assert_eq!(bytes, 0);
                        assert_eq!(wire[12], 0);
                        assert_eq!(cdb.len(), if cdb[0] == 0x35 { 10 } else { 6 });
                    }
                    0x03 => {
                        assert_eq!(cdb.len(), 6);
                        assert!(bytes >= 18);
                        data[0] = 0x70;
                        data[7] = 10;
                    }
                    0x12 => {
                        assert_eq!(cdb.len(), 6);
                        assert_eq!(bytes, 36);
                        assert_eq!(cdb[4], 36);
                        data[2] = 5;
                        data[3] = 2;
                        data[4] = 31;
                        data[8..16].copy_from_slice(b"MODEL   ");
                        data[16..32].copy_from_slice(b"Topology BOT RAM");
                    }
                    0x1a => {
                        assert_eq!(cdb.len(), 6);
                        assert_eq!(bytes, 192);
                        assert_eq!(cdb[4], 192);
                        data[0] = 191;
                        data[4] = 8;
                        data[5] = 18;
                    }
                    0x25 => {
                        assert_eq!(cdb.len(), 10);
                        assert_eq!(bytes, 8);
                        data[..4].copy_from_slice(&((BLOCKS - 1) as u32).to_be_bytes());
                        data[4..].copy_from_slice(&(BLOCK_BYTES as u32).to_be_bytes());
                    }
                    0x28 | 0x2a => {
                        assert_eq!(cdb.len(), 10);
                        let lba = u32::from_be_bytes(cdb[2..6].try_into().unwrap()) as usize;
                        let blocks = usize::from(u16::from_be_bytes(cdb[7..9].try_into().unwrap()));
                        assert!(lba <= BLOCKS && blocks <= BLOCKS - lba);
                        assert_eq!(bytes, blocks * BLOCK_BYTES);
                        let offset = lba * BLOCK_BYTES;
                        if cdb[0] == 0x28 {
                            data.copy_from_slice(&self.media[offset..offset + bytes]);
                        } else {
                            write_offset = Some(offset);
                        }
                    }
                    opcode => panic!("unmodeled device-side SCSI opcode {opcode:#x}"),
                }
                self.phase = if bytes == 0 {
                    Phase::Status(tag)
                } else {
                    let input = wire[12] == 0x80;
                    assert_eq!(input, write_offset.is_none());
                    Phase::Data {
                        tag,
                        input,
                        bytes: data,
                        write_offset,
                    }
                };
            }
            Phase::Data {
                tag,
                input: expected,
                bytes,
                write_offset,
            } => {
                assert_eq!(input, expected);
                assert_eq!(length, bytes.len());
                if let Some(offset) = write_offset {
                    self.media[offset..offset + length].copy_from_slice(wire);
                } else {
                    wire.copy_from_slice(&bytes);
                }
                self.phase = Phase::Status(tag);
            }
            Phase::Status(tag) => {
                assert!(input);
                assert_eq!(length, 13);
                wire.fill(0);
                wire[..4].copy_from_slice(b"USBS");
                wire[4..8].copy_from_slice(&tag.to_le_bytes());
            }
        }
    }
}
