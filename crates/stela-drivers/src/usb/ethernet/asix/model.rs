//! Independent ASIX register device: selected PHY pages, MMD addressing,
//! asynchronous EEPROM reads and FIFO ownership. No production decode helpers.
use super::*;
use alloc::{collections::BTreeMap, rc::Rc};
use core::cell::RefCell;

pub(crate) struct Hardware {
    pub(crate) mac: [u8; 256],
    pub(crate) phy: BTreeMap<(u16, u16, u16), u16>,
    pub(crate) eeprom: [u16; 256],
    pub(crate) efuse: [u8; 64],
    page: u16,
    bank: u16,
    mmd_control: u16,
    mmd_address: u16,
    pub(crate) mmd: BTreeMap<(u16, u16), u16>,
    power: u8,
    srom: Option<(u8, usize)>,
    pub(crate) stuck_srom: bool,
    pub(crate) fifo: usize,
    pub(crate) stuck_fifo: bool,
    pub(crate) fail_at: Option<usize>,
    pub(crate) short_at: Option<usize>,
    pub(crate) trace: Vec<([u8; 8], Vec<u8>)>,
    pub(crate) delays: Vec<u64>,
    pub(crate) minimum_elapsed: u64,
}
impl Hardware {
    pub(crate) fn new(bus: u8, ua2: bool) -> Self {
        let mut hw = Self {
            mac: [0; 256],
            phy: BTreeMap::new(),
            eeprom: [0xffff; 256],
            efuse: [0xff; 64],
            page: 0,
            bank: 0,
            mmd_control: 0,
            mmd_address: 0,
            mmd: BTreeMap::new(),
            power: 0,
            srom: None,
            stuck_srom: false,
            fifo: 3,
            stuck_fifo: false,
            fail_at: None,
            short_at: None,
            trace: Vec::new(),
            delays: Vec::new(),
            minimum_elapsed: 0,
        };
        hw.mac[0x10..0x16].copy_from_slice(&[2, 0x34, 0x56, 0x78, 0x9a, 0xbc]);
        hw.mac[2] = bus;
        hw.mac[3] = if ua2 { 4 } else { 0 };
        // Synthetic, nonzero PHY identification; no claim of a captured board.
        hw.phy.insert((0, 0, 2), 0x0007);
        hw.phy.insert((0, 0, 3), 0xc001);
        hw.phy.insert((0, 0, 0), 0x1000);
        hw.phy.insert((0, 0, 0x11), 0xa400);
        hw.phy.insert((7, 0x2c, 0x1a), 0xa55a);
        hw.phy.insert((7, 0x2c, 0x1c), 0x5aa5);
        hw
    }
    pub(crate) fn word(&self, address: usize) -> u16 {
        u16::from_le_bytes(self.mac[address..address + 2].try_into().unwrap())
    }
    pub(crate) fn request(
        &mut self,
        request: [u8; 8],
        output: &[u8],
    ) -> Result<Vec<u8>, BlockError> {
        let at = self.trace.len();
        self.trace.push((request, output.to_vec()));
        if self.fail_at == Some(at) {
            return Err(BlockError::Timeout);
        }
        let read = request[0] == 0xc0;
        assert!(read || request[0] == 0x40);
        let command = request[1];
        let value = u16::from_le_bytes([request[2], request[3]]);
        let index = u16::from_le_bytes([request[4], request[5]]);
        let length = usize::from(u16::from_le_bytes([request[6], request[7]]));
        assert_eq!(output.len(), if read { 0 } else { length });
        let mut reply = match command {
            1 => {
                assert_eq!(index as usize, length);
                let address = value as usize;
                assert!(address + length <= 256);
                if read {
                    if address == 0x0a
                        && let Some((word, count)) = &mut self.srom
                        && !self.stuck_srom
                    {
                        *count -= 1;
                        if *count == 0 {
                            let word = *word;
                            self.srom = None;
                            self.mac[0x0a] &= !0x10;
                            self.mac[8..10]
                                .copy_from_slice(&self.eeprom[word as usize].to_le_bytes());
                        }
                    }
                    if address == 8 {
                        assert!(self.srom.is_none(), "EEPROM data before completion");
                    }
                    self.mac[address..address + length].to_vec()
                } else {
                    if address == 0x26 {
                        assert_eq!(length, 2);
                        let value = u16::from_le_bytes(output.try_into().unwrap());
                        if value == 0 {
                            self.power = 1;
                        } else if value == 0x20 {
                            assert_eq!(self.power, 1);
                            self.power = 2;
                        }
                    } else if address == 0x33 {
                        assert_eq!(self.power, 2, "clock before PHY power release");
                    } else {
                        assert_eq!(self.power, 2);
                    }
                    self.mac[address..address + length].copy_from_slice(output);
                    if address == 0x0a && output[0] & 4 != 0 {
                        self.srom = Some((self.mac[7], 3));
                        self.mac[0x0a] |= 0x10;
                    }
                    Vec::new()
                }
            }
            2 => {
                assert_eq!((value, length), (3, 2));
                assert_eq!(self.power, 2);
                if read {
                    let data = if index == 31 {
                        self.page
                    } else if index == 30 {
                        self.bank
                    } else if self.page == 0 && index == 14 && self.mmd_control & 0x4000 != 0 {
                        *self
                            .mmd
                            .get(&(self.mmd_control & 31, self.mmd_address))
                            .unwrap_or(&0xffff)
                    } else {
                        *self
                            .phy
                            .get(&(self.page, if self.page == 7 { self.bank } else { 0 }, index))
                            .unwrap_or(&0)
                    };
                    data.to_le_bytes().to_vec()
                } else {
                    let data = u16::from_le_bytes(output.try_into().unwrap());
                    if index == 31 {
                        self.page = data;
                    } else if index == 30 {
                        self.bank = data;
                    } else if self.page == 0 && index == 13 {
                        self.mmd_control = data;
                    } else if self.page == 0 && index == 14 {
                        if self.mmd_control & 0x4000 != 0 {
                            self.mmd
                                .insert((self.mmd_control & 31, self.mmd_address), data);
                        } else {
                            self.mmd_address = data;
                        }
                    } else {
                        self.phy.insert(
                            (self.page, if self.page == 7 { self.bank } else { 0 }, index),
                            data,
                        );
                    }
                    Vec::new()
                }
            }
            4 => {
                assert!(read);
                assert_eq!((index, length), (1, 2));
                self.eeprom[value as usize].to_le_bytes().to_vec()
            }
            5 => {
                assert!(read);
                assert_eq!((value, index, length), (0, 64, 64));
                self.efuse.to_vec()
            }
            0x81 => {
                assert!(read);
                assert_eq!((value, index, length), (0x8c, 0, 4));
                if self.fifo > 0 && !self.stuck_fifo {
                    self.fifo -= 1;
                }
                (if self.fifo == 0 { 0u32 } else { 0x4000_0000 })
                    .to_le_bytes()
                    .to_vec()
            }
            _ => panic!("unsupported ASIX request: {request:?}"),
        };
        if read && self.short_at == Some(at) {
            reply.pop();
        }
        Ok(reply)
    }
}

pub(super) struct Io(pub Rc<RefCell<Hardware>>);
impl Control for Io {
    async fn control(&mut self, request: [u8; 8], output: &[u8]) -> Result<Vec<u8>, BlockError> {
        self.0.borrow_mut().request(request, output)
    }
    async fn delay_us(&mut self, microseconds: u64) {
        self.0.borrow_mut().delays.push(microseconds);
    }
    async fn minimum_wait(&mut self, microseconds: u64) -> Result<(), BlockError> {
        let mut state = self.0.borrow_mut();
        state.minimum_elapsed = state
            .minimum_elapsed
            .checked_add(microseconds)
            .ok_or(BlockError::PciPower(crate::bus::pci::power::Error::Clock))?;
        state.delays.push(microseconds);
        Ok(())
    }
}
