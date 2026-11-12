// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2003-2006 David Hollis <dhollis@davehollis.com>
// Copyright (C) 2005 Phil Chang <pchang23@sbcglobal.net>
// Copyright (C) 2006 James Painter <jamie.painter@iname.com>
// Copyright (C) 2002-2003 TiVo Inc.
// ASIX PHY reset/status adaptation: Michael Schmitz <schmitzmic@gmail.com>.
// Generic PHY reference: Copyright (C) 2004 Freescale Semiconductor, Inc.; Andy Fleming.
//! AX88772 original/A/B vendor protocol with acknowledged MII ownership.
//! Only identified embedded ASIX PHYs are initialized. External PHY boards
//! require their own vendor driver; USB identities alone never establish one.
use super::{Control, RegisterData, RegisterRequest, request};
use crate::{block::BlockError, net::Mac};
use alloc::vec::Vec;
pub(super) mod framing;
#[cfg(test)]
pub(crate) mod model;
mod registers;
use registers::*;

pub(in crate::usb::ethernet) struct Chip<C> {
    io: super::super::MinimumControl<C>,
    accessible: bool,
    initialized: bool,
    pub(super) raw_status: u8,
    pub(super) phy_id: u32,
    phy: u16,
    advertised: u16,
    medium: u16,
    was_link: bool,
    decoder: framing::Decoder,
}
impl<C: Control> Chip<C> {
    async fn read(
        &mut self,
        command: u8,
        value: u16,
        index: u16,
        n: usize,
    ) -> Result<Vec<u8>, BlockError> {
        request(
            &mut self.io,
            &mut self.accessible,
            RegisterRequest {
                command,
                value,
                index,
                data: RegisterData::Read(n),
            },
        )
        .await
    }
    async fn write(
        &mut self,
        command: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<(), BlockError> {
        request(
            &mut self.io,
            &mut self.accessible,
            RegisterRequest {
                command,
                value,
                index,
                data: RegisterData::Write(data),
            },
        )
        .await
        .map(|_| ())
    }
    async fn word(&mut self, command: u8) -> Result<u16, BlockError> {
        self.read(command, 0, 0, 2)
            .await
            .map(|v| u16::from_le_bytes(v.try_into().unwrap()))
    }
    async fn command(&mut self, command: u8, value: u16) -> Result<(), BlockError> {
        self.write(command, value, 0, &[]).await
    }
    async fn minimum_wait(&mut self, us: u64) -> Result<(), BlockError> {
        let result = self.io.minimum_wait(us).await;
        if result.is_err() {
            self.initialized = false;
        }
        result
    }
    fn timeout(&mut self) -> BlockError {
        self.accessible = false;
        self.initialized = false;
        BlockError::Timeout
    }
    async fn own_mii(&mut self) -> Result<(), BlockError> {
        for _ in 0..30 {
            self.command(SET_SW_MII, 0).await?;
            self.minimum_wait(1000).await?;
            if self.read(STATUS, 0, 0, 1).await?[0] & HOST_EN != 0 {
                return Ok(());
            }
        }
        Err(self.timeout())
    }
    async fn phy_read(&mut self, address: u16) -> Result<u16, BlockError> {
        self.own_mii().await?;
        let bytes = self.read(READ_MII, self.phy, address, 2).await?;
        self.command(SET_HW_MII, 0).await?;
        Ok(u16::from_le_bytes(bytes.try_into().unwrap()))
    }
    async fn phy_write(&mut self, address: u16, value: u16) -> Result<(), BlockError> {
        self.own_mii().await?;
        self.write(WRITE_MII, self.phy, address, &value.to_le_bytes())
            .await?;
        self.command(SET_HW_MII, 0).await
    }
    pub(super) async fn attach(io: C, eeprom_mac: bool) -> Result<(Self, Mac), BlockError> {
        let mut c = Self {
            io: super::super::MinimumControl::new(io),
            accessible: true,
            initialized: false,
            raw_status: 0,
            phy_id: 0,
            phy: 0,
            advertised: 0,
            medium: 0,
            was_link: false,
            decoder: framing::Decoder::default(),
        };
        let mut address = [0; 6];
        if eeprom_mac {
            for i in 0..3 {
                address[i * 2..i * 2 + 2]
                    .copy_from_slice(&c.read(EEPROM, 4 + i as u16, 0, 2).await?);
            }
        } else {
            address.copy_from_slice(&c.read(READ_MAC, 0, 0, 6).await?);
        }
        let mac = Mac(address);
        if address[0] & 1 != 0 || address.iter().all(|b| *b == 0) {
            return Err(BlockError::Unsupported(
                "AX88772 invalid permanent station address",
            ));
        }
        let phy = c.read(READ_PHY, 0, 0, 2).await?;
        c.phy = u16::from(phy[1]);
        if c.phy != INTERNAL_PHY {
            return Err(BlockError::Unsupported(
                "AX88772 external/unknown PHY requires its own initialization",
            ));
        }
        c.raw_status = c.read(STATUS, 0, 0, 1).await?[0];
        if !matches!(c.raw_status & CHIP_MASK, ORIGINAL | A | B) {
            return Err(BlockError::Unsupported("AX88772 unimplemented chip code"));
        }
        c.reset(&mac).await?;
        c.phy_id =
            (u32::from(c.phy_read(PHY_ID1).await?) << 16) | u32::from(c.phy_read(PHY_ID2).await?);
        if c.phy_id != PHY_A && c.phy_id != PHY_C && c.phy_id & !0xf != PHY_96B {
            return Err(BlockError::Unsupported(
                "AX88772 unimplemented embedded PHY identity",
            ));
        }
        c.phy_start().await?;
        // Driver reset/open follows bind in usbnet; preserve this order after
        // physical reset and PHY attach, then let polling resolve the medium.
        c.write(WRITE_MAC, 0, 0, &mac.0).await?;
        c.command(WRITE_RX, RX_DEFAULT).await?;
        c.command(WRITE_MEDIUM, MEDIUM_DEFAULT).await?;
        // Native Link has no multicast filter programming API. Source
        // asix_set_multicast's all-multicast mode preserves guest IPv6/etc.
        c.command(WRITE_RX, RX_DEFAULT | RX_ALL_MULTICAST).await?;
        c.command(WRITE_MEDIUM, 0).await?; // phylink begins with carrier down.
        c.initialized = true;
        Ok((c, mac))
    }
    async fn reset(&mut self, mac: &Mac) -> Result<(), BlockError> {
        let code = self.raw_status & CHIP_MASK;
        let gpio = if code == ORIGINAL {
            GPIO_RELOAD | GPIO_2 | GPIO_2_ENABLE
        } else {
            GPIO_RELOAD
        };
        self.command(WRITE_GPIO, gpio).await?;
        self.minimum_wait(5000).await?;
        self.command(PHY_SELECT, 1 | if code == ORIGINAL { 0 } else { SSEN })
            .await?;
        if code == ORIGINAL {
            self.command(SW_RESET, RESET_IPPD).await?;
            self.minimum_wait(10_000).await?;
            self.command(SW_RESET, 0).await?;
            self.minimum_wait(60_000).await?;
            self.command(SW_RESET, RESET_IPRL | RESET_PRL).await?;
            self.minimum_wait(150_000).await?;
            self.command(WRITE_RX, RX_DEFAULT).await?;
            self.command(WRITE_MEDIUM, MEDIUM_DEFAULT).await?;
        } else {
            self.minimum_wait(10_000).await?;
            self.command(SW_RESET, RESET_IPPD | RESET_IPRL).await?;
            self.minimum_wait(10_000).await?;
            self.command(SW_RESET, RESET_IPRL).await?;
            self.minimum_wait(160_000).await?;
            self.command(SW_RESET, 0).await?;
            self.command(SW_RESET, RESET_IPRL).await?;
            self.minimum_wait(200_000).await?;
            if code == B {
                self.write(QUEUE, 0x8000, 0x8001, &[]).await?;
            } else {
                let mut observed = [0; 3];
                for (i, (register, _)) in A_DEFAULTS.iter().enumerate() {
                    observed[i] = self.phy_read(*register).await?;
                }
                for ((register, value), actual) in A_DEFAULTS.iter().zip(observed) {
                    if *value != actual {
                        self.phy_write(*register, *value).await?;
                    }
                }
            }
        }
        self.write(WRITE_IPG, IPG0 | IPG1, IPG2, &[]).await?;
        self.write(WRITE_MAC, 0, 0, &mac.0).await?;
        self.command(WRITE_RX, RX_DEFAULT).await?;
        if code != ORIGINAL {
            self.command(WRITE_MEDIUM, MEDIUM_DEFAULT).await?;
            self.command(WRITE_RX, RX_DEFAULT).await?;
        }
        self.word(READ_RX).await?;
        self.word(READ_MEDIUM).await?;
        Ok(())
    }
    async fn phy_start(&mut self) -> Result<(), BlockError> {
        // ax88796b.c asix_soft_reset must toggle RESET low first.
        self.phy_write(BMCR, 0).await?;
        let old = self.phy_read(BMCR).await?;
        self.phy_write(BMCR, (old & !ISOLATE) | RESET | AN_RESTART)
            .await?;
        let mut done = false;
        for _ in 0..12 {
            self.minimum_wait(50_000).await?;
            if self.phy_read(BMCR).await? & RESET == 0 {
                done = true;
                break;
            }
        }
        if !done {
            return Err(self.timeout());
        }
        self.minimum_wait(1000).await?;
        let capabilities = self.phy_read(BMSR).await?;
        if capabilities & AN_CAPABLE == 0 {
            return Err(BlockError::Unsupported("AX88772 PHY lacks autonegotiation"));
        }
        let mut adv = ADV_PAUSE | ADV_ASYM;
        for (cap, bit) in [
            (CAP_100_FULL, ADV_100_FULL),
            (CAP_100_HALF, ADV_100_HALF),
            (CAP_10_FULL, ADV_10_FULL),
            (CAP_10_HALF, ADV_10_HALF),
        ] {
            if capabilities & cap != 0 {
                adv |= bit;
            }
        }
        if adv & (ADV_100_FULL | ADV_100_HALF | ADV_10_FULL | ADV_10_HALF) == 0 {
            return Err(BlockError::Unsupported(
                "AX88772 PHY lacks 10/100 capability",
            ));
        }
        let old = self.phy_read(ADVERTISE).await?;
        self.advertised = (old & !ADV_MASK) | adv;
        if old != self.advertised {
            self.phy_write(ADVERTISE, self.advertised).await?;
        }
        let control = self.phy_read(BMCR).await?;
        self.phy_write(
            BMCR,
            (control & !(ISOLATE | POWER_DOWN)) | AN_ENABLE | AN_RESTART,
        )
        .await
    }
    pub(super) async fn carrier(&mut self) -> Result<bool, BlockError> {
        if let Some(error) = self.io.error() {
            return Err(error.clone());
        }
        if !self.initialized {
            return Err(BlockError::Unsupported("AX88772 not initialized"));
        }
        let control = self.phy_read(BMCR).await?;
        if control & AN_ENABLE == 0 {
            return Err(BlockError::Unsupported(
                "AX88772 autonegotiation unexpectedly disabled",
            ));
        }
        let status = if control & (AN_RESTART | RESET | POWER_DOWN | ISOLATE) != 0 {
            0
        } else if self.was_link {
            // Poll once while up so the latched-low indication of a brief
            // disconnect is observed instead of cleared by a second read.
            self.phy_read(BMSR).await?
        } else {
            let first = self.phy_read(BMSR).await?;
            if first & LINK != 0 {
                first
            } else {
                self.phy_read(BMSR).await?
            }
        };
        let up = status & (LINK | AN_COMPLETE) == LINK | AN_COMPLETE;
        if !up {
            if self.medium != 0 {
                self.command(WRITE_MEDIUM, 0).await?;
                self.medium = 0;
            }
            self.decoder.reset();
            // AX88772A's PHY_NOLINK callback reinitializes the PHY to clear
            // stale partner advertisement. Bound this to a down transition.
            if self.was_link && self.phy_id == PHY_A {
                self.phy_start().await?;
            }
            self.was_link = false;
            return Ok(false);
        }
        let partner = self.phy_read(LPA).await?;
        let common = partner & self.advertised;
        let (hundred, full) = if common & ADV_100_FULL != 0 {
            (true, true)
        } else if common & ADV_100_HALF != 0 {
            (true, false)
        } else if common & ADV_10_FULL != 0 {
            (false, true)
        } else if common & ADV_10_HALF != 0 {
            (false, false)
        } else if self.phy_id == PHY_A {
            (control & SPEED_100 != 0, control & FULL != 0)
        } else {
            return Err(BlockError::Unsupported(
                "AX88772 unresolved negotiated link mode",
            ));
        };
        let mut medium = MEDIUM_AC | MEDIUM_RE;
        if hundred {
            medium |= MEDIUM_100;
        }
        if full {
            medium |= MEDIUM_FD;
        }
        if full {
            if common & ADV_PAUSE != 0 {
                medium |= MEDIUM_TX_PAUSE | MEDIUM_RX_PAUSE;
            } else if common & ADV_ASYM != 0 {
                if self.advertised & ADV_PAUSE != 0 {
                    medium |= MEDIUM_RX_PAUSE;
                } else if partner & ADV_PAUSE != 0 {
                    medium |= MEDIUM_TX_PAUSE;
                }
            }
        }
        if self.medium != medium {
            self.command(WRITE_MEDIUM, medium).await?;
            self.medium = medium;
        }
        self.was_link = true;
        Ok(true)
    }
    pub(super) async fn link_down(&mut self) -> Result<(), BlockError> {
        self.decoder.reset();
        if self.medium != 0 {
            self.command(WRITE_MEDIUM, 0).await?;
            self.medium = 0;
        }
        Ok(())
    }
    pub(super) async fn stop(&mut self) -> Result<(), BlockError> {
        self.initialized = false;
        self.was_link = false;
        self.decoder.reset();
        self.command(WRITE_MEDIUM, 0).await?;
        self.command(WRITE_RX, 0).await?;
        let old = self.phy_read(BMCR).await?;
        self.phy_write(BMCR, old | POWER_DOWN).await
    }
    pub(super) fn decode(&mut self, bytes: &[u8]) -> Result<Vec<Vec<u8>>, &'static str> {
        self.decoder.decode(bytes)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use alloc::rc::Rc;
    use core::cell::RefCell;
    use stela_driver_base::stepped::block_on;
    fn attached(
        code: u8,
        id: u32,
        eeprom: bool,
    ) -> (Chip<model::Io>, Rc<RefCell<model::Hardware>>) {
        let hw = Rc::new(RefCell::new(model::Hardware::new(code, id)));
        let (chip, mac) = block_on(Chip::attach(model::Io(hw.clone()), eeprom)).unwrap();
        assert_eq!(mac.0, hw.borrow().mac);
        (chip, hw)
    }
    #[test]
    fn revision_phy_reset_autoneg_carrier_and_power_down_are_stateful() {
        for code in [0, 0x10, 0x20] {
            for id in [PHY_A, PHY_C, PHY_96B | 1] {
                for eeprom in [false, true] {
                    let (mut c, hw) = attached(code, id, eeprom);
                    assert_eq!(hw.borrow().rx, 0x8a);
                    assert_eq!(hw.borrow().medium, 0);
                    assert_eq!(hw.borrow().resets, 1);
                    assert_eq!(hw.borrow().phy[4] & 0xfe0, 0xde0);
                    // Grant delays advance the separate autonegotiation clock.
                    for _ in 0..4 {
                        if block_on(c.carrier()).unwrap() {
                            break;
                        }
                    }
                    assert!(block_on(c.carrier()).unwrap());
                    assert_eq!(hw.borrow().medium, 0x336);
                    hw.borrow_mut().link = false;
                    assert!(!block_on(c.carrier()).unwrap());
                    assert_eq!(hw.borrow().medium, 0);
                    assert_eq!(hw.borrow().resets, if id == PHY_A { 2 } else { 1 });
                    hw.borrow_mut().link = true;
                    for _ in 0..4 {
                        if block_on(c.carrier()).unwrap() {
                            break;
                        }
                    }
                    assert!(block_on(c.carrier()).unwrap());
                    block_on(c.stop()).unwrap();
                    assert_eq!(hw.borrow().medium, 0);
                    assert_eq!(hw.borrow().rx, 0);
                    assert_ne!(hw.borrow().phy[0] & 0x800, 0);
                    assert!(block_on(c.carrier()).is_err());
                }
            }
        }
    }
    #[test]
    fn every_request_failure_and_short_read_poison_access() {
        for code in [0, 0x10, 0x20] {
            for eeprom in [false, true] {
                let (_, reference) = attached(code, PHY_A, eeprom);
                for (at, (q, _)) in reference.borrow().trace.iter().enumerate() {
                    for short in [false, true] {
                        if short && q[0] != 0xc0 {
                            continue;
                        }
                        let hw = Rc::new(RefCell::new(model::Hardware::new(code, PHY_A)));
                        if short {
                            hw.borrow_mut().short_at = Some(at);
                        } else {
                            hw.borrow_mut().fail_at = Some(at);
                        }
                        assert!(
                            block_on(Chip::attach(model::Io(hw.clone()), eeprom)).is_err(),
                            "code {code} at {at}"
                        );
                        assert_eq!(hw.borrow().trace.len(), at + 1);
                    }
                }
            }
        }
        let (mut c, hw) = attached(0x20, PHY_C, true);
        let at = hw.borrow().trace.len();
        hw.borrow_mut().fail_at = Some(at + 2);
        assert!(block_on(c.carrier()).is_err());
        let count = hw.borrow().trace.len();
        assert!(block_on(c.carrier()).is_err());
        assert!(block_on(c.stop()).is_err());
        assert_eq!(hw.borrow().trace.len(), count);
    }
    #[test]
    fn missing_grant_reset_timeout_and_unimplemented_phy_refuse() {
        for mode in 0..7 {
            let hw = Rc::new(RefCell::new(model::Hardware::new(0x10, PHY_A)));
            match mode {
                0 => hw.borrow_mut().stuck_mii = true,
                1 => hw.borrow_mut().stuck_reset = true,
                2 => hw.borrow_mut().phy_address = 3,
                3 => hw.borrow_mut().code = 0x70,
                4 => hw.borrow_mut().phy[2] = 0xffff,
                5 => hw.borrow_mut().mac = [0; 6],
                _ => hw.borrow_mut().phy[1] = 0,
            }
            assert!(block_on(Chip::attach(model::Io(hw.clone()), false)).is_err());
            assert_eq!(hw.borrow().rx & !0x88, 0);
        }
    }
    #[test]
    fn all_negotiated_modes_pause_and_a_zero_partner_workaround() {
        for id in [PHY_A, PHY_C] {
            for partner in [0, 0x21, 0x41, 0x81, 0x101, 0x501, 0x901, 0xd01] {
                let (mut c, hw) = attached(0x10, id, false);
                hw.borrow_mut().delay(100_000);
                hw.borrow_mut().phy[5] = partner;
                if partner == 0 && id != PHY_A {
                    assert!(block_on(c.carrier()).is_err());
                    continue;
                }
                assert!(block_on(c.carrier()).unwrap());
                let expected = match partner {
                    0 => 0x104,
                    0x21 => 0x104,
                    0x41 => 0x106,
                    0x81 => 0x304,
                    0x101 => 0x306,
                    0x501 | 0xd01 => 0x336,
                    0x901 => 0x316,
                    _ => unreachable!(),
                };
                assert_eq!(hw.borrow().medium, expected, "{partner:04x}");
            }
        }
    }
    #[test]
    #[ignore = "oracle: STELA_AX88772_RESET_ORACLE"]
    fn unchanged_linux_ax88772_reset_register_sequences() {
        let text =
            std::fs::read_to_string(std::env::var("STELA_AX88772_RESET_ORACLE").unwrap()).unwrap();
        assert_eq!(text.lines().count(), 24);
        for line in text.lines() {
            let fields: Vec<_> = line.split_whitespace().collect();
            let code: u8 = fields[0].parse().unwrap();
            let mask: u8 = fields[1].parse().unwrap();
            let hw = Rc::new(RefCell::new(model::Hardware::new(code, PHY_A)));
            for (j, (reg, val)) in A_DEFAULTS.iter().enumerate() {
                hw.borrow_mut().phy[*reg as usize] = if mask & (1 << j) != 0 { *val } else { 0 };
            }
            let mut chip = Chip {
                io: crate::usb::ethernet::MinimumControl::new(model::Io(hw.clone())),
                accessible: true,
                initialized: false,
                raw_status: code,
                phy_id: PHY_A,
                phy: 16,
                advertised: 0,
                medium: 0,
                was_link: false,
                decoder: framing::Decoder::default(),
            };
            block_on(chip.reset(&Mac([2, 0x27, 0x72, 0x43, 0x54, 0x65]))).unwrap();
            let hw = hw.borrow();
            let mut trace = alloc::string::String::new();
            use core::fmt::Write;
            for (q, data) in &hw.trace {
                for b in q {
                    write!(&mut trace, "{b:02x}").unwrap();
                }
                trace.push(':');
                for b in data {
                    write!(&mut trace, "{b:02x}").unwrap();
                }
                trace.push(';');
            }
            assert_eq!(trace, fields[2], "code {code} mask {mask}");
            assert_eq!(
                hw.delays,
                fields[3]
                    .split(',')
                    .map(|v| v.parse::<u64>().unwrap())
                    .collect::<Vec<_>>()
            );
        }
    }
    #[test]
    #[ignore = "oracle: STELA_AX88772_IDS_ORACLE"]
    fn unchanged_linux_ax88772_candidates_preserve_flags_and_eeprom_selection() {
        use crate::usb::ethernet::asix::identity::{Identity, LEGACY_IDS};
        let text =
            std::fs::read_to_string(std::env::var("STELA_AX88772_IDS_ORACLE").unwrap()).unwrap();
        assert_eq!(text.lines().count(), 39);
        let mut count = 0;
        for line in text.lines() {
            let f: Vec<_> = line.split_whitespace().collect();
            let vid = u16::from_str_radix(f[0], 16).unwrap();
            let pid = u16::from_str_radix(f[1], 16).unwrap();
            let actual = LEGACY_IDS.iter().find(|x| x.0 == vid && x.1 == pid);
            if f[2] != "4" {
                assert!(actual.is_none());
                continue;
            }
            let (_, _, Identity::Ax88772 { eeprom_mac }) = actual.unwrap() else {
                panic!("wrong protocol")
            };
            assert_eq!(&f[3..7], &["5", "0", "0", "6"]);
            assert_eq!(
                f[7],
                if (vid, pid) == (0x066b, 0x20f9) {
                    "0"
                } else {
                    "7"
                }
            );
            assert_eq!(&f[8..11], &["8288", "8", "9"]);
            assert_eq!(*eeprom_mac, f[11] == "1");
            count += 1;
        }
        assert_eq!(count, LEGACY_IDS.len());
        assert_eq!(count, 14);
    }
    #[test]
    fn polling_observes_latched_loss_and_reassembles_only_current_link_frames() {
        let (mut c, hw) = attached(0x20, PHY_C, true);
        hw.borrow_mut().delay(100_000);
        assert!(block_on(c.carrier()).unwrap());
        let n = 60u32;
        let header = (n | ((!n & 0xffff) << 16)).to_le_bytes();
        let mut part = header.to_vec();
        part.extend_from_slice(&[0x93; 20]);
        assert!(c.decode(&part).unwrap().is_empty());
        hw.borrow_mut().latched_low = true;
        assert!(!block_on(c.carrier()).unwrap());
        assert!(block_on(c.carrier()).unwrap());
        let mut frame = header.to_vec();
        frame.extend_from_slice(&[0x65; 60]);
        assert_eq!(c.decode(&frame).unwrap(), alloc::vec![alloc::vec![0x65;60]]);
        hw.borrow_mut().phy[0] &= !0x1000;
        assert!(block_on(c.carrier()).is_err());
    }
    #[test]
    fn phy_advertisement_is_intersected_with_basic_capabilities() {
        for mask in 0u16..16 {
            let hw = Rc::new(RefCell::new(model::Hardware::new(0x20, PHY_C)));
            hw.borrow_mut().phy[1] = AN_CAPABLE | (mask << 11);
            hw.borrow_mut().phy[4] = 0xa201;
            let result = block_on(Chip::attach(model::Io(hw.clone()), false));
            if mask == 0 {
                assert!(result.is_err());
                continue;
            }
            let (mut c, _) = result.unwrap();
            assert_eq!(
                hw.borrow().phy[4],
                (0xa201 & !ADV_MASK) | ADV_PAUSE | ADV_ASYM | (mask << 5)
            );
            let mut failures = 0;
            // Fault every operation of final PHY stop after a ready link.
            hw.borrow_mut().delay(100_000);
            block_on(c.carrier()).unwrap();
            let start = hw.borrow().trace.len();
            block_on(c.stop()).unwrap();
            let n = hw.borrow().trace.len() - start;
            for at in 0..n {
                let (mut other, other_hw) = attached(0x20, PHY_C, false);
                other_hw.borrow_mut().delay(100_000);
                assert!(block_on(other.carrier()).unwrap());
                let begin = other_hw.borrow().trace.len();
                other_hw.borrow_mut().fail_at = Some(begin + at);
                assert!(block_on(other.stop()).is_err());
                let end = other_hw.borrow().trace.len();
                assert!(block_on(other.stop()).is_err());
                assert_eq!(other_hw.borrow().trace.len(), end);
                failures += 1;
            }
            assert!(failures >= 10);
        }
    }
    mod minimum;
}
