// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
// Copyright (C) 2020-2023 MediaTek Inc.
//! Initial source MAC/WTBL/basic-rate effects, with the explicit raw-802.11
//! global translation profile. These observations grant no station or policy.
use super::{
    BarIo, Control, Error, MT_CONN_ON_LPCTL, MT_WFDMA0_TX_RING0_EXT_CTRL, Observation, rx_ring,
    tx_ring,
};
use crate::wifi::{
    mediatek::identity::Chip,
    stepped::{block_on, until},
};

#[derive(Debug)]
pub struct MacInitialized {
    pub global_format: Observation,
    pub wtbl: [Observation; 20],
    pub fixed_rate_entries: u8,
}
impl<I: BarIo> Control<I> {
    pub fn chip(&self) -> Chip {
        self.registers.chip()
    }
    /// C2 normal-mode write precedes the source PCI MCU driver-own
    /// handshake. C3 first performs its source FW-own→driver-own sequence.
    /// All operations keep the existing exclusive controller and real clock.
    pub async fn prepare_firmware(&mut self, aspm_supported: bool) -> Result<(), Error> {
        self.guard(false)?;
        let result = async {
            if self.registers.chip() != Chip::Mt7925 {
                self.registers.write32(0x41f23c, 0)?;
                let actual = self.read_control(0x41f23c)?;
                if actual != 0 {
                    return Err(Error::Readback {
                        address: 0x41f23c,
                        mask: u32::MAX,
                        expected: 0,
                        actual,
                    });
                }
                self.driver_owned = false;
                // mt7921e_driver_own uses the TOP LPCR FW-own bit, rather
                // than substituting the common driver's bit2 sync poll.
                self.registers.write32(0x18060010, 2)?;
                self.poll(0x18060010, 1, 0, 500_000).await?;
                self.driver_owned = true;
            } else {
                self.driver_owned = false;
                for attempt in 0..10 {
                    self.registers.write32(MT_CONN_ON_LPCTL, 1)?;
                    match self.poll(MT_CONN_ON_LPCTL, 4, 4, 50_000).await {
                        Ok(_) => break,
                        Err(Error::Timeout { .. }) if attempt < 9 => {}
                        Err(e) => return Err(e),
                    }
                }
                self.driver_own(aspm_supported).await?;
            }
            self.modify(0x10194, 0, 1 << 8)?;
            let actual = self.read_control(0x10194)?;
            if actual & (1 << 8) == 0 {
                return Err(Error::Readback {
                    address: 0x10194,
                    mask: 1 << 8,
                    expected: 1 << 8,
                    actual,
                });
            }
            Ok(())
        }
        .await;
        self.finish(result)
    }
    fn mac_modify(&mut self, address: u32, mask: u32, set: u32) -> Result<(), Error> {
        self.registers.modify32(address, mask, set)?;
        Ok(())
    }
    fn mac_band(&mut self, band: u32) -> Result<(), Error> {
        let stride = band * 0x10000;
        self.mac_modify(0x820e40f4 + stride, 0x3f, 0x3f)?;
        self.mac_modify(0x820e40f4 + stride, 0, (1 << 17) | (1 << 18))?;
        self.mac_modify(0x820e53c4 + stride, 0, 1 << 30)?;
        self.mac_modify(0x820e5380 + stride, 0, 1 << 30)?;
        self.mac_modify(0x820ed004 + stride, 0, 1 << 8)?;
        self.mac_modify(0x820ed004 + stride, 0, 1 << 9)?;
        self.mac_modify(0x820e7000 + stride, 0xfff8, 1536 << 3)?;
        self.mac_modify(0x820e7000 + stride, 1 << 23, 0)?;
        self.mac_modify(0x820e9008 + stride, (3 << 30) | (3 << 24), 3 << 24)?;
        Ok(())
    }
    async fn wtbl_clear_admission(&mut self, index: u32) -> Result<Observation, Error> {
        let c3 = self.registers.chip() == Chip::Mt7925;
        let address = if c3 { 0x820d4380 } else { 0x820d4230 };
        self.mac_modify(
            address,
            if c3 { 0xfff } else { 0x3ff },
            index | if c3 { 1 << 14 } else { 1 << 12 },
        )?;
        // Full source __mt76_poll:501 possible reads,10us delay on each
        // failed observation. Source mac_init ignores timeout; this profile
        // explicitly refuses and poisons instead of publishing false success.
        let mut last = 0;
        for reads in 1..=501 {
            last = self.read_control(address)?;
            if last & (1 << 31) == 0 {
                return Ok(Observation {
                    address,
                    value: last,
                    reads,
                });
            }
            until(self.clock.deadline(10)).await;
        }
        Err(Error::Timeout {
            address,
            mask: 1 << 31,
            expected: 0,
            last,
        })
    }
    pub(in crate::wifi::mediatek) fn clear_peer_admission(
        &mut self,
        index: u16,
    ) -> Result<Observation, Error> {
        self.guard(true)?;
        // A connected peer's clear: the same poll, at most 501 reads 10 us
        // apart, run to completion within the caller's step.
        let result = block_on(self.wtbl_clear_admission(u32::from(index)));
        self.finish(result)
    }
    /// # Safety
    /// This is initial post-EFUSE MAC setup on freshly booted firmware
    /// with no live peer/key/WTBL/TX/RX consumer. The owning lifecycle serializes
    /// the source whole-WTBL admission-counter clear and basic-rate writes with
    /// all real users and keeps all owners through any partial failure.
    /// Both later firmware/per-WCID translation controls must also be disabled
    /// before claiming that a peer's payload is raw80211. This method itself
    /// establishes only the global bit and source initialization effects.
    pub async unsafe fn initialize_mac_raw(&mut self) -> Result<MacInitialized, Error> {
        self.guard(true)?;
        let result = async {
            let c3 = self.registers.chip() == Chip::Mt7925;
            let mdp = if c3 { 0x820cc800 } else { 0x820cd000 };
            self.mac_modify(mdp + 4, 0xfff8, 1536 << 3)?;
            self.mac_modify(mdp, 0, 1 << 15)?;
            // Explicit supported global raw profile. C2 donor sets this bit;
            // C3 donor leaves it untouched. Neither may silently enable a
            // descriptor layout rejected by the native 802.11 decoder.
            self.mac_modify(mdp, 1 << 19, 0)?;
            let mut wtbl = [Observation {
                address: 0,
                value: 0,
                reads: 0,
            }; 20];
            for (index, receipt) in wtbl.iter_mut().enumerate() {
                *receipt = self.wtbl_clear_admission(index as u32).await?;
            }
            for band in 0..2 {
                self.mac_band(band)?;
            }
            let rates = [
                0u16, 1, 2, 3, 0x4b, 0x4f, 0x4a, 0x4e, 0x49, 0x4d, 0x48, 0x4c,
            ];
            if c3 {
                for (index, rate) in rates.into_iter().enumerate() {
                    self.registers.write32(0x820d43b8, u32::from(rate))?;
                    self.registers.write32(0x820d43bc, 1 << 6)?;
                    self.registers
                        .write32(0x820d43b0, (1 << 16) | (1 << 31) | (14 + index as u32))?;
                }
            }
            let value = self.read_control(mdp)?;
            if value & (1 << 19) != 0 {
                return Err(Error::Readback {
                    address: mdp,
                    mask: 1 << 19,
                    expected: 0,
                    actual: value,
                });
            }
            Ok(MacInitialized {
                global_format: Observation {
                    address: mdp,
                    value,
                    reads: 1,
                },
                wtbl,
                fixed_rate_entries: if c3 { 12 } else { 0 },
            })
        }
        .await;
        self.finish(result)
    }
}

impl<I: BarIo> Control<I> {
    pub(in crate::wifi::mediatek) fn disable_global_translation(&mut self) -> Result<(), Error> {
        self.guard(true)?;
        let result = (|| {
            let address = if self.chip() == Chip::Mt7925 {
                0x820cc800
            } else {
                0x820cd000
            };
            self.mac_modify(address, 1 << 19, 0)?;
            let actual = self.read_control(address)?;
            if actual & (1 << 19) != 0 {
                return Err(Error::Readback {
                    address,
                    mask: 1 << 19,
                    expected: 0,
                    actual,
                });
            }
            Ok(())
        })();
        self.finish(result)
    }
    pub(in crate::wifi::mediatek) fn initial_tx_ext(&mut self) -> Result<(), Error> {
        self.guard(true)?;
        let r = self
            .registers
            .write32(MT_WFDMA0_TX_RING0_EXT_CTRL, 4)
            .map_err(Error::from);
        self.finish(r)
    }
    /// Empty every ring outside this chip's layout that another owner (an
    /// earlier OS's driver, or a reset default) left with a count, while the
    /// engine is stopped, so none stays reachable once WFDMA is enabled.
    /// Linux never reads these rings; refusing the device for them would be
    /// stricter than it. Returns how many rings were emptied.
    pub(in crate::wifi::mediatek) fn clear_unused_queues(&mut self) -> Result<u8, Error> {
        self.guard(true)?;
        let result = (|| {
            let c3 = self.chip() == Chip::Mt7925;
            let tx = (0..=17)
                .filter(|&qid| qid != 0 && qid != 16 && qid != if c3 { 15 } else { 17 })
                .map(tx_ring);
            let rx = (0..=5)
                .filter(|&qid| qid != 0 && qid != 2 && (c3 || qid != 4))
                .map(rx_ring);
            let mut cleared = 0;
            for ring in tx.chain(rx) {
                if self.read_control(ring + 4)? == 0 {
                    continue;
                }
                // Base, count, then CPU index: the ring holds nothing.
                for register in [ring, ring + 4, ring + 8] {
                    self.registers.write32(register, 0)?;
                }
                let actual = self.read_control(ring + 4)?;
                if actual != 0 {
                    return Err(Error::Readback {
                        address: ring + 4,
                        mask: u32::MAX,
                        expected: 0,
                        actual,
                    });
                }
                cleared += 1;
            }
            Ok(cleared)
        })();
        self.finish(result)
    }
    /// Read-to-clear and airtime reset effects. Host accounting/time
    /// is held by the lifecycle; no air/duration result is invented.
    pub(in crate::wifi::mediatek) fn reset_mac_counters(&mut self) -> Result<[u32; 11], Error> {
        self.guard(true)?;
        let result = (|| {
            let mut reads = [0; 11];
            for i in 0..4 {
                reads[2 * i] = self.registers.read32(0x820ed7dc + 4 * i as u32)?;
                reads[2 * i + 1] = self.registers.read32(0x820ed7ec + 4 * i as u32)?;
            }
            for (i, address) in [0x820ed02c, 0x820ed054, 0x820ed058].into_iter().enumerate() {
                reads[8 + i] = self.registers.read32(address)?;
            }
            self.mac_modify(0x820e53c4, 0, 1 << 31)?;
            self.mac_modify(0x820e5380, 0, 1 << 31)?;
            Ok(reads)
        })();
        self.finish(result)
    }
}
