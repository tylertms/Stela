use super::*;
use crate::usb::ethernet::{firmware::tests::synthetic_upload, rtl815x::tests::Model};
use alloc::{collections::BTreeMap, rc::Rc};
use core::cell::RefCell;
use stela_driver_base::stepped::block_on;

/// Stateful PHY/MCU model independent of the uploader: SRAM auto-increment,
/// asynchronous patch-ready, staged 2 KiB USB RAM download ownership and
/// firmware version changes occur only after complete, ordered payloads.
pub(crate) struct Hardware {
    pub(crate) core: Model,
    pub(crate) sram: BTreeMap<u16, u16>,
    pub(crate) sram2: BTreeMap<u16, u16>,
    pointer2: u16,
    pub(crate) mdi: [u16; 4],
    pub(crate) dsp: BTreeMap<u16, u16>,
    dsp_pointer: u16,
    pub(crate) flash_pending: Option<usize>,
    pub(crate) stuck_flash: bool,
    pointer: u16,
    patch_pending: Option<(bool, usize)>,
    speed_expected: Vec<u8>,
    speed_received: Vec<u8>,
    speed_version: u16,
    staged: usize,
    speed_pending: usize,
    pub(crate) stuck_patch: bool,
    pub(crate) stuck_speed: bool,
    wrong_version: bool,
    pub(crate) efuse: u32,
    pub(crate) phy_states: alloc::collections::VecDeque<u32>,
    pub(crate) ups_exit_power_cut: bool,
}

impl Hardware {
    pub(crate) fn new(revision: u16, bytes: &[u8]) -> Self {
        let mut hardware = Self {
            core: Model::new(revision),
            sram: BTreeMap::new(),
            sram2: BTreeMap::new(),
            pointer2: 0,
            mdi: [0; 4],
            dsp: BTreeMap::new(),
            dsp_pointer: 0,
            flash_pending: None,
            stuck_flash: false,
            pointer: 0,
            patch_pending: None,
            speed_expected: Vec::new(),
            speed_received: Vec::new(),
            speed_version: 0,
            staged: 0,
            speed_pending: 0,
            stuck_patch: false,
            stuck_speed: false,
            wrong_version: false,
            efuse: 0x1ffff,
            phy_states: Default::default(),
            ups_exit_power_cut: false,
        };
        // Independent raw wire inventory of a speed-up block: do not read
        // expectations back from the production parser/uploader objects.
        let mut at = 64;
        while at + 8 <= bytes.len() {
            let kind = u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
            let length = u32::from_le_bytes(bytes[at + 4..at + 8].try_into().unwrap()) as usize;
            if kind == 0 {
                break;
            }
            if kind == 13 {
                let offset = usize::from(u16::from_le_bytes(
                    bytes[at + 8..at + 10].try_into().unwrap(),
                ));
                hardware.speed_version =
                    u16::from_le_bytes(bytes[at + 10..at + 12].try_into().unwrap());
                hardware.speed_expected = bytes[at + offset..at + length].to_vec();
            }
            at += length.next_multiple_of(8);
        }
        hardware
    }

    pub(crate) fn request(
        &mut self,
        request: [u8; 8],
        output: &[u8],
    ) -> Result<Vec<u8>, BlockError> {
        let address = u32::from(u16::from_le_bytes([request[2], request[3]]));
        let bank = u16::from_le_bytes([request[4], request[5]]) & 0xff00;
        let mapped = if bank == PLA && address & 0xf000 == 0xb000 {
            (address & 0xfff) | self.core.base
        } else {
            address
        };
        let lanes = request[4] & 0xf;
        let target = mapped
            + match lanes {
                2 => 1,
                4 | 12 => 2,
                8 => 3,
                _ => 0,
            };
        let read = request[0] == 0xc0;
        if read
            && bank == USB
            && mapped == USB_GPHY_CTRL
            && !self.stuck_flash
            && let Some(remaining) = &mut self.flash_pending
        {
            *remaining -= 1;
            if *remaining == 0 {
                self.flash_pending = None;
                let value = self.core.get(USB, USB_GPHY_CTRL, 2);
                self.core
                    .set(USB, USB_GPHY_CTRL, 2, value | GPHY_PATCH_DONE);
            }
        }
        if read && bank == PLA {
            // SRAM2 data is the upper half of the aligned four-byte read.
            if mapped == OCP_SRAM2_ADDR {
                self.core.set(
                    PLA,
                    OCP_SRAM2_DATA,
                    2,
                    u32::from(*self.sram2.get(&self.pointer2).unwrap_or(&0)),
                );
            }
            if mapped == 0xd068 {
                let port = (self.core.get(PLA, 0xd068, 2) >> 3 & 3) as usize;
                self.core.set(PLA, 0xd06a, 2, u32::from(self.mdi[port]));
            }

            if target == OCP_PHY_STATUS
                && let Some(state) = self.phy_states.pop_front()
            {
                self.core.set(PLA, OCP_PHY_STATUS, 2, state);
            }
            if address == PLA_EFUSE_DATA {
                let command = self.core.get(PLA, PLA_EFUSE_CMD, 2);
                self.core.set(PLA, PLA_EFUSE_DATA, 2, self.efuse & 0xffff);
                self.core.set(
                    PLA,
                    PLA_EFUSE_CMD,
                    2,
                    (command & !EFUSE_DATA_BIT16)
                        | if self.efuse & 0x10000 != 0 {
                            EFUSE_DATA_BIT16
                        } else {
                            0
                        },
                );
            }
            if target == OCP_SRAM_DATA {
                self.core.set(
                    PLA,
                    OCP_SRAM_DATA,
                    2,
                    u32::from(*self.sram.get(&self.pointer).unwrap_or(&0)),
                );
            }
            if target == OCP_PHY_PATCH_STAT
                && !self.stuck_patch
                && let Some((ready, count)) = &mut self.patch_pending
            {
                *count -= 1;
                if *count == 0 {
                    self.core.set(
                        PLA,
                        OCP_PHY_PATCH_STAT,
                        2,
                        if *ready { PATCH_READY } else { 0 },
                    );
                    self.patch_pending = None;
                }
            }
            if target == PLA_POL_GPIO_CTRL && self.speed_pending != 0 && !self.stuck_speed {
                self.speed_pending -= 1;
                if self.speed_pending == 0 {
                    let gpio = self.core.get(PLA, PLA_POL_GPIO_CTRL, 2);
                    self.core
                        .set(PLA, PLA_POL_GPIO_CTRL, 2, gpio & !POL_GPHY_PATCH);
                    if self.speed_received.len() == self.speed_expected.len() && !self.wrong_version
                    {
                        self.sram
                            .insert(SRAM_GPHY_FW_VER as u16, self.speed_version);
                    }
                }
            }
        }
        let reply = self.core.request(request, output)?;
        if !read && bank == USB && target == USB_MISC_2 && self.ups_exit_power_cut {
            self.ups_exit_power_cut = false;
            self.core.set(USB, USB_MISC_0, 2, PCUT_STATUS);
        }
        if !read && bank == PLA {
            if target == 0xb88e {
                self.dsp_pointer = self.core.get(PLA, 0xb88e, 2) as u16;
            }
            if target == 0xb890 && (self.core.get(PLA, PLA_TCR0, 4) >> 16 & VERSION_MASK) == 0x7400
            {
                assert_eq!(
                    self.core.get(PLA, OCP_PHY_PATCH_STAT, 2) & PATCH_READY,
                    PATCH_READY
                );
                assert_eq!(self.core.get(PLA, 0xb896, 2) & 1, 0);
                self.dsp
                    .insert(self.dsp_pointer, self.core.get(PLA, 0xb890, 2) as u16);
            }

            if target == OCP_SRAM2_ADDR {
                self.pointer2 = self.core.get(PLA, OCP_SRAM2_ADDR, 2) as u16;
            }
            if target == OCP_SRAM2_DATA {
                self.sram2
                    .insert(self.pointer2, self.core.get(PLA, OCP_SRAM2_DATA, 2) as u16);
                self.pointer2 = self.pointer2.wrapping_add(2);
            }
            if target == 0xd06a {
                let port = (self.core.get(PLA, 0xd068, 2) >> 3 & 3) as usize;
                self.mdi[port] = self.core.get(PLA, 0xd06a, 2) as u16;
            }

            if target == OCP_SRAM_ADDR {
                self.pointer = self.core.get(PLA, OCP_SRAM_ADDR, 2) as u16;
            }
            if target == OCP_SRAM_DATA {
                self.sram
                    .insert(self.pointer, self.core.get(PLA, OCP_SRAM_DATA, 2) as u16);
                self.pointer = self.pointer.wrapping_add(2);
            }
            if target == OCP_PHY_PATCH_CMD {
                self.patch_pending = Some((
                    self.core.get(PLA, OCP_PHY_PATCH_CMD, 2) & PATCH_REQUEST != 0,
                    3,
                ));
            }
            if target == PLA_POL_GPIO_CTRL
                && self.core.get(PLA, PLA_POL_GPIO_CTRL, 2) & POL_GPHY_PATCH != 0
            {
                assert_eq!(
                    self.core.get(USB, USB_GPHY_CTRL, 2) & (GPHY_PATCH_DONE | BACKUP_RESTRORE),
                    GPHY_PATCH_DONE | BACKUP_RESTRORE
                );
                assert!(self.staged > 0 && self.staged <= 2048);
                let start = self.speed_received.len();
                let chunk: Vec<u8> = (0..self.staged)
                    .map(|i| self.core.get(USB, 0x9a00 + i as u32, 1) as u8)
                    .collect();
                assert_eq!(chunk, self.speed_expected[start..start + self.staged]);
                self.speed_received.extend(chunk);
                self.staged = 0;
                self.speed_pending = 3;
            }
        }
        if !read && bank == USB && (0x9a00..0xa200).contains(&address) {
            assert_eq!(address as usize - 0x9a00, self.staged);
            self.staged += output.len();
        }
        Ok(reply)
    }
}
pub(crate) struct Io(pub(crate) Rc<RefCell<Hardware>>);
impl Control for Io {
    async fn control(&mut self, request: [u8; 8], output: &[u8]) -> Result<Vec<u8>, BlockError> {
        self.0.borrow_mut().request(request, output)
    }
    async fn delay_us(&mut self, us: u64) {
        self.0.borrow_mut().core.delays.push(us);
    }
    async fn minimum_wait(&mut self, us: u64) -> Result<(), BlockError> {
        let mut state = self.0.borrow_mut();
        state.core.minimum_elapsed = state
            .core
            .minimum_elapsed
            .checked_add(us)
            .ok_or(BlockError::PciPower(crate::bus::pci::power::Error::Clock))?;
        state.core.delays.push(us);
        Ok(())
    }
}

fn execute(bytes: &[u8], revision: u16) -> (Chip<Io>, Rc<RefCell<Hardware>>) {
    let model = Rc::new(RefCell::new(Hardware::new(revision, bytes)));
    let mut chip = block_on(Chip::probe(Io(model.clone()), Revision(revision))).unwrap();
    block_on(chip.upload_firmware(&Firmware::parse(bytes, Revision(revision)).unwrap(), false))
        .unwrap();
    assert!(!chip.initialized);
    assert!(block_on(chip.carrier()).is_err());
    assert_eq!(chip.base, None);
    (chip, model)
}

#[test]
fn link_effects_require_completion_before_carrier() {
    for revision in [
        0x5c10, 0x5c20, 0x5c30, 0x6010, 0x6400, 0x7030, 0x7410, 0x7420,
    ] {
        let bytes = synthetic_upload(revision);
        let (mut chip, model) = execute(&bytes, revision);
        {
            let state = model.borrow();
            let core = &state.core;
            assert_eq!(core.get(USB, 0xcfd7, 1), 4);
            assert_eq!(core.get(USB, 0xcfd6, 1), 6);
            assert_eq!(core.get(PLA, 0xf800, 4), 0x5a5a_5a5a);
            if matches!(revision, 0x7410 | 0x7420) {
                assert_eq!(state.speed_received.len(), 4224);
                assert_eq!(state.sram[&(SRAM_GPHY_FW_VER as u16)], 0x21);
            }
        }
        let before = model.borrow().core.trace.len();
        block_on(
            chip.upload_firmware(&Firmware::parse(&bytes, Revision(revision)).unwrap(), false),
        )
        .unwrap();
        let model = model.borrow();
        assert!(
            !model.core.trace[before..]
                .iter()
                .any(|(request, data)| request[0] == 0x40
                    && u16::from_le_bytes([request[2], request[3]]) == 0xf800
                    && data.len() > 4)
        );
    }
}

#[test]
fn failures_at_every_mac_upload_request_poison_and_forbid_retries() {
    let bytes = synthetic_upload(0x6010);
    let (_, model) = execute(&bytes, 0x6010);
    let count = model.borrow().core.trace.len();
    for at in 1..count {
        let model = Rc::new(RefCell::new(Hardware::new(0x6010, &bytes)));
        let mut chip = block_on(Chip::probe(Io(model.clone()), Revision(0x6010))).unwrap();
        model.borrow_mut().core.fail_at = Some(at);
        let firmware = Firmware::parse(&bytes, Revision(0x6010)).unwrap();
        assert!(block_on(chip.upload_firmware(&firmware, false)).is_err());
        assert!(!chip.accessible);
        assert_eq!(model.borrow().core.trace.len(), at + 1);
        assert!(block_on(chip.upload_firmware(&firmware, false)).is_err());
        assert_eq!(model.borrow().core.trace.len(), at + 1);
    }
}

#[test]
fn patch_ready_speedup_and_final_version_failures_remain_explicit() {
    for case in 0..3 {
        let revision = if case == 0 { 0x5c10 } else { 0x7410 };
        let bytes = synthetic_upload(revision);
        let model = Rc::new(RefCell::new(Hardware::new(revision, &bytes)));
        match case {
            0 => model.borrow_mut().stuck_patch = true,
            1 => model.borrow_mut().stuck_speed = true,
            _ => model.borrow_mut().wrong_version = true,
        }
        let mut chip = block_on(Chip::probe(Io(model.clone()), Revision(revision))).unwrap();
        let firmware = Firmware::parse(&bytes, Revision(revision)).unwrap();
        assert!(block_on(chip.upload_firmware(&firmware, false)).is_err());
        assert!(!chip.accessible);
        assert!(!chip.initialized);
        if case == 1 {
            assert_eq!(
                model.borrow().core.get(PLA, OCP_PHY_PATCH_CMD, 2) & PATCH_REQUEST,
                0
            );
        }
        let count = model.borrow().core.trace.len();
        assert!(block_on(chip.upload_firmware(&firmware, false)).is_err());
        assert_eq!(model.borrow().core.trace.len(), count);
    }
}

#[test]
#[ignore = "oracle: STELA_RTL815X_FIRMWARE"]
fn real_firmware_uploads_preserve_payloads_order_and_final_version() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let directory = std::env::var("STELA_RTL815X_FIRMWARE").unwrap();
    for (name, revision, _, hash) in crate::usb::ethernet::firmware::tests::FIXTURES {
        let bytes = std::fs::read(std::path::Path::new(&directory).join(name)).unwrap();
        assert_eq!(alloc::format!("{:x}", Sha256::digest(&bytes)), hash);
        execute(&bytes, revision);
    }
}
