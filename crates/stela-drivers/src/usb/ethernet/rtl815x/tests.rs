use super::*;
use crate::usb::descriptor::*;
use crate::usb::ethernet;
use crate::usb::xhci::{model, model::*, *};
use alloc::string::String;
use alloc::{collections::BTreeMap, rc::Rc};
use core::cell::RefCell;
use stela_driver_base::stepped::block_on;

/// Independent byte-lane/OCP model. The window maps through the hardware
/// base register, partial writes preserve disabled lanes, and reset/LL
/// completion is driven by later reads instead of write echo.
pub(crate) struct Model {
    registers: BTreeMap<(u16, u32), u8>,
    pub(super) base: u32,
    reset: usize,
    phy_reset: usize,
    pub trace: Vec<([u8; 8], Vec<u8>)>,
    pub delays: Vec<u64>,
    pub minimum_elapsed: u64,
    pub fail_at: Option<usize>,
    pub refuse_minimum: bool,
    pub short_at: Option<usize>,
    pub link: bool,
    pub(crate) phy_status: u32,
    pub stuck_reset: bool,
    pub stuck_phy: bool,
    pub stuck_list: bool,
}

impl Model {
    pub(crate) fn new(revision: u16) -> Self {
        let mut model = Self {
            registers: BTreeMap::new(),
            base: 0,
            reset: 0,
            phy_reset: 0,
            trace: Vec::new(),
            delays: Vec::new(),
            minimum_elapsed: 0,
            fail_at: None,
            refuse_minimum: false,
            short_at: None,
            link: true,
            phy_status: 0x0a,
            stuck_reset: false,
            stuck_phy: false,
            stuck_list: false,
        };
        model.set(
            PLA,
            PLA_TCR0,
            4,
            (u32::from(revision) << 16) | TCR0_TX_EMPTY,
        );
        model.set(PLA, PLA_BACKUP, 4, 0x33221102);
        model.set(PLA, PLA_BACKUP + 4, 4, 0x5544);
        model.set(PLA, PLA_IDR, 4, 0x33221102);
        model.set(PLA, PLA_IDR + 4, 4, 0x5544);
        model.set(PLA, PLA_OOB_CTRL, 1, NOW_IS_OOB | LINK_LIST_READY | 0x30);
        model.set(PLA, PLA_BOOT_CTRL, 2, AUTOLOAD_DONE);
        model.set(PLA, OCP_PHY_STATUS, 2, PHY_STAT_LAN_ON);
        model.set(PLA, 0xe000, 2, 0x0100);
        model
    }

    pub(super) fn get(&self, bank: u16, address: u32, width: u32) -> u32 {
        (0..width).fold(0, |value, lane| {
            value
                | (u32::from(*self.registers.get(&(bank, address + lane)).unwrap_or(&0))
                    << (lane * 8))
        })
    }
    pub(super) fn set(&mut self, bank: u16, address: u32, width: u32, value: u32) {
        for lane in 0..width {
            self.registers
                .insert((bank, address + lane), (value >> (lane * 8)) as u8);
        }
    }

    pub(crate) fn request(&mut self, setup: [u8; 8], output: &[u8]) -> Result<Vec<u8>, BlockError> {
        let at = self.trace.len();
        self.trace.push((setup, output.to_vec()));
        if self.fail_at == Some(at) {
            return Err(BlockError::Timeout);
        }
        assert_eq!(setup[1], 5);
        let length = usize::from(u16::from_le_bytes([setup[6], setup[7]]));
        assert!(length > 0 && length <= 512 && length.is_multiple_of(4));
        let address = u32::from(u16::from_le_bytes([setup[2], setup[3]]));
        assert_eq!(address & 3, 0);
        let index = u16::from_le_bytes([setup[4], setup[5]]);
        let bank = index & 0xff00;
        assert!(matches!(bank, USB | PLA));
        let mapped = if bank == PLA && address & 0xf000 == 0xb000 {
            (address & 0xfff) | self.base
        } else {
            address
        };
        match setup[0] {
            0xc0 => {
                assert_eq!(length, 4);
                assert!(output.is_empty());
                if address == PLA_CR & !3 && self.reset > 0 && !self.stuck_reset {
                    self.reset -= 1;
                    if self.reset == 0 {
                        self.set(PLA, PLA_CR, 1, 0);
                    }
                }
                if mapped == MII_BMCR && self.phy_reset > 0 && !self.stuck_phy {
                    self.phy_reset -= 1;
                    if self.phy_reset == 0 {
                        self.set(PLA, MII_BMCR, 2, BMCR_ANENABLE_RESTART);
                    }
                }
                if address == PLA_PHYSTATUS {
                    self.set(
                        PLA,
                        PLA_PHYSTATUS,
                        2,
                        if self.link { self.phy_status } else { 0 },
                    );
                }
                let mut value = self.get(bank, mapped, 4);
                if address == PLA_OOB_CTRL & !3 && self.stuck_list {
                    value &= !(LINK_LIST_READY << 24);
                }
                let mut bytes = value.to_le_bytes().to_vec();
                if self.short_at == Some(at) {
                    bytes.pop();
                }
                Ok(bytes)
            }
            0x40 => {
                assert_eq!(output.len(), length);
                let enables = index as u8;
                assert_eq!(enables & 0xf, enables >> 4);
                let lanes = enables & 0xf;
                assert!(matches!(lanes, 1 | 2 | 4 | 8 | 3 | 12 | 15));
                let teredo_events = self.get(PLA, PLA_TEREDO_CFG, 1);
                let teredo_w1c = bank == PLA
                    && mapped == PLA_TEREDO_CFG
                    && lanes & 1 != 0
                    && (self.get(PLA, PLA_TCR0, 4) >> 16 & VERSION_MASK) >= 0x6000;
                for (lane, byte) in output.iter().enumerate() {
                    if lanes & (1 << (lane % 4)) != 0 {
                        self.registers.insert((bank, mapped + lane as u32), *byte);
                    }
                }
                if teredo_w1c {
                    self.set(
                        PLA,
                        PLA_TEREDO_CFG,
                        1,
                        teredo_events & !u32::from(output[0]),
                    );
                }
                if address == PLA_OCP_GPHY_BASE {
                    self.base = self.get(PLA, PLA_OCP_GPHY_BASE, 2);
                }
                if address == PLA_CR & !3 && self.get(PLA, PLA_CR, 1) & CR_RST != 0 {
                    self.reset = 3;
                }
                if mapped == MII_BMCR && self.get(PLA, MII_BMCR, 2) & BMCR_RESET != 0 {
                    self.phy_reset = 3;
                }
                Ok(Vec::new())
            }
            _ => panic!("unexpected vendor control type"),
        }
    }
}

pub(crate) struct Io(pub Rc<RefCell<Model>>);
impl Control for Io {
    async fn control(&mut self, setup: [u8; 8], output: &[u8]) -> Result<Vec<u8>, BlockError> {
        self.0.borrow_mut().request(setup, output)
    }
    async fn delay_us(&mut self, us: u64) {
        self.0.borrow_mut().delays.push(us);
    }
    async fn minimum_wait(&mut self, us: u64) -> Result<(), BlockError> {
        let mut state = self.0.borrow_mut();
        state.delays.push(us);
        if state.refuse_minimum {
            return Err(BlockError::PciPower(
                crate::bus::pci::power::Error::Interrupted,
            ));
        }
        state.minimum_elapsed = state
            .minimum_elapsed
            .checked_add(us)
            .ok_or(BlockError::PciPower(crate::bus::pci::power::Error::Clock))?;
        Ok(())
    }
}

#[test]
fn rtl8152_revisions_initialize_both_usb_fifo_modes_and_follow_carrier() {
    for revision in [0x4c00, 0x4c10, 0x4800] {
        for full_speed in [false, true] {
            let model = Rc::new(RefCell::new(Model::new(revision)));
            let (mut chip, mac) = block_on(Chip::attach(
                Io(model.clone()),
                Revision(revision),
                full_speed,
            ))
            .unwrap();
            assert_eq!(mac, Mac([2, 0x11, 0x22, 0x33, 0x44, 0x55]));
            assert_eq!(
                model.borrow().get(PLA, PLA_RXFIFO_CTRL1, 4),
                if full_speed { 0x60 } else { 0x38 }
            );
            assert_eq!(
                model.borrow().get(PLA, PLA_RXFIFO_CTRL2, 4),
                if full_speed { 0x78 } else { 0x48 }
            );
            assert_eq!(model.borrow().get(PLA, PLA_CR, 1), 0);
            assert!(block_on(chip.carrier()).unwrap());
            assert_eq!(model.borrow().get(PLA, PLA_CR, 1), 12);
            assert_eq!(model.borrow().get(PLA, PLA_RCR, 4) & 15, 14);
            model.borrow_mut().link = false;
            assert!(!block_on(chip.carrier()).unwrap());
            block_on(chip.stop()).unwrap();
            assert_eq!(model.borrow().get(PLA, PLA_CR, 1), 0);
            model.borrow_mut().link = true;
            assert!(block_on(chip.carrier()).unwrap());
        }
    }
}

#[test]
fn no_further_register_access_after_any_failed_initialization_transaction() {
    for revision in [0x4c00, 0x4c10, 0x4800] {
        let model = Rc::new(RefCell::new(Model::new(revision)));
        block_on(Chip::attach(Io(model.clone()), Revision(revision), false)).unwrap();
        let operations = model.borrow().trace.len();
        assert!(operations > 80);
        for fail_at in 0..operations {
            let model = Rc::new(RefCell::new(Model::new(revision)));
            model.borrow_mut().fail_at = Some(fail_at);
            assert!(block_on(Chip::attach(Io(model.clone()), Revision(revision), false)).is_err());
            assert_eq!(model.borrow().trace.len(), fail_at + 1);
        }
    }
    let model = Rc::new(RefCell::new(Model::new(0x4c10)));
    let (mut chip, _) = block_on(Chip::attach(Io(model.clone()), Revision(0x4c10), false)).unwrap();
    let next = model.borrow().trace.len();
    model.borrow_mut().short_at = Some(next);
    assert!(block_on(chip.carrier()).is_err());
    assert!(block_on(chip.carrier()).is_err());
    assert_eq!(model.borrow().trace.len(), next + 1);
}

#[test]
fn revision01_reads_idr_preserves_station_and_masks_only_led_mode() {
    for full_speed in [false, true] {
        let model = Rc::new(RefCell::new(Model::new(0x4c00)));
        model.borrow_mut().set(PLA, PLA_IDR, 4, 0x05040302);
        model.borrow_mut().set(PLA, PLA_IDR + 4, 4, 0x88770706);
        model.borrow_mut().set(PLA, PLA_BACKUP, 4, 0);
        model.borrow_mut().set(PLA, PLA_BACKUP + 4, 4, 0);
        model.borrow_mut().set(PLA, PLA_LED_FEATURE, 2, 0xa75a);
        let (mut chip, mac) = block_on(Chip::attach(
            Io(model.clone()),
            Revision(0x4c00),
            full_speed,
        ))
        .unwrap();
        assert_eq!(mac.0, [2, 3, 4, 5, 6, 7]);
        assert_eq!(model.borrow().get(PLA, PLA_LED_FEATURE, 2), 0xa05a);
        assert_eq!(model.borrow().get(PLA, PLA_IDR + 4, 4), 0x88770706);
        assert!(model.borrow().trace.iter().all(|(request, _)| {
            request[0] != 0x40
                || !matches!(
                    u16::from_le_bytes([request[2], request[3]]),
                    0xc000 | 0xc004
                )
        }));
        assert!(block_on(chip.carrier()).unwrap());
        block_on(chip.stop()).unwrap();
    }
    for case in 0..3 {
        let model = Rc::new(RefCell::new(Model::new(0x4c00)));
        match case {
            0 => model.borrow_mut().stuck_reset = true,
            1 => model.borrow_mut().stuck_phy = true,
            _ => model.borrow_mut().stuck_list = true,
        }
        assert!(block_on(Chip::attach(Io(model), Revision(0x4c00), false)).is_err());
    }
}

#[test]
fn an_invalid_station_address_is_replaced_by_a_programmed_random_one() {
    // determine_ethernet_addr: an invalid address gets eth_hw_addr_random;
    // set_ethernet_addr writes it to IDR on every revision but 01.
    let model = Rc::new(RefCell::new(Model::new(0x4c00)));
    model.borrow_mut().set(PLA, PLA_IDR, 4, 0x05040303);
    let (_, mac) = block_on(Chip::attach(Io(model), Revision(0x4c00), false)).unwrap();
    assert!(mac.is_valid() && mac.0[0] & 2 != 0, "{mac}");
    let model = Rc::new(RefCell::new(Model::new(0x4c10)));
    model.borrow_mut().set(PLA, PLA_BACKUP, 4, 0);
    model.borrow_mut().set(PLA, PLA_BACKUP + 4, 4, 0);
    let (_, mac) = block_on(Chip::attach(Io(model.clone()), Revision(0x4c10), false)).unwrap();
    assert!(mac.is_valid() && mac.0[0] & 2 != 0, "{mac}");
    let [a, b, c, d, e, f] = mac.0;
    assert_eq!(
        model.borrow().get(PLA, PLA_IDR, 4),
        u32::from_le_bytes([a, b, c, d])
    );
    assert_eq!(
        model.borrow().get(PLA, PLA_IDR + 4, 2),
        u32::from(u16::from_le_bytes([e, f]))
    );
}

#[test]
#[ignore = "oracle: STELA_RTL8152_REVISION_ORACLE"]
fn unchanged_linux_rtl8152_revision_init_and_station_effects() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let trace =
        std::fs::read_to_string(std::env::var("STELA_RTL8152_REVISION_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(trace.as_bytes())),
        "982fba3636eed37444bd94b3a862144b26731b163e72bbbe0d7cd1b2b128d4a2"
    );
    let fields = [
        (PLA, PLA_LED_FEATURE, 2),
        (USB, USB_UPS_CTRL, 2),
        (USB, USB_PM_CTRL_STATUS, 2),
        (PLA, PLA_PHY_PWR, 2),
        (PLA, PLA_MAC_PWR_CTRL, 4),
        (PLA, PLA_GPHY_INTR_IMR, 2),
        (PLA, PLA_RSTTALLY, 2),
        (USB, USB_USB_CTRL, 2),
    ];
    let mut count = 0;
    for line in trace.lines() {
        let values: Vec<_> = line.split_whitespace().collect();
        assert_eq!(values.len(), 17);
        let revision = u16::from_str_radix(values[0], 16).unwrap();
        let seed = values[1].parse::<u32>().unwrap();
        let model = Rc::new(RefCell::new(Model::new(revision)));
        for (bank, address, width) in fields {
            model.borrow_mut().set(
                bank,
                address,
                width,
                if width == 4 {
                    seed | (seed << 16)
                } else {
                    seed
                },
            );
        }
        model.borrow_mut().set(PLA, PLA_IDR, 4, 0x05040302);
        model.borrow_mut().set(PLA, PLA_IDR + 4, 4, 0x0706);
        let (_, mac) =
            block_on(Chip::attach(Io(model.clone()), Revision(revision), false)).unwrap();
        for (actual, expected) in mac.0.into_iter().zip(&values[3..9]) {
            assert_eq!(
                u32::from(actual),
                expected.parse::<u32>().unwrap(),
                "{line}"
            );
        }
        let writes_idr = model
            .borrow()
            .trace
            .iter()
            .any(|(request, _)| request[0] == 0x40 && request[2..4] == [0, 0xc0]);
        assert_eq!(
            u32::from(writes_idr),
            values[2].parse::<u32>().unwrap(),
            "{line}"
        );
        for ((bank, address, width), expected) in fields.into_iter().zip(&values[9..]) {
            assert_eq!(
                model.borrow().get(bank, address, width),
                expected.parse::<u32>().unwrap(),
                "{line}"
            );
        }
        count += 1;
    }
    assert_eq!(count, 12);
}

#[test]
fn timeout_unknown_revision_and_alignment_are_explicit() {
    for case in 0..3 {
        let model = Rc::new(RefCell::new(Model::new(0x4c10)));
        match case {
            0 => model.borrow_mut().stuck_reset = true,
            1 => model.borrow_mut().stuck_phy = true,
            _ => model.borrow_mut().stuck_list = true,
        }
        assert!(matches!(
            block_on(Chip::attach(Io(model), Revision(0x4c10), false)),
            Err(BlockError::Timeout)
        ));
    }
    // Unimplemented revisions refuse after the identity read. Patched
    // revisions start without their file (every_patched_revision_...).
    for revision in [0x4c20, 0xffff] {
        let model = Rc::new(RefCell::new(Model::new(revision)));
        assert!(block_on(Chip::attach(Io(model.clone()), Revision(revision), false)).is_err());
        assert_eq!(model.borrow().trace.len(), 1);
    }
    let model = Rc::new(RefCell::new(Model::new(0x4c10)));
    let (mut chip, _) = block_on(Chip::attach(Io(model.clone()), Revision(0x4c10), false)).unwrap();
    let before = model.borrow().trace.len();
    for (address, width) in [(0xe813, 2), (0xe812, 4), (0x10000, 1), (0xe810, 3)] {
        assert!(block_on(chip.read(PLA, address, width)).is_err());
        assert!(block_on(chip.write(PLA, address, width, 0)).is_err());
    }
    assert_eq!(model.borrow().trace.len(), before);
}

#[test]
fn backup_station_address_is_validated_then_restored_with_write_gate() {
    let model = Rc::new(RefCell::new(Model::new(0x4c10)));
    let (_chip, mac) = block_on(Chip::attach(Io(model.clone()), Revision(0x4c10), false)).unwrap();
    assert_eq!(mac.0, [2, 0x11, 0x22, 0x33, 0x44, 0x55]);
    assert_eq!(model.borrow().get(PLA, PLA_IDR, 4), 0x33221102);
    assert_eq!(model.borrow().get(PLA, PLA_IDR + 4, 2), 0x5544);
    assert_eq!(model.borrow().get(PLA, PLA_CRWECR, 1), 0);
    for first in [0, 1, u32::MAX] {
        let model = Rc::new(RefCell::new(Model::new(0x4c10)));
        model.borrow_mut().set(PLA, PLA_BACKUP, 4, first);
        model.borrow_mut().set(PLA, PLA_BACKUP + 4, 4, 0);
        // Invalid (zero, multicast, all ones): a random address instead.
        let (_chip, mac) =
            block_on(Chip::attach(Io(model.clone()), Revision(0x4c10), false)).unwrap();
        assert!(mac.is_valid() && mac.0[0] & 2 != 0, "{mac}");
        assert_ne!(model.borrow().get(PLA, PLA_IDR, 4), first);
    }
}

#[test]
#[ignore = "oracle: STELA_RTL8152_ORACLE"]
fn unchanged_linux_ocp_request_trace_matches_native_helpers() {
    extern crate std;
    use core::fmt::Write;
    let model = Rc::new(RefCell::new(Model::new(0x4c10)));
    let mut chip = Chip {
        io: crate::usb::ethernet::MinimumControl::new(Io(model.clone())),
        revision: Revision(0x4c10),
        base: None,
        accessible: true,
        enabled: false,
        initialized: false,
        usb_speed: 0,
        policy: Policy::default(),
    };
    for bank in [USB, PLA] {
        for address in 0xc000..0xc020 {
            block_on(chip.write(bank, address, 1, address * 7)).unwrap();
            block_on(chip.read(bank, address, 1)).unwrap();
        }
        for address in (0xe000..0xe020).step_by(2) {
            block_on(chip.write(bank, address, 2, address * 11)).unwrap();
            block_on(chip.read(bank, address, 2)).unwrap();
        }
        for address in (0xd000..0xd020).step_by(4) {
            block_on(chip.write(bank, address, 4, address * 0x1234)).unwrap();
            block_on(chip.read(bank, address, 4)).unwrap();
        }
    }
    for address in (0x2010..0xf000).step_by(0x102) {
        block_on(chip.phy_write(address, address * 7)).unwrap();
        let mapped = block_on(chip.phy_address(address)).unwrap();
        block_on(chip.read(PLA, mapped, 2)).unwrap();
    }
    let mut actual = alloc::string::String::new();
    for (request, bytes) in &model.borrow().trace {
        for byte in request {
            write!(actual, "{byte:02x}").unwrap();
        }
        actual.push(':');
        for byte in bytes {
            write!(actual, "{byte:02x}").unwrap();
        }
        actual.push('\n');
    }
    let path = std::env::var("STELA_RTL8152_ORACLE").expect("path to generated donor trace");
    assert_eq!(actual, std::fs::read_to_string(path).unwrap());
}

/// A frame whose transmit TD the controller has not finished is queued and
/// the call returns at once, as usbnet submits its URB; the next frame is
/// busy until the TD completes, and a TD past its 1 s deadline is cancelled
/// and reported before the buffer is reused.
#[test]
fn a_transmit_in_flight_makes_the_next_busy_until_it_ends_or_times_out() {
    use crate::net::{Link, LinkError};
    let mut dma = TestRegion::new(128 * 1024 * 8);
    let (mut controller, model) = unconfigured_cancellation_controller_with_configurations(
        &mut dma,
        &[vec![9, 2, 9, 0, 0, 7, 0, 0x80, 50]],
    );
    controller.set_slot_speed(1, HIGH_SPEED);
    model.borrow_mut().ethernet = Some(ethernet::rtl815x::tests::Model::new(0x4c00));
    let configuration = vec![
        9, 2, 32, 0, 1, 7, 0, 0x80, 50, 9, 4, 0, 0, 2, 0xff, 0, 0, 0, 7, 5, 0x81, 2, 0, 2, 0, 7, 5,
        0x02, 2, 0, 2, 0,
    ];
    let mut device = model::addressed_device(&controller, vec![configuration]);
    device.speed = HIGH_SPEED;
    device.descriptor[8..12].copy_from_slice(&[0xda, 0x0b, 0x52, 0x81]);
    controller.preflight_vendor_ethernet(&mut device).unwrap();
    model::select(&mut device).unwrap();
    let shared = Rc::new(RefCell::new(controller));
    let mut found = Found::default();
    bind(&shared, device, &mut dma, &mut found);
    let mut link = found.vendor_links.pop().unwrap();
    assert!(link.carrier());
    let clock = shared.borrow().clock;
    let elapsed = |since: u64| clock.now_us() - since;

    model.borrow_mut().silence_transfer = Some(4);
    let before = clock.now_us();
    link.transmit(&[1; 60]).unwrap();
    assert!(link.sending.is_some());
    let sent = link.send.bytes().to_vec();
    assert_eq!(link.transmit(&[2; 60]), Err(LinkError::Busy));
    assert_eq!(link.send.bytes(), sent);
    assert!(elapsed(before) < 100_000);
    crate::test_clock::advance(clock, 1_000_000);
    assert!(matches!(
        link.transmit(&[3; 60]),
        Err(LinkError::Adapter(message)) if message.ends_with("device timeout")
    ));
    assert!(link.sending.is_none());

    model.borrow_mut().silence_transfer = None;
    let outgoing = model.borrow().outgoing.len();
    link.transmit(&[4; 60]).unwrap();
    assert!(link.sending.is_none());
    assert_eq!(model.borrow().outgoing.len(), outgoing + 1);
    link.transmit(&[5; 60]).unwrap();
    assert_eq!(model.borrow().outgoing.len(), outgoing + 2);
}

#[test]
fn rtl8152b_binds_real_xhci_dma_moves_aggregates_wraps_and_quiesces() {
    use crate::net::Link;
    for (revision, scenario) in [0x4c00, 0x4c10, 0x4800]
        .into_iter()
        .flat_map(|r| (0..4).map(move |s| (r, s)))
    {
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (mut controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 7, 0, 0x80, 50]],
        );
        controller.set_slot_speed(1, HIGH_SPEED);
        model::minimum_on_controller_clock(&mut controller);
        model.borrow_mut().ethernet = Some(ethernet::rtl815x::tests::Model::new(revision));
        let configuration = vec![
            9, 2, 32, 0, 1, 7, 0, 0x80, 50, 9, 4, 0, 0, 2, 0xff, 0, 0, 0, 7, 5, 0x81, 2, 0, 2, 0,
            7, 5, 0x02, 2, 0, 2, 0,
        ];
        let mut device = model::addressed_device(&controller, vec![configuration]);
        device.speed = HIGH_SPEED;
        device.descriptor[8..12].copy_from_slice(&[0xda, 0x0b, 0x52, 0x81]);
        controller.preflight_vendor_ethernet(&mut device).unwrap();
        assert_eq!(device.vendor_ethernet_revision, Some(revision));
        assert_eq!(
            device.vendor_ethernet_tcr,
            Some((u32::from(revision) << 16) | 0x0800)
        );
        model::select(&mut device).unwrap();
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        bind(&shared, device, &mut dma, &mut found);
        assert!(found.failures.is_empty(), "{:?}", found.failures);
        assert_eq!(found.vendor_links.len(), 1);
        assert_eq!(model.borrow().configuration_seen_before_endpoints, [7]);
        let mut link = found.vendor_links.pop().unwrap();
        assert_eq!(link.mac().0, [2, 0x11, 0x22, 0x33, 0x44, 0x55]);
        assert!(link.carrier());
        for value in 0..300 {
            link.transmit(&[value as u8; 60]).unwrap();
        }
        assert_eq!(model.borrow().outgoing.len(), 300);
        let packet = model.borrow().outgoing[299].clone();
        assert_eq!(&packet[..8], &[60, 0, 0, 0xc0, 0, 0, 0, 0]);
        assert_eq!(&packet[8..], &[299u16 as u8; 60]);
        assert_eq!(link.receive().unwrap(), None);
        assert!(model.borrow().active[3].is_some());
        let mut aggregate = vec![0; 176];
        aggregate[..4].copy_from_slice(&64u32.to_le_bytes());
        aggregate[24..84].fill(0x5a);
        aggregate[88..92].copy_from_slice(&64u32.to_le_bytes());
        aggregate[112..172].fill(0x6a);
        model.borrow_mut().push_event(aggregate.clone());
        assert_eq!(link.receive().unwrap(), Some(vec![0x5a; 60]));
        assert_eq!(link.receive().unwrap(), Some(vec![0x6a; 60]));
        assert_eq!(link.receive().unwrap(), None);
        // r8152's rx_bottom and read_bulk_callback: an aggregate that fails
        // its checks and a STALLed transfer are counted and dropped, and
        // the link keeps receiving.
        let mut runt = aggregate.clone();
        runt[..4].copy_from_slice(&5u32.to_le_bytes());
        model.borrow_mut().push_event(runt);
        assert_eq!(link.receive().unwrap(), None);
        model.borrow_mut().halt_transfer = Some(3);
        assert_eq!(link.receive().unwrap(), None);
        model.borrow_mut().halt_transfer = None;
        model.borrow_mut().push_event(aggregate);
        let received = (0..2).find_map(|_| link.receive().unwrap());
        assert_eq!(received, Some(vec![0x5a; 60]));
        assert_eq!(link.receive().unwrap(), Some(vec![0x6a; 60]));
        assert_eq!(link.rx_dropped(), 2);
        assert_eq!(link.receive().unwrap(), None);
        match scenario {
            0 => {
                model.borrow_mut().ethernet.as_mut().unwrap().link = false;
                assert!(!link.checked_carrier());
                assert!(model.borrow().active[3].is_none());
                model.borrow_mut().ethernet.as_mut().unwrap().link = true;
                assert!(link.checked_carrier());
                link.transmit(&[1; 60]).unwrap();
                link.stop().unwrap();
                assert!(link.transmit(&[2; 60]).is_err());
            }
            1 => {
                model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
                assert!(link.stop().is_err());
                assert!(model.borrow().active[3].is_some());
                assert!(link.receive().is_err());
                assert!(link.transmit(&[0; 60]).is_err());
            }
            2 => {
                model.borrow_mut().connected = false;
                assert!(!link.carrier());
                assert!(link.receive().is_err());
                assert!(link.transmit(&[0; 60]).is_err());
                // Disconnect is not proof of DMA quiescence.
                assert!(model.borrow().active[3].is_some());
            }
            _ => {
                model.borrow_mut().halt_transfer = Some(4);
                model.borrow_mut().fail = Some((SET_DEQUEUE, CONTEXT_STATE_ERROR));
                assert!(link.transmit(&[0x77; 60]).is_err());
                let state = model.borrow();
                let at = state.rings[4] + (state.indices[4] * TRB) as u64;
                // SAFETY: the failed TD remains in the live ring.
                let (buffer, bytes) = unsafe {
                    (
                        (at as *const u64).read_volatile(),
                        ((at + 8) as *const u32).read_volatile() as usize & 0x1ffff,
                    )
                };
                drop(state);
                // SAFETY: model performs no DMA here; hardware still owns
                // this sentinel and the next attempt must not mutate it.
                let before =
                    unsafe { core::slice::from_raw_parts(buffer as *const u8, bytes).to_vec() };
                assert!(link.transmit(&[0x88; 60]).is_err());
                assert_eq!(
                    // SAFETY: the pointer addresses this test's own live pages, and the stopped
                    // model writes nothing concurrently.
                    unsafe { core::slice::from_raw_parts(buffer as *const u8, bytes) },
                    before
                );
            }
        }
    }
}

#[test]
fn rtl815x_firmware_uses_ep0_dma_wrapping_and_poison_guards() {
    use ethernet::{
        firmware::{Firmware, tests::synthetic_upload},
        rtl815x::{Chip, FirmwareModel, Revision},
    };
    for revision in [
        0x5c10, 0x5c20, 0x5c30, 0x6010, 0x6400, 0x7030, 0x7410, 0x7420,
    ] {
        let bytes = synthetic_upload(revision);
        let firmware = Firmware::parse(&bytes, Revision(revision)).unwrap();
        for fail in [false, true] {
            let mut dma = TestRegion::new(128 * 1024 * 8);
            let (mut controller, model) = unconfigured_cancellation_controller_with_configurations(
                &mut dma,
                &[vec![9, 2, 9, 0, 0, 7, 0, 0x80, 50]],
            );
            controller.set_configuration(1, 7).unwrap();
            model.borrow_mut().ethernet_upload = Some(FirmwareModel::new(revision, &bytes));
            let shared = Rc::new(RefCell::new(controller));
            let io = ethernet::HostControl {
                controller: shared.clone(),
                id: shared.borrow().test_device_id(1).unwrap(),
                stepped: Default::default(),
            };
            let mut chip = block_on(Chip::probe(io, Revision(revision))).unwrap();
            if fail {
                model.borrow_mut().silence_transfer = Some(1);
                model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
                assert!(block_on(chip.upload_firmware(&firmware, false)).is_err());
                assert!(shared.borrow_mut().endpoint_ready(1, 1).is_err());
                let control = shared.borrow().control_bytes(1);
                let requests = model.borrow().setup.len();
                assert!(block_on(chip.upload_firmware(&firmware, false)).is_err());
                assert_eq!(model.borrow().setup.len(), requests);
                assert_eq!(shared.borrow().control_bytes(1), control);
            } else {
                block_on(chip.upload_firmware(&firmware, false)).unwrap();
                assert!(block_on(chip.carrier()).is_err()); // Upload alone is not initialized PHY/MAC.
                let state = model.borrow();
                assert_eq!(state.configuration_value, 7);
                assert!(state.configuration_seen_before_endpoints.is_empty());
                assert!(
                    state
                        .setup
                        .iter()
                        .any(|(request, data)| request[0] == 0x40 && data.len() > 4)
                );
                assert!(state.setup.iter().all(|(_, data)| data.len() <= 512));
                assert!(state.active[1].is_none());
            }
        }
    }
}

#[test]
fn rtl8153_missing_firmware_starts_the_chip_unpatched() {
    for revision in [
        0x5c10, 0x5c20, 0x5c30, 0x6010, 0x6400, 0x7030, 0x7410, 0x7420,
    ] {
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (mut controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 7, 0, 0x80, 50]],
        );
        controller.set_slot_speed(1, HIGH_SPEED);
        model.borrow_mut().ethernet_upload =
            Some(ethernet::rtl815x::FirmwareModel::new(revision, &[]));
        let config = vec![
            9, 2, 32, 0, 1, 7, 0, 0x80, 50, 9, 4, 0, 0, 2, 0xff, 0, 0, 0, 7, 5, 0x81, 2, 0, 2, 0,
            7, 5, 2, 2, 0, 2, 0,
        ];
        let mut dev = model::addressed_device(&controller, vec![config]);
        dev.speed = HIGH_SPEED;
        let is_8156 = ethernet::rtl815x::Revision(revision).is_8156();
        dev.descriptor[8..14].copy_from_slice(&[
            0xda,
            0x0b,
            if is_8156 { 0x56 } else { 0x53 },
            0x81,
            0x00,
            0x30,
        ]);
        dev.vendor_ethernet_revision = Some(revision);
        model::select(&mut dev).unwrap();
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        bind(&shared, dev, &mut dma, &mut found);
        // rtl8152_request_firmware: without the file the chip starts
        // unpatched, and the link names what it runs without.
        assert!(
            found.vendor_failures.is_empty() && found.failures.is_empty(),
            "{revision:04x}: {:?} {:?}",
            found.vendor_failures,
            found.failures
        );
        assert_eq!(found.vendor_links.len(), 1, "{revision:04x}");
        let link = &found.vendor_links[0];
        assert!(link.firmware.is_none());
        assert!(
            link.location.ends_with("without its firmware patch"),
            "{}",
            link.location
        );
        let Some(ethernet::Error::Firmware { request, .. }) = &link.firmware_skipped else {
            panic!("{:?}", link.firmware_skipped)
        };
        assert_eq!(
            request.name,
            ethernet::rtl815x::Revision(revision).firmware().unwrap()
        );
        assert_eq!(
            request.hardware,
            if is_8156 {
                "usb:0bda:8156:3000"
            } else {
                "usb:0bda:8153:3000"
            }
        );
        assert_eq!(request.revision, format!("tcr-{revision:04x}"));
        assert_eq!(model.borrow().configuration_seen_before_endpoints, [7]);
        assert!(
            model
                .borrow()
                .setup
                .iter()
                .any(|(request, _)| *request == setup(0, SET_CONFIGURATION, 7, 0, 0))
        );
        assert!(model.borrow().seen.contains(&CONFIGURE_ENDPOINT));
        assert!(model.borrow().active[3].is_none() && model.borrow().outgoing.is_empty());
        // The probe account names the missing patch once, for the log.
        let location = link.location.clone();
        let mut devices = crate::probe::Devices::default();
        let empty = crate::firmware::Catalog::from_handoff(Vec::new()).unwrap();
        devices.absorb_usb(None, found, &empty);
        assert_eq!(devices.usb_vendor_warnings.len(), 1);
        assert_eq!(devices.usb_vendor_warnings[0].0, location);
        assert!(devices.usb_vendor_failures.is_empty() && devices.nics.len() == 1);
    }
}

#[test]
fn rtl8153_native_usb2_usb3_binding_uploads_exact_provider_and_keeps_owned_receipt() {
    use crate::{
        firmware::{Catalog, Provenance, Provider, Resource, Target},
        net::Link,
    };
    use sha2::{Digest, Sha256};
    struct Recording {
        catalog: Catalog,
        requests: RefCell<Vec<(String, String, String)>>,
    }
    impl Provider for Recording {
        fn load(
            &self,
            requirement: crate::firmware::Requirement<'_>,
        ) -> Result<crate::firmware::Image<'_>, crate::firmware::Error> {
            self.requests.borrow_mut().push((
                requirement.name.into(),
                requirement.target.hardware.into(),
                requirement.target.revision.into(),
            ));
            self.catalog.load(requirement)
        }
    }
    let native = [
        0x5c00, 0x5c10, 0x5c20, 0x5c30, 0x6000, 0x6010, 0x6400, 0x7020, 0x7030, 0x7400, 0x7410,
        0x7420,
    ]
    .into_iter()
    .flat_map(|revision| [HIGH_SPEED, SUPER_SPEED, SUPER_SPEED_PLUS].map(|speed| (revision, speed)))
    .map(|(revision, speed)| {
        let pid = if ethernet::rtl815x::Revision(revision).is_8156() {
            0x8156
        } else {
            0x8153
        };
        (0x0bda, pid, revision, speed, 0x3000u16, None)
    });
    let aliases = ethernet::USB_IDS
        .iter()
        .copied()
        .map(|(vid, pid)| (vid, pid, 0x5c10, HIGH_SPEED, 0x3000, None));
    let dock = [0x3011, 0x3010].into_iter().flat_map(|bcd| {
        ["000001000000", "000002000000", "000003000000"]
            .map(|serial| (0x413c, 0xb097, 0x5c10, HIGH_SPEED, bcd, Some(serial)))
    });
    for (vid, pid, revision, speed, bcd, serial) in native.chain(aliases).chain(dock) {
        let image = if matches!(revision, 0x5c00 | 0x6000 | 0x7020 | 0x7400) {
            Vec::new()
        } else {
            ethernet::firmware::tests::synthetic_upload(revision)
        };
        let is_8156 = ethernet::rtl815x::Revision(revision).is_8156();
        let hardware_label = format!("usb:{vid:04x}:{pid:04x}:{bcd:04x}");
        let name = ethernet::rtl815x::Revision(revision).firmware();
        let revision_label = format!("tcr-{revision:04x}");
        let digest: [u8; 32] = Sha256::digest(&image).into();
        let catalog = if let Some(name) = name {
            Catalog::from_handoff(vec![Resource {
                name,
                target: Target {
                    driver: "usb-rtl815x",
                    hardware: &hardware_label,
                    revision: &revision_label,
                    board: None,
                },
                provenance: Provenance {
                    source: "source-model-fixture",
                    revision: "immutable-test",
                    member: name,
                    license: "synthetic",
                },
                expected_length: image.len(),
                expected_sha256: digest,
                bytes: image.clone(),
            }])
            .unwrap()
        } else {
            Catalog::from_handoff(Vec::new()).unwrap()
        };
        let provider = Recording {
            catalog,
            requests: RefCell::new(Vec::new()),
        };
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (mut controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 7, 0, 0x80, 50]],
        );
        controller.set_slot_speed(1, speed);
        model.borrow_mut().ethernet_upload =
            Some(ethernet::rtl815x::FirmwareModel::new(revision, &image));
        let mut configuration = vec![
            9, 2, 32, 0, 1, 7, 0, 0x80, 50, 9, 4, 0, 0, 2, 0xff, 0, 0, 0, 7, 5, 0x81, 2, 0, 2, 0,
            7, 5, 0x02, 2, 0, 2, 0,
        ];
        if speed >= SUPER_SPEED {
            configuration = vec![
                9,
                2,
                44,
                0,
                1,
                7,
                0,
                0x80,
                50,
                9,
                4,
                0,
                0,
                2,
                0xff,
                0,
                0,
                0,
                7,
                5,
                0x81,
                2,
                0,
                4,
                0,
                6,
                SS_ENDPOINT_COMPANION,
                7,
                0,
                0,
                0,
                7,
                5,
                2,
                2,
                0,
                4,
                0,
                6,
                SS_ENDPOINT_COMPANION,
                7,
                0,
                0,
                0,
            ];
        }
        let mut device = model::addressed_device(&controller, vec![configuration]);
        device.speed = speed;
        device.descriptor[8..14].copy_from_slice(&[
            vid as u8,
            (vid >> 8) as u8,
            pid as u8,
            (pid >> 8) as u8,
            bcd as u8,
            (bcd >> 8) as u8,
        ]);
        device.serial = serial.map(Into::into);
        controller.preflight_vendor_ethernet(&mut device).unwrap();
        model::select(&mut device).unwrap();
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        bind_with_provider(&shared, device, &mut dma, &mut found, &provider);
        assert!(found.failures.is_empty(), "{:?}", found.failures);
        assert!(
            found.vendor_failures.is_empty(),
            "{:?}",
            found.vendor_failures
        );
        assert_eq!(found.vendor_links.len(), 1);
        assert_eq!(
            provider.requests.borrow().len(),
            usize::from(name.is_some())
        );
        let bytes = shared
            .borrow_mut()
            .control(1, setup(0xc0, 5, 0xd404, 0, 4), &[])
            .unwrap();
        let aggregation = u16::from_le_bytes([bytes[2], bytes[3]]) & 0x90;
        assert_eq!(
            aggregation,
            if bcd == 0x3011 && matches!(serial, Some("000001000000" | "000002000000")) {
                0x10
            } else {
                0
            }
        );
        let mut link = found.vendor_links.pop().unwrap();
        if let Some(name) = name {
            let receipt = link.firmware.as_ref().unwrap();
            assert_eq!(receipt.request.name, name);
            assert_eq!(receipt.request.hardware, hardware_label);
            assert_eq!(receipt.request.revision, revision_label);
            assert_eq!(receipt.sha256, digest);
            assert_eq!(receipt.length, image.len());
            assert_eq!(receipt.source_revision, "immutable-test");
        } else {
            assert!(link.firmware.is_none());
        }
        drop(provider); // Native link must not borrow the image/catalog.
        assert!(link.settled_carrier());
        link.transmit(&[0x5a; 1514]).unwrap();
        assert_eq!(model.borrow().outgoing[0].len(), 1522);
        assert_eq!(link.receive().unwrap(), None);
        if is_8156 {
            let hw = model.borrow();
            let (_, address, length) = hw.active[3].unwrap();
            assert_eq!(address & 0xffff, 0);
            assert_eq!(length, 49152);
        }
        let frames = if is_8156 {
            32
        } else if speed >= SUPER_SPEED {
            21
        } else {
            1
        };
        let frame_size = if frames >= 21 { 1500 } else { 60 };
        let mut aggregate = Vec::new();
        for value in 0..frames {
            let mut packet = vec![0; 24 + frame_size + 4];
            packet[..4].copy_from_slice(&((frame_size + 4) as u32).to_le_bytes());
            packet[24..24 + frame_size].fill(value as u8);
            aggregate.extend(packet);
            aggregate.resize(aggregate.len().next_multiple_of(8), 0);
        }
        if frames >= 21 {
            assert!(aggregate.len() > 16384);
        }
        model.borrow_mut().push_event(aggregate);
        for value in 0..frames {
            assert_eq!(link.receive().unwrap(), Some(vec![value as u8; frame_size]));
        }
        link.stop().unwrap();
        assert!(link.transmit(&[0; 60]).is_err());
    }
}

#[test]
fn every_patched_revision_starts_without_its_firmware_file() {
    use ethernet::rtl815x::{FirmwareIo, FirmwareModel};
    // rtl8152_apply_firmware skips a patch request_firmware did not find:
    // each revision with a firmware file starts and initializes without it.
    for revision in [
        0x5c10, 0x5c20, 0x5c30, 0x6010, 0x6400, 0x7030, 0x7410, 0x7420,
    ] {
        assert!(Revision(revision).firmware().is_some());
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &[])));
        let (chip, mac) = block_on(Chip::attach_with_firmware(
            FirmwareIo(model),
            Revision(revision),
            4,
            None,
        ))
        .unwrap_or_else(|error| panic!("{revision:04x}: {error:?}"));
        assert!(chip.initialized, "{revision:04x}");
        assert!(!mac.is_multicast() && mac.0 != [0; 6]);
    }
}

mod minimum;

/// The RTL8152B link of [`a_transmit_in_flight_makes_the_next_busy_until_it_ends_or_times_out`]
/// through carrier, a frame each way, a lost link and its return, with
/// calls bounded by `calls`; what the device saw.
fn rtl_link_through_loss(late: bool) -> ethernet::tests::stepped::Effects {
    use crate::net::{Link, LinkError};
    use ethernet::tests::stepped::{Calls, effects};
    let mut dma = TestRegion::new(128 * 1024 * 8);
    let (mut controller, model) = unconfigured_cancellation_controller_with_configurations(
        &mut dma,
        &[vec![9, 2, 9, 0, 0, 7, 0, 0x80, 50]],
    );
    controller.set_slot_speed(1, HIGH_SPEED);
    model::minimum_on_controller_clock(&mut controller);
    model.borrow_mut().ethernet = Some(ethernet::rtl815x::tests::Model::new(0x4c10));
    let configuration = vec![
        9, 2, 32, 0, 1, 7, 0, 0x80, 50, 9, 4, 0, 0, 2, 0xff, 0, 0, 0, 7, 5, 0x81, 2, 0, 2, 0, 7, 5,
        0x02, 2, 0, 2, 0,
    ];
    let mut device = model::addressed_device(&controller, vec![configuration]);
    device.speed = HIGH_SPEED;
    device.descriptor[8..12].copy_from_slice(&[0xda, 0x0b, 0x52, 0x81]);
    controller.preflight_vendor_ethernet(&mut device).unwrap();
    model::select(&mut device).unwrap();
    let shared = Rc::new(RefCell::new(controller));
    let mut found = Found::default();
    bind(&shared, device, &mut dma, &mut found);
    let mut link = found.vendor_links.pop().unwrap();
    let clock = shared.borrow().clock;
    let mut calls = Calls::new(model.clone(), clock, late);
    let not_busy = |result: &Result<(), LinkError>| result != &Err(LinkError::Busy);

    calls.until(|| link.carrier(), |up| *up);
    calls
        .until(|| link.transmit(&[0x5a; 60]), not_busy)
        .unwrap();
    calls
        .until(|| link.transmit(&[0x6a; 60]), not_busy)
        .unwrap();
    calls.until(
        || link.receive().unwrap(),
        |_| model.borrow().active[3].is_some(),
    );
    let mut aggregate = vec![0; 88];
    aggregate[..4].copy_from_slice(&64u32.to_le_bytes());
    aggregate[24..84].fill(0x7a);
    model.borrow_mut().push_event(aggregate);
    let frame = calls.until(|| link.receive().unwrap(), Option::is_some);
    assert_eq!(frame, Some(vec![0x7a; 60]));

    // The lost link: both bulk endpoints cancelled, then the MAC stopped.
    model.borrow_mut().ethernet.as_mut().unwrap().link = false;
    calls.wait(ethernet::LINK_POLL_US);
    calls.until(
        || (link.carrier(), link.running.is_some()),
        |(up, busy)| !up && !busy,
    );
    assert!(model.borrow().active[3].is_none());
    model.borrow_mut().ethernet.as_mut().unwrap().link = true;
    calls.wait(ethernet::LINK_POLL_US);
    calls.until(|| link.carrier(), |up| *up);
    calls
        .until(|| link.transmit(&[0x8a; 60]), not_busy)
        .unwrap();
    calls
        .until(|| link.transmit(&[0x9a; 60]), not_busy)
        .unwrap();
    assert_eq!(calls.answered_late > 0, late);
    let seen = effects(&model.borrow(), |_| false);
    calls.answer_at_once();
    seen
}

/// A device that answers each control transfer and command only after the
/// link call that asked returned: the carrier check, the MAC set up for the
/// new link, transmit, receive and the lost link's quiesce and MAC stop with
/// its 20 ms ALDPS wait run in steps, no call holding the processor beyond
/// its bound, and the device sees what one that answers at once sees.
#[test]
fn rtl8152_link_calls_run_in_steps_when_the_device_answers_late() {
    assert_eq!(rtl_link_through_loss(true), rtl_link_through_loss(false));
}
