//! Independent synthetic AX88772 control model. Numeric wire encodings only;
//! MII grants, PHY reset completion and AN completion advance with time.
use crate::{block::BlockError, usb::ethernet::Control};
use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;
pub(crate) struct Hardware {
    pub(crate) phy: [u16; 32],
    pub(crate) mac: [u8; 6],
    pub(crate) phy_address: u8,
    pub(crate) code: u8,
    pub(crate) rx: u16,
    pub(crate) medium: u16,
    pub(crate) trace: Vec<([u8; 8], Vec<u8>)>,
    pub(crate) delays: Vec<u64>,
    pub(crate) minimum_failure: bool,
    pub(crate) minimum_attempts: Vec<u64>,
    pub(crate) fail_at: Option<usize>,
    pub(crate) short_at: Option<usize>,
    pub(crate) stuck_mii: bool,
    pub(crate) stuck_reset: bool,
    pub(crate) link: bool,
    pub(crate) latched_low: bool,
    pub(crate) resets: usize,
    grants: usize,
    owned: bool,
    elapsed: u64,
    requested: u64,
    phy_ready: u64,
    negotiated: u64,
    phase: usize,
}
impl Hardware {
    pub(crate) fn new(code: u8, phy_id: u32) -> Self {
        let mut phy = [0; 32];
        phy[0] = 0x3100;
        phy[1] = 0x782d;
        phy[2] = (phy_id >> 16) as u16;
        phy[3] = phy_id as u16;
        phy[4] = 1;
        phy[5] = 0x0de1;
        Self {
            phy,
            mac: [2, 0x27, 0x72, 0x43, 0x54, 0x65],
            phy_address: 16,
            code,
            rx: 0,
            medium: 0,
            trace: Vec::new(),
            delays: Vec::new(),
            minimum_failure: false,
            minimum_attempts: Vec::new(),
            fail_at: None,
            short_at: None,
            stuck_mii: false,
            stuck_reset: false,
            link: true,
            latched_low: false,
            resets: 0,
            grants: 0,
            owned: false,
            elapsed: 0,
            requested: 0,
            phy_ready: 0,
            negotiated: 0,
            phase: 0,
        }
    }
    pub(crate) fn delay(&mut self, us: u64) {
        self.elapsed += us;
        self.delays.push(us);
    }
    pub(crate) fn request(&mut self, q: [u8; 8], data: &[u8]) -> Result<Vec<u8>, BlockError> {
        let at = self.trace.len();
        self.trace.push((q, data.to_vec()));
        if self.fail_at == Some(at) {
            return Err(BlockError::Timeout);
        }
        let read = q[0] == 0xc0;
        assert!(read || q[0] == 0x40);
        let v = u16::from_le_bytes([q[2], q[3]]);
        let i = u16::from_le_bytes([q[4], q[5]]);
        let n = u16::from_le_bytes([q[6], q[7]]) as usize;
        assert_eq!(data.len(), if read { 0 } else { n });
        let mut reply = match (read, q[1]) {
            (true, 0x13) => {
                assert_eq!((v, i, n), (0, 0, 6));
                self.mac.to_vec()
            }
            (true, 0x0b) => {
                assert_eq!((i, n), (0, 2));
                assert!((4..=6).contains(&v));
                self.mac[(v as usize - 4) * 2..(v as usize - 3) * 2].to_vec()
            }
            (true, 0x19) => {
                assert_eq!((v, i, n), (0, 0, 2));
                alloc::vec![0, self.phy_address]
            }
            (false, 0x06) => {
                assert_eq!((v, i, n), (0, 0, 0));
                self.grants += 1;
                self.requested = self.elapsed;
                Vec::new()
            }
            (true, 0x09) => {
                assert_eq!((v, i, n), (0, 0, 1));
                if self.grants > 0 {
                    assert!(self.elapsed >= self.requested + 1000);
                }
                if self.grants >= 3 && !self.stuck_mii {
                    self.owned = true;
                }
                alloc::vec![self.code | u8::from(self.owned)]
            }
            (false, 0x0a) => {
                assert!(self.owned);
                assert_eq!((v, i, n), (0, 0, 0));
                self.owned = false;
                self.grants = 0;
                Vec::new()
            }
            (true, 0x07) => {
                assert!(self.owned);
                assert_eq!((v, n), (16, 2));
                assert!(i < 32);
                if self.elapsed >= self.phy_ready && !self.stuck_reset {
                    self.phy[0] &= !0x8000;
                }
                if self.elapsed >= self.negotiated && !self.stuck_reset {
                    self.phy[0] &= !0x0200;
                }
                if i == 1 {
                    self.phy[1] = (self.phy[1] & !0x24)
                        | if self.link && self.phy[0] & 0x8a00 == 0 {
                            0x24
                        } else {
                            0
                        };
                }
                let mut value = self.phy[i as usize];
                if i == 1 && self.latched_low {
                    value &= !4;
                    self.latched_low = false;
                }
                value.to_le_bytes().to_vec()
            }
            (false, 0x08) => {
                assert!(self.owned);
                assert_eq!((v, n), (16, 2));
                assert!(i < 32);
                let val = u16::from_le_bytes(data.try_into().unwrap());
                if i == 0 {
                    if val & 0x8000 != 0 {
                        assert_eq!(self.phy[0] & 0x8000, 0);
                        self.phy_ready = self.elapsed + 100_000;
                        self.resets += 1;
                    }
                    if val & 0x0200 != 0 {
                        self.negotiated = self.elapsed + 10_000;
                    }
                }
                self.phy[i as usize] = val;
                Vec::new()
            }
            (false, 0x1f) => {
                assert_eq!(v, if self.code & 0x70 == 0 { 0xb0 } else { 0x80 });
                self.phase = 1;
                self.requested = self.elapsed;
                Vec::new()
            }
            (false, 0x22) => {
                assert_eq!(self.phase, 1);
                assert!(self.elapsed >= self.requested + 5000);
                assert_eq!(v, if self.code & 0x70 == 0 { 1 } else { 0x11 });
                self.phase = 2;
                Vec::new()
            }
            (false, 0x20) => {
                assert!(self.phase >= 2);
                self.phase += 1;
                Vec::new()
            }
            (false, 0x2a) => {
                assert_eq!(self.code & 0x70, 0x20);
                assert_eq!((v, i, n), (0x8000, 0x8001, 0));
                Vec::new()
            }
            (false, 0x12) => {
                assert_eq!((v, i, n), (0x1d, 0x12, 0));
                Vec::new()
            }
            (false, 0x14) => {
                assert_eq!((v, i, n), (0, 0, 6));
                self.mac.copy_from_slice(data);
                Vec::new()
            }
            (false, 0x10) => {
                assert_eq!((i, n), (0, 0));
                self.rx = v;
                Vec::new()
            }
            (false, 0x1b) => {
                assert_eq!((i, n), (0, 0));
                self.medium = v;
                Vec::new()
            }
            (true, 0x0f) => {
                assert_eq!((v, i, n), (0, 0, 2));
                self.rx.to_le_bytes().to_vec()
            }
            (true, 0x1a) => {
                assert_eq!((v, i, n), (0, 0, 2));
                self.medium.to_le_bytes().to_vec()
            }
            _ => panic!("unmodeled AX88772 request {q:?}"),
        };
        if read && self.short_at == Some(at) {
            reply.pop();
        }
        Ok(reply)
    }
}
pub(super) struct Io(pub Rc<RefCell<Hardware>>);
impl Control for Io {
    async fn control(&mut self, q: [u8; 8], data: &[u8]) -> Result<Vec<u8>, BlockError> {
        self.0.borrow_mut().request(q, data)
    }
    async fn delay_us(&mut self, us: u64) {
        self.0.borrow_mut().delay(us);
    }
    async fn minimum_wait(&mut self, us: u64) -> Result<(), BlockError> {
        let mut state = self.0.borrow_mut();
        state.minimum_attempts.push(us);
        if state.minimum_failure {
            return Err(BlockError::PciPower(
                crate::bus::pci::power::Error::Interrupted,
            ));
        }
        state.elapsed = state
            .elapsed
            .checked_add(us)
            .ok_or(BlockError::PciPower(crate::bus::pci::power::Error::Clock))?;
        state.delays.push(us);
        Ok(())
    }
}
