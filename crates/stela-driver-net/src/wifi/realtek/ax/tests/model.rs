//! Fake rtw89 register file for tests: byte-addressed values, a full
//! operation trace, per-register read hooks and a virtual clock that only
//! `delay_us` advances, so polls are deterministic and instant.
use crate::wifi::realtek::ax::{AxError, AxIo};
use alloc::{boxed::Box, collections::BTreeMap, vec::Vec};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    R8(u32),
    R16(u32),
    R32(u32),
    W8(u32, u8),
    W16(u32, u16),
    W32(u32, u32),
    Delay(u64),
}

pub type ReadHook = Box<dyn FnMut(&mut RegisterModel) -> u32>;

pub struct RegisterModel {
    regs: BTreeMap<u32, u8>,
    pub trace: Vec<Op>,
    hooks: BTreeMap<u32, ReadHook>,
    pub now: u64,
    /// Keep `now_us` constant across delays, as a stalled clock would.
    pub freeze_clock: bool,
    /// Microseconds each register read adds to the clock, as PCIe latency.
    pub read_cost_us: u64,
}

impl RegisterModel {
    pub fn new() -> Self {
        Self {
            regs: BTreeMap::new(),
            trace: Vec::new(),
            hooks: BTreeMap::new(),
            now: 1_000,
            freeze_clock: false,
            read_cost_us: 0,
        }
    }
    fn get(&self, reg: u32, width: u32) -> u32 {
        (0..width).fold(0, |v, i| {
            v | (*self.regs.get(&(reg + i)).unwrap_or(&0) as u32) << (8 * i)
        })
    }
    fn put(&mut self, reg: u32, width: u32, value: u32) {
        for i in 0..width {
            self.regs.insert(reg + i, (value >> (8 * i)) as u8);
        }
    }
    /// Preset a register without tracing.
    pub fn set32(&mut self, reg: u32, value: u32) {
        self.put(reg, 4, value);
    }
    /// Current value of a register without tracing.
    pub fn get32(&self, reg: u32) -> u32 {
        self.get(reg, 4)
    }
    /// Replace the value every read of `reg` returns; the hook may change
    /// the model first. The returned value is also stored.
    pub fn on_read(&mut self, reg: u32, hook: ReadHook) {
        self.hooks.insert(reg, hook);
    }
    /// Writes and delays only, in order.
    pub fn writes(&self) -> Vec<Op> {
        self.trace
            .iter()
            .copied()
            .filter(|op| !matches!(op, Op::R8(_) | Op::R16(_) | Op::R32(_)))
            .collect()
    }
    fn read(&mut self, reg: u32, width: u32) -> u32 {
        if !self.freeze_clock {
            self.now += self.read_cost_us;
        }
        if let Some(mut hook) = self.hooks.remove(&reg) {
            let v = hook(self);
            self.put(reg, width, v);
            self.hooks.entry(reg).or_insert(hook);
        }
        self.get(reg, width)
    }
}

impl AxIo for RegisterModel {
    fn read8(&mut self, reg: u32) -> Result<u8, AxError> {
        self.trace.push(Op::R8(reg));
        Ok(self.read(reg, 1) as u8)
    }
    fn read16(&mut self, reg: u32) -> Result<u16, AxError> {
        self.trace.push(Op::R16(reg));
        Ok(self.read(reg, 2) as u16)
    }
    fn read32(&mut self, reg: u32) -> Result<u32, AxError> {
        self.trace.push(Op::R32(reg));
        Ok(self.read(reg, 4))
    }
    fn write8(&mut self, reg: u32, v: u8) -> Result<(), AxError> {
        self.trace.push(Op::W8(reg, v));
        self.put(reg, 1, v as u32);
        Ok(())
    }
    fn write16(&mut self, reg: u32, v: u16) -> Result<(), AxError> {
        self.trace.push(Op::W16(reg, v));
        self.put(reg, 2, v as u32);
        Ok(())
    }
    fn write32(&mut self, reg: u32, v: u32) -> Result<(), AxError> {
        self.trace.push(Op::W32(reg, v));
        self.put(reg, 4, v);
        Ok(())
    }
    fn delay_us(&mut self, us: u64) {
        self.trace.push(Op::Delay(us));
        if !self.freeze_clock {
            self.now += us;
        }
    }
    fn now_us(&mut self) -> Result<u64, AxError> {
        Ok(self.now)
    }
}
