//! The generic timers of Arm ARM DDI0487 D12: the EL1 physical and virtual
//! timers (CNTP, CNTV) and the EL2 physical and virtual timers (CNTHP,
//! CNTHV), whose CTL and CVAL registers live in the System register file. The
//! counter comes from the host. The EL1 virtual timer counts the physical
//! count minus CNTVOFF_EL2; the EL2 virtual timer counts without an offset,
//! because HCR_EL2.E2H is RES1. Apple cores signal the timers as FIQ, and
//! VM_TMR_FIQ_ENA_EL2 gates the EL1 timers (m1n1 `cpu_regs.h`). The event
//! streams of CNTHCTL_EL2 and CNTKCTL_EL1 set the event register.
//!
//! From the M3 on, these registers form the Arm bank, counting the system
//! count scaled to 1 GHz, beside the Apple generic timer's bank on the
//! unscaled count ([`agt`]); each bank has its own timers and event streams.
//! Counts here are system counts, converted to a bank's count to compare.
use super::{
    cpu::{Cpu, VM_TIMER_FIQ_PHYSICAL, VM_TIMER_FIQ_VIRTUAL},
    system::Register,
};
use agt::Bank;

pub mod agt;

pub const ENABLE: u64 = 1;
pub const IMASK: u64 = 2;
pub const ISTATUS: u64 = 4;

/// A channel's comparator or interrupt control, including its relative alias.
pub(super) fn channel_register(register: Register) -> bool {
    use Register::*;
    matches!(
        register,
        CNTP_CTL_EL0
            | CNTP_CVAL_EL0
            | CNTP_TVAL_EL0
            | CNTV_CTL_EL0
            | CNTV_CVAL_EL0
            | CNTV_TVAL_EL0
            | CNTHP_CTL_EL2
            | CNTHP_CVAL_EL2
            | CNTHP_TVAL_EL2
            | CNTHV_CTL_EL2
            | CNTHV_CVAL_EL2
            | CNTHV_TVAL_EL2
    )
}

/// One timer channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel {
    pub control: Register,
    pub compare: Register,
    /// Counts the physical count minus CNTVOFF_EL2.
    pub offset: bool,
    /// The VM_TMR_FIQ_ENA_EL2 bit that lets it signal FIQ, if any.
    pub gate: Option<u64>,
}

pub const CHANNELS: [Channel; 4] = [
    Channel {
        control: Register::CNTP_CTL_EL0,
        compare: Register::CNTP_CVAL_EL0,
        offset: false,
        gate: Some(VM_TIMER_FIQ_PHYSICAL),
    },
    Channel {
        control: Register::CNTV_CTL_EL0,
        compare: Register::CNTV_CVAL_EL0,
        offset: true,
        gate: Some(VM_TIMER_FIQ_VIRTUAL),
    },
    Channel {
        control: Register::CNTHP_CTL_EL2,
        compare: Register::CNTHP_CVAL_EL2,
        offset: false,
        gate: None,
    },
    Channel {
        control: Register::CNTHV_CTL_EL2,
        compare: Register::CNTHV_CVAL_EL2,
        offset: false,
        gate: None,
    },
];

/// The banks `cpu` has: the Arm bank, and the Apple bank on a chip with the
/// Apple generic timer.
pub fn banks(cpu: &Cpu) -> &'static [Bank] {
    if agt::present(cpu.registers.chip()) {
        &[Bank::Arm, Bank::Apple]
    } else {
        &[Bank::Arm]
    }
}

/// `bank`'s copy of a generic timer register.
pub fn value(cpu: &Cpu, bank: Bank, register: Register) -> u64 {
    match bank {
        Bank::Apple => cpu
            .apple
            .generic_timer
            .get(register)
            .unwrap_or_else(|| cpu.read(register)),
        Bank::Arm => cpu.read(register),
    }
}

/// Whether `bank` counts the system count scaled to 1 GHz.
pub fn scaled(cpu: &Cpu, bank: Bank) -> bool {
    bank == Bank::Arm && agt::present(cpu.registers.chip())
}

/// `bank`'s count at system count `physical`.
pub fn count(cpu: &Cpu, bank: Bank, physical: u64) -> u64 {
    if scaled(cpu, bank) {
        cpu.apple.generic_timer.scale(physical)
    } else {
        physical
    }
}

/// The first system count at which `bank` counts `count`.
pub fn system_count(cpu: &Cpu, bank: Bank, count: u64) -> u64 {
    if scaled(cpu, bank) {
        cpu.apple.generic_timer.unscale(count)
    } else {
        count
    }
}

impl Channel {
    pub fn by_control(register: Register) -> Option<Self> {
        CHANNELS
            .into_iter()
            .find(|channel| channel.control == register)
    }

    fn offset_of(&self, cpu: &Cpu, bank: Bank) -> u64 {
        if self.offset {
            value(cpu, bank, Register::CNTVOFF_EL2)
        } else {
            0
        }
    }

    /// The timer condition, independent of IMASK.
    pub fn met(&self, cpu: &Cpu, bank: Bank, physical: u64) -> bool {
        let count = count(cpu, bank, physical).wrapping_sub(self.offset_of(cpu, bank));
        value(cpu, bank, self.control) & ENABLE != 0 && count >= value(cpu, bank, self.compare)
    }

    /// CTL with ISTATUS.
    pub fn read_control(&self, cpu: &Cpu, bank: Bank, physical: u64) -> u64 {
        let status = if self.met(cpu, bank, physical) {
            ISTATUS
        } else {
            0
        };
        (value(cpu, bank, self.control) & !ISTATUS) | status
    }

    fn gated(&self, cpu: &Cpu) -> bool {
        self.gate
            .is_some_and(|bit| cpu.apple.vm_timer_fiq & bit == 0)
    }

    /// The FIQ output: enabled, unmasked, met and not gated off.
    pub fn asserted(&self, cpu: &Cpu, bank: Bank, physical: u64) -> bool {
        !self.gated(cpu)
            && value(cpu, bank, self.control) & IMASK == 0
            && self.met(cpu, bank, physical)
    }

    /// The system count at which the output can assert. Without an offset
    /// it is the compare value's. With one, the bank's count less the offset
    /// wraps as the architecture's virtual count does, so the answer is the
    /// one seen at system count `physical`, read only then: `physical`
    /// itself while the condition holds ([`Self::met`]: an offset above the
    /// count leaves the virtual count high), else the count at which the
    /// virtual count reaches the compare value, none when that is only after
    /// the count wraps (a compare value plus the offset that overflows,
    /// which is no deadline in the past). Either stays right as the count
    /// advances, until the virtual count wraps.
    pub fn deadline(
        &self,
        cpu: &Cpu,
        bank: Bank,
        physical: &mut impl FnMut() -> u64,
    ) -> Option<u64> {
        if self.gated(cpu) || value(cpu, bank, self.control) & (ENABLE | IMASK) != ENABLE {
            return None;
        }
        let offset = self.offset_of(cpu, bank);
        let compare = value(cpu, bank, self.compare);
        if offset == 0 {
            return Some(system_count(cpu, bank, compare));
        }
        let physical = physical();
        if self.met(cpu, bank, physical) {
            return Some(physical);
        }
        let compare = compare.wrapping_add(offset);
        if compare < count(cpu, bank, physical) {
            return None;
        }
        Some(system_count(cpu, bank, compare))
    }
}

/// The bank that counts the system count unscaled: the Apple bank where
/// the chip has one.
pub fn system_bank(cpu: &Cpu) -> Bank {
    if agt::present(cpu.registers.chip()) {
        Bank::Apple
    } else {
        Bank::Arm
    }
}

/// The system counter's frequency: [`system_bank`]'s CNTFRQ_EL0.
pub fn system_frequency(cpu: &Cpu) -> u64 {
    value(cpu, system_bank(cpu), Register::CNTFRQ_EL0)
}

/// Every timer of every bank.
fn timers(cpu: &Cpu) -> impl Iterator<Item = (Bank, Channel)> + '_ {
    banks(cpu)
        .iter()
        .flat_map(|&bank| CHANNELS.into_iter().map(move |channel| (bank, channel)))
}

/// Whether any timer signals FIQ.
pub fn asserted(cpu: &Cpu, physical: u64) -> bool {
    timers(cpu).any(|(bank, channel)| channel.asserted(cpu, bank, physical))
}

/// The earliest system count at which a timer can signal FIQ
/// ([`Channel::deadline`]). `physical` gives the system count now; it is
/// read at most once, and only for a timer with an offset.
pub fn deadline(cpu: &Cpu, mut physical: impl FnMut() -> u64) -> Option<u64> {
    let mut now = None;
    let mut physical = || *now.get_or_insert_with(&mut physical);
    timers(cpu)
        .filter_map(|(bank, channel)| channel.deadline(cpu, bank, &mut physical))
        .min()
}

/// Timer observation cutoff for a bounded native epoch. The caller polls and
/// delivers pending interrupts before entry, and the epoch cannot change timer
/// controls or interrupt masks/routing. An unscaled, zero-offset asserted channel
/// stays asserted
/// as the machine's monotonic system count advances, so it has no new edge to
/// observe. Keep scaled/offset channels conservative because their count can
/// wrap. This does not change FIQ levels, waiting or ordinary timer deadlines.
pub(crate) fn epoch_deadline(cpu: &Cpu, physical: u64) -> Option<u64> {
    timers(cpu)
        .filter(|(bank, channel)| {
            scaled(cpu, *bank)
                || channel.offset_of(cpu, *bank) != 0
                || !channel.asserted(cpu, *bank, physical)
        })
        .filter_map(|(bank, channel)| channel.deadline(cpu, bank, &mut || physical))
        .min()
}

/// CNTKCTL_EL1 and CNTHCTL_EL2 share the event stream fields.
const EVNTEN: u64 = 1 << 2;
const EVNTDIR: u64 = 1 << 3;
const EVNTI_SHIFT: u32 = 4;
const EVNTIS: u64 = 1 << 17;
const HCR_TGE: u64 = 1 << 27;
const HCR_E2H: u64 = 1 << 34;

/// One generic timer event stream (Arm `TestEventCNTP`, `TestEventCNTV`): an
/// event each time the selected bit of its bank's count minus `offset`
/// rises, or falls under EVNTDIR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stream {
    bank: Bank,
    offset: u64,
    bit: u32,
    falling: bool,
}

impl Stream {
    fn from_control(bank: Bank, control: u64, offset: impl FnOnce() -> u64) -> Option<Self> {
        if control & EVNTEN == 0 {
            return None;
        }
        // FEAT_ECV's EVNTIS selects bits 8 to 23.
        let scale = if control & EVNTIS != 0 { 8 } else { 0 };
        Some(Self {
            bank,
            offset: offset(),
            bit: (control >> EVNTI_SHIFT) as u32 % 16 + scale,
            falling: control & EVNTDIR != 0,
        })
    }

    fn period(&self) -> u64 {
        1 << (self.bit + 1)
    }

    /// How far the bank's `count` is past this stream's last edge, within
    /// one period.
    fn phase(&self, count: u64) -> u64 {
        let edge = if self.falling { 0 } else { 1 << self.bit };
        count.wrapping_sub(self.offset).wrapping_sub(edge) & (self.period() - 1)
    }

    /// Whether an edge falls in the system counts (previous, current].
    pub fn signals(&self, cpu: &Cpu, previous: u64, current: u64) -> bool {
        let previous = count(cpu, self.bank, previous);
        let current = count(cpu, self.bank, current);
        current > previous && current - previous >= self.period() - self.phase(previous)
    }

    /// The first system count after `now` with an edge.
    pub fn next(&self, cpu: &Cpu, now: u64) -> u64 {
        let at = count(cpu, self.bank, now);
        let edge = at.saturating_add(self.period() - self.phase(at));
        system_count(cpu, self.bank, edge)
    }
}

/// The enabled event streams of each bank: CNTHCTL_EL2's on the physical
/// count, and CNTKCTL_EL1's on the virtual count unless HCR_EL2.{E2H,TGE}
/// is {1,1}.
pub fn streams(cpu: &Cpu) -> impl Iterator<Item = Stream> + '_ {
    let host = cpu.read(Register::HCR_EL2) & (HCR_E2H | HCR_TGE) == HCR_E2H | HCR_TGE;
    banks(cpu).iter().flat_map(move |&bank| {
        let read = |register| value(cpu, bank, register);
        let physical = Stream::from_control(bank, read(Register::CNTHCTL_EL2), || 0);
        let virtual_stream = (!host)
            .then(|| {
                Stream::from_control(bank, read(Register::CNTKCTL_EL1), || {
                    read(Register::CNTVOFF_EL2)
                })
            })
            .flatten();
        physical.into_iter().chain(virtual_stream)
    })
}

/// Whether an enabled event stream signals in the system counts
/// (previous, current] (Arm `GenericCounterTick`).
pub fn event(cpu: &Cpu, previous: u64, current: u64) -> bool {
    // A held count, or a serialized one, may read the same twice.
    current != previous && streams(cpu).any(|stream| stream.signals(cpu, previous, current))
}

/// The first system count after `now` at which an event stream signals.
pub fn next_event(cpu: &Cpu, now: u64) -> Option<u64> {
    streams(cpu).map(|stream| stream.next(cpu, now)).min()
}

#[cfg(test)]
mod tests {
    use super::*;
    use {crate::machine::exception::Pstate, stela_contracts::Chip};

    #[test]
    fn disabled_stream_does_not_read_its_offset() {
        assert!(Stream::from_control(Bank::Arm, 0, || panic!("disabled stream offset")).is_none());
        let stream = Stream::from_control(Bank::Arm, EVNTEN, || 9).unwrap();
        assert_eq!(stream.offset, 9);
    }

    #[test]
    fn native_epoch_omits_stable_assertions_but_keeps_every_future_channel() {
        let mut cpu = Cpu::of(Chip::AppleM2, 0, Pstate::reset(2));
        cpu.registers.set(Register::CNTHV_CVAL_EL2, 0, 100);
        cpu.registers.set(Register::CNTHV_CTL_EL2, 0, ENABLE);
        cpu.registers.set(Register::CNTHP_CVAL_EL2, 0, 200);
        cpu.registers.set(Register::CNTHP_CTL_EL2, 0, ENABLE);
        assert_eq!(epoch_deadline(&cpu, 99), Some(100));
        for now in [100, 101, 199] {
            assert!(asserted(&cpu, now));
            assert_eq!(deadline(&cpu, || now), Some(100));
            assert_eq!(epoch_deadline(&cpu, now), Some(200));
        }
        assert_eq!(epoch_deadline(&cpu, 200), None);
        assert!(asserted(&cpu, u64::MAX));
        assert_eq!(epoch_deadline(&cpu, u64::MAX), None);
        // Reprogramming is a source service boundary, and immediately restores
        // a future cutoff. Masking/disable do not create an asserted level.
        cpu.registers.set(Register::CNTHV_CVAL_EL2, 0, 300);
        assert_eq!(epoch_deadline(&cpu, 200), Some(300));
        cpu.registers
            .set(Register::CNTHV_CTL_EL2, 0, ENABLE | IMASK);
        assert_eq!(epoch_deadline(&cpu, 200), None);
        cpu.registers.set(Register::CNTHP_CTL_EL2, 0, 0);
        assert!(!asserted(&cpu, 200));
    }

    #[test]
    fn native_epoch_keeps_scaled_offset_and_gated_timer_rules() {
        let mut cpu = Cpu::of(Chip::AppleM2, 0, Pstate::reset(2));
        cpu.registers.set(Register::CNTVOFF_EL2, 0, 100);
        cpu.registers.set(Register::CNTV_CVAL_EL0, 0, 50);
        cpu.registers.set(Register::CNTV_CTL_EL0, 0, ENABLE);
        assert!(asserted(&cpu, 150));
        assert_eq!(epoch_deadline(&cpu, 150), Some(150));
        cpu.apple.vm_timer_fiq = 0;
        assert!(!asserted(&cpu, 150));
        assert_eq!(epoch_deadline(&cpu, 150), None);

        let mut cpu = Cpu::of(Chip::AppleM3, 0, Pstate::reset(2));
        cpu.registers.set(Register::CNTHV_CVAL_EL2, 0, 125);
        cpu.registers.set(Register::CNTHV_CTL_EL2, 0, ENABLE);
        assert!(asserted(&cpu, 3));
        assert_eq!(epoch_deadline(&cpu, 3), Some(3));
        cpu.registers.set(Register::CNTHV_CTL_EL2, 0, 0);
        assert!(cpu.apple.generic_timer.set(Register::CNTHV_CVAL_EL2, 2));
        assert!(cpu.apple.generic_timer.set(Register::CNTHV_CTL_EL2, ENABLE));
        assert!(asserted(&cpu, 3));
        assert_eq!(deadline(&cpu, || 0), Some(2));
        assert_eq!(epoch_deadline(&cpu, 3), None);
    }

    /// The one bank of a chip without the Apple generic timer.
    #[test]
    fn compare_offset_gating_and_status() {
        let mut cpu = Cpu::of(Chip::AppleM2, 0, Pstate::reset(1));
        let virtual_timer = CHANNELS[1];
        cpu.registers.set(Register::CNTV_CVAL_EL0, 0, 1500);
        cpu.registers.set(Register::CNTVOFF_EL2, 0, 100);
        assert_eq!(deadline(&cpu, || 0), None);
        cpu.registers.set(Register::CNTV_CTL_EL0, 0, ENABLE);
        assert_eq!(deadline(&cpu, || 200), Some(1600));
        assert!(!asserted(&cpu, 1599) && asserted(&cpu, 1600));
        assert_eq!(
            virtual_timer.read_control(&cpu, Bank::Arm, 1600),
            ENABLE | ISTATUS
        );
        cpu.apple.vm_timer_fiq = 0;
        assert!(!asserted(&cpu, 1600) && deadline(&cpu, || 0).is_none());
        assert!(
            virtual_timer.met(&cpu, Bank::Arm, 1600),
            "the condition ignores the gate"
        );
        // The EL2 timers are never gated.
        cpu.registers.set(Register::CNTHP_CVAL_EL2, 0, 10);
        cpu.registers
            .set(Register::CNTHP_CTL_EL2, 0, ENABLE | IMASK);
        assert!(!asserted(&cpu, 20));
        cpu.registers.set(Register::CNTHP_CTL_EL2, 0, ENABLE);
        assert!(asserted(&cpu, 20));
    }

    /// The virtual count is the count less CNTVOFF, modulo 2^64: a compare
    /// value plus the offset that overflows is not a deadline in the past,
    /// and an offset above the count leaves the condition met already. The
    /// deadline agrees with the condition either way.
    #[test]
    fn a_deadline_follows_the_wrapping_virtual_count() {
        let mut cpu = Cpu::of(Chip::AppleM2, 0, Pstate::reset(1));
        cpu.registers.set(Register::CNTVOFF_EL2, 0, 100);
        cpu.registers.set(Register::CNTV_CVAL_EL0, 0, u64::MAX);
        cpu.registers.set(Register::CNTV_CTL_EL0, 0, ENABLE);
        // Count past the offset: the virtual count never reaches "never".
        assert!(!asserted(&cpu, 150));
        assert_eq!(deadline(&cpu, || 150), None);
        // Count below it: the virtual count is just below 2^64 and reaches
        // all ones at count 99.
        assert!(!asserted(&cpu, 50));
        assert_eq!(deadline(&cpu, || 50), Some(99));
        assert!(asserted(&cpu, 99));
        // A guest hypervisor's VM whose virtual count runs 1000 ahead.
        cpu.registers
            .set(Register::CNTVOFF_EL2, 0, 1000u64.wrapping_neg());
        cpu.registers.set(Register::CNTV_CVAL_EL0, 0, 5000);
        assert_eq!(deadline(&cpu, || 0), Some(4000));
        assert!(!asserted(&cpu, 3999) && asserted(&cpu, 4000));
        // One already due is due now, not after the count wraps.
        cpu.registers.set(Register::CNTV_CVAL_EL0, 0, 500);
        assert!(asserted(&cpu, 0));
        assert_eq!(deadline(&cpu, || 0), Some(0));
        assert_eq!(deadline(&cpu, || 7), Some(7));
    }

    #[test]
    fn event_streams_signal_on_the_selected_edge() {
        let mut cpu = Cpu::of(Chip::AppleM2, 0, Pstate::reset(2));
        cpu.registers.set(Register::HCR_EL2, 0, HCR_E2H | HCR_TGE);
        assert_eq!(next_event(&cpu, 0), None);
        // CNTHCTL_EL2: bit 2 rising, at counts 4, 12, 20, ...
        cpu.registers
            .set(Register::CNTHCTL_EL2, 0, EVNTEN | 2 << EVNTI_SHIFT);
        assert!(!event(&cpu, 0, 3) && event(&cpu, 3, 4) && !event(&cpu, 4, 11));
        assert!(event(&cpu, 4, 12) && event(&cpu, 0, 100) && !event(&cpu, 5, 5));
        assert_eq!(
            (next_event(&cpu, 0), next_event(&cpu, 4)),
            (Some(4), Some(12))
        );
        // Falling: at counts 8, 16, ...
        cpu.registers.set(
            Register::CNTHCTL_EL2,
            0,
            EVNTEN | EVNTDIR | 2 << EVNTI_SHIFT,
        );
        assert!(!event(&cpu, 0, 7) && event(&cpu, 7, 8));
        assert_eq!(next_event(&cpu, 8), Some(16));
        // EVNTIS (FEAT_ECV) adds 8 to EVNTI; without it (the M1) the bit is
        // RES0 and EVNTI 0 signals every count.
        cpu.registers.set(Register::CNTHCTL_EL2, 0, EVNTEN | EVNTIS);
        let ecv = cpu.registers.has(stela_contracts::Feature::FEAT_ECV);
        assert_eq!(next_event(&cpu, 0), Some(if ecv { 256 } else { 1 }));
        // The EL1 stream counts the virtual count and only outside the host.
        cpu.registers.set(Register::CNTHCTL_EL2, 0, 0);
        cpu.registers.set(Register::CNTVOFF_EL2, 0, 1);
        cpu.registers
            .set(Register::CNTKCTL_EL1, 0, EVNTEN | 2 << EVNTI_SHIFT);
        assert_eq!(next_event(&cpu, 0), None);
        cpu.registers.set(Register::HCR_EL2, 0, HCR_E2H);
        assert!(!event(&cpu, 0, 4) && event(&cpu, 4, 5));
        assert_eq!(next_event(&cpu, 5), Some(13));
    }

    /// From the M3 on, the Arm bank's timers and streams count the system
    /// count scaled to 1 GHz, and the Apple bank's the unscaled count; each
    /// signals on its own.
    #[test]
    fn both_banks_signal_on_their_own_counts() {
        let mut cpu = Cpu::of(Chip::AppleM3, 0, Pstate::reset(2));
        cpu.registers.set(Register::HCR_EL2, 0, HCR_E2H | HCR_TGE);
        // 125 Arm ticks are 3 system ticks.
        cpu.registers.set(Register::CNTV_CVAL_EL0, 0, 125);
        cpu.registers.set(Register::CNTV_CTL_EL0, 0, ENABLE);
        assert_eq!(deadline(&cpu, || 0), Some(3));
        assert!(!asserted(&cpu, 2) && asserted(&cpu, 3));
        let apple = &mut cpu.apple.generic_timer;
        assert!(
            apple.set(Register::CNTHV_CVAL_EL2, 2) && apple.set(Register::CNTHV_CTL_EL2, ENABLE)
        );
        assert_eq!(deadline(&cpu, || 0), Some(2));
        let el2_virtual = CHANNELS[3];
        assert_eq!(
            el2_virtual.read_control(&cpu, Bank::Apple, 2),
            ENABLE | ISTATUS
        );
        assert_eq!(el2_virtual.read_control(&cpu, Bank::Arm, 2), 0);
        // CNTHCTL_EL2 of each bank: the Apple one's rising bit 1 at system
        // count 2, the Arm one's rising bit 9 at scaled count 512, system
        // count 13.
        cpu.registers
            .set(Register::CNTHCTL_EL2, 0, EVNTEN | 9 << EVNTI_SHIFT);
        assert_eq!(next_event(&cpu, 0), Some(13));
        assert!(!event(&cpu, 0, 12) && event(&cpu, 12, 13));
        let control = EVNTEN | 1 << EVNTI_SHIFT;
        assert!(cpu.apple.generic_timer.set(Register::CNTHCTL_EL2, control));
        assert_eq!(next_event(&cpu, 0), Some(2));
    }
}
