//! Source selection: the invariant TSC at the boot's calibrated rate or an
//! exclusively owned HPET. No bootstrap Cpu.
use super::{WorkError, counter::Counter};
use alloc::{boxed::Box, rc::Rc, sync::Arc};
pub(crate) use stela_boot_core::hpet::ElapsedBounds;
pub(super) use stela_boot_core::source_health::Health;
use stela_drivers::bus::pci::power::{Error as TimerError, Interval, Timer};

#[derive(Clone)]
pub(crate) enum Prepared {
    Hpet(Arc<Counter>),
    Tsc(Tsc),
}

/// The invariant TSC at the boot's calibrated rate.
#[derive(Clone, Copy)]
pub(crate) struct Tsc(stela_boot_core::tsc::NominalRate);

impl Tsc {
    fn new(hz: u64) -> Result<Self, WorkError> {
        if !crate::cpu::INVARIANT_TSC.present() {
            return Err(WorkError::Unavailable);
        }
        stela_boot_core::tsc::NominalRate::new(hz)
            .map(Self)
            .map_err(|_| WorkError::Unavailable)
    }
}

/// Process-only physical minimum from a retained native HPET handoff.
struct HpetMinimum(Arc<Counter>);

struct HpetInterval {
    counter: Arc<Counter>,
    start: u64,
    ticks: u64,
}

impl Interval for HpetInterval {
    fn elapsed(&self) -> Result<bool, TimerError> {
        self.counter
            .check_process()
            .map_err(|_| TimerError::Clock)?;
        let now = self.counter.read_irq().map_err(|_| TimerError::Clock)?;
        let elapsed = now.wrapping_sub(self.start);
        if elapsed > u64::MAX / 2 {
            return Err(TimerError::Clock);
        }
        Ok(elapsed >= self.ticks)
    }
}

impl HpetMinimum {
    fn target(&self, microseconds: u64) -> Result<u64, WorkError> {
        self.0.check_process()?;
        let ns = microseconds
            .checked_mul(1_000)
            .ok_or(WorkError::Exhausted)?;
        let ticks = self.0.ticks_for_minimum_ns(ns)?;
        if ticks > u64::MAX / 2 {
            return Err(WorkError::Exhausted);
        }
        Ok(ticks)
    }

    fn validate(&self) -> Result<(), WorkError> {
        // Cover the longest PCI power readiness interval before admitting use.
        self.target(60_000_000)?;
        let start = self.0.read_irq()?;
        self.0.progress_after(start)?;
        Ok(())
    }
}

// SAFETY: the exclusive native counter retains its source and verifies root,
// process and power continuity. Only conservative HPET ticks satisfy a wait.
unsafe impl Timer for HpetMinimum {
    fn check(&self) -> Result<(), TimerError> {
        self.validate().map_err(|_| TimerError::Clock)
    }

    fn wait_us(&self, microseconds: u64) -> Result<(), TimerError> {
        let mut remaining = self.target(microseconds).map_err(|_| TimerError::Clock)?;
        let mut current = self.0.read_irq().map_err(|_| TimerError::Clock)?;
        while remaining != 0 {
            let mut next = self.0.read_irq().map_err(|_| TimerError::Clock)?;
            if next == current {
                next = self
                    .0
                    .progress_after(current)
                    .map_err(|_| TimerError::Clock)?;
            }
            let elapsed = next.wrapping_sub(current);
            if elapsed > u64::MAX / 2 {
                return Err(TimerError::Clock);
            }
            remaining = remaining.saturating_sub(elapsed);
            current = next;
        }
        Ok(())
    }

    fn begin_interval(&self, microseconds: u64) -> Result<Option<Box<dyn Interval>>, TimerError> {
        let ticks = self.target(microseconds).map_err(|_| TimerError::Clock)?;
        let start = self.0.read_irq().map_err(|_| TimerError::Clock)?;
        Ok(Some(Box::new(HpetInterval {
            counter: self.0.clone(),
            start,
            ticks,
        })))
    }
}

impl Prepared {
    /// The invariant TSC at `hz`, the boot's calibrated rate.
    pub(crate) fn tsc(hz: u64) -> Result<Self, WorkError> {
        Tsc::new(hz).map(Self::Tsc)
    }
    pub(crate) fn exclusive_native_hpet(
        owner: crate::native_mmio::NativeExclusiveHpet,
    ) -> Result<Self, WorkError> {
        Counter::attach_native_exclusive(owner).map(|counter| Self::Hpet(Arc::new(counter)))
    }
    #[cfg(feature = "diagnostics")]
    pub(crate) fn standalone_hpet(owner: crate::paging::config::Hpet) -> Result<Self, WorkError> {
        Counter::attach_standalone(owner).map(|counter| Self::Hpet(Arc::new(counter)))
    }

    pub(crate) fn pci_minimum(&self) -> Result<Rc<dyn Timer>, WorkError> {
        let counter = self.hpet_counter().ok_or(WorkError::Unavailable)?;
        let minimum = HpetMinimum(counter.clone());
        minimum.validate()?;
        Ok(Rc::new(minimum))
    }

    pub(crate) fn hpet_counter(&self) -> Option<&Arc<super::HpetCounter>> {
        match self {
            Self::Hpet(counter) => Some(counter),
            Self::Tsc(_) => None,
        }
    }
    pub(super) fn continuous_period(&self) -> Result<u32, WorkError> {
        match self {
            Self::Hpet(counter) => counter.continuous_period(),
            Self::Tsc(tsc) => Ok(tsc.0.period_fs()),
        }
    }
    pub(super) fn elapsed_bounds(&self, ticks: u64) -> Result<ElapsedBounds, WorkError> {
        match self {
            Self::Hpet(counter) => counter.elapsed_bounds(ticks),
            Self::Tsc(tsc) => tsc
                .0
                .elapsed_bounds(ticks)
                .map_err(|_| WorkError::Exhausted),
        }
    }
    pub(super) fn ticks_for_minimum_ns(&self, minimum_ns: u64) -> Result<u64, WorkError> {
        match self {
            Self::Hpet(counter) => counter.ticks_for_minimum_ns(minimum_ns),
            Self::Tsc(tsc) => tsc
                .0
                .ticks_for_minimum_ns(minimum_ns)
                .map_err(|_| WorkError::Exhausted),
        }
    }

    pub(super) fn admit(
        &self,
        identity: &Arc<crate::engine::host::RootIdentity>,
    ) -> Result<(), WorkError> {
        match self {
            Self::Tsc(tsc) => Tsc::new(tsc.0.hz()).map(|_| ()),
            Self::Hpet(counter) => {
                counter.check_process()?;
                if counter
                    .identity()
                    .is_none_or(|owner| !Arc::ptr_eq(owner, identity))
                {
                    return Err(WorkError::WrongCpu);
                }
                Ok(())
            }
        }
    }
    /// # Safety
    /// Original source and CPU/root lifetime are held; caller excludes
    /// its own IRQs while mutating the serialized previous/elapsed sample pair.
    pub(super) unsafe fn read(&self) -> Result<u64, WorkError> {
        match self {
            Self::Hpet(counter) => counter.read_irq(),
            Self::Tsc(_) => Ok(super::tsc::fenced()),
        }
    }
    pub(super) fn progress_after(&self, previous: u64) -> Result<u64, WorkError> {
        match self {
            Self::Hpet(counter) => counter.progress_after(previous),
            Self::Tsc(_) => loop {
                let current = super::tsc::fenced();
                if current != previous {
                    return self.delta(previous, current).map(|_| current);
                }
                core::hint::spin_loop();
            },
        }
    }
    pub(super) fn delta(&self, previous: u64, current: u64) -> Result<u64, WorkError> {
        match self {
            Self::Hpet(_) | Self::Tsc(_) => {
                let delta = current.wrapping_sub(previous);
                if delta > u64::MAX / 2 {
                    return Err(WorkError::corrupt());
                }
                Ok(delta)
            }
        }
    }
    pub(super) fn ns(&self, ticks: u64) -> Result<u64, WorkError> {
        match self {
            Self::Hpet(counter) => counter.ns(ticks),
            Self::Tsc(tsc) => tsc.0.ns(ticks).map_err(|_| WorkError::Exhausted),
        }
    }
}
