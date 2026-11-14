//! RTC publication, board-entry time correlation and the watchdogs.

use super::*;
use crate::Result;
use stela_contracts::refuse;

impl AppleDevices {
    pub(super) fn publish_rtc_time(&self, now: u64) -> Result<(), DeviceError> {
        if let Some(clock) = &self.rtc_clock {
            clock.publish(now).map_err(DeviceError::Refused)?;
        }
        Ok(())
    }

    /// Keep what this boot's PMU upcounts started from: the host's wall
    /// time and the machine state's anchor ([`pmu::RtcEpoch::anchored`]).
    pub fn keep_rtc_origin(
        &mut self,
        wall: Option<stela_soc_data::pmu::WallClock>,
        anchor: Option<pmu::RtcAnchor>,
    ) {
        self.rtc_origin = Some((wall, anchor));
    }

    /// The anchor for the machine state to keep now: the first PMU's upcount
    /// at the latest count a board entry was given. `None` without a PMU
    /// oscillator or kept origin, or before the first entry.
    pub fn rtc_anchor(&self) -> Option<pmu::RtcAnchor> {
        let (wall, loaded) = self.rtc_origin?;
        let now = self.time_seen.get().0;
        let ticks = self.pmu_clocks.first()?.ticks_at(now)?;
        Some(pmu::RtcAnchor::at(now, ticks, wall, loaded))
    }

    /// Record the count board entry `entry` was given.
    pub(super) fn observe_time(&self, now: u64, entry: &'static str) {
        let seen = self.time_seen.get();
        if now < seen.0 {
            if self.time_regression.get().is_none() {
                self.time_regression.set(Some([(now, entry), seen]));
            }
        } else {
            self.time_seen.set((now, entry));
        }
    }

    /// A PMU oscillator's last backwards counter, and the first board entry
    /// given an earlier count than another, for a stop's reason.
    pub(super) fn rtc_regression(&self) -> String {
        let mut reason = String::new();
        if let Some((now, published)) = self
            .rtc_clock
            .iter()
            .chain(&self.pmu_clocks)
            .find_map(pmu::RtcClock::regression)
        {
            reason += &format!(" (PMU counter {now:#x} after publication {published:#x})");
        }
        if let Some([(now, entry), (seen, before)]) = self.time_regression.get() {
            reason += &format!(" ({entry} at {now:#x} after {before} at {seen:#x})");
        }
        reason
    }

    pub(super) fn publish_rtc_time_or_halt(&mut self, now: u64) -> bool {
        if let Err(error) = self.publish_rtc_time(now) {
            if self.reset.is_none() {
                self.reset = Some(Reset {
                    transition: Transition::Halt,
                    reason: format!("{}{error:?}{}", MODEL_REFUSED[1], self.rtc_regression()),
                });
            }
            false
        } else {
            true
        }
    }

    /// The SoC watchdog; `now` is the counter value its timers start from.
    pub fn add_watchdog(
        &mut self,
        ranges: &[Range],
        irq: Option<AicInput>,
        now: u64,
    ) -> Result<(), MapError> {
        let device = self.bus.add(Box::new(AppleWdt::new(irq, now)));
        self.watchdogs.push(device);
        self.map(device, ranges)
    }

    /// Every watchdog's mechanism writes since the last call, and how many
    /// more were not kept.
    pub fn take_watchdog_mechanism_log(&mut self) -> (Vec<crate::wdt::MechanismEvent>, u32) {
        let mut events = Vec::new();
        let mut dropped = 0u32;
        for &device in &self.watchdogs {
            if let Some(watchdog) = self.bus.device_mut::<AppleWdt>(device) {
                let (log, more) = watchdog.take_mechanism_log();
                events.extend(log);
                dropped = dropped.saturating_add(more);
            }
        }
        (events, dropped)
    }

    /// The main watchdog and its independently DT-placed optional side words.
    pub fn add_watchdog_with_side(
        &mut self,
        range: Range,
        side: Option<&stela_soc_data::board::WatchdogSide>,
        irq: Option<AicInput>,
        now: u64,
    ) -> Result<()> {
        let Some(side) = side else {
            return self
                .add_watchdog(&[range], irq, now)
                .map_err(|error| refuse!("watchdog placement: {error}", ?error));
        };
        if !(2..=4).contains(&side.words.len()) {
            return Err(refuse!(
                "watchdog side controls require two to four complete words"
            ));
        }
        let shared_flags = side.words.len() == 4 && side.words[0] == side.words[3];
        let end = range
            .physical
            .checked_add(range.bytes)
            .ok_or(refuse!("watchdog primary extent overflows"))?;
        for (index, &physical) in side.words.iter().enumerate() {
            let last = physical
                .checked_add(4)
                .ok_or(refuse!("watchdog side extent overflows"))?;
            if !physical.is_multiple_of(4) {
                return Err(refuse!("watchdog side word is unaligned"));
            }
            if physical < end && range.physical < last {
                return Err(refuse!(
                    "watchdog side word overlaps primary timer registers"
                ));
            }
            if side.words[..index].contains(&physical) && !(shared_flags && index == 3) {
                return Err(refuse!(
                    "watchdog side alias lacks a qualified shared register owner"
                ));
            }
        }
        let mut triggers = Vec::new();
        for &(physical, mask, value) in &side.triggers {
            let last = physical
                .checked_add(4)
                .ok_or(refuse!("watchdog trigger extent overflows"))?;
            if !physical.is_multiple_of(4) || mask == 0 || value & !mask != 0 {
                return Err(refuse!("watchdog trigger word or mask is invalid"));
            }
            if (physical < end && range.physical < last) || side.words.contains(&physical) {
                return Err(refuse!(
                    "watchdog trigger aliases a different register role"
                ));
            }
            if let Some(&previous) = triggers
                .iter()
                .find(|&&(address, _, _)| address == physical)
            {
                if previous != (physical, mask, value) {
                    return Err(refuse!(
                        "watchdog shared trigger has conflicting configurations"
                    ));
                }
            } else {
                triggers.push((physical, mask, value));
            }
        }
        let mut model = AppleWdt::new(irq, now).with_side_controls();
        if shared_flags {
            model = model.with_shared_panic_flags();
        }
        let device = self.bus.add(Box::new(model));
        self.watchdogs.push(device);
        self.map(device, &[range])
            .map_err(|error| refuse!("watchdog placement: {error}", ?error))?;
        let offsets = [
            wdt::PANIC_SAVE,
            wdt::MECHANISM,
            wdt::CONFIGURATION,
            wdt::POWER_OFF,
        ];
        for (index, (&physical, offset)) in side.words.iter().zip(offsets).enumerate() {
            if !(shared_flags && index == 3) {
                self.bus.map(device, physical, 4, offset).map_err(|error| {
                    refuse!(
                        "watchdog side word at {physical:#x}: {error}",
                        physical,
                        ?error
                    )
                })?;
            }
        }
        for (physical, mask, _) in triggers {
            let trigger = self.bus.add(Box::new(wdt::CoreTrigger::new(mask)));
            self.bus.map(trigger, physical, 4, 0).map_err(|error| {
                refuse!(
                    "watchdog core trigger at {physical:#x}: {error}",
                    physical,
                    ?error
                )
            })?;
        }
        Ok(())
    }
}
