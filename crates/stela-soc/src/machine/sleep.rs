//! The SoC's sleep and wake.
//!
//! XNU 26A434 sleeps an Apple silicon Mac in this order (KC file offsets):
//! AppleSMC writes `WKTP` 2 at its power state 0 (`AppleSMC::setPowerState`
//! 0x2daa73c; the board's SMC has no `WKTP`, so the write fails and the
//! driver logs it), AppleSMCPMU's platform sleep action drops the key
//! endpoint's power state, which writes `MBSE` `slpa` and quiesces the
//! endpoint (`smc`), and `_ml_arm_sleep` (0x4d6f404) then powers every PE
//! down through `arm64_prepare_for_sleep(1)` (0x4b99170), a WFI in
//! CYC_OVRD's power-down mode with the IRQ and FIQ lines left enabled
//! (`engine::processors`). The boot PE goes last, once the others hold the
//! sleep token.
//!
//! With `slpa` taken and every PE down, the SoC sleeps (`info-has_slpsmc`:
//! the SMC owns the sleep). The AP's own interrupt lines are dark: no AIC
//! or fast-IPI line reaches a PE, so no AP device wakes it. The SMC wakes
//! the AP for its sources, each through the AOP AP wake source that
//! `ap-wake-sources` names `SMC.CPUWakeupAP`, which the AP must have left
//! unmasked (`wake`): the power button (`smc`) and the primary PMU's RTC
//! alarm, whose interrupt the SMC watches while the SoC sleeps (`pmu`). Its
//! wake sets `MBSW` to the source and releases the selected boot PE from
//! reset at its RVBAR, the warm path XNU resumes on. A sleep with no
//! modeled source stays asleep; the host's power button reaches the SMC
//! from the mount.
//!
//! Not modeled, so explicit: the PMU's alarm interrupting while the AP
//! runs stops the machine, since what the SMC does with it then is
//! unknown; a board without the wake masks or `SMC.CPUWakeupAP` cannot
//! sleep. The AP's device models keep their state across the sleep, as
//! the hardware's power domains would not: which blocks lose state, and
//! how XNU's drivers restore them, is not yet established block by block.
use super::{AppleDevices, Reset, Start, Transition};
use alloc::{
    collections::{BTreeSet, VecDeque},
    format,
    string::String,
};
use {crate::pmu, crate::smc::WakeSource, crate::wake::WakeMasks, stela_contracts::bus::DeviceId};

/// One step of the SoC's sleep, for the host's log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerLog {
    /// Every PE is down after `slpa`: the SoC sleeps.
    Sleep,
    /// The SoC woke from this source.
    Wake(&'static str),
    /// This source would wake the SoC, but the AP masked the SMC's wake.
    Masked(&'static str),
}

pub(super) struct SocSleep {
    /// The same existing PE that received the boot architectural state.
    pub(super) boot: usize,
    /// The PEs out of reset and not powered down.
    powered: BTreeSet<usize>,
    pub(super) asleep: bool,
    /// The times the SoC went to sleep since the boot.
    sleeps: u64,
    /// The AOP wake masks and `SMC.CPUWakeupAP`'s source number.
    pub(super) wake_masks: Option<(DeviceId, Option<u8>)>,
    /// The primary PMU's interrupt output, and its level when last seen.
    pub(super) pmu: Option<pmu::Interrupt>,
    pmu_seen: bool,
    log: VecDeque<PowerLog>,
}

impl SocSleep {
    pub(super) fn for_boot_processor(boot: usize) -> Self {
        Self {
            boot,
            powered: BTreeSet::from([boot]),
            asleep: false,
            sleeps: 0,
            wake_masks: None,
            pmu: None,
            pmu_seen: false,
            log: VecDeque::new(),
        }
    }
}

fn name(source: WakeSource) -> &'static str {
    match source {
        WakeSource::PowerButton => "power button",
        WakeSource::Rtc => "RTC alarm",
    }
}

impl AppleDevices {
    /// Whether the SoC sleeps.
    pub fn asleep(&self) -> bool {
        self.sleep.asleep
    }

    /// The times the SoC went to sleep since the boot: a host power button
    /// press after one is a new request, whatever came before it.
    pub fn sleeps(&self) -> u64 {
        self.sleep.sleeps
    }

    /// The next step of the SoC's sleep, for the host's log.
    pub fn take_power_log(&mut self) -> Option<PowerLog> {
        self.sleep.log.pop_front()
    }

    fn halt(&mut self, reason: String) {
        if self.reset.is_none() {
            self.reset = Some(Reset {
                transition: Transition::Halt,
                reason,
            });
        }
    }

    /// Whether PE `cpu` is out of reset and not powered down.
    pub(super) fn processor_running(&self, cpu: usize) -> bool {
        self.sleep.powered.contains(&cpu)
    }

    /// PE `cpu` powered down or up (`Devices::processor_power`); the last
    /// one down after `slpa` puts the SoC to sleep.
    pub(super) fn processor_powered(&mut self, cpu: usize, on: bool) {
        self.set_processor_running(cpu, on);
        if on {
            self.sleep.powered.insert(cpu);
            return;
        }
        self.sleep.powered.remove(&cpu);
        if !self.sleep.powered.is_empty() {
            return;
        }
        if !self.smc().is_ok_and(|smc| smc.sleep_pending()) {
            return;
        }
        if !matches!(self.sleep.wake_masks, Some((_, Some(_)))) {
            self.halt(
                "SoC sleep on a board whose AOP wake masks or SMC.CPUWakeupAP are not modeled: no wake could reach the AP".into(),
            );
            return;
        }
        self.smc().expect("the SMC took slpa").enter_sleep();
        self.sleep.asleep = true;
        self.sleep.sleeps += 1;
        self.sleep.log.push_back(PowerLog::Sleep);
    }

    /// The PMU's interrupt rose: an RTC alarm, which wakes a sleeping SoC.
    pub(super) fn watch_pmu(&mut self) {
        let asserted = self
            .sleep
            .pmu
            .as_ref()
            .is_some_and(pmu::Interrupt::asserted);
        let rising = asserted && !self.sleep.pmu_seen;
        self.sleep.pmu_seen = asserted;
        if !rising {
            return;
        }
        if self.sleep.asleep {
            self.wake(WakeSource::Rtc);
        } else {
            self.halt(
                "the PMU's RTC alarm interrupted the running AP; the SMC's handling of it is not modeled".into(),
            );
        }
    }

    /// The SMC wakes the sleeping SoC for `source`, if the AP left the
    /// SMC's wake unmasked.
    pub(super) fn wake(&mut self, source: WakeSource) {
        let Some((masks, Some(smc_wake))) = self.sleep.wake_masks else {
            return;
        };
        let enabled = self
            .bus
            .device::<WakeMasks>(masks)
            .expect("the wake mask id names its model")
            .enabled(smc_wake);
        match enabled {
            Ok(true) => {}
            Ok(false) => {
                self.sleep.log.push_back(PowerLog::Masked(name(source)));
                return;
            }
            Err(reason) => {
                self.halt(format!("SoC wake by the {}: {reason}", name(source)));
                return;
            }
        }
        let Some(address) = self.locked_reset_vector(self.sleep.boot) else {
            self.halt("the SoC woke, but its boot PE has no RVBAR".into());
            return;
        };
        match self.smc() {
            Ok(smc) => smc.wake(source),
            Err(reason) => {
                self.halt(format!("SoC wake: {reason}"));
                return;
            }
        }
        self.sleep.asleep = false;
        self.starts.push(Start {
            cpu: self.sleep.boot,
            address,
        });
        self.sleep.log.push_back(PowerLog::Wake(name(source)));
    }
}

#[cfg(test)]
mod tests {
    use super::super::{AicLayout, Range};
    use super::*;
    use crate::{rtkit::Endpoint, smc, spmi::SpmiSlave, wake};
    use alloc::vec::Vec;
    use stela_engine::machine::engine::Devices;
    use stela_soc_data::board::SmcController;
    use stela_soc_data::smc::WakeBits;

    const REGION: u64 = 0x2_3fe0_0000;
    const MASKS: u64 = 0x2_3d00_4008;
    const ENTRY: u64 = 0x8_0300_c000;
    const SMC_WAKE: u8 = 81;

    /// A board with an SMC, the AOP wake masks and PE 0's locked RVBAR.
    fn board() -> AppleDevices {
        board_at(0)
    }

    fn board_at(boot: usize) -> AppleDevices {
        let mut devices = AppleDevices::for_boot_processor(boot);
        devices
            .add_aic_for_processors(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0), 4)
            .unwrap();
        devices
            .add_smc(
                &SmcController {
                    wrapper: (0x2_3e40_0000, 0x6c000),
                    interrupts: [0x1f4, 0x1f3, 0x1f6, 0x1f5],
                    watchdog: None,
                    region: (REGION, 0x10_0000),
                    gpio_keys: Vec::new(),
                    quiesced: true,
                    wake_bits: WakeBits {
                        power_button: Some(0),
                        rtc: Some(48),
                    },
                },
                stela_formats::builds::DEFAULT,
            )
            .unwrap();
        devices
            .add_wake_masks(MASKS, [u32::MAX; wake::WORDS], Some(SMC_WAKE))
            .unwrap();
        let range = Range {
            physical: 0x2_1005_0000,
            bytes: 0x9010,
        };
        devices
            .add_lockdown("cpu-impl-reg", 0x5000, range, Some((boot, ENTRY)))
            .unwrap();
        devices
    }

    /// The AP writes `command` to `MBSE`.
    fn system_event(devices: &mut AppleDevices, command: &[u8; 4]) {
        devices
            .write(0, REGION, 4, u64::from(smc::code(command)), 0)
            .unwrap();
        let write = 0x11 | 4 << 16 | u64::from(smc::code(b"MBSE")) << 32;
        devices
            .smc()
            .unwrap()
            .receive(write, &mut Vec::new())
            .unwrap();
    }

    /// The AP lets the SMC wake it (`setAPWakeupSources`).
    fn unmask_smc_wake(devices: &mut AppleDevices) {
        let word = u64::from(SMC_WAKE / 32);
        let mask = !(1u32 << (SMC_WAKE % 32));
        devices
            .write(0, MASKS + word * 4, 4, u64::from(mask), 0)
            .unwrap();
    }

    fn sleep(devices: &mut AppleDevices) {
        system_event(devices, b"slpa");
        devices.processor_power(1, true);
        devices.processor_power(0, false);
        assert!(!devices.asleep());
        devices.processor_power(1, false);
    }

    fn mbsw(devices: &mut AppleDevices) -> u64 {
        let read = 0x10 | 8 << 16 | u64::from(smc::code(b"MBSW")) << 32;
        devices
            .smc()
            .unwrap()
            .receive(read, &mut Vec::new())
            .unwrap();
        devices.read(0, REGION, 8, 0).unwrap()
    }

    #[test]
    fn soc_sleep_and_wake_keep_the_nonzero_handoff_pe() {
        let mut devices = board_at(3);
        assert!(!devices.processor_running(0));
        assert!(devices.processor_running(3));
        unmask_smc_wake(&mut devices);
        assert_eq!(devices.sleeps(), 0);
        system_event(&mut devices, b"slpa");
        devices.processor_power(3, false);
        assert!(devices.asleep());
        assert_eq!(devices.sleeps(), 1, "each sleep is counted");
        assert_eq!(devices.take_power_log(), Some(PowerLog::Sleep));
        devices.power_button(true).unwrap();
        assert!(!devices.asleep());
        assert_eq!(
            devices.take_start(),
            Some(Start {
                cpu: 3,
                address: ENTRY
            })
        );
        assert_eq!(devices.take_start(), None);
        assert_eq!(
            devices.take_power_log(),
            Some(PowerLog::Wake("power button"))
        );
        assert_eq!(mbsw(&mut devices), 1);
    }

    #[test]
    fn the_last_pe_down_after_slpa_sleeps_the_soc_and_the_power_button_wakes_pe_0() {
        let mut devices = board();
        unmask_smc_wake(&mut devices);
        // Cores going down without slpa leave the SoC awake.
        devices.processor_power(0, false);
        assert!(!devices.asleep());
        devices.processor_power(0, true);
        sleep(&mut devices);
        assert!(devices.asleep());
        assert_eq!(devices.take_power_log(), Some(PowerLog::Sleep));
        // Nothing of the AP's reaches a PE.
        assert_eq!(devices.lines(0, 0), super::super::Lines::default());
        assert_eq!(devices.take_start(), None);
        // The press wakes PE 0 at its RVBAR and is the wake's source.
        devices.power_button(true).unwrap();
        devices.power_button(false).unwrap();
        assert!(!devices.asleep());
        assert_eq!(
            devices.take_start(),
            Some(Start {
                cpu: 0,
                address: ENTRY
            })
        );
        assert_eq!(
            devices.take_power_log(),
            Some(PowerLog::Wake("power button"))
        );
        assert_eq!(mbsw(&mut devices), 1);
        // `waka` before the SoC slept cancels a sleep.
        devices.processor_power(0, true);
        system_event(&mut devices, b"slpa");
        system_event(&mut devices, b"waka");
        devices.processor_power(0, false);
        assert!(!devices.asleep());
    }

    #[test]
    fn a_masked_smc_wake_keeps_the_soc_asleep_and_the_rtc_alarm_wakes_it() {
        let mut devices = board();
        let rtc = stela_soc_data::pmu::Rtc {
            upcount: 0xf802,
            clock_offset: stela_soc_data::pmu::ClockOffset::Register(0xf900),
            alarm_control: 0xf800,
            alarm_enable: 0x40,
            alarm_offset: 0xf808,
            alarm_event: 0xf80c,
            irq_mask: 0xf80e,
            event_mask: stela_soc_data::pmu::DEFAULT_EVENT_MASK,
            interrupt_mask: 1,
        };
        let mut pmu = pmu::Pmu::new(Vec::new(), Some(rtc), None);
        devices.sleep.pmu = Some(pmu.interrupt());
        sleep(&mut devices);
        devices.take_power_log();
        // Masked: the press is the SMC's alone.
        devices.power_button(true).unwrap();
        assert!(devices.asleep());
        assert_eq!(
            devices.take_power_log(),
            Some(PowerLog::Masked("power button"))
        );
        devices.power_button(false).unwrap();
        // Unmasked before the alarm (as XNU leaves it before sleeping), the
        // alarm at upcount second 2 wakes PE 0.
        unmask_smc_wake(&mut devices);
        pmu.write(rtc.alarm_offset, &2u32.to_le_bytes(), 0).unwrap();
        pmu.write(rtc.irq_mask, &[0], 0).unwrap();
        pmu.write(rtc.alarm_control, &[0x41], 0).unwrap();
        let second = crate::counter::FREQUENCY_HZ;
        assert_eq!(pmu.advance(0), Some(2 * second));
        devices.lines(0, second);
        assert!(devices.asleep());
        pmu.advance(2 * second);
        devices.deadline(2 * second);
        assert!(!devices.asleep());
        assert_eq!(devices.take_power_log(), Some(PowerLog::Wake("RTC alarm")));
        assert_eq!(mbsw(&mut devices), 1 << 48);
        // Still asserted after the wake, the interrupt does not stop the AP;
        // a new alarm while it runs does.
        devices.lines(0, 2 * second);
        assert_eq!(devices.take_reset(), None);
        pmu.write(rtc.alarm_event, &[1], 3 * second).unwrap();
        devices.lines(0, 3 * second);
        pmu.write(rtc.alarm_offset, &4u32.to_le_bytes(), 3 * second)
            .unwrap();
        pmu.advance(4 * second);
        devices.lines(0, 4 * second);
        assert_eq!(
            devices.take_reset().map(|reset| reset.transition),
            Some(Transition::Halt)
        );
    }

    #[test]
    fn a_board_without_the_smcs_wake_source_cannot_sleep() {
        let mut devices = board();
        devices.sleep.wake_masks = devices.sleep.wake_masks.map(|(id, _)| (id, None));
        sleep(&mut devices);
        assert!(!devices.asleep());
        assert_eq!(
            devices.take_reset().map(|reset| reset.transition),
            Some(Transition::Halt)
        );
    }
}
