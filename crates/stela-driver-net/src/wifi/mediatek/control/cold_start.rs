// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2023 MediaTek Inc.
// Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
//! Initial MT7925 WLAN reset from Linux mt792x_wfsys_reset_default. This private
//! install step provides neither runtime recovery nor shared reset permission.
use super::{BarIo, Control, Error, Observation};
use crate::wifi::mediatek::identity::Chip;
use stela_driver_base::stepped::until;

const WFSYS: u32 = 0x7c000140;
const RESET_B: u32 = 1;
const INIT_DONE: u32 = 1 << 4;

impl<I: BarIo> Control<I> {
    /// # Safety
    /// This is the initial exclusive MT7925 WLAN install epoch, before
    /// any rings/engine publication, with no old WLAN firmware/DMA/IRQ users.
    /// External shared power/reset operations still require retirement of all
    /// affected functions; this WLAN observation grants no such permission.
    pub(in crate::wifi::mediatek) async unsafe fn initial_wfsys_reset(
        &mut self,
    ) -> Result<Observation, Error> {
        self.guard(true)?;
        if self.registers.chip() != Chip::Mt7925 || self.initial_reset_attempted {
            return Err(Error::InitialResetRefused);
        }
        // Never reissue even if an access/ACK is uncertain. Controller install
        // also enters terminal quarantine before the first ownership request.
        self.initial_reset_attempted = true;
        self.dma_stopped = false;
        let result = async {
            self.modify(WFSYS, RESET_B, 0)?;
            until(self.clock.deadline(50_000)).await;
            self.modify(WFSYS, 0, RESET_B)?;
            // __mt76_poll_msec(..., 500) uses tick=10. The donor's do/while
            // performs 51 reads and sleeps after every failed read, including
            // the final failure. Use the lower bound of usleep_range(10,20)ms.
            for reads in 1..=51 {
                let value = self.read_control(WFSYS)?;
                if value & INIT_DONE == INIT_DONE {
                    return Ok(Observation {
                        address: WFSYS,
                        value,
                        reads,
                    });
                }
                until(self.clock.deadline(10_000)).await;
                if reads == 51 {
                    return Err(Error::Timeout {
                        address: WFSYS,
                        mask: INIT_DONE,
                        expected: INIT_DONE,
                        last: value,
                    });
                }
            }
            unreachable!("the final failed observation returns above")
        }
        .await;
        self.finish(result)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::wifi::mediatek::registers::{self, Registers};
    use alloc::vec::Vec;
    use stela_driver_base::stepped::block_on;

    struct Model {
        value: u32,
        ack_after: Option<u32>,
        polls: u32,
        asserted: bool,
        log: Vec<(u8, u32, u32, u64)>,
        fail_at: Option<usize>,
        unavailable: bool,
        clock: crate::Clock,
    }
    // SAFETY: the model answers only for this test's simulated device; nothing else reaches its
    // state.
    unsafe impl BarIo for Model {
        fn read32(&mut self, offset: u32) -> Result<u32, registers::Error> {
            let value = if offset == 0xe0010 {
                0
            } else {
                assert_eq!(offset, 0xf0140);
                if self.asserted {
                    self.polls += 1;
                    if self.ack_after.is_some_and(|n| self.polls >= n) {
                        self.value |= INIT_DONE;
                    }
                }
                if self.unavailable {
                    u32::MAX
                } else {
                    self.value
                }
            };
            self.log.push((0, offset, value, self.clock.now_us()));
            if self.fail_at == Some(self.log.len()) {
                return Err(registers::Error::AccessFailed);
            }
            Ok(value)
        }
        fn write32(&mut self, offset: u32, value: u32) -> Result<(), registers::Error> {
            self.log.push((1, offset, value, self.clock.now_us()));
            if self.fail_at == Some(self.log.len()) {
                return Err(registers::Error::AccessFailed);
            }
            if offset == 0xf0140 {
                self.value = value;
                if value & RESET_B == 0 {
                    self.value &= !INIT_DONE;
                }
                self.asserted = value & RESET_B != 0;
            } else {
                assert_eq!((offset, value), (0xe0010, 2));
            }
            Ok(())
        }
    }
    fn setup(chip: Chip, initial: u32, ack_after: Option<u32>) -> Control<Model> {
        // Compressed TSC delays exercise the real Clock lower bounds;
        // this scale is not calibrated physical-host timing validation.
        let clock = crate::Clock {
            ticks_per_microsecond: 10,
        };
        let m = Model {
            value: initial,
            ack_after,
            polls: 0,
            asserted: false,
            log: Vec::new(),
            fail_at: None,
            unavailable: false,
            clock,
        };
        // SAFETY: the model device, its pages and its domain belong to this test alone; no other
        // driver, worker or DMA touches them.
        unsafe { Control::new(Registers::new(chip, m), clock) }.unwrap()
    }
    fn run(c: &mut Control<Model>) -> Result<Observation, Error> {
        // SAFETY: the model device, its pages and its domain belong to this test alone; no other
        // driver, worker or DMA touches them.
        unsafe { block_on(c.initial_wfsys_reset()) }
    }
    #[test]
    fn initial_reset_ack_boundaries_preserve_bits_and_real_clock_delays() {
        for ack in [Some(1), Some(2), Some(50), Some(51), Some(52), None] {
            let mut c = setup(Chip::Mt7925, 0xa5a50011, ack);
            block_on(c.driver_own(false)).unwrap();
            let start = c.clock.now_us();
            let result = run(&mut c);
            let end = c.clock.now_us();
            let io = c.registers.test_io();
            let writes: Vec<_> = io
                .log
                .iter()
                .filter(|v| v.0 == 1 && v.1 == 0xf0140)
                .collect();
            assert_eq!(writes.len(), 2);
            assert_eq!(writes[0].2, 0xa5a50010);
            assert_eq!(writes[1].2, 0xa5a50001);
            assert!(writes[1].3 - writes[0].3 >= 50_000);
            let reads = ack.unwrap_or(52).min(51);
            assert_eq!(io.polls, reads);
            let failed = if ack.is_some_and(|n| n <= 51) {
                reads - 1
            } else {
                reads
            };
            assert!(end - start >= 50_000 + u64::from(failed) * 10_000);
            let poll: Vec<_> = io
                .log
                .iter()
                .filter(|v| v.0 == 0 && v.1 == 0xf0140)
                .skip(2)
                .collect();
            for pair in poll.windows(2) {
                assert!(pair[1].3 - pair[0].3 >= 10_000);
            }
            if ack.is_some_and(|n| n <= 51) {
                assert_eq!(
                    result.unwrap(),
                    Observation {
                        address: WFSYS,
                        value: 0xa5a50011,
                        reads
                    }
                );
                let n = c.registers.test_io().log.len();
                assert_eq!(run(&mut c), Err(Error::InitialResetRefused));
                assert_eq!(c.registers.test_io().log.len(), n);
            } else {
                assert_eq!(
                    result,
                    Err(Error::Timeout {
                        address: WFSYS,
                        mask: 16,
                        expected: 16,
                        last: 0xa5a50001
                    })
                );
                assert!(c.is_poisoned());
            }
        }
    }
    #[test]
    fn reset_requires_initial_chip_ownership_and_failures_cannot_retry() {
        for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
            let mut c = setup(chip, 0, Some(1));
            assert_eq!(run(&mut c), Err(Error::NotDriverOwned));
            assert!(c.registers.test_io().log.is_empty());
            if chip != Chip::Mt7925 {
                block_on(c.driver_own(false)).unwrap();
                assert_eq!(run(&mut c), Err(Error::InitialResetRefused));
                assert!(c.registers.test_io().log.iter().all(|v| v.1 != 0xf0140));
            }
        }
        // Two RMWs and the first poll: fail every MMIO boundary.
        for fail in 1..=5 {
            let mut c = setup(Chip::Mt7925, 0, Some(1));
            block_on(c.driver_own(false)).unwrap();
            let io = c.registers.test_io();
            io.fail_at = Some(io.log.len() + fail);
            assert!(matches!(run(&mut c), Err(Error::Register(_))));
            assert!(c.is_poisoned());
            let n = c.registers.test_io().log.len();
            assert_eq!(run(&mut c), Err(Error::Poisoned));
            assert_eq!(block_on(c.stop_dma(true)), Err(Error::Poisoned));
            assert_eq!(c.registers.test_io().log.len(), n);
        }
        let mut c = setup(Chip::Mt7925, 0, Some(1));
        block_on(c.driver_own(false)).unwrap();
        c.registers.test_io().unavailable = true;
        assert_eq!(run(&mut c), Err(Error::Unavailable { address: WFSYS }));
        assert!(c.is_poisoned());
    }
    #[test]
    #[ignore = "oracle: STELA_MT7925_WFSYS_ORACLE"]
    fn unchanged_donor_reset_helper_matches_native_register_trace_and_minimum_waits() {
        let path = std::env::var("STELA_MT7925_WFSYS_ORACLE").unwrap();
        let input = std::fs::read_to_string(path).unwrap();
        let mut cases = 0;
        for line in input.lines() {
            let fields: Vec<_> = line.split_whitespace().collect();
            assert_eq!(fields.len(), 6);
            let initial = u32::from_str_radix(fields[0], 16).unwrap();
            let ack: u32 = fields[1].parse().unwrap();
            assert!(cases < 18);
            assert_eq!(initial, [0, 0xa5a50011, 0x12345678][cases / 6]);
            assert_eq!(ack, [1, 2, 50, 51, 52, 0][cases % 6]);
            let status: i32 = fields[2].parse().unwrap();
            let polls: u32 = fields[3].parse().unwrap();
            let wait: u64 = fields[4].parse().unwrap();
            let mut c = setup(Chip::Mt7925, initial, (ack != 0).then_some(ack));
            block_on(c.driver_own(false)).unwrap();
            let start = c.clock.now_us();
            let result = run(&mut c);
            assert!(c.clock.now_us() - start >= wait);
            assert_eq!(if result.is_ok() { 0 } else { -110 }, status);
            let io = c.registers.test_io();
            assert_eq!(io.polls, polls);
            let mut trace = alloc::string::String::new();
            use core::fmt::Write;
            for &(kind, address, value, _) in &io.log {
                if address == 0xf0140 {
                    write!(trace, "{}:{value:08x},", if kind == 0 { 'r' } else { 'w' }).unwrap();
                }
            }
            assert_eq!(trace, fields[5], "initial={initial:x} ack={ack}");
            cases += 1;
        }
        assert_eq!(cases, 18);
    }
}
