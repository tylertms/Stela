use super::*;
use crate::wifi::mediatek::identity::Chip;
use alloc::{collections::BTreeMap, vec::Vec};
use stela_driver_base::stepped::block_on;
extern crate std;

struct Model {
    registers: BTreeMap<u32, u32>,
    operations: Vec<(u32, u32, u32)>,
    own_at_attempt: Option<u8>,
    attempts: u8,
    busy_reads: Option<usize>,
    stopping: bool,
    fail_at: Option<usize>,
    ignore_write: Option<u32>,
}
impl Default for Model {
    fn default() -> Self {
        Self {
            registers: BTreeMap::from([
                (0xe0010, 4),
                (0xe00f0, 1),
                (MT_WFDMA0_GLO_CFG, 0x5a228055),
                (MT_WFDMA0_GLO_CFG_EXT0, 0xa5a50040),
                (0xd6004, 0xaabb0000),
                (MT_WFDMA0_RST, 0xc0ff00ff),
            ]),
            operations: Vec::new(),
            own_at_attempt: Some(1),
            attempts: 0,
            busy_reads: Some(0),
            stopping: false,
            fail_at: None,
            ignore_write: None,
        }
    }
}
// SAFETY: serialized hardware state model; ownership/engine bits change
// from modeled device acknowledgements, not blind echo of host commands.
unsafe impl BarIo for Model {
    fn read32(&mut self, offset: u32) -> Result<u32, registers::Error> {
        if offset == 0xe0010 && self.own_at_attempt.is_some_and(|n| self.attempts >= n) {
            *self.registers.get_mut(&offset).unwrap() &= !4;
        }
        if offset == MT_WFDMA0_GLO_CFG
            && self.stopping
            && let Some(left) = self.busy_reads.as_mut()
        {
            if *left == 0 {
                *self.registers.get_mut(&offset).unwrap() &= !BUSY;
            } else {
                *left -= 1;
            }
        }
        let value = self.registers.get(&offset).copied().unwrap_or(0);
        self.operations.push((0, offset, value));
        if self.fail_at == Some(self.operations.len()) {
            return Err(registers::Error::AccessFailed);
        }
        Ok(value)
    }
    fn write32(&mut self, offset: u32, value: u32) -> Result<(), registers::Error> {
        self.operations.push((1, offset, value));
        if self.fail_at == Some(self.operations.len()) {
            return Err(registers::Error::AccessFailed);
        }
        if self.ignore_write == Some(offset) {
            return Ok(());
        }
        if offset == 0xe0010 && value == 2 {
            self.attempts += 1;
        } else if offset == 0xe0010 && value == 1 {
            // SET_OWN: the firmware takes the function and reports OWN_SYNC.
            *self.registers.get_mut(&offset).unwrap() |= 4;
            self.attempts = 0;
        } else {
            self.registers.insert(offset, value);
            if offset == MT_WFDMA0_GLO_CFG && value & DISABLE == 0 {
                self.stopping = true;
            }
        }
        Ok(())
    }
}
fn setup(chip: Chip, model: Model) -> Control<Model> {
    // SAFETY: exclusive serialized model; a compressed local
    // TSC scale bounds fault tests, never checks physical timing.
    unsafe {
        Control::new(
            Registers::new(chip, model),
            crate::Clock {
                ticks_per_microsecond: 10,
            },
        )
    }
    .unwrap()
}
#[test]
fn a_captured_interrupt_is_acknowledged_with_the_vector_quiet() {
    let mut m = Model::default();
    m.registers
        .insert(MT_WFDMA0_HOST_INT_STA, 1 << 29 | 1 << 2 | 1 << 31);
    m.registers.insert(MT_MCU_CMD, 1);
    let mut c = setup(Chip::Mt7925, m);
    block_on(c.driver_own(false)).unwrap();
    c.open_interrupts().unwrap();
    let start = c.registers.test_io().operations.len();
    c.capture_interrupts().unwrap();
    let io = c.registers.test_io();
    let writes: Vec<_> = io.operations[start..]
        .iter()
        .filter(|operation| operation.0 == 1)
        .map(|operation| (operation.1, operation.2))
        .collect();
    assert_eq!(
        writes,
        [
            (MT_WFDMA0_HOST_INT_ENA, 0),
            (MT_WFDMA0_HOST_INT_STA, 1 << 29 | 1 << 2),
            (MT_MCU_CMD, 1),
        ]
    );
    c.unmask_interrupts().unwrap();
    let io = c.registers.test_io();
    assert_eq!(io.registers[&MT_WFDMA0_HOST_INT_ENA], 0x2e07_fff5);
    assert_eq!(io.registers[&MT_PCIE_MAC_INT_ENABLE], 0xff);
    assert_eq!(io.registers[&MT_MCU2HOST_SW_INT_ENA], 1);
}

#[test]
fn ownership_ack_and_dma_idle_precede_scheduler_reset_writes() {
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        for force in [false, true] {
            let mut m = Model::default();
            m.registers.insert(MT_WFDMA0_GLO_CFG, 0x5a228055 | BUSY);
            m.busy_reads = Some(2);
            let mut c = setup(chip, m);
            assert_eq!(block_on(c.stop_dma(force)), Err(Error::NotDriverOwned));
            assert!(c.registers.test_io().operations.is_empty());
            let own = block_on(c.driver_own(false)).unwrap();
            assert_eq!(own.attempts, 1);
            assert_eq!(own.status.value & 4, 0);
            let stopped = block_on(c.stop_dma(force)).unwrap();
            assert!(stopped.status.reads >= 3);
            assert_eq!(stopped.status.value & (DISABLE | BUSY), 0);
            let io = &c.registers.test_io();
            let idle = io
                .operations
                .iter()
                .position(|&(k, a, v)| k == 0 && a == MT_WFDMA0_GLO_CFG && v & BUSY == 0)
                .unwrap();
            let scheduler = io
                .operations
                .iter()
                .position(|&(k, a, _)| k == 1 && a == MT_WFDMA0_GLO_CFG_EXT0)
                .unwrap();
            assert!(idle < scheduler);
            assert_eq!(io.registers[&MT_WFDMA0_GLO_CFG_EXT0] & (1 << 6), 0);
            assert_ne!(io.registers[&0xd6004] & (1 << 28), 0);
            let resets = io
                .operations
                .iter()
                .filter(|v| v.0 == 1 && v.1 == MT_WFDMA0_RST)
                .count();
            assert_eq!(resets, if force { 2 } else { 0 });
            assert_eq!(io.registers[&MT_WFDMA0_RST], 0xc0ff00ff);
        }
    }
}
#[test]
fn ownership_retries_bound_attempts_and_never_proceed_on_timeout() {
    for accept in [Some(3), None] {
        let mut c = setup(
            Chip::Mt7921,
            Model {
                own_at_attempt: accept,
                ..Model::default()
            },
        );
        let result = block_on(c.driver_own(true));
        if accept.is_some() {
            assert_eq!(result.unwrap().attempts, 3);
        } else {
            assert!(matches!(
                result,
                Err(Error::Timeout {
                    address: MT_CONN_ON_LPCTL,
                    mask: 4,
                    expected: 0,
                    ..
                })
            ));
            assert_eq!(c.registers.test_io().attempts, 10);
            let count = c.registers.test_io().operations.len();
            assert_eq!(block_on(c.stop_dma(true)), Err(Error::Poisoned));
            assert_eq!(block_on(c.driver_own(false)), Err(Error::Poisoned));
            assert_eq!(c.registers.test_io().operations.len(), count);
        }
    }
}
#[test]
fn firmware_ownership_precedes_the_driver_s_and_bounds_its_attempts() {
    let mut c = setup(Chip::Mt7925, Model::default());
    assert_eq!(block_on(c.firmware_own()).unwrap().value & 4, 4);
    assert_eq!(block_on(c.driver_own(false)).unwrap().attempts, 1);
    let mut m = Model {
        ignore_write: Some(0xe0010),
        ..Model::default()
    };
    m.registers.insert(0xe0010, 0);
    let mut c = setup(Chip::Mt7925, m);
    assert!(matches!(
        block_on(c.firmware_own()),
        Err(Error::Timeout {
            address: MT_CONN_ON_LPCTL,
            mask: 4,
            expected: 4,
            ..
        })
    ));
    let requests = c.registers.test_io().operations.iter();
    assert_eq!(requests.filter(|&&op| op == (1, 0xe0010, 1)).count(), 10);
    assert_eq!(block_on(c.driver_own(false)), Err(Error::Poisoned));
}
#[test]
fn identity_registers_read_through_the_remap_and_restore_it() {
    // (chip, selector, window base, selector value written)
    for (chip, selector, window, remap) in [
        (Chip::Mt7921, 0xfe24c, 0x40000, 0x1234_7001),
        (Chip::Mt7925, 0x155024, 0x130000, 0x7001_5678),
    ] {
        let mut m = Model::default();
        m.registers.extend([
            (selector, 0x1234_5678),
            (window + 0x200, 0x0001_7961),
            (window + 0x204, 0x8a10),
            (window + 0x20, 0x7f),
        ]);
        let mut c = setup(chip, m);
        assert_eq!(c.identity_registers(), Err(Error::NotDriverOwned));
        block_on(c.firmware_own()).unwrap();
        block_on(c.driver_own(false)).unwrap();
        let bound = (chip == Chip::Mt7921).then_some(0x7f);
        assert_eq!(c.identity_registers(), Ok((0x0001_7961, 0x8a10, bound)));
        let io = c.registers.test_io();
        assert!(io.operations.contains(&(1, selector, remap)));
        let restored = if chip == Chip::Mt7925 {
            0x1234_5678
        } else {
            remap
        };
        assert_eq!(io.registers[&selector], restored);
    }
}
#[test]
fn stuck_dma_never_resets_or_releases_unknown_owned_resources() {
    let mut m = Model {
        busy_reads: None,
        ..Model::default()
    };
    m.registers.insert(MT_WFDMA0_GLO_CFG, 0x5a228055 | BUSY);
    let mut c = setup(Chip::Mt7925, m);
    block_on(c.driver_own(false)).unwrap();
    assert!(matches!(
        block_on(c.stop_dma(true)),
        Err(Error::Timeout {
            address: MT_WFDMA0_GLO_CFG,
            mask: BUSY,
            ..
        })
    ));
    assert!(
        !c.registers
            .test_io()
            .operations
            .iter()
            .any(|&(k, a, _)| k == 1
                && matches!(a, MT_WFDMA0_GLO_CFG_EXT0 | MT_WFDMA0_RST | 0xd6004))
    );
    assert!(c.is_poisoned());
}
#[test]
fn every_ownership_and_stop_io_failure_poison_the_control_epoch() {
    let mut good = setup(Chip::Mt7921, Model::default());
    block_on(good.driver_own(false)).unwrap();
    block_on(good.stop_dma(true)).unwrap();
    let accesses = good.registers.test_io().operations.len();
    for fail_at in 1..=accesses {
        let mut c = setup(
            Chip::Mt7921,
            Model {
                fail_at: Some(fail_at),
                ..Model::default()
            },
        );
        let result = block_on(c.driver_own(false)).and_then(|_| block_on(c.stop_dma(true)));
        assert_eq!(
            result,
            Err(Error::Register(registers::Error::AccessFailed)),
            "access {fail_at}"
        );
        assert!(c.is_poisoned());
        let count = c.registers.test_io().operations.len();
        assert_eq!(block_on(c.poll_n9_ready()), Err(Error::Poisoned));
        assert_eq!(c.registers.test_io().operations.len(), count);
    }
}
#[test]
fn power_and_n9_observations_do_not_synthesize_ready_from_all_ones_or_elapsed_time() {
    for state in [0, 1, 3, u32::MAX] {
        let mut m = Model::default();
        m.registers.insert(0xe00f0, state);
        let mut c = setup(Chip::Mt7925, m);
        block_on(c.driver_own(false)).unwrap();
        let ready = block_on(c.poll_n9_ready());
        if state == 3 {
            assert_eq!(ready.unwrap().value, 3);
        } else if state == u32::MAX {
            assert_eq!(
                ready,
                Err(Error::Unavailable {
                    address: MT_CONN_ON_MISC
                })
            );
        } else {
            assert!(matches!(
                ready,
                Err(Error::Timeout {
                    address: MT_CONN_ON_MISC,
                    expected: 3,
                    ..
                })
            ));
        }
    }
    let mut c = setup(Chip::Mt7922, Model::default());
    block_on(c.driver_own(false)).unwrap();
    assert_eq!(
        block_on(c.poll_firmware_power_on()).unwrap().unwrap().value,
        1
    );
    // mt792x_load_firmware warns and downloads anyway: a late MCU is no
    // failure, and the control stays driver-owned and usable.
    c.registers.test_io().registers.insert(0xe00f0, 3);
    assert_eq!(block_on(c.poll_firmware_power_on()), Ok(None));
    assert!(!c.is_poisoned());
    assert_eq!(block_on(c.poll_n9_ready()).unwrap().value, 3);
}
#[test]
fn unavailable_control_and_zero_clock_fail_before_fabricating_ownership() {
    let mut m = Model::default();
    m.registers.insert(0xe0010, u32::MAX);
    m.own_at_attempt = None;
    let mut c = setup(Chip::Mt7921, m);
    assert_eq!(
        block_on(c.driver_own(false)),
        Err(Error::Unavailable {
            address: MT_CONN_ON_LPCTL
        })
    );
    // SAFETY: exclusive model register file, rejected before any access.
    let registers = unsafe { Registers::new(Chip::Mt7921, Model::default()) };
    assert!(matches!(
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe {
            Control::new(
                registers,
                crate::Clock {
                    ticks_per_microsecond: 0,
                },
            )
        },
        Err(Error::ClockUnavailable)
    ));
}
#[test]
#[ignore = "oracle: STELA_MT792X_CONTROL_ORACLE"]
fn unchanged_pinned_ownership_and_disable_helpers_match_register_effects() {
    let root = std::env::var_os("STELA_MT792X_CONTROL_ORACLE").expect("reference trace directory");
    for force in [false, true] {
        let bytes = std::fs::read(std::path::Path::new(&root).join(if force {
            "force.bin"
        } else {
            "stop.bin"
        }))
        .unwrap();
        let mut at = 0;
        let word = |at: &mut usize| {
            let result = u32::from_le_bytes(bytes[*at..*at + 4].try_into().unwrap());
            *at += 4;
            result
        };
        let nops = word(&mut at);
        let npolls = word(&mut at);
        let expected: Vec<_> = (0..nops)
            .map(|_| (word(&mut at), word(&mut at), word(&mut at)))
            .collect();
        let polls: Vec<_> = (0..npolls)
            .map(|_| {
                let p = (
                    word(&mut at),
                    word(&mut at),
                    word(&mut at),
                    u64::from(word(&mut at)) * 1000,
                );
                assert_eq!(word(&mut at), 1); // donor polling tick, milliseconds
                p
            })
            .collect();
        assert_eq!(at, bytes.len());
        for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
            let mut c = setup(chip, Model::default());
            block_on(c.driver_own(false)).unwrap();
            block_on(c.stop_dma(force)).unwrap();
            assert_eq!(c.polls, polls);
            let mut actual = core::mem::take(&mut c.registers.test_io().operations);
            // Stela additionally flushes the final posted write; this is
            // tested explicitly instead of hidden in donor equivalence.
            assert_eq!(actual.pop(), Some((0, MT_WFDMA0_RST, 0xc0ff00ff)));
            assert_eq!(actual, expected);
        }
    }
}
#[test]
fn polling_start_uses_chip_prefetch_and_index_reset_with_all_irq_sources_masked() {
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let mut c = setup(chip, Model::default());
        // SAFETY: isolated register-algorithm fixture, no DMA hardware.
        assert_eq!(unsafe { c.start_dma_polling() }, Err(Error::NotDriverOwned));
        block_on(c.driver_own(false)).unwrap();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        assert_eq!(unsafe { c.start_dma_polling() }, Err(Error::DmaNotStopped));
        block_on(c.stop_dma(true)).unwrap();
        let start = c.registers.test_io().operations.len();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        let enabled = unsafe { c.start_dma_polling() }.unwrap();
        assert_eq!(enabled.global_config & 5, 5);
        assert!(enabled.polling_interrupts_masked);
        let io = c.registers.test_io();
        for address in [0xd4204, 0x10188, 0xd41f4] {
            assert_eq!(io.registers[&address], 0);
        }
        assert_eq!(io.registers[&0xd420c], u32::MAX);
        assert_eq!(io.registers[&0xd42f0], 0);
        let count = io.operations[start..]
            .iter()
            .filter(|&&(kind, a, _)| kind == 1 && (0xd4600..0xd46a0).contains(&a))
            .count();
        if chip == Chip::Mt7925 {
            assert_eq!(count, 10);
            assert_eq!(io.registers[&0xd4280], u32::MAX);
            assert_eq!(io.registers[&0xd4600], 0x01000010);
            assert_eq!(io.registers[&0xd463c], 0x05000004);
            assert_eq!(io.registers[&0xd42b4] & (1 << 28), 1 << 28);
            assert_eq!(io.registers[&0xd4298] & 0xf00, 0xf00);
            assert_eq!(io.registers[&0xd429c] & 0x7f00, 0x7f00);
        } else {
            assert_eq!(count, 14);
            assert!(!io.registers.contains_key(&0xd4280));
            assert_eq!(io.registers[&0xd4600], 0x01400004);
            assert_eq!(io.registers[&0xd4644], 0x03800004);
        }
        let writes = io.operations.len();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        assert_eq!(unsafe { c.start_dma_polling() }, Err(Error::DmaNotStopped));
        assert_eq!(c.registers.test_io().operations.len(), writes);
    }
}
#[test]
fn unsupported_dma_mode_and_ignored_control_stores_never_check_start() {
    for (address, initial) in [
        (MT_WFDMA0_GLO_CFG, 1 << 26),
        (MT_WFDMA0_GLO_CFG, 1),
        (0xd4204, 1),
    ] {
        let mut c = setup(Chip::Mt7925, Model::default());
        block_on(c.driver_own(false)).unwrap();
        block_on(c.stop_dma(true)).unwrap();
        let old = c
            .registers
            .test_io()
            .registers
            .get(&address)
            .copied()
            .unwrap_or(0);
        c.registers
            .test_io()
            .registers
            .insert(address, old | initial);
        if address != MT_WFDMA0_GLO_CFG {
            c.registers.test_io().ignore_write = Some(address);
        }
        assert!(matches!(
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            unsafe { c.start_dma_polling() },
            Err(Error::UnsupportedDmaMode(_)) | Err(Error::Readback { .. })
        ));
        assert!(c.is_poisoned());
        assert!(
            !c.registers
                .test_io()
                .operations
                .iter()
                .any(|&(kind, a, v)| kind == 1 && a == MT_WFDMA0_GLO_CFG && v & 5 == 5)
        );
    }
    let mut c = setup(
        Chip::Mt7921,
        Model {
            ignore_write: Some(MT_WFDMA0_GLO_CFG),
            ..Model::default()
        },
    );
    block_on(c.driver_own(false)).unwrap();
    assert!(matches!(
        block_on(c.stop_dma(true)),
        Err(Error::Readback {
            address: MT_WFDMA0_GLO_CFG,
            expected: 0,
            ..
        })
    ));
    assert!(!c.dma_stopped);
    let mut c = setup(Chip::Mt7921, Model::default());
    block_on(c.driver_own(false)).unwrap();
    block_on(c.stop_dma(true)).unwrap();
    c.registers.test_io().ignore_write = Some(MT_WFDMA0_GLO_CFG);
    assert!(matches!(
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { c.start_dma_polling() },
        Err(Error::Readback {
            address: MT_WFDMA0_GLO_CFG,
            ..
        })
    ));
    assert!(c.is_poisoned());
}
#[test]
fn every_dma_start_publication_failure_poison_context_without_retry() {
    for chip in [Chip::Mt7921, Chip::Mt7925] {
        let mut c = setup(chip, Model::default());
        block_on(c.driver_own(false)).unwrap();
        block_on(c.stop_dma(true)).unwrap();
        let initial = c.registers.test_io().operations.len();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { c.start_dma_polling() }.unwrap();
        let end = c.registers.test_io().operations.len();
        for fail_at in initial + 1..=end {
            let mut c = setup(chip, Model::default());
            block_on(c.driver_own(false)).unwrap();
            block_on(c.stop_dma(true)).unwrap();
            c.registers.test_io().fail_at = Some(fail_at);
            assert_eq!(
                // SAFETY: the model device, its pages and its domain belong to this test alone;
                // no other driver, worker or DMA touches them.
                unsafe { c.start_dma_polling() },
                Err(Error::Register(registers::Error::AccessFailed)),
                "chip {chip:?} access {fail_at}"
            );
            assert!(c.is_poisoned());
            let count = c.registers.test_io().operations.len();
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            assert_eq!(unsafe { c.start_dma_polling() }, Err(Error::Poisoned));
            assert_eq!(block_on(c.stop_dma(true)), Err(Error::Poisoned));
            assert_eq!(c.registers.test_io().operations.len(), count);
        }
    }
}
#[test]
#[ignore = "oracle: STELA_MT792X_START_ORACLE"]
fn unchanged_pinned_prefetch_and_enable_match_except_explicit_polling_irq_policy() {
    let root = std::env::var_os("STELA_MT792X_START_ORACLE").expect("reference trace directory");
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let name = if chip == Chip::Mt7925 {
            "7925"
        } else if chip == Chip::Mt7922 {
            "7922"
        } else {
            "7921"
        };
        let bytes =
            std::fs::read(std::path::Path::new(&root).join(format!("mt{name}.bin"))).unwrap();
        let count = u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize;
        assert_eq!(bytes.len(), 4 + 12 * count);
        let mut expected: Vec<_> = bytes[4..]
            .as_chunks::<12>()
            .0
            .iter()
            .map(|b| {
                (
                    u32::from_le_bytes(b[..4].try_into().unwrap()),
                    u32::from_le_bytes(b[4..8].try_into().unwrap()),
                    u32::from_le_bytes(b[8..].try_into().unwrap()),
                )
            })
            .collect();
        assert_eq!(
            &expected[count - 3..],
            &[(2, 0, 0x20000300), (0, 0xd41f4, 0), (1, 0xd41f4, 1)]
        );
        expected.truncate(count - 3);
        let mut c = setup(chip, Model::default());
        block_on(c.driver_own(false)).unwrap();
        block_on(c.stop_dma(true)).unwrap();
        let initial = c.registers.test_io().operations.len();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        let receipt = unsafe { c.start_dma_polling() }.unwrap();
        let actual = &c.registers.test_io().operations[initial..];
        assert_eq!(
            &actual[..6],
            &[
                (1, 0xd4204, 0),
                (1, 0x10188, 0),
                (1, 0xd41f4, 0),
                (0, 0xd4204, 0),
                (0, 0x10188, 0),
                (0, 0xd41f4, 0)
            ]
        );
        assert_eq!(actual[6], (0, MT_WFDMA0_GLO_CFG, 0x5a228055 & !DISABLE));
        assert_eq!(&actual[7..actual.len() - 4], expected);
        assert_eq!(
            &actual[actual.len() - 4..],
            &[
                (0, 0xd4204, 0),
                (0, 0x10188, 0),
                (0, 0xd41f4, 0),
                (0, MT_WFDMA0_GLO_CFG, receipt.global_config)
            ]
        );
    }
}
