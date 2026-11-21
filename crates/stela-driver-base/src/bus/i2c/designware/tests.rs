use super::*;
use alloc::{collections::VecDeque, rc::Rc, vec::Vec};
use core::cell::{Cell, RefCell};
mod access;

/// A wire peer and finite hardware FIFOs, driven only when emulated time moves.
/// FIFO overflow/underflow are latched independently of the driver's accounting.
struct Model {
    words: [u32; 64],
    time: Rc<Cell<u64>>,
    tx: VecDeque<u32>,
    rx: VecDeque<u8>,
    replies: VecDeque<u8>,
    commands: Vec<u32>,
    events: Vec<Event>,
    writes: Vec<(usize, u32)>,
    tx_depth: usize,
    rx_depth: usize,
    peak_tx: usize,
    peak_rx: usize,
    enabled: bool,
    active: bool,
    direction: Option<bool>,
    raw: u32,
    abort_source: u32,
    enable_at: Option<u64>,
    disable_at: Option<u64>,
    command_at: u64,
    latency: u64,
    enable_delay: u64,
    disable_delay: u64,
    freeze_tx: bool,
    freeze_enable: bool,
    freeze_disable: bool,
    freeze_abort: bool,
    external_busy: bool,
    drop_stop: bool,
    fixed_timing: bool,
    fault_after: Option<(usize, u32)>,
    stop_after: Option<usize>,
    stop_on_enable: bool,
    aborts: usize,
    accesses: Vec<(bool, usize, u32)>,
    delay_after_access: Option<usize>,
    delay_us: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Event {
    Start { read: bool, repeated: bool },
    Write(u8),
    Read(u8),
    Stop,
}

impl Model {
    fn new(time: Rc<Cell<u64>>, tx_depth: usize, rx_depth: usize) -> Self {
        let mut words = [0; 64];
        words[COMP_TYPE / 4] = COMPONENT_TYPE;
        words[COMP_VERSION / 4] = SDA_HOLD_VERSION;
        words[COMP_PARAM_1 / 4] = ((tx_depth - 1) << 16) as u32
            | ((rx_depth - 1) << 8) as u32
            | PARAM_ENCODED
            | (2 << 2)
            | PARAM_WIDTH_32;
        Self {
            words,
            time,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            replies: VecDeque::new(),
            commands: Vec::new(),
            events: Vec::new(),
            writes: Vec::new(),
            tx_depth,
            rx_depth,
            peak_tx: 0,
            peak_rx: 0,
            enabled: false,
            active: false,
            direction: None,
            raw: 0,
            abort_source: 0,
            enable_at: None,
            disable_at: None,
            command_at: 0,
            latency: 3,
            enable_delay: 2,
            disable_delay: 2,
            freeze_tx: false,
            freeze_enable: false,
            freeze_disable: false,
            freeze_abort: false,
            external_busy: false,
            drop_stop: false,
            fixed_timing: false,
            fault_after: None,
            stop_after: None,
            stop_on_enable: false,
            aborts: 0,
            accesses: Vec::new(),
            delay_after_access: None,
            delay_us: 0,
        }
    }

    fn step(&mut self) {
        let now = self.time.get();
        if self.enable_at.is_some_and(|at| now >= at) {
            self.enabled = true;
            self.enable_at = None;
        }
        if self.disable_at.is_some_and(|at| now >= at) {
            self.enabled = false;
            self.disable_at = None;
            self.active = false;
            self.direction = None;
            self.tx.clear();
            self.rx.clear();
        }
        if !self.enabled
            || self.freeze_tx
            || self.tx.is_empty()
            || now < self.command_at
            || self.raw & INTR_TX_ABRT != 0
        {
            return;
        }
        self.command_at = now.saturating_add(self.latency);
        if let Some((after, source)) = self.fault_after
            && self.commands.len() == after
        {
            self.raw |= INTR_TX_ABRT;
            self.abort_source = source;
            self.tx.clear();
            self.rx.clear();
            self.active = false;
            return;
        }
        let command = self.tx.pop_front().unwrap();
        self.commands.push(command);
        let read = command & CMD_READ != 0;
        if self.direction.is_none() || command & CMD_RESTART != 0 {
            self.events.push(Event::Start {
                read,
                repeated: self.direction.is_some(),
            });
            self.direction = Some(read);
        } else {
            assert_eq!(
                self.direction,
                Some(read),
                "direction changed without RESTART"
            );
        }
        self.active = true;
        if read {
            let byte = self.replies.pop_front().unwrap_or(0xff);
            if self.rx.len() == self.rx_depth {
                self.raw |= INTR_RX_OVER;
            } else {
                self.rx.push_back(byte);
                self.peak_rx = self.peak_rx.max(self.rx.len());
            }
            self.events.push(Event::Read(byte));
        } else {
            self.events.push(Event::Write(command as u8));
        }
        if command & CMD_STOP != 0 || self.stop_after == Some(self.commands.len()) {
            self.active = false;
            self.direction = None;
            if !self.drop_stop {
                self.raw |= INTR_STOP_DET;
            }
            self.events.push(Event::Stop);
        }
    }

    // Advance time only after the register result/effect. This
    // models a completed slow access, independently of polling relax().
    fn complete_access(&mut self, write: bool, offset: usize, value: u32) {
        self.accesses.push((write, offset, value));
        if self.delay_after_access == Some(self.accesses.len()) {
            self.time.set(self.time.get().wrapping_add(self.delay_us));
            self.delay_after_access = None;
        }
    }

    fn read(&mut self, offset: usize) -> u32 {
        self.step();
        let value = match offset {
            ENABLE_STATUS => u32::from(self.enabled),
            STATUS => {
                (u32::from(self.active || self.external_busy) * STATUS_ACTIVE)
                    | (u32::from(self.active) * STATUS_MASTER_ACTIVE)
                    | if self.tx.is_empty() {
                        STATUS_TX_EMPTY
                    } else {
                        0
                    }
            }
            RAW_INTR_STAT => self.raw,
            TX_ABRT_SOURCE => self.abort_source,
            TXFLR => self.tx.len() as u32,
            RXFLR => self.rx.len() as u32,
            DATA_CMD => match self.rx.pop_front() {
                Some(byte) => u32::from(byte),
                None => {
                    self.raw |= INTR_RX_UNDER;
                    0
                }
            },
            CLR_INTR => {
                self.raw = 0;
                self.abort_source = 0;
                0
            }
            CLR_TX_ABRT => {
                self.raw &= !INTR_TX_ABRT;
                self.abort_source = 0;
                0
            }
            CLR_STOP_DET => {
                self.raw &= !INTR_STOP_DET;
                0
            }
            _ => self.words[offset / 4],
        };
        self.complete_access(false, offset, value);
        value
    }

    fn write(&mut self, offset: usize, value: u32) {
        self.step();
        self.writes.push((offset, value));
        match offset {
            ENABLE => {
                self.words[offset / 4] = value;
                if value & ENABLE_ABORT != 0 {
                    assert!(self.enabled, "ABORT before enable acknowledgement");
                    self.aborts += 1;
                    if !self.freeze_abort {
                        self.words[offset / 4] &= !ENABLE_ABORT;
                        self.tx.clear();
                        self.rx.clear();
                        self.active = false;
                        self.direction = None;
                        self.raw |= INTR_TX_ABRT;
                        self.abort_source = 1 << 16;
                    }
                } else if value & ENABLE_ON != 0 {
                    self.disable_at = None;
                    if self.stop_on_enable {
                        self.raw |= INTR_STOP_DET;
                    }
                    if !self.freeze_enable {
                        self.enable_at = Some(self.time.get() + self.enable_delay);
                    }
                } else {
                    self.enable_at = None;
                    if !self.freeze_disable {
                        self.disable_at = Some(self.time.get() + self.disable_delay);
                    }
                }
            }
            DATA_CMD => {
                assert!(self.enabled, "command before enable acknowledgement");
                if self.tx.len() == self.tx_depth {
                    self.raw |= INTR_TX_OVER;
                } else {
                    self.tx.push_back(value);
                    self.active = true;
                    self.peak_tx = self.peak_tx.max(self.tx.len());
                }
            }
            CON | TAR | SS_SCL_HCNT | SS_SCL_LCNT | FS_SCL_HCNT | FS_SCL_LCNT | SDA_HOLD => {
                assert!(!self.enabled, "configuration while controller enabled");
                if !self.fixed_timing || !matches!(offset, FS_SCL_HCNT | FS_SCL_LCNT) {
                    self.words[offset / 4] = value;
                }
            }
            _ => self.words[offset / 4] = value,
        }
        self.complete_access(true, offset, value);
    }
}

#[derive(Clone)]
struct RegisterModel(Rc<RefCell<Model>>);

impl Registers for RegisterModel {
    fn read32(&mut self, offset: usize) -> Result<u32, IoError> {
        Ok(self.0.borrow_mut().read(offset))
    }

    fn write32(&mut self, offset: usize, value: u32) -> Result<(), IoError> {
        self.0.borrow_mut().write(offset, value);
        Ok(())
    }
}

struct ModelTimer(Rc<Cell<u64>>);

impl Timer for ModelTimer {
    fn now_us(&self) -> u64 {
        self.0.get()
    }

    fn relax(&self) {
        self.0.set(self.0.get().wrapping_add(1));
    }
}

fn config() -> Config {
    Config {
        timing: Timing {
            speed: Speed::Fast,
            scl_high: 120,
            scl_low: 160,
            sda_hold: Some(40),
        },
        timeout_us: 20_000,
        recovery_timeout_us: 100,
    }
}

type Controller = DesignWare<RegisterModel, ModelTimer>;

fn model(tx: usize, rx: usize) -> (RegisterModel, ModelTimer) {
    let time = Rc::new(Cell::new(0));
    (
        RegisterModel(Rc::new(RefCell::new(Model::new(time.clone(), tx, rx)))),
        ModelTimer(time),
    )
}

fn controller(tx: usize, rx: usize) -> (Controller, Rc<RefCell<Model>>) {
    let (registers, timer) = model(tx, rx);
    let state = registers.0.clone();
    let controller = DesignWare::new(registers, timer, config()).unwrap();
    (controller, state)
}

#[test]
fn completed_access_deadlines_never_report_success_or_publish_later_wire_commands() {
    // First observe a real successful combined transfer, including its
    // disable/cleanup accesses. Then delay each access in turn.
    // The existing peer/FIFO model still executes every original effect.
    let (mut baseline, state) = controller(2, 2);
    state.borrow_mut().accesses.clear();
    let mut bytes = [0; 3];
    baseline
        .transaction(
            Address::SevenBit(0x2c),
            &mut [
                Operation::Write(&[0x80, 0x55, 0x33]),
                Operation::Read(&mut bytes),
            ],
        )
        .unwrap();
    let accesses = state.borrow().accesses.clone();
    assert!(
        accesses
            .iter()
            .any(|&(write, offset, _)| !write && offset == DATA_CMD)
    );
    assert!(
        accesses
            .iter()
            .any(|&(write, offset, _)| !write && offset == CLR_STOP_DET)
    );
    for delayed in 1..=accesses.len() {
        let (mut bus, state) = controller(2, 2);
        {
            let mut h = state.borrow_mut();
            h.accesses.clear();
            h.delay_after_access = Some(delayed);
            h.delay_us = config().timeout_us;
        }
        let mut data = [0xa5; 3];
        let error = bus
            .transaction(
                Address::SevenBit(0x2c),
                &mut [
                    Operation::Write(&[0x80, 0x55, 0x33]),
                    Operation::Read(&mut data),
                ],
            )
            .unwrap_err();
        assert!(
            matches!(error.kind, ErrorKind::Timeout(_)),
            "access {delayed}: {error:?}"
        );
        let h = state.borrow();
        assert_eq!(h.delay_after_access, None);
        assert_eq!(&h.accesses[..delayed], &accesses[..delayed]);
        assert!(
            h.accesses[delayed..]
                .iter()
                .all(|&(write, offset, value)| !write
                    || (offset != DATA_CMD && !(offset == ENABLE && value == ENABLE_ON))),
            "wire publication after slow access {delayed}"
        );
        if !accesses[delayed - 1].0 && accesses[delayed - 1].1 == DATA_CMD {
            let completed_before = accesses[..delayed - 1]
                .iter()
                .filter(|&&(write, offset, _)| !write && offset == DATA_CMD)
                .count();
            assert!(
                data[completed_before..].iter().all(|&byte| byte == 0xa5),
                "late RX read must not be copied into the caller's buffer"
            );
        }
        if error.recovery == Recovery::Failed {
            let writes = h.writes.len();
            drop(h);
            assert_eq!(
                bus.transaction(Address::SevenBit(0x2c), &mut [Operation::Write(&[9])])
                    .unwrap_err()
                    .kind,
                ErrorKind::NeedsReset
            );
            assert_eq!(state.borrow().writes.len(), writes);
        }
    }
}

#[test]
fn late_abort_remains_the_original_failure_through_cleanup() {
    for source in [ABRT_ADDRESS_NACK, ABRT_DATA_NACK, ABRT_ARBITRATION_LOST] {
        let (mut bus, state) = controller(2, 2);
        {
            let mut h = state.borrow_mut();
            h.raw = INTR_TX_ABRT;
            h.abort_source = source;
            // Test the common fault reader, whose clear destroys
            // the hardware source. A late observation cannot turn it into
            // an unrelated timeout or silently retry a wire operation.
            h.accesses.clear();
            h.delay_after_access = Some(2);
            h.delay_us = config().timeout_us;
        }
        let error = bus.state.faults().unwrap_err();
        assert!(matches!(error, ErrorKind::Aborted { source: actual, .. } if actual == source));
        assert_eq!(state.borrow().abort_source, 0);
        assert!(bus.state.quiesce().unwrap());
        assert!(state.borrow().commands.is_empty());
    }
}

#[test]
fn combined_messages_preserve_data_restarts_and_only_the_final_stop() {
    for (tx, rx) in [(2, 2), (3, 2), (2, 16), (16, 3), (256, 256)] {
        let (mut controller, state) = controller(tx, rx);
        state.borrow_mut().replies.extend(0..80);
        let mut read_first = [0; 63];
        let mut read_last = [0; 17];
        controller
            .transaction(
                Address::SevenBit(0x2c),
                &mut [
                    Operation::Write(&[1, 2, 3, 4, 5, 6, 7]),
                    Operation::Read(&mut read_first),
                    Operation::Write(&[9, 10]),
                    Operation::Read(&mut read_last),
                ],
            )
            .unwrap();
        assert_eq!(read_first, core::array::from_fn(|i| i as u8));
        assert_eq!(read_last, core::array::from_fn(|i| (i + 63) as u8));
        let state = state.borrow();
        assert!(state.peak_tx <= tx);
        assert!(state.peak_rx <= rx);
        assert_eq!(state.raw & INTR_FIFO_ERROR, 0);
        assert!(!state.enabled);
        assert_eq!(state.commands.len(), 89);
        let boundaries: Vec<_> = state
            .commands
            .iter()
            .enumerate()
            .filter(|(_, command)| *command & (CMD_RESTART | CMD_STOP) != 0)
            .map(|(index, command)| (index, command & (CMD_RESTART | CMD_STOP)))
            .collect();
        assert_eq!(
            boundaries,
            [
                (7, CMD_RESTART),
                (70, CMD_RESTART),
                (72, CMD_RESTART),
                (88, CMD_STOP)
            ]
        );
        assert_eq!(state.events.last(), Some(&Event::Stop));
        assert_eq!(
            state
                .events
                .iter()
                .filter(|event| **event == Event::Stop)
                .count(),
            1
        );
        assert_eq!(state.words[TAR / 4], 0x2c);
        assert_eq!(state.words[INTR_MASK / 4], 0);
        assert_eq!(state.words[SMBUS_INTR_MASK / 4], 0);
        assert_eq!(state.words[DMA_CR / 4], 0);
    }
}

#[test]
fn successive_same_direction_messages_still_generate_restarts() {
    let (mut controller, state) = controller(2, 2);
    controller
        .transaction(
            Address::TenBit(0x2ab),
            &mut [Operation::Write(&[1]), Operation::Write(&[2])],
        )
        .unwrap();
    let state = state.borrow();
    assert_eq!(state.words[TAR / 4], 0x2ab | TAR_TEN_BIT);
    assert_eq!(state.words[CON / 4] & CON_TEN_BIT, CON_TEN_BIT);
    assert_eq!(
        state.events,
        [
            Event::Start {
                read: false,
                repeated: false
            },
            Event::Write(1),
            Event::Start {
                read: false,
                repeated: true
            },
            Event::Write(2),
            Event::Stop,
        ]
    );
}

#[test]
fn abort_reasons_survive_hardware_clear_and_do_not_poison_the_next_transaction() {
    for (source, kind) in [
        (1, AbortKind::AddressNack),
        (2, AbortKind::AddressNack),
        (4, AbortKind::AddressNack),
        (8, AbortKind::DataNack),
        (1 << 12, AbortKind::ArbitrationLost),
        (1 << 11, AbortKind::Other),
    ] {
        let (mut controller, state) = controller(2, 2);
        state.borrow_mut().fault_after = Some((1, source));
        let mut data = [0; 15];
        let error = controller
            .transaction(Address::SevenBit(0x50), &mut [Operation::Read(&mut data)])
            .unwrap_err();
        assert_eq!(error.kind, ErrorKind::Aborted { kind, source });
        assert_eq!(error.recovery, Recovery::Disabled);
        {
            let mut state = state.borrow_mut();
            assert_eq!(state.abort_source, 0);
            assert_eq!(state.raw, 0);
            assert!(!state.enabled);
            assert!(state.tx.is_empty() && state.rx.is_empty());
            state.fault_after = None;
        }
        controller
            .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[55])])
            .unwrap();
        assert_eq!(state.borrow().events.last(), Some(&Event::Stop));
    }
}

#[test]
fn stalled_transfer_is_bounded_aborted_and_can_be_retried_explicitly() {
    let (mut controller, state) = controller(2, 2);
    state.borrow_mut().freeze_tx = true;
    let before = state.borrow().time.get();
    let error = controller
        .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[1; 10])])
        .unwrap_err();
    assert_eq!(error.kind, ErrorKind::Timeout(Phase::Transfer));
    assert_eq!(error.recovery, Recovery::Disabled);
    {
        let mut state = state.borrow_mut();
        assert_eq!(state.aborts, 1);
        assert!(!state.enabled);
        assert!(state.time.get() - before <= 20_100);
        state.freeze_tx = false;
    }
    controller
        .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[2])])
        .unwrap();
}

#[test]
fn an_unacknowledged_disable_poisons_the_controller() {
    let (mut controller, state) = controller(2, 2);
    state.borrow_mut().freeze_disable = true;
    let error = controller
        .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[1])])
        .unwrap_err();
    assert_eq!(error.kind, ErrorKind::Timeout(Phase::Disable));
    assert_eq!(error.recovery, Recovery::Failed);
    let writes = state.borrow().writes.len();
    let error = controller
        .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[2])])
        .unwrap_err();
    assert_eq!(error.kind, ErrorKind::NeedsReset);
    assert_eq!(state.borrow().writes.len(), writes);
}

#[test]
fn enable_bus_acquisition_and_missing_stop_all_have_finite_failure_paths() {
    for phase in [Phase::Enable, Phase::BusIdle, Phase::Stop] {
        let (mut controller, state) = controller(2, 2);
        {
            let mut state = state.borrow_mut();
            match phase {
                Phase::Enable => state.freeze_enable = true,
                Phase::BusIdle => state.external_busy = true,
                Phase::Stop => state.drop_stop = true,
                _ => unreachable!(),
            }
        }
        let error = controller
            .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[1])])
            .unwrap_err();
        assert_eq!(error.kind, ErrorKind::Timeout(phase));
        assert_eq!(error.recovery, Recovery::Disabled);
        assert!(state.borrow().time.get() <= 20_200);
    }
}

#[test]
fn a_stop_before_any_command_is_queued_is_ignored() {
    let (mut controller, state) = controller(16, 16);
    state.borrow_mut().stop_on_enable = true;
    let mut bytes = [0; 4];
    controller
        .transaction(
            Address::SevenBit(0x2c),
            &mut [Operation::Write(&[1, 0]), Operation::Read(&mut bytes)],
        )
        .unwrap();
}

#[test]
fn a_stop_before_all_commands_are_queued_is_an_error() {
    let (mut controller, state) = controller(2, 2);
    state.borrow_mut().stop_after = Some(1);
    let error = controller
        .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[1; 10])])
        .unwrap_err();
    assert_eq!(error.kind, ErrorKind::UnexpectedStop);
    assert_eq!(error.recovery, Recovery::Disabled);
}

#[test]
fn a_premature_stop_is_rejected_even_when_the_entire_write_fits_in_the_fifo() {
    let (mut controller, state) = controller(16, 16);
    state.borrow_mut().stop_after = Some(1);
    let error = controller
        .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[1, 2, 3])])
        .unwrap_err();
    assert_eq!(error.kind, ErrorKind::UnexpectedStop);
    assert_eq!(error.recovery, Recovery::Disabled);
    let state = state.borrow();
    assert!(!state.enabled);
    assert!(state.tx.is_empty());
    assert_eq!(
        state
            .events
            .iter()
            .filter(|event| **event == Event::Stop)
            .count(),
        1
    );
}

#[test]
fn invalid_addresses_and_empty_messages_do_not_touch_registers() {
    let (mut controller, state) = controller(2, 2);
    let writes = state.borrow().writes.len();
    for address in [
        Address::SevenBit(0),
        Address::SevenBit(0x80),
        Address::TenBit(0x400),
    ] {
        let error = controller
            .transaction(address, &mut [Operation::Write(&[1])])
            .unwrap_err();
        assert_eq!(error.kind, ErrorKind::InvalidAddress);
    }
    assert_eq!(
        controller
            .transaction(Address::SevenBit(0x50), &mut [])
            .unwrap_err()
            .kind,
        ErrorKind::EmptyTransaction
    );
    assert_eq!(
        controller
            .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[])])
            .unwrap_err()
            .kind,
        ErrorKind::EmptyMessage
    );
    assert_eq!(state.borrow().writes.len(), writes);
}

#[test]
fn component_timing_and_fifo_requirements_fail_explicitly() {
    for scenario in 0..6 {
        let (registers, timer) = model(2, 2);
        let mut config = config();
        {
            let mut state = registers.0.borrow_mut();
            match scenario {
                0 => state.words[COMP_TYPE / 4] = 0x0140_4457,
                1 => state.words[COMP_PARAM_1 / 4] &= !PARAM_ENCODED,
                2 => state.words[COMP_PARAM_1 / 4] &= !3,
                3 => state.words[COMP_VERSION / 4] = SDA_HOLD_VERSION - 1,
                4 => state.fixed_timing = true,
                5 => config.timing.scl_high = 0,
                _ => unreachable!(),
            }
        }
        let error = DesignWare::new(registers, timer, config).err().unwrap();
        assert!(matches!(
            (scenario, error.error.kind),
            (0, ErrorKind::ComponentType(_))
                | (1 | 2, ErrorKind::ComponentParameters(_))
                | (3, ErrorKind::UnsupportedVersion(_))
                | (4, ErrorKind::TimingRejected)
                | (5, ErrorKind::InvalidConfiguration)
        ));
    }
}

#[test]
fn seeded_mixed_transactions_match_the_wire_peer_across_fifo_sizes_and_delays() {
    let mut seed = 0x51c2_1907u32;
    let mut random = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed
    };
    for _ in 0..256 {
        let tx = [2, 3, 8, 32][random() as usize % 4];
        let rx = [2, 3, 16, 256][random() as usize % 4];
        let (mut controller, state) = controller(tx, rx);
        state.borrow_mut().latency = u64::from(random() % 6);
        let messages = random() as usize % 8 + 1;
        let mut buffers: Vec<_> = (0..messages)
            .map(|_| {
                let read = random() & 1 != 0;
                let count = random() as usize % 79 + 1;
                (read, (0..count).map(|_| random() as u8).collect::<Vec<_>>())
            })
            .collect();
        let mut expected_events = Vec::new();
        let mut next_reply = 0u8;
        for (message, (read, bytes)) in buffers.iter().enumerate() {
            expected_events.push(Event::Start {
                read: *read,
                repeated: message != 0,
            });
            for &byte in bytes {
                if *read {
                    expected_events.push(Event::Read(next_reply));
                    state.borrow_mut().replies.push_back(next_reply);
                    next_reply = next_reply.wrapping_add(1);
                } else {
                    expected_events.push(Event::Write(byte));
                }
            }
        }
        expected_events.push(Event::Stop);
        let mut operations: Vec<_> = buffers
            .iter_mut()
            .map(|(read, bytes)| {
                if *read {
                    Operation::Read(bytes)
                } else {
                    Operation::Write(bytes)
                }
            })
            .collect();
        controller
            .transaction(Address::SevenBit(0x2c), &mut operations)
            .unwrap();
        drop(operations);
        let mut next_reply = 0u8;
        for (read, bytes) in buffers {
            if read {
                for byte in bytes {
                    assert_eq!(byte, next_reply);
                    next_reply = next_reply.wrapping_add(1);
                }
            }
        }
        let state = state.borrow();
        assert_eq!(state.events, expected_events);
        assert!(state.peak_tx <= tx && state.peak_rx <= rx);
        assert!(state.raw & INTR_FIFO_ERROR == 0);
    }
}

#[test]
fn wedged_controller_requires_platform_reset() {
    let (mut controller, state) = controller(2, 2);
    {
        let mut state = state.borrow_mut();
        state.freeze_tx = true;
        state.freeze_abort = true;
        state.freeze_disable = true;
    }
    let error = controller
        .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[1; 10])])
        .unwrap_err();
    assert_eq!(error.kind, ErrorKind::Timeout(Phase::Transfer));
    assert_eq!(error.recovery, Recovery::Failed);
    assert_eq!(state.borrow().aborts, 1);
    assert!(state.borrow().time.get() <= 20_100);
    assert_eq!(
        controller
            .transaction(Address::SevenBit(0x50), &mut [Operation::Write(&[2])])
            .unwrap_err()
            .kind,
        ErrorKind::NeedsReset
    );
}
