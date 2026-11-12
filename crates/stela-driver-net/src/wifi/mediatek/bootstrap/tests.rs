extern crate std;

use crate::wifi::mediatek::mailbox::clc::{remaining::tests::*, tests::*};
use crate::wifi::mediatek::{
    firmware::{
        Requests,
        tests::{catalog, patch_bytes},
    },
    identity::Chip,
    mailbox::tests::*,
    ring::{
        self, QueueIo,
        rx::tests::device_receive,
        tests::{Domain, Io, Memory, Source, State},
    },
};
use crate::wifi::{
    Band,
    mediatek::{
        bootstrap::{Controller, Error as BootError, Phase},
        control::Control,
        mcu::{domain, power},
        registers::{self, BarIo, Registers},
    },
};
use alloc::collections::BTreeMap;
use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

#[derive(Default)]
pub(crate) struct DeviceBar {
    pub(crate) values: BTreeMap<u32, u32>,
    pub(crate) log: Vec<(u32, u32, u32)>,
    pub(crate) fail_at: Option<usize>,
    pub(crate) busy_wtbl: bool,
    pub(crate) ignore_global: bool,
    pub(crate) wfsys_cleared: bool,
    pub(crate) wfsys_no_ack: bool,
    pub(crate) fw: Option<Arc<Mutex<Response>>>,
}
#[derive(Clone)]
pub(crate) struct Bar(pub(crate) Arc<Mutex<DeviceBar>>);
// SAFETY: the model answers only for this test's simulated device; nothing else reaches its
// state.
unsafe impl BarIo for Bar {
    fn read32(&mut self, offset: u32) -> Result<u32, registers::Error> {
        let mut b = self.0.lock().unwrap();
        if offset == 0xe00f0
            && b.fw
                .as_ref()
                .is_some_and(|r| r.lock().unwrap().firmware_started)
        {
            b.values.insert(offset, 3);
        }
        let mut value = b.values.get(&offset).copied().unwrap_or(0);
        if b.busy_wtbl && matches!(offset, 0x34230 | 0x34380) {
            value |= 1 << 31;
        }
        b.log.push((0, offset, value));
        if b.fail_at == Some(b.log.len()) {
            return Err(registers::Error::AccessFailed);
        }
        Ok(value)
    }
    fn write32(&mut self, offset: u32, value: u32) -> Result<(), registers::Error> {
        let mut b = self.0.lock().unwrap();
        b.log.push((1, offset, value));
        if b.fail_at == Some(b.log.len()) {
            return Err(registers::Error::AccessFailed);
        }
        if b.ignore_global && matches!(offset, 0xf000 | 0xe800) {
            return Ok(());
        }
        // Device ownership handshake, not an echo of the host request.
        if offset == 0xe0010 {
            b.values.insert(offset, if value == 1 { 4 } else { 0 });
        } else if (0x40000..0x50000).contains(&offset)
            && b.values.get(&0xfe24c).copied().unwrap_or(0) & 0xffff == 0x1806
        {
            b.values.insert(offset, 0);
        } else if offset == 0xf0140 {
            if value & 1 == 0 {
                b.wfsys_cleared = true;
                b.values.insert(offset, value & !16);
            } else {
                let ack = b.wfsys_cleared && !b.wfsys_no_ack;
                b.values
                    .insert(offset, if ack { value | 16 } else { value & !16 });
            }
        } else {
            b.values.insert(offset, value);
        }
        Ok(())
    }
}
pub(crate) struct Q {
    pub(crate) inner: Inner,
    pub(crate) bar: Bar,
    pub(crate) offset: u32,
    pub(crate) runtime: ModelState,
    pub(crate) response: Arc<Mutex<Response>>,
}
pub(crate) enum Inner {
    Tx(DeviceIo),
    Rx(Io),
}
// SAFETY: the model answers only for this test's simulated device; nothing else reaches its
// state.
unsafe impl QueueIo for Q {
    fn configure(&mut self, address: u32, count: u16) -> Result<(), ring::Error> {
        {
            let mut b = self.bar.0.lock().unwrap();
            b.log.push((2, self.offset, address));
            b.values.insert(self.offset, address);
            b.values.insert(self.offset + 4, u32::from(count));
        }
        match &mut self.inner {
            Inner::Tx(q) => q.configure(address, count),
            Inner::Rx(q) => q.configure(address, count),
        }
    }
    fn device_index(&mut self) -> Result<u16, ring::Error> {
        match &mut self.inner {
            Inner::Tx(q) => q.device_index(),
            Inner::Rx(q) => q.device_index(),
        }
    }
    fn publish(&mut self, head: u16) -> Result<(), ring::Error> {
        self.bar
            .0
            .lock()
            .unwrap()
            .values
            .insert(self.offset + 8, u32::from(head));
        match &mut self.inner {
            Inner::Tx(q) => {
                if self.response.lock().unwrap().firmware_started && q.chip != Chip::Mt7925 {
                    q.rx = self.runtime.clone();
                }
                q.publish(head)
            }
            Inner::Rx(q) => q.publish(head),
        }
    }
}
pub(crate) struct NativeGuard {
    // Force every retained-session fixture through genuinely CPU-local custody.
    _local: core::marker::PhantomData<alloc::rc::Rc<()>>,
    pub(crate) model: Arc<Mutex<Response>>,
    pub(crate) bar: Bar,
    pub(crate) drops: Arc<AtomicUsize>,
}
impl Drop for NativeGuard {
    fn drop(&mut self) {
        let mut r = self.model.lock().unwrap();
        assert!(!r.clc_processing);
        assert_eq!(
            self.bar
                .0
                .lock()
                .unwrap()
                .values
                .get(&0xd4208)
                .copied()
                .unwrap_or(0)
                & 5,
            0
        );
        r.clc_guard = false;
        self.drops.fetch_add(1, Ordering::Relaxed);
    }
}
pub(crate) type Native = Controller<Bar, Q, NativeGuard>;
pub(crate) struct NativeFixture {
    pub(crate) controller: Native,
    pub(crate) model: Arc<Mutex<Response>>,
    pub(crate) bar: Bar,
    pub(crate) queues: BTreeMap<u32, ModelState>,
    pub(crate) drops: Arc<AtomicUsize>,
    pub(crate) memory: Arc<Mutex<Memory>>,
    pub(crate) chip: Chip,
}
impl NativeFixture {
    pub(crate) fn new(chip: Chip) -> Self {
        let model = Arc::new(Mutex::new(Response {
            capability_payload: Some(caps(chip, [true, true, false])),
            encapsulation_payload: Some(eeprom_body(chip, 0)),
            clc_payload: Some(clc_response(chip)),
            initial_execution: true,
            device_execution: true,
            require_firmware_start: true,
            clc_guard: true,
            ..Response::default()
        }));
        let bar = Bar(Arc::new(Mutex::new(DeviceBar {
            values: BTreeMap::from([(0xe00f0, 1)]),
            fw: Some(model.clone()),
            ..DeviceBar::default()
        })));
        let clock = model_clock();
        let control =
                // SAFETY: the model device, its pages and its domain belong to this test alone; no
                // other driver, worker or DMA touches them.
                unsafe { Control::new(Registers::new(chip, bar.clone()), clock) }.unwrap();
        let drops = Arc::new(AtomicUsize::new(0));
        #[expect(
            clippy::arc_with_non_send_sync,
            reason = "private local-custody model exercises the real owner retention paths"
        )]
        let guard = Arc::new(NativeGuard {
            _local: core::marker::PhantomData,
            model: model.clone(),
            bar: bar.clone(),
            drops: drops.clone(),
        });
        let controller = Controller::claim(identity(chip), control, guard, None).unwrap();
        let memory = Arc::new(Mutex::new(Memory::default()));
        let queues = [
            0xd4300, 0xd43f0, 0xd4410, 0xd4400, 0xd4500, 0xd4540, 0xd4520,
        ]
        .into_iter()
        .map(|o| {
            (
                o,
                Arc::new(Mutex::new(State {
                    memory: memory.clone(),
                    ..State::default()
                })),
            )
        })
        .collect();
        Self {
            controller,
            model,
            bar,
            queues,
            drops,
            memory,
            chip,
        }
    }
    pub(crate) fn install(&mut self) -> Result<(), BootError> {
        let domain: Arc<dyn crate::bus::dma::mapping::DmaDomain> =
            Arc::new(Domain(self.memory.clone()));
        let bar = self.bar.clone();
        let response = self.model.clone();
        let states = self.queues.clone();
        let chip = self.chip;
        let is_c3 = chip == Chip::Mt7925;
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe {
            self.controller
                .install_with(&Source, domain, false, move |offset| {
                    let state = states.get(&offset).unwrap().clone();
                    let inner =
                        if offset == 0xd4400 || offset == if is_c3 { 0xd43f0 } else { 0xd4410 } {
                            Inner::Tx(DeviceIo {
                                io: Io(state),
                                rx: states[&0xd4500].clone(),
                                chip,
                                reply: true,
                                wrong_seq: false,
                                delay: 3,
                                polls: 0,
                                pending_index: None,
                                fwdl: offset == 0xd4400,
                                response: response.clone(),
                            })
                        } else {
                            Inner::Rx(Io(state))
                        };
                    Ok(Q {
                        inner,
                        bar: bar.clone(),
                        offset,
                        runtime: states[&0xd4540].clone(),
                        response: response.clone(),
                    })
                })
        }
    }
    pub(crate) fn boot(&mut self, chip: Chip) {
        let requests = Requests::new(&identity(chip));
        let provider = catalog(&requests, patch_bytes(), ram(&clc_bytes(chip)));
        let prepared = requests.load(&provider).unwrap();
        self.controller.boot(&prepared).unwrap();
    }
    pub(crate) fn policy(&mut self, chip: Chip) {
        self.policy_bands(chip, [true, true, false]);
    }
    pub(crate) fn policy_bands(&mut self, chip: Chip, enabled: [bool; 3]) {
        let chan: [Vec<domain::Channel>; 3] = core::array::from_fn(|i| {
            if enabled[i] {
                domain_channels([Band::Ghz2, Band::Ghz5, Band::Ghz6][i])
            } else {
                vec![]
            }
        });
        self.policy_channels(chip, enabled, chan);
    }
    pub(crate) fn policy_channels(
        &mut self,
        chip: Chip,
        enabled: [bool; 3],
        chan: [Vec<domain::Channel>; 3],
    ) {
        self.policy_channel_power(chip, enabled, chan, |_, _| power::Limits::uniform(-3));
    }
    pub(crate) fn policy_channel_power(
        &mut self,
        chip: Chip,
        enabled: [bool; 3],
        chan: [Vec<domain::Channel>; 3],
        mut limits: impl FnMut(Band, u8) -> power::Limits,
    ) {
        let flags = self
            .controller
            .capabilities()
            .unwrap()
            .observed()
            .chip_flags();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { self.controller.execute_clc(inputs(flags)) }.unwrap();
        let bands: [Vec<power::Channel>; 3] = core::array::from_fn(|i| {
            if enabled[i] {
                let band = [Band::Ghz2, Band::Ghz5, Band::Ghz6][i];
                let mut rates = rates(chip, band);
                for c in &mut rates {
                    c.limits = limits(band, c.hardware_value);
                }
                rates
            } else {
                vec![]
            }
        });
        {
            let mut model = self.model.lock().unwrap();
            model.regulatory_expected_domain = Some(
                domain::Request::build(
                    chip,
                    domain::Inputs {
                        alpha2: *b"US",
                        channels_2ghz: &chan[0],
                        channels_5ghz: &chan[1],
                        channels_6ghz: &chan[2],
                    },
                )
                .unwrap()
                .body()
                .to_vec(),
            );
            for (b, cs) in bands.iter().enumerate() {
                for c in cs {
                    model.regulatory_expected_power.insert(
                        (b as u8 + 1, c.hardware_value),
                        c.limits.sku(chip, [Band::Ghz2, Band::Ghz5, Band::Ghz6][b]),
                    );
                }
            }
        }
        let channel_configuration = self.controller.policy().unwrap().channel_configuration();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe {
            self.controller.execute_remaining(
                crate::wifi::mediatek::mailbox::clc::remaining::Inputs {
                    channel_configuration,
                    domain: domain::Inputs {
                        alpha2: *b"US",
                        channels_2ghz: &chan[0],
                        channels_5ghz: &chan[1],
                        channels_6ghz: &chan[2],
                    },
                    power: power::Inputs {
                        alpha2: *b"US",
                        ghz2: enabled[0].then_some(&bands[0]),
                        ghz5: enabled[1].then_some(&bands[1]),
                        ghz6: enabled[2].then_some(&bands[2]),
                    },
                },
            )
        }
        .unwrap();
    }
    pub(crate) fn retire(&mut self) {
        self.model.lock().unwrap().clc_processing = false;
        self.bar.0.lock().unwrap().values.insert(0xd4208, 0);
        // SAFETY: the model device is stopped and drained, so nothing still uses what this
        // releases.
        unsafe { self.controller.retire_after_quiesce() }.unwrap();
    }
}

#[test]
fn initial_mt7925_wfsys_ack_precedes_dma_stop_and_every_queue_publication() {
    if crate::test_process::isolate_model() {
        return;
    }
    let mut f = NativeFixture::new(Chip::Mt7925);
    f.bar.0.lock().unwrap().values.insert(0xf0140, 0xa5a50011);
    f.install().unwrap();
    {
        let b = f.bar.0.lock().unwrap();
        let own = b.log.iter().position(|v| *v == (0, 0xe0010, 0)).unwrap();
        let clear = b
            .log
            .iter()
            .position(|v| *v == (1, 0xf0140, 0xa5a50010))
            .unwrap();
        let set = b
            .log
            .iter()
            .position(|v| *v == (1, 0xf0140, 0xa5a50001))
            .unwrap();
        let ack = b
            .log
            .iter()
            .enumerate()
            .skip(set + 1)
            .find(|(_, v)| **v == (0, 0xf0140, 0xa5a50011))
            .unwrap()
            .0;
        let dma = b
            .log
            .iter()
            .position(|v| v.0 == 1 && v.1 == 0xd4208)
            .unwrap();
        let queue = b.log.iter().position(|v| v.0 == 2).unwrap();
        assert!(own < clear && clear < set && set < ack && ack < dma && dma < queue);
        assert_eq!(
            b.log.iter().filter(|v| v.0 == 1 && v.1 == 0xf0140).count(),
            2
        );
    }
    assert!(matches!(f.install(), Err(BootError::WrongPhase)));
    f.retire();
}

#[test]
fn initial_wfsys_failure_keeps_controller_before_any_dma_publication_or_retry() {
    if crate::test_process::isolate_model() {
        return;
    }
    for fail in [None, Some(3), Some(4), Some(5), Some(6), Some(7)] {
        let mut f = NativeFixture::new(Chip::Mt7925);
        {
            let mut b = f.bar.0.lock().unwrap();
            b.wfsys_no_ack = true;
            b.fail_at = fail;
        }
        assert!(matches!(f.install(), Err(BootError::Control(_))));
        assert_eq!(f.controller.phase(), Phase::Quarantined);
        assert!(f.memory.lock().unwrap().mappings.is_empty());
        assert!(f.queues.values().all(|q| q.lock().unwrap().count == 0));
        let count = {
            let b = f.bar.0.lock().unwrap();
            assert!(!b.log.iter().any(|v| v.0 == 2 || v.1 == 0xd4208));
            b.log.len()
        };
        assert!(matches!(f.install(), Err(BootError::WrongPhase)));
        assert_eq!(f.bar.0.lock().unwrap().log.len(), count);
        let drops = f.drops.clone();
        drop(f);
        assert_eq!(drops.load(Ordering::Relaxed), 0);
    }
}

#[test]
fn complete_source_bootstrap_owns_all_queues_and_real_runtime_transition() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let mut f = NativeFixture::new(chip);
        f.install().unwrap();
        assert_eq!(f.controller.phase(), Phase::Queues);
        for (offset, count) in [
            (0xd4300, 2048),
            (
                if chip == Chip::Mt7925 {
                    0xd43f0
                } else {
                    0xd4410
                },
                256,
            ),
            (0xd4400, 128),
            (0xd4500, if chip == Chip::Mt7925 { 512 } else { 8 }),
            (0xd4520, 1536),
        ] {
            assert_eq!(f.queues[&offset].lock().unwrap().count, count);
        }
        assert_eq!(
            f.queues[&0xd4540].lock().unwrap().count,
            if chip == Chip::Mt7925 { 0 } else { 512 }
        );
        {
            let memory = f.memory.lock().unwrap();
            let mut end = 0;
            for (&a, (pins, _, _)) in &memory.mappings {
                assert!(a >= end);
                end = a + pins.bytes() as u64;
            }
        }
        f.boot(chip);
        assert_eq!(f.controller.phase(), Phase::Firmware);
        assert_eq!(f.controller.facts().len(), 1);
        assert!(matches!(
            f.controller.facts()[0],
            crate::wifi::mediatek::mailbox::init::Fact::OneWayDmaCompleted {
                kind: crate::wifi::mediatek::mcu::init::Kind::Restart,
                sequence: 1
            }
        ));
        assert!(
            f.controller
                .mailbox()
                .unwrap()
                .download_binding()
                .unwrap()
                .outcome()
                .firmware_start_acknowledged
        );
        if chip != Chip::Mt7925 {
            assert!(f.queues[&0xd4540].lock().unwrap().index > 0);
        }
        f.policy(chip);
        assert_eq!(f.controller.phase(), Phase::Policy);
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { f.controller.initialize_mac() }.unwrap();
        let mac = f.controller.mac_observations().unwrap();
        assert_eq!(mac.wtbl.len(), 20);
        assert_eq!(mac.global_format.value & (1 << 19), 0);
        assert_eq!(
            mac.fixed_rate_entries,
            if chip == Chip::Mt7925 { 12 } else { 0 }
        );
        let n = f.model.lock().unwrap().clc_commands.len();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { f.controller.start(Band::Ghz5, 36, 5180) }.unwrap();
        assert_eq!(f.controller.phase(), Phase::StartFacts);
        assert_eq!(f.model.lock().unwrap().clc_commands.len(), n);
        assert_eq!(
            f.controller.policy().unwrap().start_facts().len(),
            if chip == Chip::Mt7925 {
                1
            } else {
                1 + f.controller.policy().unwrap().remaining_facts().len() - 1
            }
        );
        let wm = if chip == Chip::Mt7925 {
            0xd43f0
        } else {
            0xd4410
        };
        let published = f.queues[&wm].lock().unwrap().completed.clone();
        let cmd = |p: &Vec<u8>| {
            if p[43] & 2 != 0 {
                (p[34], 0xff)
            } else {
                (p[36], p[41])
            }
        };
        let domain = published
            .iter()
            .rposition(|p| {
                cmd(p)
                    == if chip == Chip::Mt7925 {
                        (0x15, 0xff)
                    } else {
                        (0x0f, 0)
                    }
            })
            .unwrap();
        if chip == Chip::Mt7925 {
            let rts = published.iter().rposition(|p| cmd(p) == (8, 0xff)).unwrap();
            assert!(domain < rts);
        } else {
            let mac = published
                .iter()
                .position(|p| cmd(p) == (0xed, 0x46))
                .unwrap();
            let path = published
                .iter()
                .position(|p| cmd(p) == (0xed, 0x4e))
                .unwrap();
            let first_power = published
                .iter()
                .enumerate()
                .find(|(i, p)| *i > domain && cmd(p) == (0x5d, 0))
                .unwrap()
                .0;
            let leds = published
                .iter()
                .enumerate()
                .filter(|(_, p)| cmd(p) == (0xed, 5))
                .map(|(i, _)| i)
                .collect::<Vec<_>>();
            assert!(
                mac < domain
                    && domain < path
                    && path < first_power
                    && first_power < leds[0]
                    && leds[0] < leds[1]
            );
        }
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        assert!(unsafe { f.controller.start(Band::Ghz5, 36, 5180) }.is_err());
        f.retire();
        let drops = f.drops.clone();
        drop(f);
        assert_eq!(drops.load(Ordering::Relaxed), 1);
    }
}
#[test]
fn publication_failures_keep_guard_and_context() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7925] {
        for fail in [0, 1] {
            let mut f = NativeFixture::new(chip);
            f.install().unwrap();
            f.boot(chip);
            f.policy(chip);
            if fail == 0 {
                f.bar.0.lock().unwrap().busy_wtbl = true;
            } else {
                let mut m = f.model.lock().unwrap();
                m.initial_fail_at = Some(m.initial_commands.len() + 1);
            }
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            assert!(unsafe { f.controller.initialize_mac() }.is_err());
            assert_eq!(f.controller.phase(), Phase::Quarantined);
            let drops = f.drops.clone();
            drop(f);
            assert_eq!(drops.load(Ordering::Relaxed), 0);
        }
    }
}
#[test]
fn rings_another_owner_left_counted_are_emptied_before_dma_starts() {
    if crate::test_process::isolate_model() {
        return;
    }
    let mut f = NativeFixture::new(Chip::Mt7925);
    // RX ring 1 and TX ring 2: outside MT7925's layout, left by another OS.
    {
        let mut b = f.bar.0.lock().unwrap();
        for (register, value) in [(0xd4510, 0x1234_5000), (0xd4514, 17), (0xd4518, 3)] {
            b.values.insert(register, value);
        }
        b.values.insert(0xd4324, 64);
    }
    f.install().unwrap();
    assert_eq!(f.controller.cleared_rings(), 2);
    {
        let b = f.bar.0.lock().unwrap();
        for register in [0xd4510, 0xd4514, 0xd4518, 0xd4324] {
            assert_eq!(b.values[&register], 0, "{register:x}");
        }
        // Emptied before the engine was enabled.
        let cleared = b.log.iter().position(|v| *v == (1, 0xd4514, 0)).unwrap();
        let enabled = b
            .log
            .iter()
            .position(|v| v.0 == 1 && v.1 == 0xd4208 && v.2 & 5 == 5)
            .unwrap();
        assert!(cleared < enabled);
    }
    f.retire();
}

#[test]
fn global_and_source_per_wcid_controls_match_receive_format() {
    if crate::test_process::isolate_model() {
        return;
    }
    use crate::wifi::mediatek::{mcu::device::Context, rx};
    for chip in [Chip::Mt7921, Chip::Mt7925] {
        let mut f = NativeFixture::new(chip);
        f.install().unwrap();
        f.boot(chip);
        f.policy(chip);
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { f.controller.initialize_mac() }.unwrap();
        let context = Context {
            bss_index: 0,
            omac_index: 0,
            band_index: if chip == Chip::Mt7925 { 0xff } else { 0 },
            wmm_index: 0,
            link_index: 0,
            broadcast_wcid: 19,
            address: crate::Mac([2, 1, 2, 3, 4, 5]),
        };
        // Explicit independent peer-lifetime boundary: a checked model
        // owner reserves an already established WCID1. This bootstrap does
        // not implement or claim that missing association/STA installation.
        f.model.lock().unwrap().qualified_peer_slots.insert(1);
        if chip == Chip::Mt7925 {
            f.model.lock().unwrap().status = 0xdeadbeef;
        }
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        let interface = unsafe { f.controller.register_interface(context) }.unwrap();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        let disabled = unsafe { f.controller.disable_peer_translation(&interface, 1, true) };
        if chip == Chip::Mt7925 {
            assert!(matches!(
                disabled,
                Err(BootError::UnvalidatedHeaderFormat { wcid: 1 })
            ));
        } else {
            disabled.unwrap();
        }
        assert!(f.model.lock().unwrap().raw_disabled.contains(&1));
        if chip == Chip::Mt7925 {
            assert!(
                matches!(f.controller.facts().last(),Some(crate::wifi::mediatek::mailbox::init::Fact::GenericResponse(r)) if r.frame.bytes.ends_with(&0xdeadbeefu32.to_le_bytes()))
            );
        }

        let global = f.controller.mac_observations().unwrap().global_format.value;
        let base = if chip == Chip::Mt7925 { 32 } else { 24 };
        for wcid in [1u16, 2] {
            // Device chooses descriptor format from both independently
            // applied controls. An unconfigured peer stays explicit foreign.
            let translated =
                global & (1 << 19) != 0 || !f.model.lock().unwrap().raw_disabled.contains(&wcid);
            let mut bytes = vec![0; base + 24];
            let len = bytes.len() as u32;
            bytes[..4].copy_from_slice(&(len | (2 << 27)).to_le_bytes());
            bytes[4..8].copy_from_slice(&u32::from(wcid).to_le_bytes());
            if translated {
                bytes[8..12].copy_from_slice(
                    &(1u32 << if chip == Chip::Mt7925 { 7 } else { 13 }).to_le_bytes(),
                );
            }
            bytes[12..16].copy_from_slice(&(36u32 << 8).to_le_bytes());
            bytes[base] = 8;
            device_receive(&f.queues[&0xd4520], &bytes, true, 0);
            let frame = f.controller.poll_data().unwrap().unwrap();
            if translated {
                assert!(matches!(
                    rx::decode(chip, &frame.bytes),
                    Err(rx::Error::UnsupportedTranslation)
                ));
            } else {
                let decoded = rx::decode(chip, &frame.bytes).unwrap();
                assert_eq!(decoded.wcid, 1);
                assert_eq!(decoded.payload, &bytes[base..]);
            }
        }
        // An interface deactivation/drain is required to free it. This
        // test drops a live resource: its real guard is held
        // even after the separately checked controller/radio teardown.
        drop(interface);
        f.retire();
        let drops = f.drops.clone();
        drop(f);
        assert_eq!(drops.load(Ordering::Relaxed), 0);
    }
}
#[test]
#[ignore = "oracle: STELA_MT792X_BOOTSTRAP_ORACLE"]
fn composed_bootstrap_matches_source_mac_effects_and_all_initial_packet_bytes() {
    if crate::test_process::isolate_model() {
        return;
    }
    let root = std::path::PathBuf::from(std::env::var("STELA_MT792X_BOOTSTRAP_ORACLE").unwrap());
    for (chip, id) in [
        (Chip::Mt7921, 7921),
        (Chip::Mt7922, 7922),
        (Chip::Mt7925, 7925),
    ] {
        for seed in 0u32..=16 {
            let trace = std::fs::read(root.join(if seed == 0 {
                alloc::format!("{id}-mac.trace")
            } else {
                alloc::format!("{id}-mac-{seed}.trace")
            }))
            .unwrap();
            let n = u32::from_le_bytes(trace[..4].try_into().unwrap()) as usize;
            assert_eq!(trace.len(), 4 + n * 12);
            let mut source = trace[4..]
                .as_chunks::<12>()
                .0
                .iter()
                .map(|r| {
                    (
                        u32::from_le_bytes(r[..4].try_into().unwrap()),
                        u32::from_le_bytes(r[4..8].try_into().unwrap()),
                        u32::from_le_bytes(r[8..].try_into().unwrap()),
                    )
                })
                .filter(|r| r.0 != 2)
                .collect::<Vec<_>>();
            let initial = |offset: u32| {
                if seed == 0 {
                    0
                } else {
                    offset.wrapping_mul(0x10203) ^ seed.wrapping_mul(0x9e3779b9)
                }
            };
            let mut f = NativeFixture::new(chip);
            f.install().unwrap();
            f.boot(chip);
            f.policy(chip);
            {
                let mut bar = f.bar.0.lock().unwrap();
                for (_, offset, _) in &source {
                    let mut value = initial(*offset);
                    if matches!(*offset, 0x34230 | 0x34380) {
                        value &= !(1 << 31);
                    }
                    bar.values.insert(*offset, value);
                }
                bar.log.clear();
            }
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            unsafe { f.controller.initialize_mac() }.unwrap();
            let mut native = f.bar.0.lock().unwrap().log.clone();
            let global = if chip == Chip::Mt7925 { 0xe800 } else { 0xf000 };
            let with_deagg = initial(global) | (1 << 15);
            assert_eq!(native.pop(), Some((0, global, with_deagg & !(1 << 19))));
            if chip == Chip::Mt7925 {
                assert_eq!(
                    native.drain(4..6).collect::<Vec<_>>(),
                    vec![
                        (0, global, with_deagg),
                        (1, global, with_deagg & !(1 << 19))
                    ]
                );
            }
            let counters = source.split_off(source.len() - 15);
            if chip != Chip::Mt7925 {
                assert_eq!(source[5], (1, global, with_deagg | (1 << 19)));
                source[5].2 &= !(1 << 19);
            }
            assert_eq!(native, source, "complete source MAC {chip:?} seed{seed}");
            f.bar.0.lock().unwrap().log.clear();
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            unsafe { f.controller.start(Band::Ghz5, 36, 5180) }.unwrap();
            assert_eq!(f.bar.0.lock().unwrap().log, counters);
            let trace = std::fs::read(root.join(alloc::format!("{id}-commands.trace"))).unwrap();
            let mut at = 0;
            let mut source = Vec::<Vec<u8>>::new();
            while at < trace.len() {
                let n = u16::from_le_bytes(trace[at + 5..at + 7].try_into().unwrap()) as usize;
                at += 7 + n;
                let n = u16::from_le_bytes(trace[at..at + 2].try_into().unwrap()) as usize;
                at += 2;
                source.push(trace[at..at + n].to_vec());
                at += n;
            }
            for actual in &f.model.lock().unwrap().initial_commands {
                assert!(
                    source.iter().any(|expected| {
                        let mut expected = expected.clone();
                        expected[39] = actual[39];
                        expected == *actual
                    }),
                    "actual source packet {chip:?} {:02x?}",
                    actual
                );
            }
            f.retire();
        }
    }
}
#[test]
#[ignore = "oracle: STELA_MT792X_FIRMWARE"]
fn all_images_boot_the_full_native_layout_before_source_initialization() {
    if crate::test_process::isolate_model() {
        return;
    }
    let images = std::path::PathBuf::from(std::env::var("STELA_MT792X_FIRMWARE").unwrap());
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let mut f = NativeFixture::new(chip);
        f.install().unwrap();
        let requests = Requests::new(&identity(chip));
        let provider = catalog(
            &requests,
            std::fs::read(images.join(chip.patch_name())).unwrap(),
            std::fs::read(images.join(chip.ram_name())).unwrap(),
        );
        let prepared = requests.load(&provider).unwrap();
        f.controller.boot(&prepared).unwrap();
        drop(prepared);
        drop(provider);
        assert!(
            f.controller
                .mailbox()
                .unwrap()
                .download_binding()
                .unwrap()
                .outcome()
                .firmware_start_acknowledged
        );
        f.policy(chip);
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { f.controller.initialize_mac() }.unwrap();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { f.controller.start(Band::Ghz5, 36, 5180) }.unwrap();
        f.retire();
    }
}
#[test]
fn independent_rom_and_runtime_fragment_streams_never_concatenate() {
    if crate::test_process::isolate_model() {
        return;
    }
    let mut f = NativeFixture::new(Chip::Mt7921);
    f.install().unwrap();
    device_receive(&f.queues[&0xd4500], &[0xaa, 0xab], false, 0);
    device_receive(&f.queues[&0xd4540], &[0xbb, 0xbc], true, 0);
    let events = f.controller.poll_events().unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].bytes, [0xbb, 0xbc]);
    device_receive(&f.queues[&0xd4500], &[0xac, 0xad], true, 0);
    let events = f.controller.poll_events().unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].bytes, [0xaa, 0xab, 0xac, 0xad]);
    f.retire();
}
#[test]
fn missing_live_rfkill_service_refuses_start_before_any_radio_command() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7925] {
        let mut f = NativeFixture::new(chip);
        {
            let mut m = f.model.lock().unwrap();
            let p = m.capability_payload.as_mut().unwrap();
            let n = p.len();
            p[n - 8..].copy_from_slice(&9u64.to_le_bytes());
        }
        f.install().unwrap();
        f.boot(chip);
        f.policy(chip);
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { f.controller.initialize_mac() }.unwrap();
        let count = f.model.lock().unwrap().initial_commands.len();
        let reg = f.model.lock().unwrap().regulatory_commands.len();
        assert!(matches!(
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            unsafe { f.controller.start(Band::Ghz5, 36, 5180) },
            Err(BootError::RequiredRfkill)
        ));
        assert_eq!(f.controller.phase(), Phase::Mac);
        assert_eq!(f.model.lock().unwrap().initial_commands.len(), count);
        assert_eq!(f.model.lock().unwrap().regulatory_commands.len(), reg);
        f.retire();
    }
}
#[test]
fn wrong_prepared_input_refuses_before_restart_or_firmware_mutation() {
    if crate::test_process::isolate_model() {
        return;
    }
    let mut f = NativeFixture::new(Chip::Mt7921);
    f.install().unwrap();
    let r = Requests::new(&identity(Chip::Mt7925));
    let provider = catalog(&r, patch_bytes(), ram(&clc_bytes(Chip::Mt7925)));
    let p = r.load(&provider).unwrap();
    assert!(matches!(
        f.controller.boot(&p),
        Err(BootError::WrongFirmware)
    ));
    assert_eq!(f.controller.phase(), Phase::Queues);
    assert!(f.model.lock().unwrap().initial_commands.is_empty());
    f.retire();
}

#[test]
fn a_late_power_on_report_still_downloads_as_linux_does() {
    if crate::test_process::isolate_model() {
        return;
    }
    let mut f = NativeFixture::new(Chip::Mt7925);
    // The MCU never reports FW_PWR_ON: mt792x_load_firmware warns, then
    // downloads, and the download's own replies decide.
    f.bar.0.lock().unwrap().values.insert(0xe00f0, 0);
    f.install().unwrap();
    f.boot(Chip::Mt7925);
    assert!(f.controller.firmware_power_late());
    assert_eq!(f.controller.phase(), Phase::Firmware);
    f.retire();
}
