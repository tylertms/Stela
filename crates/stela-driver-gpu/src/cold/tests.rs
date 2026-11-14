//! Hostile inherited configuration and whole-graph cold ownership controls.
extern crate std;
use super::*;
use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;
const FLR: u32 = 1 << 28;
use stela_driver_base::bus::{
    dma::mapping::DmaError,
    pci::{Address, ConfigError},
};

#[derive(Default)]
struct State {
    events: Vec<&'static str>,
    preparation_drops: usize,
    view_drops: usize,
    fail_views: bool,
    fail_timer: bool,
    fail_wait: bool,
    minimum_waited_us: u64,
    panic_views: bool,
    panic_reset: bool,
    fail_drain: bool,
    panic_drain: bool,
    /// A bus reset is offered, and whether it fails.
    bus_reset: bool,
    fail_bus_reset: bool,
}
struct Config {
    words: Box<[u32; 128]>,
    state: Rc<RefCell<State>>,
}
impl ConfigSpace for Config {
    fn read8(&self, offset: u16) -> Result<u8, ConfigError> {
        Ok((self.read32(offset & !3)? >> (8 * (offset & 3))) as u8)
    }

    fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
        Ok((self.read32(offset & !3)? >> (8 * (offset & 3))) as u16)
    }

    fn write8(&mut self, offset: u16, value: u8) -> Result<(), ConfigError> {
        let shift = 8 * (offset & 3);
        let word = self.read32(offset & !3)?;
        self.write32(
            offset & !3,
            (word & !(0xff << shift)) | (u32::from(value) << shift),
        )
    }

    fn write16(&mut self, offset: u16, value: u16) -> Result<(), ConfigError> {
        let shift = 8 * (offset & 3);
        let word = self.read32(offset & !3)?;
        self.write32(
            offset & !3,
            (word & !(0xffff << shift)) | (u32::from(value) << shift),
        )
    }

    fn read32(&self, offset: u16) -> Result<u32, ConfigError> {
        Ok(self.words[usize::from(offset / 4)])
    }
    fn write32(&mut self, offset: u16, value: u32) -> Result<(), ConfigError> {
        let before = self.words[usize::from(offset / 4)];
        self.words[usize::from(offset / 4)] = value;
        if offset == 0x84 && value & 3 == 3 {
            self.state.borrow_mut().events.push("d3hot");
        }
        if offset == 0x84 && before & 3 == 3 && value & 3 == 0 {
            self.state.borrow_mut().events.push("d0");
            if before & 8 == 0 {
                self.words[4] = 0;
            }
        }
        if offset == 0x48 && value & 0x8000 != 0 {
            let mut state = self.state.borrow_mut();
            state.events.push("flr");
            assert!(!state.panic_reset, "first FLR write interrupted");
            self.words[0x48 / 4] = 0;
            self.words[4] = 0;
        }
        Ok(())
    }
}
struct Peer {
    function: Function<Config>,
    state: Rc<RefCell<State>>,
}
impl Drop for Peer {
    fn drop(&mut self) {
        self.state.borrow_mut().preparation_drops += 1;
    }
}
struct Bound(Peer);
// SAFETY: this is an isolated configuration/root-order diagnostic; symbolic
// values never grant native mappings, interrupts, DMA or requester admission.
unsafe impl Preparation for Peer {
    type Config = Config;
    type Bound = Bound;
    type Failure = ManuallyDrop<Peer>;
    fn function_mut(&mut self) -> &mut Function<Config> {
        &mut self.function
    }
    fn bus_reset_available(&self) -> bool {
        self.state.borrow().bus_reset
    }
    fn reset_bus(&mut self, timer: &dyn Timer) -> Result<(), BlockError> {
        timer.wait_us(100_000).map_err(BlockError::from)?;
        let mut state = self.state.borrow_mut();
        state.events.push("bus-reset");
        if state.fail_bus_reset {
            return Err(BlockError::Unsupported("bus reset failed"));
        }
        Ok(())
    }
    unsafe fn finalize(self) -> Result<Self::Bound, (preparation::Error, Self::Failure)> {
        let peer = ManuallyDrop::new(self);
        let failed = {
            let mut state = peer.state.borrow_mut();
            if state.events.contains(&"flr") {
                assert!(state.minimum_waited_us >= 100_000);
            }
            if state.events.contains(&"d0") {
                assert!(state.minimum_waited_us >= 20_000);
            }
            state.events.push("root-drain");
            assert!(!state.panic_drain, "root callback interrupted");
            state.fail_drain
        };
        if failed {
            return Err((preparation::Error::Dma(DmaError::DrainUnavailable), peer));
        }
        peer.state.borrow_mut().events.push("bind");
        Ok(Bound(ManuallyDrop::into_inner(peer)))
    }
}
struct OldViews(Rc<RefCell<State>>);
impl Drop for OldViews {
    fn drop(&mut self) {
        self.0.borrow_mut().view_drops += 1;
    }
}
// SAFETY: bounded diagnostic only; it owns no native framebuffer or PCI proof.
// Events model an independent real view owner, never a native success adapter.
unsafe impl Views<Config> for OldViews {
    fn seal(&mut self, function: &mut Function<Config>) -> Result<(), ViewError> {
        if !Rc::ptr_eq(&self.0, &function.config().state) {
            return Err(ViewError::ForeignSource);
        }
        let mut state = self.0.borrow_mut();
        state.events.push("views-seal");
        assert!(!state.panic_views, "CPU writer interrupted");
        if state.fail_views {
            Err(ViewError::Busy)
        } else {
            Ok(())
        }
    }
}
struct PhysicalTimer(Rc<RefCell<State>>);
// SAFETY: the isolated fixture advances its sole model's physical time
// exactly by each requested minimum. It owns no native timing source.
unsafe impl Timer for PhysicalTimer {
    fn check(&self) -> Result<(), power::Error> {
        if self.0.borrow().fail_timer {
            Err(power::Error::Clock)
        } else {
            Ok(())
        }
    }

    fn wait_us(&self, minimum: u64) -> Result<(), power::Error> {
        self.check()?;
        let mut state = self.0.borrow_mut();
        if state.fail_wait {
            return Err(power::Error::Clock);
        }
        state.minimum_waited_us += minimum;
        Ok(())
    }
}
fn inputs() -> (Inputs<Peer, OldViews>, Rc<RefCell<State>>) {
    let state = Rc::new(RefCell::new(State::default()));
    let mut words = [0; 128];
    words[0] = 0x1234_10de;
    words[1] = 0x0010_0402;
    words[2] = 0x0300_0001;
    words[4] = 0xfe00_0000;
    words[0x34 / 4] = 0x40;
    words[0x40 / 4] = 0x0002_8010;
    words[0x44 / 4] = FLR;
    words[0x80 / 4] = 0x0003_0001;
    words[0x100 / 4] = u32::MAX;
    let function = Function::new(
        Address {
            segment: 0,
            bus: 1,
            device: 0,
            function: 0,
        },
        Config {
            words: Box::new(words),
            state: state.clone(),
        },
    )
    .unwrap();
    (
        Inputs {
            requester: Peer {
                function,
                state: state.clone(),
            },
            views: OldViews(state.clone()),
            timer: Rc::new(PhysicalTimer(state.clone())),
        },
        state,
    )
}
fn failure(
    result: Result<Ready<Bound, OldViews>, Failure<Peer, OldViews>>,
) -> Failure<Peer, OldViews> {
    match result {
        Err(failure) => failure,
        Ok(_) => panic!("unexpected cold binding"),
    }
}
fn kept(state: &State) {
    assert_eq!(state.preparation_drops, 0);
    assert_eq!(state.view_drops, 0);
}
#[test]
fn timed_flr_precedes_original_drain_and_unique_bind() {
    let (inputs, state) = inputs();
    {
        let _ready = Cold::new(inputs)
            .quiesce_and_bind()
            .unwrap_or_else(|error| panic!("{:?}", error.error));
        assert_eq!(
            state.borrow().events,
            ["views-seal", "flr", "root-drain", "bind"]
        );
        assert!(state.borrow().minimum_waited_us >= 100_000);
    }
    kept(&state.borrow());
}
#[test]
fn preflight_never_seals_or_resets_an_unsupported_or_foreign_source() {
    for case in 0..4 {
        let (mut inputs, state) = inputs();
        match case {
            0 => state.borrow_mut().fail_timer = true,
            1 => {
                let words = &mut inputs.requester.function.config().words;
                words[0x44 / 4] = 0;
                words[0x84 / 4] = 8;
            }
            2 => inputs.requester.function.config().words[1] |= 4,
            _ => inputs.requester.function.config().words[0] ^= 1,
        }
        let failed = failure(Cold::new(inputs).quiesce_and_bind());
        assert!(state.borrow().events.is_empty());
        drop(failed);
        kept(&state.borrow());
    }
}
#[test]
fn failed_original_cpu_view_retirement_cannot_reset_or_bind() {
    let (inputs, state) = inputs();
    state.borrow_mut().fail_views = true;
    let failed = failure(Cold::new(inputs).quiesce_and_bind());
    assert_eq!(failed.error, Error::Views(ViewError::Busy));
    drop(failed);
    assert_eq!(state.borrow().events, ["views-seal"]);
    kept(&state.borrow());
}
#[test]
fn lost_timer_after_flr_retains_owners_and_prevents_root_binding() {
    let (inputs, state) = inputs();
    state.borrow_mut().fail_wait = true;
    let failed = failure(Cold::new(inputs).quiesce_and_bind());
    assert_eq!(
        failed.error,
        Error::Reset(BlockError::PciPower(power::Error::Clock))
    );
    assert_eq!(state.borrow().events, ["views-seal", "flr"]);
    drop(failed);
    kept(&state.borrow());
}

#[test]
fn failed_original_root_drain_retains_reset_function_and_views_without_binding() {
    let (inputs, state) = inputs();
    state.borrow_mut().fail_drain = true;
    let failed = failure(Cold::new(inputs).quiesce_and_bind());
    assert_eq!(
        failed.error,
        Error::Preparation(preparation::Error::Dma(DmaError::DrainUnavailable))
    );
    drop(failed);
    assert_eq!(state.borrow().events, ["views-seal", "flr", "root-drain"]);
    kept(&state.borrow());
}
#[test]
fn caught_unwind_in_each_effect_phase_keeps_whole_graph_and_never_retries() {
    for phase in 0..3 {
        let (inputs, state) = inputs();
        {
            let mut s = state.borrow_mut();
            match phase {
                0 => s.panic_views = true,
                1 => s.panic_reset = true,
                _ => s.panic_drain = true,
            }
        }
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _outcome = Cold::new(inputs).quiesce_and_bind();
        }));
        assert!(outcome.is_err());
        assert!(!state.borrow().events.contains(&"bind"));
        assert!(
            state
                .borrow()
                .events
                .iter()
                .filter(|event| **event == "flr")
                .count()
                <= 1
        );
        kept(&state.borrow());
    }
}
#[test]
fn abandoned_before_any_attempt_retains_both_original_owners() {
    let (inputs, state) = inputs();
    {
        let _cold = Cold::new(inputs);
    }
    assert!(state.borrow().events.is_empty());
    kept(&state.borrow());
}
#[test]
fn explicit_success_handover_transfers_both_owners() {
    let (inputs, state) = inputs();
    let ready = Cold::new(inputs)
        .quiesce_and_bind()
        .unwrap_or_else(|error| panic!("{:?}", error.error));
    // SAFETY: this is the complete handover/release of a symbolic CPU-only
    // diagnostic. It creates no native config/DMA/IRQ/view authority.
    let (mut bound, views) = unsafe { ready.into_parts() };
    assert_eq!(bound.0.function.config().words[4], 0xfe00_0000);
    drop(bound);
    drop(views);
    assert_eq!(state.borrow().preparation_drops, 1);
    assert_eq!(state.borrow().view_drops, 1);
}
#[test]
fn a_foreign_retained_view_owner_never_certifies_another_function() {
    let (mut inputs, state) = inputs();
    let foreign = Rc::new(RefCell::new(State::default()));
    inputs.views = OldViews(foreign.clone());
    let failed = failure(Cold::new(inputs).quiesce_and_bind());
    assert_eq!(failed.error, Error::Views(ViewError::ForeignSource));
    drop(failed);
    assert!(state.borrow().events.is_empty());
    assert!(foreign.borrow().events.is_empty());
    assert_eq!(foreign.borrow().view_drops, 0);
}
#[test]
fn a_function_without_flr_gets_a_d3hot_soft_reset() {
    let (mut inputs, state) = inputs();
    inputs.requester.function.config().words[0x44 / 4] = 0;
    let ready = Cold::new(inputs)
        .quiesce_and_bind()
        .unwrap_or_else(|error| panic!("{:?}", error.error));
    assert_eq!(
        ready.reset(),
        Some(Reset::Function(ResetMethod::PowerManagement))
    );
    assert_eq!(
        state.borrow().events,
        ["views-seal", "d3hot", "d0", "root-drain", "bind"]
    );
    // SAFETY: symbolic CPU-only diagnostic handover.
    let (mut bound, _views) = unsafe { ready.into_parts() };
    assert_eq!(bound.0.function.config().words[4], 0xfe00_0000);
}
#[test]
fn no_reset_starts_unreset_only_when_dma_is_translated() {
    for translated in [false, true] {
        let (mut inputs, state) = inputs();
        {
            let words = &mut inputs.requester.function.config().words;
            words[0x44 / 4] = 0;
            words[0x84 / 4] = 8;
        }
        let candidate = check_candidate(
            &mut inputs.requester.function,
            &PhysicalTimer(state.clone()),
            translated,
            false,
        );
        let mut cold = Cold::new(inputs);
        if translated {
            assert_eq!(candidate, Ok(None));
            // SAFETY: the symbolic domain stands for a translating one.
            cold = unsafe { cold.translated() };
            let ready = cold
                .quiesce_and_bind()
                .unwrap_or_else(|error| panic!("{:?}", error.error));
            assert_eq!(ready.reset(), None);
            assert_eq!(state.borrow().events, ["views-seal", "root-drain", "bind"]);
        } else {
            assert_eq!(candidate, Err(Error::ResetUnavailable));
            let failed = failure(cold.quiesce_and_bind());
            assert_eq!(failed.error, Error::ResetUnavailable);
            drop(failed);
            assert!(state.borrow().events.is_empty());
            kept(&state.borrow());
        }
    }
}
#[test]
fn without_a_function_reset_or_translation_the_bus_is_reset() {
    for fail in [false, true] {
        let (mut inputs, state) = inputs();
        {
            let words = &mut inputs.requester.function.config().words;
            words[0x44 / 4] = 0;
            words[0x84 / 4] = 8;
        }
        state.borrow_mut().bus_reset = true;
        state.borrow_mut().fail_bus_reset = fail;
        let candidate = check_candidate(
            &mut inputs.requester.function,
            &PhysicalTimer(state.clone()),
            false,
            true,
        );
        assert_eq!(candidate, Ok(Some(Reset::SecondaryBus)));
        let result = Cold::new(inputs).quiesce_and_bind();
        if fail {
            let failed = failure(result);
            assert_eq!(
                failed.error,
                Error::Reset(BlockError::Unsupported("bus reset failed"))
            );
            drop(failed);
            assert_eq!(state.borrow().events, ["views-seal", "bus-reset"]);
            kept(&state.borrow());
        } else {
            let ready = result.unwrap_or_else(|error| panic!("{:?}", error.error));
            assert_eq!(ready.reset(), Some(Reset::SecondaryBus));
            assert_eq!(
                state.borrow().events,
                ["views-seal", "bus-reset", "root-drain", "bind"]
            );
        }
    }
}
#[test]
fn a_function_reset_comes_before_the_bus_reset() {
    for translated in [false, true] {
        let (mut inputs, state) = inputs();
        state.borrow_mut().bus_reset = true;
        let candidate = check_candidate(
            &mut inputs.requester.function,
            &PhysicalTimer(state.clone()),
            translated,
            true,
        );
        assert_eq!(
            candidate,
            Ok(Some(Reset::Function(ResetMethod::FunctionLevel)))
        );
        let mut cold = Cold::new(inputs);
        if translated {
            // SAFETY: the symbolic domain stands for a translating one.
            cold = unsafe { cold.translated() };
        }
        let ready = cold
            .quiesce_and_bind()
            .unwrap_or_else(|error| panic!("{:?}", error.error));
        assert_eq!(
            ready.reset(),
            Some(Reset::Function(ResetMethod::FunctionLevel))
        );
        assert!(!state.borrow().events.contains(&"bus-reset"));
    }
}

/// A GPU an earlier run left holding its secure region needs a reset even
/// when its DMA is translated, so an available bus reset is not skipped.
#[test]
fn a_translated_function_without_a_function_reset_still_gets_the_bus_reset() {
    let (mut inputs, state) = inputs();
    {
        let words = &mut inputs.requester.function.config().words;
        words[0x44 / 4] = 0;
        words[0x84 / 4] = 8;
    }
    state.borrow_mut().bus_reset = true;
    let candidate = check_candidate(
        &mut inputs.requester.function,
        &PhysicalTimer(state.clone()),
        true,
        true,
    );
    assert_eq!(candidate, Ok(Some(Reset::SecondaryBus)));
    // SAFETY: the symbolic domain stands for a translating one.
    let cold = unsafe { Cold::new(inputs).translated() };
    let ready = cold
        .quiesce_and_bind()
        .unwrap_or_else(|error| panic!("{:?}", error.error));
    assert_eq!(ready.reset(), Some(Reset::SecondaryBus));
    assert_eq!(
        state.borrow().events,
        ["views-seal", "bus-reset", "root-drain", "bind"]
    );
}
