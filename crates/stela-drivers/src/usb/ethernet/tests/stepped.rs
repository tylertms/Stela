// SPDX-License-Identifier: GPL-2.0-only
//! Link calls against a device that answers late, after the call that asked
//! returned: each call stays within its step's bound on the model clock, and
//! the device sees what a device that answers at once sees.
use super::super::{LINK_POLL_US, TRANSMIT_US, UsbEthernet};
use crate::bus::dma::TestRegion;
use crate::usb::xhci::{Shared, model::CancelModel};
use crate::{Clock, test_clock};
use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;

/// No runtime link call may hold the processor longer than this.
pub(crate) const CALL_US: u64 = 60_000;
/// Model time between two link calls.
const BETWEEN_US: u64 = 1_000;

/// Link calls, each bounded and each followed by the device's late answers.
pub(crate) struct Calls {
    model: Rc<RefCell<CancelModel>>,
    clock: Clock,
    /// Completions the device gave only after the call that asked.
    pub(crate) answered_late: usize,
}

impl Calls {
    /// Calls to a link on `model`, whose device answers late when `late`.
    pub(crate) fn new(model: Rc<RefCell<CancelModel>>, clock: Clock, late: bool) -> Self {
        model.borrow_mut().late = late;
        Self {
            model,
            clock,
            answered_late: 0,
        }
    }

    /// One call, within [`CALL_US`]; then the device answers what it was
    /// asked.
    pub(crate) fn call<T>(&mut self, call: impl FnOnce() -> T) -> T {
        let start = self.clock.now_us();
        let output = call();
        let took = self.clock.now_us() - start;
        assert!(took <= CALL_US, "a link call held the processor {took} us");
        test_clock::advance(self.clock, BETWEEN_US);
        let mut model = self.model.borrow_mut();
        self.answered_late += model.held.len();
        model.release();
        output
    }

    /// Calls until `done` holds of one's output, which is returned.
    pub(crate) fn until<T>(&mut self, mut call: impl FnMut() -> T, done: impl Fn(&T) -> bool) -> T {
        for _ in 0..20_000 {
            let output = self.call(&mut call);
            if done(&output) {
                return output;
            }
        }
        panic!("link calls never reached their end");
    }

    /// From here the device answers at once again, as teardown needs.
    pub(crate) fn answer_at_once(&self) {
        let mut model = self.model.borrow_mut();
        model.late = false;
        model.release();
    }

    /// Model time passing without link calls.
    pub(crate) fn wait(&self, microseconds: u64) {
        test_clock::advance(self.clock, microseconds);
    }
}

/// What the device saw: its control writes, except those `ignored` keeps
/// out, the controller's commands and the frames sent.
pub(crate) type Effects = (Vec<([u8; 8], Vec<u8>)>, Vec<u32>, Vec<Vec<u8>>);
pub(crate) fn effects(model: &CancelModel, ignored: impl Fn(&[u8; 8]) -> bool) -> Effects {
    let writes = model
        .setup
        .iter()
        .filter(|(request, _)| request[0] & 0x80 == 0 && !ignored(request))
        .cloned()
        .collect();
    (writes, model.seen.clone(), model.outgoing.clone())
}

/// A vendor Ethernet link bound on the cancellation model, with the model's
/// way of bringing its PHY up or down. The link drops first, while the
/// controller and DMA it uses still live.
pub(crate) struct Fixture {
    pub(crate) link: UsbEthernet,
    pub(crate) model: Rc<RefCell<CancelModel>>,
    shared: Shared,
    _dma: TestRegion,
    pub(crate) set_link: fn(&mut CancelModel, bool),
}

impl Fixture {
    pub(crate) fn new(
        link: UsbEthernet,
        model: Rc<RefCell<CancelModel>>,
        shared: Shared,
        dma: TestRegion,
        set_link: fn(&mut CancelModel, bool),
    ) -> Self {
        Self {
            link,
            model,
            shared,
            _dma: dma,
            set_link,
        }
    }
    pub(crate) fn clock(&self) -> Clock {
        self.shared.borrow().clock
    }
}

/// A stop while a control transfer is under way on a device that stopped
/// answering gives the transfer the stop's bound and then cancels it on the
/// host side, instead of waiting out its 5 s; the bulk endpoints are
/// cancelled too, and the chip, whose register read failed, refuses its MAC
/// stop as after any failed transfer.
#[test]
fn a_stop_cancels_a_control_transfer_under_way_instead_of_waiting_it_out() {
    use crate::net::Link;
    let mut fixture = super::super::rtl815x::tests::rtl8152_fixture();
    assert!(fixture.link.settled_carrier());
    assert_eq!(fixture.link.receive(), Ok(None));
    let clock = fixture.clock();
    test_clock::advance(clock, LINK_POLL_US);
    let controls = fixture.model.borrow().setup.len();
    fixture.model.borrow_mut().silence_control_from = Some(controls);
    // The carrier check's PHY status read goes out and is not answered.
    assert!(fixture.link.carrier());
    assert!(fixture.link.running.is_some());
    assert_eq!(fixture.model.borrow().setup.len(), controls + 1);
    assert!(fixture.model.borrow().active[1].is_some());
    let started = clock.now_us();
    assert!(fixture.link.stop().is_err());
    let took = clock.now_us() - started;
    let bound = super::super::STOP_US;
    assert!(took >= bound && took < 2 * bound, "{took} us");
    let model = fixture.model.borrow();
    assert!(model.active[1].is_none() && model.active[3].is_none());
    // No MAC stop request followed the cancelled read.
    assert_eq!(model.setup.len(), controls + 1);
}

/// An unplugged adapter's stop: only the host side cancels its bulk
/// endpoints, as r8152 skips register access once RTL8152_UNPLUG is set,
/// and the MAC stop's control transfers, which could only fail, are never
/// sent.
#[test]
fn an_unplugged_adapter_stops_with_the_host_side_cancellation_alone() {
    use crate::net::Link;
    let mut fixture = super::super::rtl815x::tests::rtl8152_fixture();
    assert!(fixture.link.settled_carrier());
    // A transmit TD the device never answers and a receive TD waiting.
    fixture.model.borrow_mut().silence_transfer = Some(4);
    fixture.link.transmit(&[0x5a; 60]).unwrap();
    assert_eq!(fixture.link.receive(), Ok(None));
    fixture.model.borrow_mut().connected = false;
    let controls = fixture.model.borrow().setup.len();
    assert_eq!(fixture.link.stop(), Ok(()));
    let model = fixture.model.borrow();
    assert_eq!(model.setup.len(), controls);
    assert!(model.active[3].is_none() && model.active[4].is_none());
    drop(model);
    assert!(fixture.link.transmit(&[0x6a; 60]).is_err());
}

/// The link's work in steps, each from the state before it.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Sequence {
    /// The first carrier check, which sets the MAC up for the link.
    Carrier,
    /// The lost link's check, quiesce and MAC stop.
    LinkDown,
    /// A STALLed receive transfer's endpoint reset and halt clear.
    ClearInput,
    /// A STALLed transmit transfer's endpoint reset and halt clear.
    ClearOutput,
    /// A transmit transfer past its deadline, cancelled.
    CancelOutput,
}

const SEQUENCES: [Sequence; 5] = [
    Sequence::Carrier,
    Sequence::LinkDown,
    Sequence::ClearInput,
    Sequence::ClearOutput,
    Sequence::CancelOutput,
];

/// What goes wrong in the middle of a sequence.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Fault {
    None,
    /// The device stops answering EP0 from the sequence's control transfer
    /// at this index on.
    Control(usize),
    /// The controller never completes a command of this type.
    Command(u32),
    /// The adapter is unplugged before this call of the sequence.
    Unplugged(usize),
    /// The device is taken away, its slot retiring, before this call.
    Removed(usize),
    /// The controller dies before this call.
    Died(usize),
}

/// What a sequence did: its control transfers, its commands and the calls
/// it took.
pub(crate) struct Sequenced {
    pub(crate) controls: usize,
    pub(crate) commands: Vec<u32>,
    pub(crate) calls: usize,
}

const INPUT: usize = 3;
const OUTPUT: usize = 4;
/// xHCI's time for a control transfer and for a command.
pub(crate) const DEVICE_TIMEOUT_US: u64 = 5_000_000;

/// Fault calls past a sequence's start, where it is under way: the first,
/// the middle and the last of the `calls` a healthy run took.
pub(crate) fn during(calls: usize) -> Vec<usize> {
    let mut at = alloc::vec![0, calls / 2, calls.saturating_sub(1)];
    at.dedup();
    at
}

/// One sequence on a fresh link whose device answers late, with `fault`;
/// every call is bounded, and the link either ends the work as a healthy
/// device lets it or closes.
fn sequence(build: fn() -> Fixture, sequence: Sequence, fault: Fault) -> Sequenced {
    use crate::net::{Link, LinkError};
    let mut fixture = build();
    let model = fixture.model.clone();
    let clock = fixture.clock();
    let mut calls = Calls::new(model.clone(), clock, true);
    let link = &mut fixture.link;
    if sequence != Sequence::Carrier {
        calls.until(|| link.carrier(), |up| *up);
    }
    let (controls, commands) = {
        let model = model.borrow();
        (model.setup.len(), model.seen.len())
    };
    match fault {
        Fault::Control(index) => model.borrow_mut().silence_control_from = Some(controls + index),
        Fault::Command(kind) => model.borrow_mut().silence = Some(kind),
        _ => {}
    }
    let started = clock.now_us();
    match sequence {
        Sequence::Carrier => {}
        Sequence::LinkDown => {
            (fixture.set_link)(&mut model.borrow_mut(), false);
            calls.wait(LINK_POLL_US);
        }
        Sequence::ClearInput => {
            model.borrow_mut().halt_transfer = Some(INPUT);
            assert_eq!(calls.call(|| link.receive()), Ok(None));
            model.borrow_mut().halt_transfer = None;
        }
        Sequence::ClearOutput => {
            model.borrow_mut().halt_transfer = Some(OUTPUT);
            assert_eq!(calls.call(|| link.transmit(&[0x5a; 60])), Ok(()));
            model.borrow_mut().halt_transfer = None;
        }
        Sequence::CancelOutput => {
            model.borrow_mut().silence_transfer = Some(OUTPUT);
            assert_eq!(calls.call(|| link.transmit(&[0x5a; 60])), Ok(()));
            calls.wait(TRANSMIT_US);
            model.borrow_mut().silence_transfer = None;
            // A carrier check that came due runs first; then a transmit
            // finds the TD past its deadline and starts its cancellation.
            for _ in 0..1_000 {
                assert_eq!(
                    calls.call(|| link.transmit(&[0x6a; 60])),
                    Err(LinkError::Busy)
                );
                if matches!(
                    &link.running,
                    Some(super::super::Running {
                        work: super::super::Work::ResetOutput,
                        ..
                    })
                ) {
                    break;
                }
            }
        }
    }
    let mut taken = 0;
    let ended = loop {
        match fault {
            Fault::Unplugged(at) if at == taken => model.borrow_mut().connected = false,
            Fault::Removed(at) if at == taken => {
                let id = link.id;
                fixture.shared.borrow_mut().device(id).unwrap().quarantine();
            }
            Fault::Died(at) if at == taken => model.borrow_mut().host_system_error(),
            _ => {}
        }
        let result = calls.call(|| link.update_carrier());
        taken += 1;
        if result.is_err() || link.running.is_none() {
            break result;
        }
        assert!(taken < 20_000, "{sequence:?} {fault:?} never ended");
    };
    let sequenced = {
        let model = model.borrow();
        Sequenced {
            controls: model.setup.len() - controls,
            commands: model.seen[commands..].to_vec(),
            calls: taken,
        }
    };
    if let Fault::Control(_) | Fault::Command(_) = fault {
        // The silent transfer or command failed at its own time, not before.
        assert!(
            clock.now_us() - started >= DEVICE_TIMEOUT_US,
            "{sequence:?} {fault:?}"
        );
    }
    let healthy = matches!(fault, Fault::None);
    if healthy {
        assert_eq!(ended, Ok(()), "{sequence:?}");
        match sequence {
            Sequence::Carrier => assert!(link.carrier),
            Sequence::LinkDown => {
                assert!(!link.carrier && !link.closed);
                assert!(model.borrow().active[INPUT].is_none());
            }
            Sequence::ClearInput => assert_eq!(link.rx_dropped(), 1),
            Sequence::ClearOutput | Sequence::CancelOutput => {
                // The failure is reported once, after any carrier check
                // that came due meanwhile, then frames go out again.
                let not_busy = |result: &Result<(), LinkError>| result != &Err(LinkError::Busy);
                let failed = calls.until(|| link.transmit(&[0x7a; 60]), not_busy);
                assert!(matches!(failed, Err(LinkError::Adapter(_))), "{failed:?}");
                calls
                    .until(|| link.transmit(&[0x8a; 60]), not_busy)
                    .unwrap();
            }
        }
    } else {
        // The work ended in failure, or, for a controller that died at its
        // last call, ended before the death showed: either way the link is
        // down and refuses frames from here on.
        assert!(
            ended.is_err() || matches!(fault, Fault::Died(_)),
            "{sequence:?} {fault:?}"
        );
        assert!(!calls.call(|| link.carrier()), "{sequence:?} {fault:?}");
        let refused = calls.call(|| link.transmit(&[0x9a; 60]));
        assert!(
            matches!(refused, Err(LinkError::Adapter(_))),
            "{sequence:?} {fault:?} {refused:?}"
        );
        assert!(calls.call(|| link.receive()).is_err());
    }
    {
        let mut model = model.borrow_mut();
        model.silence_control_from = None;
        model.silence = None;
    }
    calls.answer_at_once();
    if let Fault::Unplugged(_) = fault {
        // An unplugged adapter's stop is the host side's cancellation
        // alone, within a call's bound.
        assert_eq!(calls.call(|| link.stop()), Ok(()));
        assert!(model.borrow().active[INPUT].is_none());
        assert!(model.borrow().active[OUTPUT].is_none());
    }
    sequenced
}

/// Every sequence of the link `build` makes against a device that answers
/// late: healthy, with each of its control transfers and command types
/// timing out in turn, and with the adapter unplugged, the device taken
/// away and the controller dying at its first, middle and last call.
pub(crate) fn every_fault(build: fn() -> Fixture) {
    for each in SEQUENCES {
        let healthy = sequence(build, each, Fault::None);
        assert!(healthy.calls > 0, "{each:?}");
        // Each sequence's faults are its own transfers and commands, with
        // any carrier check that comes due meanwhile.
        let (controls, commands) = match each {
            Sequence::Carrier => (true, false),
            Sequence::LinkDown | Sequence::ClearInput | Sequence::ClearOutput => (true, true),
            Sequence::CancelOutput => (false, true),
        };
        assert!(!controls || healthy.controls > 0, "{each:?}");
        assert!(!commands || !healthy.commands.is_empty(), "{each:?}");
        for index in 0..healthy.controls {
            sequence(build, each, Fault::Control(index));
        }
        let mut kinds = healthy.commands.clone();
        kinds.sort_unstable();
        kinds.dedup();
        for kind in kinds {
            sequence(build, each, Fault::Command(kind));
        }
        for at in during(healthy.calls) {
            sequence(build, each, Fault::Unplugged(at));
            sequence(build, each, Fault::Removed(at));
            sequence(build, each, Fault::Died(at));
        }
    }
}

#[test]
fn rtl8152_sequences_end_or_close_on_every_fault_within_each_call() {
    every_fault(super::super::rtl815x::tests::rtl8152_fixture);
}

#[test]
fn ax88179_sequences_end_or_close_on_every_fault_within_each_call() {
    every_fault(|| super::super::asix::tests::asix_fixture(false));
}

#[test]
fn ax88772_sequences_end_or_close_on_every_fault_within_each_call() {
    every_fault(|| super::super::asix::tests::asix_fixture(true));
}
