// SPDX-License-Identifier: GPL-2.0-only
//! Link calls against a device that answers late, after the call that asked
//! returned: each call stays within its step's bound on the model clock, and
//! the device sees what a device that answers at once sees.
use super::super::UsbEthernet;
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
