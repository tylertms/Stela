// SPDX-License-Identifier: GPL-2.0-or-later
//! Isolated control owners; no physical permission from numeric model time.
use super::*;
use crate::bus::pci::power::Error;
use alloc::rc::Rc;
use core::cell::{Cell, RefCell};
use stela_driver_base::stepped::block_on;
#[derive(Default)]
struct Effects {
    controls: RefCell<Vec<[u8; 8]>>,
    waits: RefCell<Vec<u64>>,
    elapsed: Cell<u64>,
    refuse: Cell<bool>,
    drops: Cell<usize>,
}
struct Owner(Rc<Effects>);
impl Drop for Owner {
    fn drop(&mut self) {
        self.0.drops.set(self.0.drops.get() + 1);
    }
}
impl Control for Owner {
    async fn control(&mut self, request: [u8; 8], _: &[u8]) -> Result<Vec<u8>, BlockError> {
        self.0.controls.borrow_mut().push(request);
        Ok(Vec::new())
    }
    async fn delay_us(&mut self, _: u64) {
        panic!("physical sequence used nominal bridge");
    }
    async fn minimum_wait(&mut self, us: u64) -> Result<(), BlockError> {
        self.0.waits.borrow_mut().push(us);
        if self.0.refuse.get() {
            return Err(BlockError::PciPower(Error::Interrupted));
        }
        self.0.elapsed.set(self.0.elapsed.get() + us);
        Ok(())
    }
}
#[test]
fn minimum_failure_is_sticky_before_control_effects_and_owner_destruction() {
    let effects = Rc::new(Effects::default());
    let mut owner = MinimumControl::new(Owner(effects.clone()));
    block_on(owner.minimum_wait(73)).unwrap();
    assert_eq!(effects.elapsed.get(), 73);
    effects.refuse.set(true);
    assert_eq!(
        block_on(owner.minimum_wait(500)),
        Err(BlockError::PciPower(Error::Interrupted))
    );
    effects.refuse.set(false);
    assert_eq!(
        block_on(owner.minimum_wait(800)),
        Err(BlockError::PciPower(Error::Interrupted))
    );
    assert_eq!(
        block_on(owner.control([0; 8], &[])),
        Err(BlockError::PciPower(Error::Interrupted))
    );
    assert_eq!(*effects.waits.borrow(), [73, 500]);
    assert!(effects.controls.borrow().is_empty());
    drop(owner);
    assert_eq!(effects.drops.get(), 0);
}
#[test]
fn ax88179_power_reset_failure_stops_before_clock_select_and_retains_partial_owner() {
    let effects = Rc::new(Effects::default());
    effects.refuse.set(true);
    assert!(matches!(
        block_on(super::super::asix::Chip::attach(Owner(effects.clone()))),
        Err(BlockError::PciPower(Error::Interrupted))
    ));
    assert_eq!(effects.controls.borrow().len(), 2);
    assert_eq!(*effects.waits.borrow(), [500_000]);
    assert_eq!(effects.drops.get(), 0);
}
#[test]
fn actual_host_bridge_reports_retained_model_timer_failure_without_later_device_access() {
    use crate::usb::xhci::{HIGH_SPEED, model};
    let mut memory = alloc::vec![0u64;1024*1024];
    let (controller, device, hardware, _dma) = model::reset_fixture(&mut memory, HIGH_SPEED, 32, 1);
    let mut control = MinimumControl::new(super::super::HostControl {
        controller: controller.clone(),
        id: device.id,
        stepped: Default::default(),
    });
    assert_eq!(
        block_on(control.minimum_wait(u64::MAX)),
        Err(BlockError::PciPower(Error::Clock))
    );
    let accesses = hardware.borrow().accesses;
    assert_eq!(
        block_on(control.control([0; 8], &[])),
        Err(BlockError::PciPower(Error::Clock))
    );
    assert_eq!(hardware.borrow().accesses, accesses);
    assert_eq!(controller.borrow().minimum_error(), Some(Error::Clock));
}
