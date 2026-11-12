// SPDX-License-Identifier: GPL-2.0-or-later
use super::*;
use stela_driver_base::stepped::block_on;
#[test]
fn gpio_minimum_failure_never_selects_phy_and_keeps_original_control_owner() {
    let hw = Rc::new(RefCell::new(model::Hardware::new(A, PHY_A)));
    hw.borrow_mut().minimum_failure = true;
    assert!(matches!(
        block_on(Chip::attach(model::Io(hw.clone()), false)),
        Err(BlockError::PciPower(
            crate::bus::pci::power::Error::Interrupted
        ))
    ));
    assert_eq!(hw.borrow().trace.last().unwrap().0[1], WRITE_GPIO);
    assert_eq!(hw.borrow().minimum_attempts, [5000]);
    assert!(hw.borrow().delays.is_empty());
    assert_eq!(Rc::strong_count(&hw), 2);
}
