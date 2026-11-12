// SPDX-License-Identifier: GPL-2.0-only
use super::*;
use stela_driver_base::stepped::block_on;
#[test]
fn aldps_quiet_failure_preserves_error_no_following_transfer_and_original_control_owner() {
    let model = Rc::new(RefCell::new(Model::new(0x4c00)));
    let mut chip = block_on(Chip::probe(Io(model.clone()), Revision(0x4c00))).unwrap();
    block_on(chip.minimum_wait(17)).unwrap();
    assert_eq!(model.borrow().minimum_elapsed, 17);
    model.borrow_mut().refuse_minimum = true;
    let error = BlockError::PciPower(crate::bus::pci::power::Error::Interrupted);
    assert_eq!(block_on(chip.aldps(false)), Err(error.clone()));
    let accesses = model.borrow().trace.len();
    model.borrow_mut().refuse_minimum = false;
    assert_eq!(block_on(chip.carrier()), Err(error.clone()));
    assert_eq!(block_on(chip.read(PLA, PLA_TCR0, 2)), Err(error));
    assert_eq!(model.borrow().trace.len(), accesses);
    assert!(!chip.enabled && !chip.initialized);
    drop(chip);
    assert_eq!(Rc::strong_count(&model), 2);
}
