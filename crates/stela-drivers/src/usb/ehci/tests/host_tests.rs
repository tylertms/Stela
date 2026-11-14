//! Root enumeration and Bulk-Only binding over the native-layout peer.
use super::*;
use crate::{Clock, usb::ehci::host::enumerate};
use alloc::{rc::Rc, string::String};
use core::cell::RefCell;

/// What enumerating the peer's root left unused, and how many failures.
fn enumerated(m: &Arc<Mutex<Shared>>) -> (Vec<(String, &'static str)>, usize) {
    let controller = Rc::new(RefCell::new(controller(m.clone())));
    let clock = Clock {
        ticks_per_microsecond: 1_000,
    };
    let found = enumerate(&controller, "ehci", clock);
    assert!(found.disks.is_empty());
    (found.unused, found.failures.len())
}

#[test]
fn a_bulk_only_root_is_addressed_configured_and_sent_its_first_command_block() {
    let _time = crate::test_clock::virtualize();
    let m = Arc::new(Mutex::new(Shared::new()));
    let (unused, _) = enumerated(&m);
    let m = m.lock().unwrap();
    assert_eq!(m.addresses[0], Some(1), "the lowest free address");
    assert_eq!(m.configurations[0], 1);
    // The peer answers bulk IN with a pattern, so the INQUIRY's CSW is
    // malformed; its CBW still went out on the OUT pipe.
    assert_eq!(m.received.get(..4), Some(&b"USBC"[..]));
    assert!(unused.is_empty());
}

#[test]
fn a_full_speed_root_is_left_to_the_companion() {
    let _time = crate::test_clock::virtualize();
    let m = Arc::new(Mutex::new(Shared::new()));
    m.lock().unwrap().high_speed = false;
    let (unused, failures) = enumerated(&m);
    assert_eq!(unused.len(), 1);
    assert_eq!(unused[0].0, "ehci port 1");
    assert!(unused[0].1.contains("companion"));
    assert_eq!(failures, 0);
}

#[test]
fn a_root_without_bulk_only_storage_is_addressed_and_left_unconfigured() {
    let _time = crate::test_clock::virtualize();
    let m = Arc::new(Mutex::new(Shared::new()));
    m.lock().unwrap().description[14] = 0xff;
    let (unused, failures) = enumerated(&m);
    let m = m.lock().unwrap();
    assert_eq!(m.addresses[0], Some(1));
    assert_eq!(m.configurations[0], 0);
    assert_eq!(unused.len(), 1);
    assert!(unused[0].1.contains("Bulk-Only"));
    assert_eq!(failures, 0);
}

/// usb-storage's `delay_use` on EHCI: the configured Bulk-Only root gets
/// its first command block only once its second has passed, and a look
/// before then sends nothing.
#[test]
fn a_bulk_only_root_gets_its_first_command_block_a_second_after_its_configuration() {
    use crate::usb::storage::settle::{SETTLE_US, settle};
    let _time = crate::test_clock::virtualize();
    let m = Arc::new(Mutex::new(Shared::new()));
    let controller = Rc::new(RefCell::new(controller(m.clone())));
    let clock = Clock {
        ticks_per_microsecond: 1_000,
    };
    let mut found = crate::usb::ehci::host::enumerate_unsettled(&controller, "ehci", clock);
    assert!(found.failures.is_empty() && found.disks.is_empty());
    assert_eq!(m.lock().unwrap().configurations[0], 1);
    assert!(m.lock().unwrap().received.is_empty());
    let mut settling = found.take_settling();
    assert_eq!(settling.len(), 1);
    crate::test_clock::advance(clock, SETTLE_US / 2);
    assert!(!settling[0].due());
    let early = settle(&mut settling, None, false);
    assert!(early.failures.is_empty() && early.disks.is_empty() && early.storage.is_empty());
    assert_eq!(settling.len(), 1);
    assert!(m.lock().unwrap().received.is_empty());
    crate::test_clock::advance(clock, SETTLE_US / 2);
    assert!(settling[0].due());
    settle(&mut settling, None, false);
    assert!(settling.is_empty());
    assert_eq!(m.lock().unwrap().received.get(..4), Some(&b"USBC"[..]));
}
