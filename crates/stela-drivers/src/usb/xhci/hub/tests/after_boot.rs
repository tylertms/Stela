//! Devices that arrive after boot, polled as the guest's service turns poll
//! them: a hub with input devices behind it, unplugged and replugged as a
//! KVM switch does, a hub port that never powers and a device that stops
//! answering.
use super::super::{model::*, tests::controller, *};
use crate::bus::dma::TestRegion;
use crate::input::Input;
use alloc::collections::BTreeMap;

/// Polls one late arrival takes at most before it settles.
const POLLS: usize = 4000;

/// A controller that booted with an empty root port 1, and its runtime
/// owner, polled after boot.
struct Late {
    model: Rc<RefCell<Hardware>>,
    shared: Shared,
    host: Host,
    dma: TestRegion,
    _clock: crate::test_clock::Guard,
}

/// What every poll since the last look found.
#[derive(Default)]
struct Seen {
    devices: Vec<Device>,
    removed: Vec<Device>,
    inputs: Vec<hid::UsbInput>,
    unused: Vec<(String, &'static str)>,
    failures: Vec<(String, BlockError)>,
}

impl Late {
    fn boot(nodes: BTreeMap<u32, Node>) -> Self {
        let clock = crate::test_clock::virtualize();
        let mut dma = TestRegion::new(1024 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 32, nodes);
        model.borrow_mut().root_status = Some(PP);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        assert!(devices.is_empty() && failures.is_empty(), "{failures:?}");
        let shared = Rc::new(RefCell::new(c));
        let host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        Self {
            model,
            shared,
            host,
            dma,
            _clock: clock,
        }
    }

    fn plug(&mut self) {
        self.model.borrow_mut().root_status = Some(PP | CCS | CSC | u32::from(HIGH_SPEED) << 10);
    }

    fn unplug(&mut self) {
        self.model.borrow_mut().root_status = Some(PP | CSC);
    }

    fn poll(&mut self, seen: &mut Seen) -> bool {
        let firmware = crate::firmware::HandoffProvider::new(None);
        let changes = self.host.poll_after_boot(&mut self.dma, &firmware);
        let mut added = changes.added;
        let any = !(added.devices.is_empty()
            && changes.removed.is_empty()
            && added.inputs.is_empty()
            && added.unused.is_empty()
            && added.failures.is_empty());
        seen.devices.append(&mut added.devices);
        seen.removed.extend(changes.removed);
        seen.inputs.append(&mut added.inputs);
        seen.unused.append(&mut added.unused);
        seen.failures.append(&mut added.failures);
        any
    }

    /// Poll until the host has nothing left to do, a millisecond apart.
    fn settle(&mut self) -> Seen {
        let mut seen = Seen::default();
        for _ in 0..POLLS {
            let any = self.poll(&mut seen);
            if !any && !self.host.due() && !self.host.attachment_due() {
                return seen;
            }
            let clock = self.shared.borrow().clock;
            crate::test_clock::advance(clock, 1000);
        }
        panic!("the late arrival never settled");
    }

    fn enabled_slots(&self) -> usize {
        let c = self.shared.borrow();
        (1..c.slots.len())
            .filter(|&slot| c.slot_ownership[slot] == reset::Ownership::Enabled)
            .count()
    }

    fn report(&self, route: u32, report: &[u8]) {
        let mut model = self.model.borrow_mut();
        let hid = model.nodes.get_mut(&route).unwrap().hid.as_mut().unwrap();
        hid.reports.push_back(report.to_vec());
    }
}

/// A high-speed hub with a full-speed boot keyboard on port 1 and a mouse
/// on port 2.
fn hub_with_input() -> BTreeMap<u32, Node> {
    let mut hub = Node::hub(HIGH_SPEED, 2);
    for port in hub.ports.as_mut().unwrap() {
        port.child_speed = FULL_SPEED;
    }
    BTreeMap::from([(0, hub), (1, Node::hid(1)), (2, Node::hid(2))])
}

fn hub_location(seen: &Seen) -> String {
    let hub = seen.devices.iter().find(|d| d.descriptor[4] == HUB_CLASS);
    hub.expect("no hub").location.clone()
}

fn input_named<'a>(seen: &'a mut Seen, location: &str) -> &'a mut hid::UsbInput {
    seen.inputs
        .iter_mut()
        .find(|input| input.name.ends_with(&format!("{location})")))
        .unwrap_or_else(|| panic!("no input at {location}"))
}

fn poll_input(input: &mut hid::UsbInput) -> Vec<Input> {
    let mut inputs = Vec::new();
    for _ in 0..8 {
        inputs.extend(input.poll());
    }
    inputs
}

/// A hub that arrives after boot with a keyboard and a mouse behind it is
/// brought up, and both input devices start and report.
#[test]
fn a_hub_with_a_keyboard_and_a_mouse_that_arrives_after_boot_starts_both() {
    let mut late = Late::boot(hub_with_input());
    late.plug();
    let mut seen = late.settle();
    assert!(seen.failures.is_empty(), "{:?}", seen.failures);
    assert_eq!(seen.devices.len(), 3, "{:?}", seen.unused);
    assert_eq!(seen.inputs.len(), 2);
    let location = hub_location(&seen);
    late.report(1, &[0, 0, 4, 0, 0, 0, 0, 0]);
    let keyboard = input_named(&mut seen, &format!("{location}.1"));
    assert!(poll_input(keyboard).contains(&Input::Key {
        usage: 4,
        pressed: true
    }));
    late.report(2, &[1, 5, 0xfb]);
    let mouse = input_named(&mut seen, &format!("{location}.2"));
    let inputs = poll_input(mouse);
    assert!(inputs.contains(&Input::MouseButtons(1)), "{inputs:?}");
    assert!(
        inputs.contains(&Input::Motion { dx: 5, dy: -5 }),
        "{inputs:?}"
    );
}

/// The same hub unplugged and plugged in again, as a KVM switch does:
/// every owner of each round is removed with its slot and current, a key
/// held at the unplug is released, and each round starts both devices
/// again.
#[test]
fn a_hub_that_a_kvm_switch_unplugs_and_replugs_leaks_no_owner_or_key() {
    let mut late = Late::boot(hub_with_input());
    for round in 0..5 {
        late.plug();
        let mut seen = late.settle();
        assert!(seen.failures.is_empty(), "{round}: {:?}", seen.failures);
        assert_eq!((seen.devices.len(), seen.inputs.len()), (3, 2), "{round}");
        assert_eq!(late.enabled_slots(), 3);
        let location = hub_location(&seen);
        late.report(1, &[0, 0, 4, 0, 0, 0, 0, 0]);
        let keyboard = input_named(&mut seen, &format!("{location}.1"));
        let pressed = poll_input(keyboard);
        assert!(pressed.contains(&Input::Key {
            usage: 4,
            pressed: true
        }));
        late.unplug();
        let gone = late.settle();
        assert_eq!(gone.removed.len(), 3, "{round}");
        assert_eq!(late.enabled_slots(), 0, "{round}");
        assert!(!late.shared.borrow().power.retained(seen.devices[1].slot));
        let keyboard = input_named(&mut seen, &format!("{location}.1"));
        let released = poll_input(keyboard);
        assert!(
            released.contains(&Input::Key {
                usage: 4,
                pressed: false
            }),
            "{round}: {released:?}"
        );
        assert!(poll_input(keyboard).is_empty());
    }
}

/// A hub port whose power never comes up is dropped after the hub's power
/// on wait, reported once; the keyboard on the other port starts.
#[test]
fn a_hub_port_that_never_powers_is_dropped_and_reported_once() {
    let mut nodes = hub_with_input();
    nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[1].fault = Fault::PowerAbsent;
    let mut late = Late::boot(nodes);
    late.plug();
    let seen = late.settle();
    assert_eq!(seen.failures.len(), 1, "{:?}", seen.failures);
    assert!(seen.failures[0].0.contains(".2 "), "{:?}", seen.failures);
    assert_eq!((seen.devices.len(), seen.inputs.len()), (2, 1));
    let again = late.settle();
    assert!(again.failures.is_empty(), "{:?}", again.failures);
}

/// A device behind the hub that stops answering at any point of its
/// enumeration or start is dropped and reported once, within its
/// transfers' deadlines; the hub and the other device are unaffected.
#[test]
fn a_device_behind_a_hub_that_stops_answering_is_dropped_at_every_step() {
    // The mouse answers its descriptor reads, SET_CONFIGURATION,
    // GET_DESCRIPTOR(report), SET_PROTOCOL and SET_IDLE: seven requests.
    for answers in 0..7 {
        let mut late = Late::boot(hub_with_input());
        late.model.borrow_mut().answers = Some((2, answers));
        late.plug();
        let seen = late.settle();
        assert_eq!(seen.failures.len(), 1, "{answers}: {:?}", seen.failures);
        assert!(seen.failures[0].0.contains(".2"), "{:?}", seen.failures);
        assert_eq!(seen.inputs.len(), 1, "{answers}");
        assert!(seen.inputs[0].name.ends_with(".1)"));
        assert!(late.settle().failures.is_empty(), "{answers}");
    }
}
