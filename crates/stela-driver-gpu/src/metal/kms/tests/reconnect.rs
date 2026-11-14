//! A shown output connected again, perhaps to another monitor.
use super::outputs::{at, two_monitors};
use super::*;

#[test]
fn a_returning_monitor_runs_a_mode_from_its_new_list() {
    let mut card = Card::new(vec![at(2, 2, PREFERRED, 60)]);
    let front;
    {
        let mut display = Display::take(&mut card, (2, 2)).unwrap();
        front = display.buffers[0].framebuffer;
        display.drm.connected = false;
        let lost = Watched {
            lost: 1,
            ..Watched::default()
        };
        assert_eq!(display.watch(), lost);
        // Another monitor: larger, and the display's size only at 75 Hz.
        display.drm.modes = vec![at(4, 2, PREFERRED, 60), at(2, 2, 0, 75)];
        display.drm.connected = true;
        let restored = Watched {
            restored: 1,
            ..Watched::default()
        };
        assert_eq!(display.watch(), restored);
    }
    assert_eq!(card.set, Some((at(2, 2, 0, 75), front)));
}

#[test]
fn a_returning_monitor_without_the_size_is_left_dark_and_counted_once() {
    let mut card = Card::new(vec![at(2, 2, PREFERRED, 60)]);
    let mut display = Display::take(&mut card, (2, 2)).unwrap();
    display.drm.connected = false;
    display.watch();
    display.drm.modes = vec![at(4, 2, PREFERRED, 60)];
    display.drm.connected = true;
    let sets = display.drm.mode_sets;
    // Probed at each of the tries, in case its EDID comes late; counted at
    // the last; the controller is never touched.
    assert_eq!(display.watch(), Watched::default());
    assert_eq!(display.watch(), Watched::default());
    let modeless = Watched {
        modeless: 1,
        ..Watched::default()
    };
    assert_eq!(display.watch(), modeless);
    let probes = display.drm.probes;
    assert_eq!(display.watch(), Watched::default());
    assert_eq!(display.drm.probes, probes);
    assert_eq!(display.drm.mode_sets, sets);
    // Connected again with the size: set.
    display.drm.connected = false;
    assert_eq!(display.watch(), Watched::default());
    display.drm.modes = vec![at(4, 2, PREFERRED, 60), at(2, 2, 0, 60)];
    display.drm.connected = true;
    let restored = Watched {
        restored: 1,
        ..Watched::default()
    };
    assert_eq!(display.watch(), restored);
    assert_eq!(display.drm.set.map(|(mode, _)| mode), Some(at(2, 2, 0, 60)));
}

#[test]
fn an_output_that_cannot_be_set_again_does_not_stop_the_next() {
    let mut card = two_monitors();
    let front;
    {
        let mut display = Display::take(&mut card, (6, 4)).unwrap();
        assert_eq!(display.outputs(), 2);
        front = display.buffers[0].framebuffer;
        display.drm.first_connected = false;
        display.drm.connected = false;
        let lost = Watched {
            lost: 2,
            ..Watched::default()
        };
        assert_eq!(display.watch(), lost);
        display.drm.set = None;
        display.drm.first_connected = true;
        display.drm.connected = true;
        // The first output's controller refuses every request.
        display.drm.refused_crtc = Some(40);
        let refusal = Some(Error::Request(-22));
        let restored = Watched {
            restored: 1,
            error: refusal,
            ..Watched::default()
        };
        assert_eq!(display.watch(), restored);
        let tried = Watched {
            error: refusal,
            ..Watched::default()
        };
        assert_eq!(display.watch(), tried);
        let refused = Watched {
            refused: 1,
            error: refusal,
            ..Watched::default()
        };
        assert_eq!(display.watch(), refused);
        assert_eq!(display.watch(), Watched::default());
    }
    assert_eq!(card.set, Some((at(6, 4, PREFERRED, 60), front)));
}
