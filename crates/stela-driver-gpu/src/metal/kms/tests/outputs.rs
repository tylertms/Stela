//! Several connected outputs: one picture on every output that runs its
//! size.
use super::*;

pub(super) fn at(width: u16, height: u16, kind: u32, vrefresh: u32) -> ModeInfo {
    ModeInfo {
        vrefresh,
        ..mode(width, height, kind)
    }
}

/// A card with a monitor on each connector: a large one on the first that
/// also runs the second's native size, and the second's.
pub(super) fn two_monitors() -> Card {
    let mut card = Card::new(vec![at(6, 4, PREFERRED, 60), at(4, 2, 0, 60)]);
    card.first = vec![at(8, 4, PREFERRED, 60), at(6, 4, 0, 75), at(4, 2, 0, 60)];
    card.first_connected = true;
    card
}

#[test]
fn the_size_most_outputs_run_wins_before_the_guest_mode_is_set() {
    let large = [at(8, 4, PREFERRED, 60), at(6, 4, 0, 60)];
    let small = [at(6, 4, PREFERRED, 60)];
    let any: Fits<'_> = &|_, _, _| true;
    let native = Wanted::Native((4, 2));
    // Both run 6x4; the second output's preferred size, listed second.
    assert_eq!(plan(&[&large, &small], native, any), Some(((6, 4), 0)));
    // No shared size: the first output's own.
    let other = [at(4, 2, PREFERRED, 60)];
    assert_eq!(plan(&[&large, &other], native, any), Some(((8, 4), 0)));
    assert_eq!(plan(&[&other, &large], native, any), Some(((4, 2), 0)));
    // A shared size the guest cannot be told of loses to one it can.
    let not_6x4: Fits<'_> = &|width, _, _| width != 6;
    assert_eq!(plan(&[&large, &small], native, not_6x4), Some(((8, 4), 0)));
    // The guest's set size wins whenever an output has it, which gives
    // the mode even when it is not the first.
    let size = Wanted::Size((6, 4));
    assert_eq!(plan(&[&other, &small], size, any), Some(((6, 4), 1)));
    assert_eq!(plan(&[&other, &other], size, any), Some(((4, 2), 0)));
    assert_eq!(plan(&[], native, any), None);
}

#[test]
fn two_monitors_show_one_picture_in_a_size_both_run() {
    let mut card = two_monitors();
    let (first, second) = (card.first[1], card.modes[0]);
    let (front, spare);
    {
        let mut display =
            Display::take_fitting(&mut card, Wanted::Native((4, 2)), &|_, _, _| true).unwrap();
        assert_eq!(display.size(), (6, 4));
        assert_eq!((display.outputs(), display.unshown()), (2, 0));
        // The first in DRM order gives the mode the guest is told of.
        assert_eq!(display.refresh_millihertz(), 75_000);
        front = display.buffers[0].framebuffer;
        spare = display.buffers[2].framebuffer;
        display.present(&[7; 96]).unwrap();
        assert!(!display.queued());
    }
    // Each runs its own mode of the size, on its own controller, from the
    // same buffer, and both flip to the frame.
    assert_eq!(card.set_40, Some((first, front)));
    assert_eq!(card.set, Some((second, front)));
    assert_eq!(card.flips_40, [spare]);
    assert_eq!(card.flips, [spare]);
}

#[test]
fn the_output_the_firmware_lit_wins_ties() {
    let mut card = Card::new(vec![at(6, 4, PREFERRED, 50)]);
    card.first = vec![at(8, 4, PREFERRED, 60)];
    card.first_connected = true;
    {
        let display =
            Display::take_fitting(&mut card, Wanted::Native((4, 2)), &|_, _, _| true).unwrap();
        assert_eq!(display.size(), (8, 4));
        assert_eq!((display.outputs(), display.unshown()), (1, 1));
    }
    assert!(card.set_40.is_some() && card.set.is_none());

    let mut lit = Card::new(vec![at(6, 4, PREFERRED, 50)]);
    lit.first = vec![at(8, 4, PREFERRED, 60)];
    lit.first_connected = true;
    lit.lit = 51;
    {
        let display =
            Display::take_fitting(&mut lit, Wanted::Native((4, 2)), &|_, _, _| true).unwrap();
        assert_eq!(display.size(), (6, 4));
        assert_eq!(display.refresh_millihertz(), 50_000);
        assert_eq!((display.outputs(), display.unshown()), (1, 1));
    }
    assert!(lit.set_40.is_none() && lit.set.is_some());

    // Where both share a size, the lit one gives the guest's mode.
    let mut shared = two_monitors();
    shared.lit = 51;
    let display =
        Display::take_fitting(&mut shared, Wanted::Native((4, 2)), &|_, _, _| true).unwrap();
    assert_eq!(display.refresh_millihertz(), 60_000);
    assert_eq!(display.outputs(), 2);
}

#[test]
fn an_output_that_follows_flips_once_its_engine_is_free() {
    let mut card = two_monitors();
    // Controller 41 runs the second output, which follows the first.
    card.busy = 1;
    let spare;
    {
        let mut display = Display::take(&mut card, (6, 4)).unwrap();
        spare = display.buffers[2].framebuffer;
        display.present(&[7; 96]).unwrap();
        assert_eq!(display.buffers[0].framebuffer, spare);
        assert!(display.queued());
        display.drm.immediate = false;
        display.flush().unwrap();
        assert!(display.queued());
        display.drm.immediate = true;
        display.flush().unwrap();
        assert!(!display.queued());
        display.flush().unwrap();
        assert_eq!(display.drm.flip_requests, 3);
    }
    assert_eq!(card.flips_40, [spare]);
    assert_eq!(card.flips, [spare]);
}

#[test]
fn an_output_that_cannot_be_set_stays_dark_and_the_others_keep_the_buffers() {
    let mut card = two_monitors();
    card.refused_crtc = Some(41);
    {
        let display = Display::take(&mut card, (6, 4)).unwrap();
        assert_eq!((display.outputs(), display.unshown()), (1, 1));
    }
    assert!(card.set_40.is_some() && card.set.is_none());
    assert!(card.destroyed.is_empty() && card.removed.is_empty());

    // The output that gives the mode is refused: all three buffers go, and
    // no other output was set.
    let mut first = two_monitors();
    first.refused_crtc = Some(40);
    assert_eq!(
        Display::take(&mut first, (6, 4)).err(),
        Some(Error::Request(-22))
    );
    assert_eq!(first.destroyed.len(), 3);
    assert_eq!(first.removed.len(), 3);
    assert_eq!(first.unmapped, 3);
    assert_eq!(first.set, None);

    // A buffer that cannot be made: the ones made go, nothing is set.
    let mut mapped = two_monitors();
    mapped.map_refusals = 1;
    assert_eq!(Display::take(&mut mapped, (6, 4)).err(), Some(Error::Map));
    assert_eq!(mapped.destroyed.len(), 1);
    assert_eq!(mapped.mode_sets, 0);

    // Outputs that would share an encoder: only the first is shown.
    let mut shared = two_monitors();
    shared.shared_encoder = true;
    shared.possible_crtcs = 0b11;
    let display = Display::take(&mut shared, (6, 4)).unwrap();
    assert_eq!((display.outputs(), display.unshown()), (1, 1));
}

#[test]
fn a_set_guest_size_shows_only_where_an_output_runs_it() {
    let mut card = Card::new(vec![at(8, 4, PREFERRED, 60)]);
    card.first = vec![at(8, 4, PREFERRED, 60), at(6, 4, 0, 60)];
    card.first_connected = true;
    {
        let display =
            Display::take_fitting(&mut card, Wanted::Size((6, 4)), &|_, _, _| true).unwrap();
        assert_eq!(display.size(), (6, 4));
        assert_eq!((display.outputs(), display.unshown()), (1, 1));
    }
    assert!(card.set_40.is_some() && card.set.is_none());
}

#[test]
fn a_monitor_connected_later_joins_the_picture_in_its_size() {
    let mut card = two_monitors();
    card.first_connected = false;
    let joining = card.first[1];
    let (front, spare);
    {
        let mut display = Display::take(&mut card, (6, 4)).unwrap();
        assert_eq!((display.outputs(), display.unshown()), (1, 0));
        front = display.buffers[0].framebuffer;
        let unchanged = Watched::default();
        assert_eq!(display.watch(), unchanged);
        display.drm.first_connected = true;
        let joined = Watched {
            joined: 1,
            ..unchanged
        };
        assert_eq!(display.watch(), joined);
        assert_eq!(display.outputs(), 2);
        assert_eq!(display.drm.set_40, Some((joining, front)));
        assert_eq!(display.watch(), unchanged);
        spare = display.buffers[2].framebuffer;
        display.present(&[7; 96]).unwrap();
        // It is shown now: unplugged and plugged again, its mode is set again.
        display.drm.first_connected = false;
        let lost = Watched {
            lost: 1,
            ..unchanged
        };
        assert_eq!(display.watch(), lost);
        display.drm.first_connected = true;
        let restored = Watched {
            restored: 1,
            ..unchanged
        };
        assert_eq!(display.watch(), restored);
    }
    assert_eq!(card.flips, [spare]);
    assert_eq!(card.flips_40, [spare]);
    assert_eq!(card.set_40, Some((joining, spare)));
}

#[test]
fn a_monitor_connected_later_without_the_size_is_tried_once_per_connection() {
    let mut card = two_monitors();
    card.first = vec![at(8, 4, PREFERRED, 60)];
    card.first_connected = false;
    let mut display = Display::take(&mut card, (6, 4)).unwrap();
    display.drm.first_connected = true;
    let unchanged = Watched::default();
    let unshown = Watched {
        unshown: 1,
        ..unchanged
    };
    assert_eq!(display.watch(), unshown);
    assert_eq!(display.unshown(), 1);
    let probes = display.drm.probes;
    assert_eq!(display.watch(), unchanged);
    assert_eq!(display.drm.probes, probes);
    display.drm.first_connected = false;
    assert_eq!(display.watch(), unchanged);
    assert_eq!(display.unshown(), 0);
    display.drm.first_connected = true;
    assert_eq!(display.watch(), unshown);
    assert_eq!(display.outputs(), 1);
    assert!(display.drm.set_40.is_none());
}
