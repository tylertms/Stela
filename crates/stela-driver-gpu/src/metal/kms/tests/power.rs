//! Display sleep: the outputs off and on again.
use super::*;

#[test]
fn display_sleep_turns_the_output_off_and_a_wake_shows_the_newest_frame() {
    let modes = vec![mode(2, 2, PREFERRED)];
    let mut card = Card::new(modes.clone());
    let (front, hidden);
    {
        let mut display = Display::take(&mut card, (2, 2)).unwrap();
        front = display.buffers[0].framebuffer;
        hidden = display.buffers[2].framebuffer;
        // Where a request must wait, the change waits for a flush.
        display.drm.immediate = false;
        display.set_power(false).unwrap();
        assert!(display.queued() && !display.dark);
        assert_eq!(display.drm.mode_sets, 1);
        display.drm.immediate = true;
        display.flush().unwrap();
        assert!(display.dark && !display.queued());
        assert_eq!(display.drm.set, None, "the controller is off");
        // A frame while the output is off is kept, not flipped.
        display.present(&[7; 16]).unwrap();
        assert!(display.queued());
        assert!(display.drm.flips.is_empty());
        // A returning connector is not lit while the outputs are off.
        assert_eq!(display.watch(), Watched::default());
        assert_eq!(display.drm.set, None);
        // The wake sets the mode on the buffer on screen, then flips the
        // newest frame.
        display.set_power(true).unwrap();
        assert_eq!(display.drm.set, Some((modes[0], front)));
        assert!(display.queued());
        display.flush().unwrap();
        assert!(!display.queued());
        // Asking for the state it is in changes nothing.
        display.set_power(true).unwrap();
    }
    assert_eq!(card.flips, [hidden]);
    assert_eq!(card.mode_sets, 3);
}

#[test]
fn an_output_that_cannot_be_lit_again_is_left_for_the_next_look() {
    let modes = vec![mode(2, 2, PREFERRED)];
    let mut card = Card::new(modes);
    let mut display = Display::take(&mut card, (2, 2)).unwrap();
    display.set_power(false).unwrap();
    display.drm.set_error = Some(-22);
    assert_eq!(display.set_power(true), Err(Error::Request(-22)));
    assert!(!display.dark && display.heads[0].lost);
    // Still connected: the next look sets it again.
    let restored = Watched {
        restored: 1,
        ..Watched::default()
    };
    assert_eq!(display.watch(), restored);
    assert!(display.drm.set.is_some());
}
