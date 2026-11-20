//! Which of the firmware's graphics modes the mount runs the screen in.
//!
//! A boot that keeps the mode the firmware left often runs below the
//! panel's resolution (QEMU's OVMF leaves 1280x800 whatever the display),
//! and the monitor scales that up into large, soft text. Boot managers
//! switch to a larger mode first, as OpenCore's `Resolution=Max` does, and
//! Linux's own display drivers set the panel's. The mount chooses once,
//! while boot services run, and keeps that mode for the whole boot: its
//! console and the guest's display share it, and the mode cannot change
//! after ExitBootServices.
//!
//! [`choose`] takes, among the modes Stela can draw in and the guest's
//! display can run, the panel's native resolution, the preferred timing of
//! its EDID ([`native`]), when the firmware offers it. It never takes a mode
//! larger than one the panel is known to show: without that mode it takes
//! the largest within it that has the panel's shape ([`same_shape`]),
//! because a monitor stretches any other picture over its whole panel (a
//! firmware on a 3840x2160 panel that lists 1920x1440 above 1920x1080
//! showed macOS a third too wide), and only when no mode has that shape the
//! largest of any. Without an EDID it keeps the firmware's mode,
//! which the firmware set up on that panel, because a firmware may list
//! modes the panel cannot show (QEMU's OVMF lists 2048x2048 on any display).
//!
//! The guest's display caps the choice ([`guest_takes`]): the modeled DCP
//! advertises the screen's mode to macOS (`stela_soc::dcp::modes`) with at
//! most 4095 pixels on each axis. 3840x2160 fits; a 5K or 6K panel runs the
//! largest smaller mode its firmware offers.
//!
//! A monitor whose fastest timing needs more than a detailed timing's
//! 655.35 MHz lists it in a DisplayID extension block, and one with no
//! detailed timing at all in its base block (QEMU's generator for such a
//! timing) has its preferred timing only there ([`native`], [`refresh`]).
use core::{cmp::Reverse, fmt};
use stela_soc::scanout::DEFAULT_REFRESH_MILLIHERTZ;

/// One of the firmware's graphics modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mode {
    /// The mode's number, which selects it.
    pub index: u32,
    pub width: u32,
    pub height: u32,
    /// Whether Stela can draw in it: a linear framebuffer of 32-bit pixels.
    pub drawable: bool,
}

impl Mode {
    fn pixels(&self) -> u64 {
        u64::from(self.width) * u64::from(self.height)
    }
}

/// Why [`choose`] took its mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    /// The panel's EDID prefers its resolution.
    Native,
    /// The largest mode of the panel's shape within its native resolution,
    /// which no usable mode has.
    WithinNative,
    /// No usable mode has the panel's shape: the largest within its native
    /// resolution, which the monitor may stretch.
    OtherShape,
    /// No EDID: the firmware's mode.
    Firmware,
    /// No EDID, and the guest cannot run the firmware's mode: the largest
    /// mode within it.
    WithinFirmware,
}

impl fmt::Display for Reason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Reason::Native => "the panel's native resolution",
            Reason::WithinNative => {
                "the largest mode of the panel's shape within its native resolution"
            }
            Reason::OtherShape => {
                "the largest mode within the panel's native resolution; none has the panel's shape, so the monitor may stretch the picture"
            }
            Reason::Firmware => "the firmware's mode, as no EDID names the panel's",
            Reason::WithinFirmware => "the largest mode within the firmware's",
        })
    }
}

/// The EDID base block's fixed header.
const EDID_HEADER: [u8; 8] = [0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00];
const EDID_BLOCK: usize = 128;
/// The first 18-byte detailed timing descriptor, the preferred timing
/// (VESA E-EDID 1.4, section 3.10.1; 1.3 requires it to be the
/// preferred one too).
const PREFERRED_TIMING: usize = 0x36;

/// An EDID's preferred timing descriptor. `None` for a block whose header or
/// checksum is wrong, or whose first descriptor is not a detailed timing
/// (pixel clock 0).
fn preferred_timing(edid: &[u8]) -> Option<&[u8]> {
    let block = base_block(edid)?;
    let d = &block[PREFERRED_TIMING..PREFERRED_TIMING + 18];
    (d[0] != 0 || d[1] != 0).then_some(d)
}

fn base_block(edid: &[u8]) -> Option<&[u8]> {
    let block = edid.get(..EDID_BLOCK)?;
    (block[..8] == EDID_HEADER && block.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) == 0)
        .then_some(block)
}

/// The printable product name in a monitor's EDID base block, if supplied.
pub fn product_name(edid: &[u8]) -> Option<&str> {
    let block = base_block(edid)?;
    block[PREFERRED_TIMING..126]
        .chunks_exact(18)
        .find_map(|descriptor| {
            if descriptor[..5] != [0, 0, 0, 0xfc, 0] {
                return None;
            }
            let text = descriptor[5..]
                .split(|byte| matches!(*byte, b'\n' | 0))
                .next()?;
            if !text
                .iter()
                .all(|byte| byte.is_ascii_graphic() || *byte == b' ')
            {
                return None;
            }
            core::str::from_utf8(text)
                .ok()
                .map(str::trim)
                .filter(|name| !name.is_empty())
        })
}

/// The active width and height of an EDID's preferred timing: the panel's
/// native resolution. Without a detailed timing in the base block, its
/// DisplayID extension's preferred timing ([`displayid_timing`]).
pub fn native(edid: &[u8]) -> Option<(u32, u32)> {
    let Some(d) = preferred_timing(edid) else {
        return displayid_timing(edid).map(|(width, height, _)| (width, height));
    };
    let width = u32::from(d[2]) | u32::from(d[4] >> 4) << 8;
    let height = u32::from(d[5]) | u32::from(d[7] >> 4) << 8;
    (width != 0 && height != 0).then_some((width, height))
}

/// The extension tag of a DisplayID section in an EDID (VESA DisplayID 1.3
/// section 2.2; E-EDID block maps).
const DISPLAYID_EXTENSION: u8 = 0x70;
/// DisplayID 1.3's type I detailed timing data block, its clock in 10 kHz
/// units, and DisplayID 2.0's type VII one, in 1 kHz units: each a list of
/// 20-byte descriptors (Linux `drm_mode_displayid_detailed`).
const TYPE_I_TIMING: u8 = 0x03;
const TYPE_VII_TIMING: u8 = 0x22;
const DISPLAYID_TIMING_BYTES: usize = 20;

/// The first preferred timing, else the first timing, of an EDID's
/// DisplayID extension blocks: `(width, height, refresh in millihertz)`.
/// Each field of a descriptor is stored less one (DisplayID 1.3 section
/// 4.4.1), as Linux reads them. `None` without a valid base block, a
/// DisplayID extension or a timing in it.
fn displayid_timing(edid: &[u8]) -> Option<(u32, u32, u32)> {
    let block = edid.get(..EDID_BLOCK)?;
    if block[..8] != EDID_HEADER || block.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) != 0 {
        return None;
    }
    let mut first = None;
    for extension in edid[EDID_BLOCK..]
        .as_chunks::<EDID_BLOCK>()
        .0
        .iter()
        .take(usize::from(block[126]))
    {
        if extension[0] != DISPLAYID_EXTENSION {
            continue;
        }
        // Section: version, payload bytes, product type, extension count,
        // then the data blocks, each a tag, a revision and its length.
        let end = (5 + usize::from(extension[2])).min(EDID_BLOCK - 1);
        let mut at = 5;
        while at + 3 <= end {
            let (tag, length) = (extension[at], usize::from(extension[at + 2]));
            let payload = extension.get(at + 3..(at + 3 + length).min(end))?;
            let unit_khz = match tag {
                TYPE_I_TIMING => 10,
                TYPE_VII_TIMING => 1,
                _ => {
                    at += 3 + length;
                    continue;
                }
            };
            for d in payload.as_chunks::<DISPLAYID_TIMING_BYTES>().0 {
                let field = |at: usize, mask: u16| {
                    u32::from(u16::from_le_bytes([d[at], d[at + 1]]) & mask) + 1
                };
                let clock_khz =
                    (u64::from(d[0]) | u64::from(d[1]) << 8 | u64::from(d[2]) << 16) + 1;
                let (width, height) = (field(4, 0xffff), field(12, 0xffff));
                let total =
                    u64::from(width + field(6, 0xffff)) * u64::from(height + field(14, 0xffff));
                let Some(refresh) = u32::try_from(clock_khz * unit_khz * 1_000_000 / total)
                    .ok()
                    .filter(|&refresh| refresh != 0)
                else {
                    continue;
                };
                let timing = (width, height, refresh);
                if d[3] & 0x80 != 0 {
                    return Some(timing);
                }
                first = first.or(Some(timing));
            }
            at += 3 + length;
        }
    }
    first
}

/// The smallest image an EDID's size is believed for: a 7" panel's width.
/// Projectors and TVs report 0 or an aspect ratio in centimetres.
const MINIMUM_SIZE_MM: (u32, u32) = (100, 50);

/// The image size in millimetres an EDID reports: its preferred timing's,
/// else the base block's centimetres (E-EDID 1.4 sections 3.6.2 and
/// 3.10.2). `None` when neither is a plausible monitor size.
pub fn physical_size(edid: &[u8]) -> Option<(u32, u32)> {
    let plausible =
        |(w, h): (u32, u32)| (w >= MINIMUM_SIZE_MM.0 && h >= MINIMUM_SIZE_MM.1).then_some((w, h));
    let base = || plausible((u32::from(edid[21]) * 10, u32::from(edid[22]) * 10));
    let Some(d) = preferred_timing(edid) else {
        // A base block with no detailed timing still has its centimetres.
        return displayid_timing(edid).and_then(|_| base());
    };
    plausible((
        u32::from(d[12]) | u32::from(d[14] >> 4) << 8,
        u32::from(d[13]) | u32::from(d[14] & 0xf) << 8,
    ))
    .or_else(base)
}

/// The refresh of an EDID's preferred timing in millihertz, rounded: its
/// pixel clock over its total pixels, active and blanking, a frame (a
/// field, for an interlaced timing, whose descriptor counts field lines).
/// Without a detailed timing in the base block, its DisplayID extension's
/// preferred timing's, truncated. A base block's detailed timing wins over
/// a faster DisplayID one: it is what a firmware's mode of the size runs.
pub fn refresh(edid: &[u8]) -> Option<u32> {
    let Some(d) = preferred_timing(edid) else {
        return displayid_timing(edid).map(|(_, _, refresh)| refresh);
    };
    let clock = u64::from(u16::from_le_bytes([d[0], d[1]])) * 10_000;
    let axis = |active: u8, blank: u8, high: u8| {
        u64::from(u32::from(active) | u32::from(high >> 4) << 8)
            + u64::from(u32::from(blank) | u32::from(high & 0xf) << 8)
    };
    let total = axis(d[2], d[3], d[4]) * axis(d[5], d[6], d[7]);
    (total != 0)
        .then(|| (clock * 1000 + total / 2) / total)
        .and_then(|refresh| u32::try_from(refresh).ok())
        .filter(|&refresh| refresh != 0)
}

/// Whether the guest's display can run a `width` by `height` screen: the
/// DCP model can advertise it at 60 Hz.
pub fn guest_takes(width: u32, height: u32) -> bool {
    guest_takes_at(width, height, DEFAULT_REFRESH_MILLIHERTZ)
}

/// The refresh in millihertz a `width` by `height` screen runs at, given
/// the EDID's preferred timing `native` as width, height and millihertz:
/// that timing's when it is the screen's size and the guest's display can
/// advertise it, else 60 Hz, which every monitor runs a mode it lists at.
pub fn screen_refresh(native: Option<(u32, u32, u32)>, width: u32, height: u32) -> u32 {
    match native {
        Some((w, h, refresh)) if (w, h) == (width, height) && guest_takes_at(w, h, refresh) => {
            refresh
        }
        _ => DEFAULT_REFRESH_MILLIHERTZ,
    }
}

/// Whether the DCP model can advertise a `width` by `height` screen at
/// `refresh` millihertz.
pub fn guest_takes_at(width: u32, height: u32, refresh: u32) -> bool {
    stela_soc::dcp::modes::advertisable(width, height, refresh)
}

/// Whether a `width` by `height` picture has the shape of `panel` within
/// 1%, so 1366x768 counts on a 16:9 panel.
fn same_shape((width, height): (u32, u32), panel: (u32, u32)) -> bool {
    let (a, b) = (
        u64::from(width) * u64::from(panel.1),
        u64::from(height) * u64::from(panel.0),
    );
    a.abs_diff(b) * 100 <= a.max(b)
}

/// The mode to run among `modes`, given the one in use (`current`) and the
/// panel's native resolution when known: that resolution, else the
/// firmware's, or the largest mode within it, one of its shape before any
/// other. Among equals the current mode
/// wins, then the lowest number, so a firmware already in the best mode is
/// left alone. `None` when no such mode is drawable and fits the guest: the
/// caller keeps the current one.
pub fn choose(modes: &[Mode], current: u32, native: Option<(u32, u32)>) -> Option<(Mode, Reason)> {
    let (bound, exact, within) = match native {
        Some(native) => (native, Reason::Native, Reason::WithinNative),
        None => {
            let mode = modes.iter().find(|mode| mode.index == current)?;
            (
                (mode.width, mode.height),
                Reason::Firmware,
                Reason::WithinFirmware,
            )
        }
    };
    let usable = modes.iter().filter(|mode| {
        mode.drawable
            && mode.width <= bound.0
            && mode.height <= bound.1
            && guest_takes(mode.width, mode.height)
    });
    // Lower ranks first: the current mode, then the lowest number.
    let rank = |mode: &Mode| (mode.index != current, mode.index);
    let shaped = |mode: &Mode| same_shape((mode.width, mode.height), bound);
    let mode = usable.min_by_key(|mode| (!shaped(mode), Reverse(mode.pixels()), rank(mode)))?;
    let reason = if (mode.width, mode.height) == bound {
        exact
    } else if shaped(mode) || native.is_none() {
        within
    } else {
        Reason::OtherShape
    };
    Some((*mode, reason))
}

#[cfg(test)]
mod tests {
    use super::*;
    use stela_soc::scanout::{Mode as ScanoutMode, PixelFormat};

    #[test]
    fn monitor_name_requires_a_valid_printable_edid_descriptor() {
        let mut bytes = edid(1920, 1080);
        bytes[72..90].copy_from_slice(b"\0\0\0\xfc\0LED Cinema\n  ");
        reseal(&mut bytes);
        assert_eq!(product_name(&bytes), Some("LED Cinema"));
        assert_eq!(product_name(&bytes[..90]), None);
        bytes[77] = 1;
        assert_eq!(product_name(&bytes), None);
        reseal(&mut bytes);
        assert_eq!(product_name(&bytes), None);
    }

    fn mode(index: u32, width: u32, height: u32) -> Mode {
        Mode {
            index,
            width,
            height,
            drawable: true,
        }
    }

    #[test]
    fn a_monitors_size_comes_from_its_timing_else_its_base_block() {
        let mut e = stela_soc::dcp::modes::edid_sized(
            ScanoutMode {
                width: 3840,
                height: 2160,
                format: PixelFormat::Bgra8888,
            },
            60_000,
            Some((597, 336)),
        )
        .unwrap();
        assert_eq!(physical_size(&e), Some((597, 336)));
        // A timing without a size leaves the base block's centimetres.
        e[54 + 12..54 + 15].fill(0);
        reseal(&mut e);
        assert_eq!(physical_size(&e), Some((600, 340)));
        // A projector's aspect ratio is no size.
        e[21] = 16;
        e[22] = 0;
        reseal(&mut e);
        assert_eq!(physical_size(&e), None);
        assert_eq!(physical_size(&e[..64]), None);
    }

    #[test]
    fn a_mode_of_the_panels_shape_wins_over_a_larger_one_the_monitor_would_stretch() {
        // A 3840x2160 panel whose firmware lists 4:3 modes above 16:9 ones.
        let modes = [
            mode(0, 1920, 1440),
            mode(1, 1600, 1200),
            mode(2, 1920, 1080),
            mode(3, 1366, 768),
            mode(4, 1920, 1200),
        ];
        assert_eq!(
            choose(&modes, 2, Some((3840, 2160))),
            Some((modes[2], Reason::WithinNative))
        );
        // 1366x768 is 16:9 within a percent.
        assert_eq!(
            choose(&modes[3..], 4, Some((3840, 2160))),
            Some((modes[3], Reason::WithinNative))
        );
        // Without a mode of that shape the largest runs, and says so.
        assert_eq!(
            choose(&modes[..2], 1, Some((3840, 2160))),
            Some((modes[0], Reason::OtherShape))
        );
        // A 16:10 panel takes its own shape too.
        assert_eq!(
            choose(&modes, 0, Some((2560, 1600))),
            Some((modes[4], Reason::WithinNative))
        );
    }

    fn reseal(e: &mut [u8; 128]) {
        e[127] = 0;
        e[127] = 0u8.wrapping_sub(e.iter().fold(0u8, |a, b| a.wrapping_add(*b)));
    }

    /// A base block whose preferred timing is `width` by `height`, built by
    /// the DCP model's own EDID encoder.
    fn edid(width: u32, height: u32) -> [u8; 128] {
        stela_soc::dcp::modes::edid(
            ScanoutMode {
                width,
                height,
                format: PixelFormat::Bgra8888,
            },
            60_000,
        )
        .unwrap()
    }

    #[test]
    fn the_refresh_is_the_preferred_timings() {
        assert_eq!(refresh(&edid(1920, 1080)), Some(60_000));
        let fast = stela_soc::dcp::modes::edid(
            ScanoutMode {
                width: 2560,
                height: 1440,
                format: PixelFormat::Bgra8888,
            },
            144_000,
        )
        .unwrap();
        assert_eq!(refresh(&fast), Some(144_000));
        let mut bad = fast;
        bad[127] ^= 1;
        assert_eq!(refresh(&bad), None);
    }

    /// QEMU 10.2's EDID for its standard VGA at `xres=2560,yres=1440,
    /// refresh_rate=240000`, read from the device's EDID registers: a base
    /// block with no detailed timing (its clock passes 655.35 MHz), a CTA
    /// block and a DisplayID 1.3 block with one preferred type I timing.
    fn qemu_large(clock: [u8; 3]) -> alloc::vec::Vec<u8> {
        let mut e = alloc::vec![0u8; 3 * EDID_BLOCK];
        e[..8].copy_from_slice(&EDID_HEADER);
        e[8..24].copy_from_slice(&[
            0x49, 0x14, 0x34, 0x12, 0, 0, 0, 0, 0x2a, 0x18, 1, 4, 0xa5, 0x41, 0x24, 0x78,
        ]);
        // Established timings III, no detailed timing.
        e[0x36..0x3c].copy_from_slice(&[0, 0, 0, 0xf7, 0, 0x0a]);
        e[126] = 2;
        e[127] = 0u8.wrapping_sub(e[..127].iter().fold(0u8, |a, b| a.wrapping_add(*b)));
        e[128..132].copy_from_slice(&[0x02, 0x03, 0x0b, 0x00]);
        let did = &mut e[256..];
        did[..8].copy_from_slice(&[0x70, 0x13, 0x17, 0x03, 0x00, 0x03, 0x00, 0x14]);
        did[8..11].copy_from_slice(&clock);
        did[11..28].copy_from_slice(&[
            0x88, 0xff, 0x09, 0x7f, 0x03, 0x7f, 0x02, 0x4b, 0x00, 0x9f, 0x05, 0x31, 0x00, 0x06,
            0x00, 0x06, 0x00,
        ]);
        e
    }

    #[test]
    fn a_base_block_without_a_detailed_timing_takes_its_displayid_timing() {
        // 1235.86 MHz over 3456 by 1490: 240 Hz (QEMU stores the clock
        // without the less-one, so it reads 10 kHz fast: 240.0009 Hz).
        let e = qemu_large([0xc2, 0xe2, 0x01]);
        assert_eq!(native(&e), Some((2560, 1440)));
        assert_eq!(refresh(&e), Some(240_000));
        assert_eq!(physical_size(&e), Some((650, 360)));
        assert!(guest_takes_at(2560, 1440, 240_000));
        // A type VII block counts kilohertz.
        let mut seven = qemu_large([0x99, 0xdb, 0x12]);
        seven[256 + 5] = TYPE_VII_TIMING;
        assert_eq!(refresh(&seven), Some(240_000));
        // A detailed timing in the base block wins: the firmware runs it.
        let mut both = qemu_large([0xc2, 0xe2, 0x01]);
        both[..128].copy_from_slice(
            &stela_soc::dcp::modes::edid(
                ScanoutMode {
                    width: 2560,
                    height: 1440,
                    format: PixelFormat::Bgra8888,
                },
                60_000,
            )
            .unwrap(),
        );
        both[126] = 2;
        both[127] = 0;
        both[127] = 0u8.wrapping_sub(both[..128].iter().fold(0u8, |a, b| a.wrapping_add(*b)));
        assert_eq!(refresh(&both), Some(60_000));
        // No extension, or a broken base block: nothing.
        assert_eq!(refresh(&e[..128]), None);
        let mut bad = e.clone();
        bad[127] ^= 1;
        assert_eq!(native(&bad), None);
    }

    #[test]
    fn the_native_resolution_is_the_preferred_timing() {
        assert_eq!(native(&edid(2560, 1440)), Some((2560, 1440)));
        assert_eq!(native(&edid(3840, 2160)), Some((3840, 2160)));
        let mut bad = edid(1920, 1080);
        bad[127] ^= 1;
        assert_eq!(native(&bad), None, "checksum");
        let mut bad = edid(1920, 1080);
        bad[0] = 1;
        assert_eq!(native(&bad), None, "header");
        let mut descriptor = edid(1920, 1080);
        descriptor[0x36..0x38].fill(0);
        assert_eq!(native(&descriptor), None, "display descriptor");
        assert_eq!(native(&edid(1920, 1080)[..127]), None, "short");
    }

    #[test]
    fn the_guest_takes_up_to_4k() {
        assert!(guest_takes(1920, 1080));
        assert!(guest_takes(3840, 2160));
        assert!(!guest_takes(4096, 2160));
        assert!(!guest_takes(5120, 2880));
        assert!(!guest_takes(0, 768));
    }

    #[test]
    fn the_native_mode_wins_and_nothing_larger_is_taken() {
        let modes = [mode(0, 800, 600), mode(1, 1920, 1080), mode(2, 2560, 1600)];
        assert_eq!(
            choose(&modes, 0, Some((1920, 1080))),
            Some((modes[1], Reason::Native))
        );
        // A native resolution the firmware does not offer.
        assert_eq!(
            choose(&modes, 0, Some((1366, 768))),
            Some((modes[0], Reason::OtherShape))
        );
        assert_eq!(choose(&modes, 0, Some((640, 480))), None);
    }

    #[test]
    fn without_an_edid_the_firmware_mode_stays() {
        let modes = [mode(0, 1280, 800), mode(1, 2048, 2048), mode(2, 1024, 768)];
        assert_eq!(choose(&modes, 0, None), Some((modes[0], Reason::Firmware)));
        // A firmware mode the guest cannot run gives way to one within it.
        let modes = [
            mode(0, 5120, 2880),
            mode(1, 3840, 2160),
            mode(2, 6016, 3384),
        ];
        assert_eq!(
            choose(&modes, 0, None),
            Some((modes[1], Reason::WithinFirmware))
        );
        // An unlisted firmware mode leaves the choice to the caller.
        assert_eq!(choose(&modes, 9, None), None);
    }

    #[test]
    fn modes_stela_cannot_draw_or_the_guest_cannot_run_are_skipped() {
        let blt_only = Mode {
            drawable: false,
            ..mode(1, 2560, 1440)
        };
        let modes = [mode(0, 1024, 768), blt_only, mode(2, 5120, 2880)];
        assert_eq!(
            choose(&modes, 0, Some((5120, 2880))),
            Some((modes[0], Reason::OtherShape))
        );
        assert_eq!(choose(&[blt_only], 1, None), None);
    }

    #[test]
    fn ties_keep_the_current_mode_then_the_lowest_number() {
        let modes = [
            mode(0, 1920, 1080),
            mode(1, 1080, 1920),
            mode(2, 1920, 1080),
        ];
        let native = Some((1920, 1080));
        assert_eq!(choose(&modes, 2, native), Some((modes[2], Reason::Native)));
        assert_eq!(choose(&modes, 1, native), Some((modes[0], Reason::Native)));
        assert_eq!(choose(&modes, 7, native), Some((modes[0], Reason::Native)));
        assert_eq!(
            choose(&modes, 1, Some((1920, 1920))),
            Some((modes[1], Reason::OtherShape))
        );
    }

    #[test]
    fn a_screen_runs_at_its_edids_preferred_rate_in_its_native_size() {
        let native = Some((1600, 900, 240_000));
        assert_eq!(screen_refresh(native, 1600, 900), 240_000);
        assert_eq!(
            screen_refresh(Some((2560, 1440, 239_970)), 2560, 1440),
            239_970
        );
        assert_eq!(
            screen_refresh(native, 1280, 800),
            DEFAULT_REFRESH_MILLIHERTZ
        );
        assert_eq!(screen_refresh(None, 1600, 900), DEFAULT_REFRESH_MILLIHERTZ);
        // A timing the guest's display cannot publish is not run.
        assert_eq!(
            screen_refresh(Some((5120, 2880, 60_000)), 5120, 2880),
            DEFAULT_REFRESH_MILLIHERTZ
        );
    }
}
