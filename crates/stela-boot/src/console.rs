//! Text on the firmware's linear framebuffer, so a PC without a serial port
//! still shows the mount's progress, from the startup choice until the
//! guest's display takes the screen.
//!
//! The text sits under Stela's header ([`ui`]), in its colours. Glyphs come
//! from one 8x16 bitmap font drawn at the pages' [`ui::scale`]. The screen
//! runs in the panel's native mode where the firmware offers it ([`mode`]),
//! so each font pixel is whole panel pixels instead of ones the monitor
//! scales up, as in OpenCore's console. The text is a grid of cells kept in
//! memory, and drawing writes only the cells that changed: video memory is
//! write-combining and slow to read back, so scrolling redraws from the grid
//! instead of moving pixels.
pub(crate) mod font;
mod mode;

pub(crate) fn monitor_audio_name() -> Option<alloc::string::String> {
    mode::monitor_name()
}

use crate::ui;
use alloc::{string::String, vec, vec::Vec};
use core::{
    arch::x86_64::_rdtsc,
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, Ordering},
};
use font::Font;
use stela_boot_core::screen::{Access, Permit};
use stela_soc::machine::ScanoutMasks;
use uefi::{
    Handle,
    boot::{self, OpenProtocolAttributes, OpenProtocolParams, ScopedProtocol},
    proto::console::gop::{GraphicsOutput, ModeInfo, PixelFormat},
};

const BYTES_PER_PIXEL: usize = 4;
/// The most columns of text: wider than the pages' column, as log lines
/// are long.
const COLUMNS: usize = 140;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Color {
    Text,
    Dim,
    Good,
    Warn,
    Fatal,
    Busy,
}

/// How a GOP mode lays out a 32-bit pixel.
#[derive(Debug, Clone, Copy)]
enum Encoding {
    /// `PixelBlueGreenRedReserved8BitPerColor`: blue in the low byte, as the
    /// pixels Stela draws and presents already are.
    Bgr,
    /// `PixelRedGreenBlueReserved8BitPerColor`: red in the low byte.
    Rgb,
    /// `PixelBitMask`: the layout its channel masks give.
    Masks(ScanoutMasks),
}

impl Encoding {
    /// The mode's pixel for one with blue in the low byte.
    fn encode(self, pixel: u32) -> u32 {
        match self {
            Encoding::Bgr => pixel,
            Encoding::Rgb => pixel & 0xff00_ff00 | (pixel & 0xff) << 16 | (pixel >> 16) & 0xff,
            Encoding::Masks(masks) => {
                let [blue, green, red, _] = pixel.to_le_bytes();
                masks.encode(red, green, blue)
            }
        }
    }
}

/// A mode's pixel layout, when Stela can draw in it.
fn encoding(mode: &ModeInfo) -> Option<Encoding> {
    match (mode.pixel_format(), mode.pixel_bitmask()) {
        (PixelFormat::Rgb, _) => Some(Encoding::Rgb),
        (PixelFormat::Bgr, _) => Some(Encoding::Bgr),
        (PixelFormat::Bitmask, Some(masks)) => {
            ScanoutMasks::new(masks.red, masks.green, masks.blue)
                .ok()
                .map(Encoding::Masks)
        }
        (PixelFormat::Bitmask | PixelFormat::BltOnly, _) => None,
    }
}

/// The first GOP, read-only (see [`Framebuffer::from_firmware`]), and its
/// handle.
fn gop() -> Option<(Handle, ScopedProtocol<GraphicsOutput>)> {
    let handle = boot::get_handle_for_protocol::<GraphicsOutput>().ok()?;
    // SAFETY: the protocol is only used while boot services run.
    let gop = unsafe {
        boot::open_protocol::<GraphicsOutput>(
            OpenProtocolParams {
                handle,
                agent: boot::image_handle(),
                controller: None,
            },
            OpenProtocolAttributes::GetProtocol,
        )
    }
    .ok()?;
    Some((handle, gop))
}

static FIRMWARE_ACCESS: Access = Access::new();
/// The display refused to show its second frame: every frame goes to the
/// first.
static FLIP_REFUSED: AtomicBool = AtomicBool::new(false);

struct ScreenUse(Option<Permit<'static>>);

impl Drop for ScreenUse {
    fn drop(&mut self) {
        if self.0.is_some() {
            // SAFETY: SFENCE is available on x86-64. Complete this CPU's
            // write-combining stores before releasing its writer permit.
            unsafe { core::arch::asm!("sfence", options(nostack, preserves_flags)) };
        }
    }
}

/// The GOP mode in use when Stela takes over.
#[derive(Debug, Clone, Copy)]
pub struct Framebuffer {
    pub base: usize,
    pub bytes: usize,
    width: usize,
    height: usize,
    stride: usize,
    encoding: Encoding,
    firmware: bool,
    /// The monitor's refresh in millihertz.
    refresh_millihertz: u32,
    /// The monitor's image size in millimetres, from its EDID.
    size_mm: Option<(u32, u32)>,
    /// The monitor's identity, from its EDID.
    monitor: Option<stela_soc::machine::MonitorId>,
    /// The address of the registers that choose which of two frames in
    /// video memory shows, the one at `base` or the one after it, when the
    /// display has them (a virtual display's Bochs DISPI registers).
    pub(crate) flip: Option<usize>,
    pub(crate) pci_source: Option<crate::pci_io::FramebufferSource>,
}

impl Framebuffer {
    /// The first GOP's mode, when it is a linear 32-bit framebuffer: RGB,
    /// BGR or channel masks (`scanout::ChannelMasks`). The first call
    /// switches the GOP to the best mode ([`mode`]). A `PixelBltOnly` mode
    /// has no framebuffer to draw in once boot services end, so it
    /// gives none, as do masks that `ChannelMasks` refuses. Boot managers
    /// keep their own handle open, so this only reads the protocol rather
    /// than opening it exclusively.
    pub fn from_firmware() -> Option<Self> {
        Self::firmware_mode().map(|(_, screen)| screen)
    }

    fn firmware_mode() -> Option<(Handle, Self)> {
        let (handle, mut gop) = gop()?;
        mode::select(handle, &mut gop);
        Some((handle, Self::current(&mut gop)?))
    }

    /// Capture the original display function before the engine handoff.
    pub(crate) fn for_engine() -> crate::Result<Option<Self>> {
        let Some((handle, mut screen)) = Self::firmware_mode() else {
            return Ok(None);
        };
        match crate::pci_io::framebuffer_source(handle, screen.base as u64, screen.bytes as u64) {
            Ok(source) => screen.pci_source = source,
            Err(crate::pci_io::Error::Release(status)) => {
                return Err(
                    stela_contracts::refuse!("GOP PCI resource buffer remains retained: {status}", status = ?status),
                );
            }
            Err(error) => crate::output::arguments(format_args!(
                "GOP PCI source observation refused: {error}\n"
            )),
        }
        let frame = screen.frame_bytes();
        if let Some(registers) = crate::pci_io::bochs_display_flip(
            handle,
            screen.base as u64,
            screen.stride,
            frame as u64,
        ) {
            screen.flip = Some(registers as usize);
            screen.bytes = screen.bytes.max(2 * frame);
            crate::output::arguments(format_args!(
                "DISPLAY presents into a hidden second frame and flips to it\n"
            ));
        }
        Ok(Some(screen))
    }

    /// `gop`'s current mode, when Stela can draw in it.
    fn current(gop: &mut GraphicsOutput) -> Option<Self> {
        let mode = gop.current_mode_info();
        let encoding = encoding(&mode)?;
        let (width, height) = mode.resolution();
        let mut buffer = gop.frame_buffer();
        let framebuffer = Self {
            base: buffer.as_mut_ptr() as usize,
            bytes: buffer.size(),
            width,
            height,
            stride: mode.stride(),
            encoding,
            firmware: true,
            refresh_millihertz: mode::refresh(width, height),
            size_mm: mode::size_mm(),
            monitor: mode::monitor(),
            flip: None,
            pci_source: None,
        };
        (framebuffer.stride * framebuffer.height * BYTES_PER_PIXEL <= framebuffer.bytes)
            .then_some(framebuffer)
    }

    /// A blue-low `width` by `height` screen at `base`, `stride` pixels a row.
    #[cfg(feature = "nvk")]
    pub fn linear(base: usize, width: usize, height: usize, stride: usize) -> Self {
        Self {
            base,
            bytes: stride * height * BYTES_PER_PIXEL,
            width,
            height,
            stride,
            encoding: Encoding::Bgr,
            firmware: false,
            refresh_millihertz: stela_soc::scanout::DEFAULT_REFRESH_MILLIHERTZ,
            size_mm: None,
            monitor: None,
            flip: None,
            pci_source: None,
        }
    }

    /// The same mode reached at another address (the engine's physmap,
    /// which moves its flip registers by as much).
    pub fn relocated(self, base: usize) -> Self {
        Self {
            base,
            flip: self
                .flip
                .map(|registers| registers.wrapping_add(base.wrapping_sub(self.base))),
            ..self
        }
    }

    /// The bytes of one frame in video memory.
    fn frame_bytes(&self) -> usize {
        self.stride * self.height * BYTES_PER_PIXEL
    }

    /// Visible pixels across and down.
    pub fn resolution(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// The monitor's refresh in millihertz: its EDID's for its native mode,
    /// else 60 Hz.
    pub fn refresh_millihertz(&self) -> u32 {
        self.refresh_millihertz
    }

    /// The monitor's image size in millimetres, when its EDID gave one.
    pub fn size_mm(&self) -> Option<(u32, u32)> {
        self.size_mm
    }

    /// The monitor's identity, when the firmware gave its EDID.
    pub fn monitor(&self) -> Option<stela_soc::machine::MonitorId> {
        self.monitor
    }

    pub(crate) fn available(&self) -> bool {
        !self.firmware || !FIRMWARE_ACCESS.is_retired()
    }

    fn use_screen(&self) -> Option<ScreenUse> {
        if self.firmware {
            FIRMWARE_ACCESS
                .enter()
                .map(|permit| ScreenUse(Some(permit)))
        } else {
            Some(ScreenUse(None))
        }
    }

    /// Present one complete frame with a single permit, including the
    /// console's transfer to guest scanout. With two frames in video memory
    /// it fills the hidden one and then shows it, so the display never reads
    /// a frame half written.
    pub(crate) fn show_frame(&self, rows: &[u8], before: impl FnOnce() -> bool) -> bool {
        let Some(_access) = self.use_screen() else {
            return false;
        };
        let row_bytes = self.width * BYTES_PER_PIXEL;
        if row_bytes == 0 || !before() {
            return false;
        }
        let hidden = self.hidden_frame();
        let first = hidden.map_or(0, |lines| lines * self.stride * BYTES_PER_PIXEL);
        for (y, pixels) in rows.chunks_exact(row_bytes).take(self.height).enumerate() {
            self.write_row(self.base + first, y, pixels);
        }
        if let Some(lines) = hidden {
            let shown = self.show_lines_from(lines);
            if shown != u16::try_from(lines).ok() {
                // The display kept a frame: present into the one it shows.
                FLIP_REFUSED.store(true, Ordering::Relaxed);
                crate::output::arguments(format_args!(
                    "DISPLAY refused the flip to its second frame; presents into the one it shows\n"
                ));
                let shown = shown.map_or(0, usize::from);
                let at = if shown == self.height { shown } else { 0 };
                for (y, pixels) in rows.chunks_exact(row_bytes).take(self.height).enumerate() {
                    self.write_row(self.base + at * self.stride * BYTES_PER_PIXEL, y, pixels);
                }
            }
        }
        true
    }

    /// Which of the display's two frames is not on screen, when it has two:
    /// 0 at `base`, 1 after it.
    pub(crate) fn hidden_half(&self) -> Option<usize> {
        self.hidden_frame().map(|lines| usize::from(lines != 0))
    }

    /// Fill rows `lines` of the hidden frame `half` from `fill`, through the
    /// RAM row `row`. Nothing is shown: [`Self::show_half`] shows the frame
    /// once every row it lacks is filled, by this call or later ones.
    pub(crate) fn fill_rows(
        &self,
        half: usize,
        lines: core::ops::Range<usize>,
        row: &mut [u8],
        fill: &mut dyn FnMut(usize, &mut [u8]) -> crate::Result<()>,
    ) -> crate::Result<()> {
        let Some(_access) = self.use_screen() else {
            return Err(stela_contracts::refuse!(
                "firmware framebuffer access was refused"
            ));
        };
        let frame = self.height * self.stride * BYTES_PER_PIXEL;
        for y in lines.start..lines.end.min(self.height) {
            fill(y, row)?;
            self.write_row(self.base + half * frame, y, row);
        }
        Ok(())
    }

    /// Show the hidden frame `half`, which [`Self::fill_rows`] filled, once
    /// `before` has run. A display that refuses the flip gets the whole
    /// frame, from `fill` through the RAM row `row`, in the one it shows,
    /// as [`Self::show_frame`] does.
    pub(crate) fn show_half(
        &self,
        half: usize,
        row: &mut [u8],
        fill: &mut dyn FnMut(usize, &mut [u8]) -> crate::Result<()>,
        before: impl FnOnce() -> bool,
    ) -> crate::Result<()> {
        let refused = || stela_contracts::refuse!("firmware framebuffer access was refused");
        let Some(_access) = self.use_screen() else {
            return Err(refused());
        };
        if !before() {
            return Err(refused());
        }
        let lines_from = half * self.height;
        let shown = self.show_lines_from(lines_from);
        if shown != u16::try_from(lines_from).ok() {
            // The display kept a frame: present into the one it shows.
            FLIP_REFUSED.store(true, Ordering::Relaxed);
            crate::output::arguments(format_args!(
                "DISPLAY refused the flip to its second frame; presents into the one it shows\n"
            ));
            let shown = shown.map_or(0, usize::from);
            let at = if shown == self.height { shown } else { 0 };
            for y in 0..self.height {
                fill(y, row)?;
                self.write_row(self.base + at * self.stride * BYTES_PER_PIXEL, y, row);
            }
        }
        Ok(())
    }

    /// The first line of the frame not on screen, when the display has two.
    fn hidden_frame(&self) -> Option<usize> {
        if FLIP_REFUSED.load(Ordering::Relaxed) {
            return None;
        }
        let shown = usize::from(self.dispi(crate::pci_io::DISPI_Y_OFFSET, None)?);
        Some(if shown == 0 { self.height } else { 0 })
    }

    /// Show the frame from line `lines`, once this CPU's stores to it are
    /// complete. The first line the display then shows.
    fn show_lines_from(&self, lines: usize) -> Option<u16> {
        // SAFETY: SFENCE is available on x86-64. The frame's write-combining
        // stores complete before the register write that shows it.
        unsafe { core::arch::asm!("sfence", options(nostack, preserves_flags)) };
        self.dispi(
            crate::pci_io::DISPI_Y_OFFSET,
            Some(u16::try_from(lines).ok()?),
        )
    }

    /// Write DISPI register `index` when given a value, then read it.
    fn dispi(&self, index: u64, value: Option<u16>) -> Option<u16> {
        let register = (self.flip? + index as usize * 2) as *mut u16;
        // SAFETY: `flip` is the uncached mapping of the display's DISPI
        // registers, 16-bit each, that `bochs_display_flip` observed.
        unsafe {
            if let Some(value) = value {
                register.write_volatile(value);
            }
            Some(register.read_volatile())
        }
    }

    /// Row `y` from 32-bit pixels with blue in the low byte, as many as fit.
    pub fn show_row(&self, y: usize, pixels: &[u8]) {
        if y >= self.height {
            return;
        }
        let Some(_access) = self.use_screen() else {
            return;
        };
        self.write_row(self.base, y, pixels);
    }

    /// Row `y` of the frame at `base`. A blue-low mode takes the pixels as
    /// they are, in one copy whose wide stores combine; the screen use's
    /// closing SFENCE, opaque to the compiler, keeps them.
    fn write_row(&self, base: usize, y: usize, pixels: &[u8]) {
        let line = (base + y * self.stride * BYTES_PER_PIXEL) as *mut u32;
        let (pixels, _) = pixels.as_chunks::<BYTES_PER_PIXEL>();
        let pixels = &pixels[..pixels.len().min(self.width)];
        if let Encoding::Bgr = self.encoding {
            // SAFETY: at most `width` pixels of row `y < height`, inside one
            // of the mode's frames in video memory, which no Rust reference
            // covers.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    pixels.as_ptr().cast::<u8>(),
                    line.cast::<u8>(),
                    pixels.len() * BYTES_PER_PIXEL,
                )
            };
            return;
        }
        for (column, pixel) in pixels.iter().enumerate() {
            let value = self.encoding.encode(u32::from_le_bytes(*pixel));
            // SAFETY: `column < width` and `y < height`, inside one of the
            // mode's frames in video memory.
            unsafe { line.add(column).write_volatile(value) };
        }
    }

    /// Pixels per row in memory, at least the width.
    #[cfg(feature = "diagnostics")]
    pub fn stride(&self) -> usize {
        self.stride
    }

    /// The mode's pixel layout, as GOP names it.
    #[cfg(feature = "diagnostics")]
    pub fn format(&self) -> &'static str {
        match self.encoding {
            Encoding::Bgr => "BGR",
            Encoding::Rgb => "RGB",
            Encoding::Masks(_) => "bitmask",
        }
    }

    /// The same mode with only its top `height` rows, so that text written
    /// to it leaves the rows below for something else.
    #[cfg(feature = "diagnostics")]
    pub fn top(self, height: usize) -> Self {
        Self {
            height: height.min(self.height),
            ..self
        }
    }

    fn pixel(&self, color: Color) -> u32 {
        self.encoding.encode(match color {
            Color::Text => ui::TEXT,
            Color::Dim => ui::DIM,
            Color::Good => ui::BLUE,
            Color::Warn | Color::Busy => ui::ORANGE,
            Color::Fatal => ui::RED,
        })
    }

    /// The pixel value of a colour in this mode's layout.
    #[cfg(feature = "diagnostics")]
    pub fn rgb(&self, red: u8, green: u8, blue: u8) -> u32 {
        let (red, green, blue) = (u32::from(red), u32::from(green), u32::from(blue));
        self.encoding.encode((red << 16) | (green << 8) | blue)
    }

    /// Fill `width` by `height` pixels from (`x`, `y`), cut to the mode's
    /// visible rows and columns.
    pub fn rectangle(&self, x: usize, y: usize, width: usize, height: usize, value: u32) {
        let Some(_access) = self.use_screen() else {
            return;
        };
        for row in y..(y + height).min(self.height) {
            let line = (self.base + row * self.stride * BYTES_PER_PIXEL) as *mut u32;
            for column in x..(x + width).min(self.width) {
                // SAFETY: inside the mode's visible framebuffer.
                unsafe { line.add(column).write_volatile(value) };
            }
        }
    }

    /// The pixel at (`x`, `y`), which is visible.
    #[cfg(feature = "diagnostics")]
    pub fn read(&self, x: usize, y: usize) -> Option<u32> {
        assert!(x < self.width && y < self.height, "pixel outside the mode");
        let _access = self.use_screen()?;
        let line = (self.base + y * self.stride * BYTES_PER_PIXEL) as *const u32;
        // SAFETY: inside the mode's visible framebuffer, as checked.
        Some(unsafe { line.add(x).read_volatile() })
    }

    /// Stela's background under its header.
    fn clear(&self) {
        // Text is drawn in the first frame: show it.
        if self.flip.is_some() {
            let _ = self.dispi(crate::pci_io::DISPI_Y_OFFSET, Some(0));
        }
        let background = self.encoding.encode(ui::BACKGROUND);
        self.rectangle(0, 0, self.width, self.height, background);
        if let Some(font) = font::console() {
            let mut header = ui::Canvas::header_band(*self, font);
            header.header();
            header.show();
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct Cell {
    character: u8,
    color: Color,
}

const BLANK: Cell = Cell {
    character: b' ',
    color: Color::Text,
};

/// The text grid and what the screen shows of it.
struct Text {
    screen: Framebuffer,
    font: Font,
    /// Screen pixels per font pixel.
    scale: usize,
    /// The screen pixel the grid starts at, across and down.
    left: usize,
    top: usize,
    columns: usize,
    rows: usize,
    cells: Vec<Cell>,
    shown: Vec<Cell>,
    column: usize,
    row: usize,
    /// A bounded present left cells undrawn ([`present_some`]).
    pending: bool,
}

impl Text {
    fn new(screen: Framebuffer) -> Option<Self> {
        let (width, height) = screen.resolution();
        let font = font::console()?;
        let scale = ui::scale(height);
        let top = ui::HEADER * scale;
        // A centred column, at least a cell from either side, over a margin
        // of a cell below.
        let columns = (width / (font.width * scale)).checked_sub(2)?.min(COLUMNS);
        let left = (width - columns * font.width * scale) / 2;
        let rows = (height.checked_sub(top)? / (font.height * scale)).checked_sub(1)?;
        (columns > 0 && rows > 0).then(|| Self {
            screen,
            font,
            scale,
            left,
            top,
            columns,
            rows,
            cells: vec![BLANK; columns * rows],
            shown: vec![BLANK; columns * rows],
            column: 0,
            row: 0,
            pending: false,
        })
    }

    fn newline(&mut self) {
        self.column = 0;
        if self.row + 1 < self.rows {
            self.row += 1;
            return;
        }
        self.cells.copy_within(self.columns.., 0);
        let last = self.cells.len() - self.columns;
        self.cells[last..].fill(BLANK);
    }

    /// Carry on at `indent` on a blank row under the cursor's.
    fn wrap(&mut self, indent: usize) {
        self.newline();
        let start = self.row * self.columns;
        self.cells[start..start + self.columns].fill(BLANK);
        self.column = indent.min(self.columns - 1);
    }

    fn put(&mut self, character: u8, color: Color) {
        match character {
            b'\n' => self.newline(),
            b'\r' => self.column = 0,
            _ => {
                if self.column == self.columns {
                    self.newline();
                }
                self.cells[self.row * self.columns + self.column] = Cell {
                    character: printable(character),
                    color,
                };
                self.column += 1;
            }
        }
    }

    /// Draw every cell that differs from what the screen shows.
    /// Draw the cells that changed: all of them, or with a `bound` of TSC
    /// ticks, what that allows and at least one, leaving the rest pending
    /// ([`present_some`]).
    fn present(&mut self) {
        self.present_within(0);
    }

    /// Replace the cursor's line with `segments`, cut at the edge.
    fn rewrite(&mut self, segments: &[(&str, Color)]) {
        let start = self.row * self.columns;
        self.cells[start..start + self.columns].fill(BLANK);
        self.column = 0;
        for (part, color) in segments {
            for character in part.bytes().filter(|&c| c != b'\n') {
                if self.column == self.columns {
                    break;
                }
                self.put(character, *color);
            }
        }
    }

    /// Put `lines` on the rows under the cursor's ([`below`]).
    fn below(&mut self, lines: &[String], indent: usize, color: Color) {
        let width = self.columns.saturating_sub(indent).max(1);
        let rows_of = |line: &String| line.len().div_ceil(width).max(1);
        let first = self.row + 1;
        let total: usize = lines.iter().map(rows_of).sum();
        let skip = total.saturating_sub(self.rows - first);
        let mut pieces = lines.iter().flat_map(|line| {
            let bytes = line.as_bytes();
            let end = |k: usize| (k * width).min(bytes.len());
            (0..rows_of(line)).map(move |k| &bytes[end(k)..end(k + 1)])
        });
        for _ in 0..skip {
            pieces.next();
        }
        for row in first..self.rows {
            let cells = &mut self.cells[row * self.columns..(row + 1) * self.columns];
            cells.fill(BLANK);
            if let Some(piece) = pieces.next() {
                for (cell, &character) in cells.iter_mut().skip(indent).zip(piece) {
                    *cell = Cell {
                        character: printable(character),
                        color,
                    };
                }
            }
        }
    }

    fn present_within(&mut self, bound: u64) {
        let Some(_access) = self.screen.use_screen() else {
            return;
        };
        // SAFETY: RDTSC has no memory effects.
        let started = unsafe { _rdtsc() };
        let mut first = true;
        let more = || {
            // SAFETY: as above.
            let within = bound == 0 || unsafe { _rdtsc() }.wrapping_sub(started) < bound;
            core::mem::take(&mut first) || within
        };
        let s = self.scale;
        let (on_width, on_height) = (self.font.width * s, self.font.height * s);
        let off = self.screen.encoding.encode(ui::BACKGROUND);
        let (screen, font, columns) = (&self.screen, &self.font, self.columns);
        let (left, top) = (self.left, self.top);
        let draw = |index: usize, cell: Cell| {
            let x = left + index % columns * on_width;
            let y = top + index / columns * on_height;
            let on = screen.pixel(cell.color);
            for row in 0..on_height {
                let line = (screen.base + (y + row) * screen.stride * BYTES_PER_PIXEL) as *mut u32;
                for column in 0..on_width {
                    let value = if font.pixel(cell.character, column / s, row / s) {
                        on
                    } else {
                        off
                    };
                    // SAFETY: the grid and its margins lie inside the mode's
                    // visible framebuffer.
                    unsafe { line.add(x + column).write_volatile(value) };
                }
            }
        };
        self.pending =
            !stela_boot_core::screen::present_changed(&self.cells, &mut self.shown, draw, more);
    }
}

/// Draw for `ticks` at most what earlier bounded presents left undrawn
/// ([`running`]); whether some is still left.
pub fn present_some(ticks: u64) -> bool {
    with(|text| {
        if text.pending {
            text.present_within(ticks);
        }
        text.pending
    })
    .unwrap_or(false)
}

fn printable(character: u8) -> u8 {
    match character {
        b' '..=b'~' => character,
        _ => b'?',
    }
}

/// Firmware timer callbacks may interrupt the boot CPU. A nested operation
/// skips the console while its current owner holds BUSY.
struct Console(UnsafeCell<Option<Text>>);
// SAFETY: every access is exclusive through BUSY.
unsafe impl Sync for Console {}
static CONSOLE: Console = Console(UnsafeCell::new(None));
/// The text [`release`] keeps, which [`reclaim`] shows again.
static PARKED: Console = Console(UnsafeCell::new(None));
static BUSY: AtomicBool = AtomicBool::new(false);

fn with_consoles<R>(f: impl FnOnce(&mut Option<Text>, &mut Option<Text>) -> R) -> Option<R> {
    if BUSY.swap(true, Ordering::Acquire) {
        return None;
    }
    // SAFETY: BUSY makes these the only live references to either console.
    let result = unsafe { f(&mut *CONSOLE.0.get(), &mut *PARKED.0.get()) };
    BUSY.store(false, Ordering::Release);
    Some(result)
}

fn with<R>(f: impl FnOnce(&mut Text) -> R) -> Option<R> {
    with_consoles(|active, _| {
        active
            .as_mut()
            .filter(|text| text.screen.available())
            .map(f)
    })
    .flatten()
}

/// Clear the screen and send later text to it.
pub fn install(screen: Framebuffer) {
    with_consoles(|active, parked| {
        if screen.available() {
            screen.clear();
            *active = Text::new(screen);
            *parked = None;
        }
    });
}

/// Keep the text [`release`] kept for `screen`, which now shows.
#[cfg(feature = "nvk")]
pub fn parked_on(screen: Framebuffer) {
    with_consoles(|_, parked| {
        if let Some(text) = parked {
            text.screen = screen;
        }
    });
}

/// Stop drawing text: the screen now shows something else. The text is
/// kept for [`reclaim`]. False means another console operation was active.
pub fn release() -> bool {
    with_consoles(|active, parked| {
        if let Some(text) = active.take() {
            *parked = Some(text);
        }
    })
    .is_some()
}

/// Stop drawing for good on the screen, whose memory may no longer be
/// mapped, as a GPU's display after amdgpu's reset unmapped it: neither
/// [`reclaim`] nor a stop draws on it again. False means another console
/// operation was active.
#[cfg(feature = "nvk")]
pub fn forget_screen() -> bool {
    with_consoles(|active, parked| {
        *active = None;
        *parked = None;
    })
    .is_some()
}

/// Permanently stop the boot CPU and guest compositor from touching GOP.
/// Parked text can still be rebound to a native scanout.
pub(crate) fn retire_firmware() -> bool {
    with_consoles(|active, parked| {
        if !FIRMWARE_ACCESS.retire() {
            return false;
        }
        if active.as_ref().is_some_and(|text| text.screen.firmware) {
            *parked = active.take();
        }
        true
    })
    .unwrap_or(false)
}

/// Undo [`retire_firmware`] for a GPU claim that stopped before the GPU was
/// sealed and reset, so the firmware's picture still shows: the boot CPU and
/// the guest's frames may draw there again, and the parked text returns.
pub(crate) fn restore_firmware() -> bool {
    let restored = FIRMWARE_ACCESS.restore();
    if restored {
        reclaim();
    }
    restored
}

/// Take the screen back after [`release`] and draw the kept text again, so
/// a stop shows its reason instead of the guest's last frame.
pub fn reclaim() {
    with_consoles(|active, parked| {
        if active.is_none()
            && parked.as_ref().is_some_and(|text| text.screen.available())
            && let Some(mut text) = parked.take()
        {
            text.screen.clear();
            text.shown.fill(BLANK);
            text.present();
            *active = Some(text);
        }
    });
}

/// Whether text reaches the screen.
pub fn active() -> bool {
    with(|_| ()).is_some()
}

/// Write `segments` at the cursor.
pub fn write(segments: &[(&str, Color)]) {
    with(|text| {
        for (part, color) in segments {
            for character in part.bytes() {
                text.put(character, *color);
            }
        }
        text.present();
    });
}

/// Replace the cursor's line with `segments`, cut to the line, leaving the
/// cursor on it, and show `lines` [`below`] it, drawing for `ticks` of the
/// TSC at most (0 for no bound): a change of
/// many cells, such as lines that scrolled, reaches the screen over the
/// next [`present_some`] calls instead of holding the boot CPU for one long
/// draw.
pub fn running(segments: &[(&str, Color)], lines: Option<(&[String], usize, Color)>, ticks: u64) {
    with(|text| {
        text.rewrite(segments);
        if let Some((lines, indent, color)) = lines {
            text.below(lines, indent, color);
        }
        text.present_within(ticks);
    });
}

/// Replace the cursor's line with `segments` and carry on under it: a word
/// that would pass the edge starts the next row, `indent` cells in. The
/// cursor ends on the last row.
pub fn rewrite_wrapped(segments: &[(&str, Color)], indent: usize) {
    with(|text| {
        let start = text.row * text.columns;
        text.cells[start..start + text.columns].fill(BLANK);
        text.column = 0;
        let room = text.columns.saturating_sub(indent);
        for (part, color) in segments {
            for word in part.split_inclusive(' ') {
                let length = word.trim_end().len();
                if text.column > indent && text.column + length > text.columns && length <= room {
                    text.wrap(indent);
                }
                for character in word.bytes() {
                    // A line break in the text starts the next row, so
                    // lines a message lists never run together.
                    if character == b'\n' {
                        text.wrap(indent);
                        continue;
                    }
                    if text.column == text.columns {
                        if character == b' ' {
                            continue;
                        }
                        text.wrap(indent);
                    }
                    text.put(character, *color);
                }
            }
        }
        text.present();
    });
}

/// Show `lines` on the rows under the cursor's, `indent` cells in and each
/// wrapped onto as many rows as it takes, the last rows when not all fit,
/// and blank the rest; the cursor stays. An empty `lines` clears what an
/// earlier call showed.
pub fn below(lines: &[String], indent: usize, color: Color) {
    with(|text| {
        text.below(lines, indent, color);
        text.present();
    });
}
