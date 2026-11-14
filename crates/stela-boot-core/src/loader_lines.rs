//! The loader's lines that no log stored: those written after the
//! firmware's log file closed with its boot services and before the engine
//! starts (the xHCI handoff, the IOMMU units the firmware left on, the APIC
//! mode, the firmware processors' INIT). The engine runs from a fresh copy
//! of the image with an empty record, so the loader hands them over in a
//! bounded page and the engine puts them at the head of its record, which
//! the USB's log receives when it resumes.

/// The most bytes handed over: the newest whole lines that fit.
pub const BYTES: usize = 16 * 1024;

/// Room kept for the closing note on lines left out.
const NOTE_BYTES: usize = 96;

/// Where in the loader's record to copy from: the first byte no log stored
/// (`unkept`), or later when the record since then holds more than `room`
/// bytes, so that the newest lines fit.
pub fn first(unkept: u64, pushed: u64, room: usize) -> u64 {
    let room = room.saturating_sub(NOTE_BYTES) as u64;
    unkept.max(pushed.saturating_sub(room))
}

/// Whole lines copied into `into`, at most its length.
pub struct Carry<'a> {
    into: &'a mut [u8],
    length: usize,
    /// The copy starts inside a line, which is left out up to its newline.
    partial: bool,
    /// Bytes no log stored that are not carried: those before the copy's
    /// start, the partial line and any past the room.
    lost: u64,
}

impl<'a> Carry<'a> {
    /// Copy into `into`, starting `skipped` bytes after the first byte no
    /// log stored; a copy with `skipped` above zero starts inside a line.
    pub fn new(into: &'a mut [u8], skipped: u64) -> Self {
        Self {
            into,
            length: 0,
            partial: skipped != 0,
            lost: skipped,
        }
    }

    pub fn push(&mut self, mut bytes: &[u8]) {
        if self.partial {
            match bytes.iter().position(|&byte| byte == b'\n') {
                Some(end) => {
                    self.lost += end as u64 + 1;
                    bytes = &bytes[end + 1..];
                    self.partial = false;
                }
                None => {
                    self.lost += bytes.len() as u64;
                    return;
                }
            }
        }
        let room = self.into.len().saturating_sub(NOTE_BYTES);
        let taken = bytes.len().min(room.saturating_sub(self.length));
        self.into[self.length..self.length + taken].copy_from_slice(&bytes[..taken]);
        self.length += taken;
        self.lost += (bytes.len() - taken) as u64;
    }

    /// End at the last whole line, with a line saying how many bytes were
    /// left out when any were; the bytes used.
    pub fn finish(mut self) -> usize {
        let whole = self.into[..self.length]
            .iter()
            .rposition(|&byte| byte == b'\n')
            .map_or(0, |end| end + 1);
        self.lost += (self.length - whole) as u64;
        self.length = whole;
        if self.lost != 0 {
            let mut note = Note::default();
            let _ = core::fmt::write(
                &mut note,
                format_args!(
                    "[loader lines: {} bytes no log stored are not here]\n",
                    self.lost
                ),
            );
            let end = (self.length + note.length).min(self.into.len());
            self.into[self.length..end].copy_from_slice(&note.bytes[..end - self.length]);
            self.length = end;
        }
        self.length
    }
}

struct Note {
    bytes: [u8; NOTE_BYTES],
    length: usize,
}

impl Default for Note {
    fn default() -> Self {
        Self {
            bytes: [0; NOTE_BYTES],
            length: 0,
        }
    }
}

impl core::fmt::Write for Note {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        let end = self.length + text.len();
        self.bytes
            .get_mut(self.length..end)
            .ok_or(core::fmt::Error)?
            .copy_from_slice(text.as_bytes());
        self.length = end;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn carried(skipped: u64, room: usize, chunks: &[&[u8]]) -> alloc::string::String {
        let mut into = alloc::vec![0; room];
        let mut carry = Carry::new(&mut into, skipped);
        for chunk in chunks {
            carry.push(chunk);
        }
        let length = carry.finish();
        alloc::string::String::from_utf8(into[..length].to_vec()).unwrap()
    }

    #[test]
    fn every_unstored_line_is_carried_when_it_fits() {
        assert_eq!(first(100, 300, BYTES), 100);
        assert_eq!(
            carried(
                0,
                BYTES,
                &[
                    b"HOST APIC x2APIC\nNATIVE FIRMWARE ",
                    b"INIT deliveries=7\n"
                ]
            ),
            "HOST APIC x2APIC\nNATIVE FIRMWARE INIT deliveries=7\n"
        );
        assert_eq!(carried(0, BYTES, &[]), "");
    }

    #[test]
    fn a_long_record_keeps_its_newest_whole_lines() {
        let pushed = 100_000;
        let from = first(10, pushed, BYTES);
        assert_eq!(from, pushed - (BYTES - NOTE_BYTES) as u64);
        // The copy starts inside a line, which is left out.
        assert_eq!(
            carried(from - 10, BYTES, &[b"ART of a line\nHOST APIC xAPIC\n"]),
            alloc::format!(
                "HOST APIC xAPIC\n[loader lines: {} bytes no log stored are \
                 not here]\n",
                from - 10 + 14
            )
        );
    }

    #[test]
    fn bytes_past_the_room_and_an_unended_line_are_counted_as_lost() {
        let room = NOTE_BYTES + 12;
        assert_eq!(
            carried(0, room, &[b"first line\nsecond line\n"]),
            "first line\n[loader lines: 12 bytes no log stored are not here]\n"
        );
        assert_eq!(
            carried(0, BYTES, &[b"whole\nno newline"]),
            "whole\n[loader lines: 10 bytes no log stored are not here]\n"
        );
    }
}
