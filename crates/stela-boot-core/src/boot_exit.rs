//! Leaving the firmware's boot services: the final memory map's buffer and
//! the bounded retry of `ExitBootServices` (UEFI 2.10 section 7.4.6).
//!
//! The loader allocates the buffer once, before it closes its firmware log
//! and timer, and never again: those closes and the firmware's own
//! notifications change the map, and after a refused exit UEFI 2.8 and
//! earlier allow only `GetMemoryMap` and `ExitBootServices`. A refusal is
//! retried with a fresh map read into the same buffer, nothing in between,
//! as Linux's EFI stub does (`efi_exit_boot_services`), up to [`ATTEMPTS`]
//! times. A final refusal is returned with the firmware's status: the
//! firmware's boot services still run then, and the caller stops with the
//! reason kept for the next start instead of resetting the PC.
use core::fmt;
use uefi_raw::Status;

/// Descriptors the buffer holds beyond the map it was sized from, at least.
/// Linux keeps 32 spare; firmware whose exit notifications split many
/// ranges needs more, and the pages cost nothing after the exit.
pub const SPARE_DESCRIPTORS: usize = 64;
/// Exit tries before the boot stops.
pub const ATTEMPTS: u32 = 4;
/// The smallest descriptor (`EFI_MEMORY_DESCRIPTOR`, section 7.2.3).
const DESCRIPTOR_BYTES: usize = 40;

/// The bytes the final map's buffer takes, for a map of `map_bytes` in
/// descriptors of `descriptor_bytes`: the map, and as much again or
/// [`SPARE_DESCRIPTORS`] more descriptors, whichever is larger.
pub fn buffer_bytes(map_bytes: usize, descriptor_bytes: usize) -> usize {
    let descriptor_bytes = descriptor_bytes.max(DESCRIPTOR_BYTES);
    map_bytes.saturating_add(map_bytes.max(SPARE_DESCRIPTORS * descriptor_bytes))
}

/// What `GetMemoryMap` returned with the map it wrote.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Map {
    pub bytes: usize,
    pub key: usize,
    pub descriptor_bytes: usize,
    pub version: u32,
}

impl Map {
    /// Whether the map fills `capacity` bytes or fewer with whole
    /// descriptors of a size the specification allows.
    fn fits(&self, capacity: usize) -> bool {
        self.bytes <= capacity
            && self.descriptor_bytes >= DESCRIPTOR_BYTES
            && self.descriptor_bytes.is_multiple_of(8)
            && self.bytes.is_multiple_of(self.descriptor_bytes)
    }
}

/// The two boot services the exit uses.
pub trait Firmware {
    /// `GetMemoryMap` into `buffer`, whole.
    fn memory_map(&mut self, buffer: &mut [u8]) -> Result<Map, Status>;
    /// `ExitBootServices` with the map's `key`.
    fn exit(&mut self, key: usize) -> Status;
}

/// The call that stopped the exit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// `GetMemoryMap` failed: the map outgrew the buffer, or worse.
    MemoryMap,
    /// `GetMemoryMap` returned a map with a size it cannot have.
    Malformed,
    /// `ExitBootServices` refused every try.
    Exit,
}

/// Why the firmware's boot services did not end; they still run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Refused {
    pub step: Step,
    pub status: Status,
    /// The tries made, the failing one included.
    pub attempts: u32,
}

impl fmt::Display for Refused {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Self {
            step,
            status,
            attempts,
        } = self;
        match step {
            Step::MemoryMap => write!(
                f,
                "the firmware gave no final memory map at try {attempts} of {ATTEMPTS}: {status:?}"
            ),
            Step::Malformed => write!(
                f,
                "the firmware's final memory map at try {attempts} of {ATTEMPTS} has an \
                 impossible size"
            ),
            Step::Exit => write!(
                f,
                "the firmware refused to end its boot services {attempts} times, last with \
                 {status:?}"
            ),
        }
    }
}

/// The exit's map and the tries it took.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Exited {
    pub map: Map,
    pub attempts: u32,
}

/// End `firmware`'s boot services with a map read into `buffer` right
/// before each try ([`ATTEMPTS`] at most).
pub fn exit(firmware: &mut impl Firmware, buffer: &mut [u8]) -> Result<Exited, Refused> {
    let mut status = Status::ABORTED;
    for attempts in 1..=ATTEMPTS {
        let refused = |step, status| Refused {
            step,
            status,
            attempts,
        };
        let map = firmware
            .memory_map(buffer)
            .map_err(|status| refused(Step::MemoryMap, status))?;
        if !map.fits(buffer.len()) {
            return Err(refused(Step::Malformed, Status::BAD_BUFFER_SIZE));
        }
        status = firmware.exit(map.key);
        if status == Status::SUCCESS {
            return Ok(Exited { map, attempts });
        }
    }
    Err(Refused {
        step: Step::Exit,
        status,
        attempts: ATTEMPTS,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;

    /// Firmware whose map grows by `growth` descriptors at every read and
    /// whose key changes with it; `answers` are its exit statuses in turn.
    struct Fake {
        descriptors: usize,
        growth: usize,
        key: usize,
        answers: Vec<Status>,
        keys: Vec<usize>,
    }

    impl Fake {
        fn new(descriptors: usize, growth: usize, answers: &[Status]) -> Self {
            Self {
                descriptors,
                growth,
                key: 0,
                answers: answers.to_vec(),
                keys: Vec::new(),
            }
        }
    }

    impl Firmware for Fake {
        fn memory_map(&mut self, buffer: &mut [u8]) -> Result<Map, Status> {
            self.descriptors += self.growth;
            self.key += 1;
            let bytes = self.descriptors * 48;
            if bytes > buffer.len() {
                return Err(Status::BUFFER_TOO_SMALL);
            }
            Ok(Map {
                bytes,
                key: self.key,
                descriptor_bytes: 48,
                version: 1,
            })
        }

        fn exit(&mut self, key: usize) -> Status {
            self.keys.push(key);
            if self.answers.is_empty() {
                Status::SUCCESS
            } else {
                self.answers.remove(0)
            }
        }
    }

    #[test]
    fn buffer_holds_the_map_twice_or_sixty_four_spare_descriptors() {
        assert_eq!(buffer_bytes(48 * 10, 48), 48 * 10 + 48 * 64);
        assert_eq!(buffer_bytes(48 * 200, 48), 48 * 400);
        // A descriptor size below the specification's counts as its least.
        assert_eq!(buffer_bytes(0, 0), 40 * 64);
    }

    #[test]
    fn exit_retries_with_a_fresh_map_when_the_key_changed() {
        let mut firmware = Fake::new(
            100,
            20,
            &[Status::INVALID_PARAMETER, Status::INVALID_PARAMETER],
        );
        let mut buffer = vec![0; buffer_bytes(100 * 48, 48)];
        let exited = exit(&mut firmware, &mut buffer).unwrap();
        assert_eq!(exited.attempts, 3);
        assert_eq!(exited.map.bytes, 160 * 48);
        // Each try used the key of the map read just before it.
        assert_eq!(firmware.keys, [1, 2, 3]);
    }

    #[test]
    fn exit_stops_with_the_status_after_every_try_is_refused() {
        let mut firmware = Fake::new(10, 1, &[Status::INVALID_PARAMETER; 8]);
        let mut buffer = vec![0; buffer_bytes(10 * 48, 48)];
        assert_eq!(
            exit(&mut firmware, &mut buffer),
            Err(Refused {
                step: Step::Exit,
                status: Status::INVALID_PARAMETER,
                attempts: ATTEMPTS,
            })
        );
        assert_eq!(firmware.keys.len(), ATTEMPTS as usize);
    }

    #[test]
    fn exit_stops_when_the_map_outgrows_the_buffer() {
        let mut firmware = Fake::new(100, 70, &[Status::INVALID_PARAMETER]);
        let mut buffer = vec![0; buffer_bytes(100 * 48, 48)];
        assert_eq!(
            exit(&mut firmware, &mut buffer),
            Err(Refused {
                step: Step::MemoryMap,
                status: Status::BUFFER_TOO_SMALL,
                attempts: 2,
            })
        );
    }

    struct Odd;

    impl Firmware for Odd {
        fn memory_map(&mut self, _: &mut [u8]) -> Result<Map, Status> {
            Ok(Map {
                bytes: 100,
                key: 1,
                descriptor_bytes: 48,
                version: 1,
            })
        }

        fn exit(&mut self, _: usize) -> Status {
            panic!("a map of partial descriptors must not be exited with")
        }
    }

    #[test]
    fn exit_refuses_a_map_of_partial_descriptors_before_exiting() {
        let mut buffer = vec![0; 4096];
        assert_eq!(
            exit(&mut Odd, &mut buffer).map_err(|refused| refused.step),
            Err(Step::Malformed)
        );
    }

    #[test]
    fn refusal_names_the_status_and_tries() {
        let refused = Refused {
            step: Step::Exit,
            status: Status::INVALID_PARAMETER,
            attempts: ATTEMPTS,
        };
        let text = alloc::format!("{refused}");
        assert!(text.contains("4 times"), "{text}");
        assert!(text.contains("INVALID_PARAMETER"), "{text}");
    }
}
