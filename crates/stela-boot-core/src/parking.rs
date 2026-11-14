//! The processors that run no guest PE. Each is started once into a small
//! idle loop of its own (`native_smp::executor::park`) instead of staying in
//! INIT's wait-for-SIPI state for the whole boot: there CR4.MCE is clear, and
//! a broadcast machine check that finds CR4.MCE clear on any processor shuts
//! the PC down with no record. Linux boots every logical CPU at least once
//! for the same reason (`kernel/cpu.c`). A parked processor keeps the
//! firmware's microcode, runs with interrupts masked and does no host
//! service: it wakes only for NMI, SMI, INIT or a machine check, which stops
//! the boot with the engine's fatal report.

use core::fmt;

/// How a parked processor waits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Idle {
    /// MWAIT with this hint and no interrupt break, so only NMI, SMI, INIT
    /// and machine checks end it.
    Mwait { hint: u32 },
    /// HLT with interrupts masked.
    Halt,
}

/// The CPUID values [`idle`] reads, from the parked processor itself.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IdleCpuid {
    /// "GenuineIntel".
    pub intel: bool,
    pub max_leaf: u32,
    pub leaf1_ecx: u32,
    pub leaf1_edx: u32,
    pub leaf5_ecx: u32,
    pub leaf5_edx: u32,
}

/// CPUID.01H:ECX[3], MONITOR/MWAIT.
const MWAIT: u32 = 1 << 3;
/// CPUID.01H:EDX[19], CLFLUSH.
const CLFLUSH: u32 = 1 << 19;
/// CPUID.05H:ECX[0], the MWAIT extensions that enumerate sub-states.
const EXTENSIONS: u32 = 1;

/// The deepest wait the processor's CPUID offers, as Linux's
/// `mwait_play_dead` chooses it for an offline CPU: on Intel with MONITOR,
/// MWAIT, CLFLUSH and leaf 5, the deepest C-state with a sub-state in leaf
/// 5 EDX and its deepest sub-state; C1 where leaf 5 enumerates none. AMD
/// processors take MWAIT to C1 alone and halt, as Linux's offline AMD CPUs
/// do; so does any processor without MWAIT or without a C1 or deeper
/// sub-state enumerated.
pub fn idle(cpu: IdleCpuid) -> Idle {
    if !cpu.intel || cpu.leaf1_ecx & MWAIT == 0 || cpu.leaf1_edx & CLFLUSH == 0 || cpu.max_leaf < 5
    {
        return Idle::Halt;
    }
    if cpu.leaf5_ecx & EXTENSIONS == 0 {
        return Idle::Mwait { hint: 0 };
    }
    // EDX holds four bits of sub-state count per C-state, C0 first; state
    // `n` here is C(n+1), whose hint is `n << 4`.
    let deepest = (0..7u32)
        .rev()
        .map(|state| (state, (cpu.leaf5_edx >> (4 * (state + 1))) & 0xf))
        .find(|&(_, substates)| substates != 0);
    match deepest {
        Some((state, substates)) => Idle::Mwait {
            hint: state << 4 | (substates - 1),
        },
        None => Idle::Halt,
    }
}

impl Idle {
    /// The word a parked processor publishes its wait in ([`Self::decode`]).
    pub fn encode(self) -> u32 {
        match self {
            Self::Mwait { hint } => hint & 0xff,
            Self::Halt => HALT,
        }
    }

    /// The wait [`Self::encode`] published; `None` before one was.
    pub fn decode(word: u32) -> Option<Self> {
        match word {
            HALT => Some(Self::Halt),
            UNPUBLISHED => None,
            hint => Some(Self::Mwait { hint: hint & 0xff }),
        }
    }
}

const HALT: u32 = 0x100;
/// The word before a processor publishes its wait.
pub const UNPUBLISHED: u32 = u32::MAX;

/// Why processors that run no PE stay in wait-for-SIPI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Left {
    /// The boot runs on one core and took no startup page below 1 MiB.
    NoStartupPage,
    /// A PE's processor did not start: the startup page may still be read
    /// by it, so no other processor starts from it.
    StartupFailed,
    /// A parked processor did not reach its idle loop in time, or faulted
    /// on the way: the processors after it stay in wait-for-SIPI.
    ParkFailed { apic: u64 },
}

/// What the parking did, for its log line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Census {
    /// Enabled processors that run no PE, the boot processor excepted.
    pub unused: usize,
    pub mwait: usize,
    pub halted: usize,
    /// The deepest MWAIT hint a parked processor took.
    pub deepest: Option<u32>,
    /// Processors whose APIC ID the boot processor cannot address.
    pub unreachable: usize,
    pub left: Option<Left>,
}

impl Census {
    /// Count a processor that reached its idle loop with `idle`.
    pub fn parked(&mut self, idle: Idle) {
        match idle {
            Idle::Mwait { hint } => {
                self.mwait += 1;
                self.deepest = Some(self.deepest.map_or(hint, |deepest| deepest.max(hint)));
            }
            Idle::Halt => self.halted += 1,
        }
    }

    /// The processors that stay in wait-for-SIPI.
    pub fn waiting(&self) -> usize {
        self.unused - self.mwait - self.halted
    }
}

impl fmt::Display for Census {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "NATIVE PARKED {} of {} unused processors with CR4.MCE set: mwait={}",
            self.mwait + self.halted,
            self.unused,
            self.mwait,
        )?;
        if let Some(hint) = self.deepest {
            write!(f, " deepest_hint={hint:#x}")?;
        }
        write!(f, " hlt={} wait_for_sipi={}", self.halted, self.waiting())?;
        if self.unreachable != 0 {
            write!(f, " unreachable={}", self.unreachable)?;
        }
        match self.left {
            None => Ok(()),
            Some(Left::NoStartupPage) => f.write_str(" (no startup page: one-core boot)"),
            Some(Left::StartupFailed) => f.write_str(" (a PE's processor did not start)"),
            Some(Left::ParkFailed { apic }) => {
                write!(f, " (processor {apic:#x} did not reach its idle loop)")
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;

    fn intel(leaf5_ecx: u32, leaf5_edx: u32) -> IdleCpuid {
        IdleCpuid {
            intel: true,
            max_leaf: 0x20,
            leaf1_ecx: MWAIT,
            leaf1_edx: CLFLUSH,
            leaf5_ecx,
            leaf5_edx,
        }
    }

    #[test]
    fn intel_waits_in_its_deepest_enumerated_substate() {
        // C1 and C3 with two sub-states each: C3's second.
        assert_eq!(idle(intel(3, 0x0000_2020)), Idle::Mwait { hint: 0x21 });
        // C1 to C7 enumerated: the C7 entry (index 6) wins.
        assert_eq!(idle(intel(3, 0x1111_1110)), Idle::Mwait { hint: 0x60 });
        // Only C1, one sub-state.
        assert_eq!(idle(intel(3, 0x10)), Idle::Mwait { hint: 0 });
    }

    #[test]
    fn intel_without_substates_waits_in_c1_or_halts() {
        assert_eq!(idle(intel(0, 0)), Idle::Mwait { hint: 0 }, "no extensions");
        assert_eq!(idle(intel(3, 0x1)), Idle::Halt, "C0 alone");
    }

    #[test]
    fn other_processors_halt() {
        let amd = IdleCpuid {
            intel: false,
            ..intel(3, 0x11)
        };
        assert_eq!(idle(amd), Idle::Halt);
        let no_mwait = IdleCpuid {
            leaf1_ecx: 0,
            ..intel(3, 0x11)
        };
        assert_eq!(idle(no_mwait), Idle::Halt);
        let no_clflush = IdleCpuid {
            leaf1_edx: 0,
            ..intel(3, 0x11)
        };
        assert_eq!(idle(no_clflush), Idle::Halt);
        let no_leaf5 = IdleCpuid {
            max_leaf: 4,
            ..intel(3, 0x11)
        };
        assert_eq!(idle(no_leaf5), Idle::Halt);
    }

    #[test]
    fn idle_round_trips_through_its_word() {
        for idle in [
            Idle::Halt,
            Idle::Mwait { hint: 0 },
            Idle::Mwait { hint: 0x60 },
        ] {
            assert_eq!(Idle::decode(idle.encode()), Some(idle));
        }
        assert_eq!(Idle::decode(UNPUBLISHED), None);
    }

    #[test]
    fn census_names_the_waits_and_what_stayed_in_wait_for_sipi() {
        let mut census = Census {
            unused: 8,
            ..Census::default()
        };
        census.parked(Idle::Mwait { hint: 0x21 });
        census.parked(Idle::Mwait { hint: 0x60 });
        census.parked(Idle::Halt);
        census.left = Some(Left::ParkFailed { apic: 0x0c });
        assert_eq!(census.waiting(), 5);
        assert_eq!(
            format!("{census}"),
            "NATIVE PARKED 3 of 8 unused processors with CR4.MCE set: mwait=2 \
             deepest_hint=0x60 hlt=1 wait_for_sipi=5 (processor 0xc did not reach its idle loop)"
        );
        let none = Census {
            unused: 0,
            ..Census::default()
        };
        assert_eq!(
            format!("{none}"),
            "NATIVE PARKED 0 of 0 unused processors with CR4.MCE set: mwait=0 hlt=0 wait_for_sipi=0"
        );
    }
}
