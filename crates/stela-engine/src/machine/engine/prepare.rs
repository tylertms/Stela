//! Block preparation: what the engine compiles for the source window at an
//! entry PC, as a pure function of the window's bytes, the PC and a
//! [`Preparation`] (the chip, the x86 target and the block variant).
//!
//! The engine prepares every block it fetches through it, and so do the
//! threads that prepare blocks off the guest's path: the translation
//! store's provers and ahead-of-time translation of mapped images. A block
//! prepared anywhere is the block the engine would compile at that PC from
//! those bytes in that variant, which is what lets the store key it by its
//! page bytes and context alone.
use super::{FETCH_PAGE, INSTRUCTION, MAX_BLOCK_BYTES, PROFILE, is_floating_point};
use alloc::{format, sync::Arc, vec, vec::Vec};
use stela_compiler::{
    CompileError, GuestAddress, TargetCpu,
    block::{self, CompilationPlan, CompiledBlock},
    x86_64::{MemoryChecks, NativeSystem},
};
use {
    crate::translations, crate::translations::Digest, stela_contracts::chip::AdmitTable,
    stela_contracts::chip::Check, stela_contracts::chip::Chip,
};

/// The bytes of the source window of a block at `pc`: to the end of its
/// fetch page, which no block leaves, and at most a block's.
pub fn window(pc: u64) -> usize {
    (FETCH_PAGE - (pc & (FETCH_PAGE - 1))).min(MAX_BLOCK_BYTES) as usize
}

/// How blocks of one variant are compiled.
#[derive(Debug, Clone)]
pub struct Preparation {
    chip: Chip,
    admit: Arc<AdmitTable>,
    target: TargetCpu,
    /// FP/SIMD enabled: otherwise a block ends before the first FP/SIMD
    /// instruction.
    fp: bool,
    checks: MemoryChecks,
    /// The PE has SME, and PSTATE.SM.
    scalable: Option<bool>,
    system: NativeSystem,
    /// [`Self::context`] of the lower and upper halves' address classes,
    /// which every fetch page lies in: each install of a block prepared
    /// elsewhere checks it.
    contexts: [Digest; 2],
}

/// The address classes of the lower and upper halves of a 48-bit address
/// space.
const HALVES: [u64; 2] = [0, translations::address_class(u64::MAX)];

/// The inputs of a [`Preparation`], serializable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Description {
    pub chip: Chip,
    pub target: TargetCpu,
    pub fp: bool,
    pub checks: MemoryChecks,
    pub scalable: Option<bool>,
    pub system: NativeSystem,
    /// The features the PE withholds (`AdmitTable::withheld`); none in a
    /// description from before it was recorded.
    #[serde(default)]
    pub withheld: u64,
}

/// What a source window prepares to.
#[derive(Debug)]
pub enum Prepared {
    /// The block, the bytes of the window the compiler was given, the
    /// bytes the analysis read (the block's, or the whole window when the
    /// block runs to its end), and the entry PCs its static successors
    /// (call continuations included) have.
    Block {
        compiled: CompiledBlock,
        input: usize,
        consumed: usize,
        successors: Vec<u64>,
    },
    /// Its first instruction, an FP/SIMD or scalable one, has an access
    /// check the engine applies before it runs.
    Checked(u32),
    /// Its first instruction is an FP/SIMD one while FP/SIMD is trapped.
    FloatingPointTrap,
}

/// The first word of a window no block can be compiled from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refused {
    pub pc: u64,
    pub error: CompileError,
}

impl Preparation {
    pub(super) fn new(
        chip: Chip,
        admit: Arc<AdmitTable>,
        target: TargetCpu,
        fp: bool,
        checks: MemoryChecks,
        scalable: Option<bool>,
        system: NativeSystem,
    ) -> Self {
        let mut preparation = Self {
            chip,
            admit,
            target,
            fp,
            checks,
            scalable,
            system,
            contexts: [Digest::default(); 2],
        };
        preparation.contexts = HALVES.map(|class| preparation.describe(class));
        preparation
    }

    /// What decides this preparation, for another process to make it again
    /// ([`Self::from_description`]).
    pub fn description(&self) -> Description {
        Description {
            chip: self.chip,
            target: self.target,
            fp: self.fp,
            checks: self.checks,
            scalable: self.scalable,
            system: self.system,
            withheld: self.admit.withheld(),
        }
    }

    /// The preparation `description` records, or why it records none: a
    /// stored description may be old or damaged, and its withheld set one
    /// [`Chip::admit_table`] refuses.
    pub fn from_description(description: &Description) -> Result<Self, stela_contracts::Refusal> {
        let Description {
            chip,
            target,
            fp,
            checks,
            scalable,
            system,
            withheld,
        } = *description;
        Ok(Self::new(
            chip,
            Arc::new(chip.admit_table(withheld)?),
            target,
            fp,
            checks,
            scalable,
            system,
        ))
    }
    /// Replace each word of `source` with the one the chip executes: a
    /// form the chip lacks compiles as the UDF or NOP it executes as.
    pub fn admit(&self, source: &mut [u8]) {
        for bytes in source.as_chunks_mut::<4>().0 {
            *bytes = self.admit.admit(u32::from_le_bytes(*bytes)).to_le_bytes();
        }
    }

    /// The entry context of blocks prepared with this in the page at
    /// `page`: everything besides their bytes and offset that shapes their
    /// compilation.
    pub fn context(&self, page: u64) -> Digest {
        let class = translations::address_class(page);
        match HALVES.iter().position(|&half| half == class) {
            Some(half) => self.contexts[half],
            None => self.describe(class),
        }
    }

    fn describe(&self, class: u64) -> Digest {
        let Self {
            chip,
            admit,
            target,
            fp,
            checks,
            scalable,
            system,
            contexts: _,
        } = self;
        let described = format!(
            "chip {chip:?} withheld {:#x} target {target:?} fp {fp} checks {checks:?} scalable {scalable:?} \
             system {system:?} class {class:#x}",
            admit.withheld(),
        );
        Digest::of(described.as_bytes())
    }

    /// Compile the admitted words `input` at `pc` as the engine compiles a
    /// block's consumed prefix.
    pub fn compile_input(&self, input: &[u8], pc: u64) -> Result<CompiledBlock, CompileError> {
        stela_compiler::compile(input, GuestAddress(pc), &self.options())
    }

    pub(super) fn qualify_recovery(
        &self,
        input: &[u8],
        pc: u64,
        template: block::RecoveryTemplate<'_>,
    ) -> Result<CompiledBlock, block::RecoveryQualificationError> {
        CompilationPlan::new(input, GuestAddress(pc), &self.options())
            .map_err(block::RecoveryQualificationError::Compile)?
            .qualify_recovery(template)
    }

    /// What the engine compiles this preparation's blocks for.
    fn options(&self) -> stela_compiler::Options {
        stela_compiler::Options::new(PROFILE, self.target)
            .with_memory_checks(self.checks)
            .with_native_system(self.system)
    }

    /// Prepare the block at `pc` from the admitted window `source`, which
    /// holds the bytes to the end of its fetch page (at most a block's),
    /// ending it at `limit` bytes, before a host breakpoint or trace point.
    /// From a prefix of the window, a block whose analysis did not reach
    /// the prefix's end (`consumed`) is the block the whole window
    /// prepares: the analysis reads words in order and stops at the
    /// block's last.
    pub fn prepare(&self, source: &[u8], pc: u64, limit: usize) -> Result<Prepared, Refused> {
        self.prepare_window(source, pc, limit, true)
            .expect("complete window")
    }

    /// A short probe that reaches the end of its input needs the full window.
    /// Decide from analysis before emitting code that the caller would discard.
    pub(super) fn prepare_window(
        &self,
        source: &[u8],
        pc: u64,
        limit: usize,
        whole: bool,
    ) -> Option<Result<Prepared, Refused>> {
        self.prepare_prefix(source, pc, limit, whole).transpose()
    }

    fn prepare_prefix(
        &self,
        source: &[u8],
        pc: u64,
        limit: usize,
        whole: bool,
    ) -> Result<Option<Prepared>, Refused> {
        let word = |index: usize| {
            u32::from_le_bytes(source[index * 4..index * 4 + 4].try_into().expect("a word"))
        };
        let mut length = limit.min(source.len());
        if !self.fp {
            let first = source
                .as_chunks::<4>()
                .0
                .iter()
                .position(|word| is_floating_point(u32::from_le_bytes(*word)));
            match first {
                // CheckFPMREnabled comes before the FP access check.
                Some(0) if self.chip.checks(word(0)).contains(&Check::Fpmr) => {
                    return Ok(Some(Prepared::Checked(word(0))));
                }
                Some(0) => return Ok(Some(Prepared::FloatingPointTrap)),
                Some(index) => length = length.min(index * 4),
                None => {}
            }
        }
        if let Some(streaming) = self.scalable {
            let first = source.as_chunks::<4>().0[..length / 4]
                .iter()
                .map(|word| u32::from_le_bytes(*word))
                .position(|word| {
                    let checks = self.chip.checks(word);
                    if streaming {
                        !checks.is_empty()
                    } else {
                        checks
                            .iter()
                            .any(|check| !matches!(check, Check::Fp | Check::FpAdvSimd))
                    }
                });
            match first {
                Some(0) => return Ok(Some(Prepared::Checked(word(0)))),
                Some(index) => length = index * 4,
                None => {}
            }
        }
        let refused = |error: block::BlockAnalysisError| Refused {
            pc: error.pc.0,
            error: error.into(),
        };
        let analyze = |length: usize| {
            block::analyze_and_plan(&source[..length], GuestAddress(pc), &self.options())
        };
        let (analysis, plan) = match analyze(length) {
            Ok(analyzed) => analyzed,
            Err(error) if error.pc.0 > pc => {
                analyze((error.pc.0 - pc) as usize).map_err(refused)?
            }
            Err(error) => return Err(refused(error)),
        };
        let consumed = analysis.source_bytes;
        if !whole && consumed == source.len() {
            return Ok(None);
        }
        let (compiled, input) = self.compile_prefix(&source[..consumed], pc, plan)?;
        // A block a host refusal cut short falls through to the refused
        // word; a whole one has the analysis's static successors.
        let successors = if input < consumed {
            vec![pc + input as u64]
        } else {
            analysis
                .static_successors
                .iter()
                .map(|successor| successor.target.0)
                .collect()
        };
        Ok(Some(Prepared::Block {
            compiled,
            input,
            consumed,
            successors,
        }))
    }

    /// Compile the longest prefix of `source` the target can emit, starting
    /// from `plan`, the whole source's, and the bytes of it the compiler was
    /// given.
    fn compile_prefix(
        &self,
        source: &[u8],
        pc: u64,
        plan: Result<CompilationPlan, CompileError>,
    ) -> Result<(CompiledBlock, usize), Refused> {
        let refused = |error: CompileError| Refused {
            pc: error.instruction.map_or(pc, |source| source.address.0),
            error,
        };
        match plan.and_then(CompilationPlan::compile) {
            Ok(compiled) => Ok((compiled, source.len())),
            Err(error) => {
                // A host-capability refusal names no PC; find the longest
                // prefix that compiles and report the first refused word.
                let words = source.len() / INSTRUCTION as usize;
                let (mut good, mut bad) = (0, words);
                while bad - good > 1 {
                    let middle = (good + bad) / 2;
                    if self.compile_input(&source[..middle * 4], pc).is_ok() {
                        good = middle;
                    } else {
                        bad = middle;
                    }
                }
                if good == 0 {
                    return Err(refused(error));
                }
                self.compile_input(&source[..good * 4], pc)
                    .map(|compiled| (compiled, good * 4))
                    .map_err(refused)
            }
        }
    }
}

/// A preparation of the test chip for the baseline target, as a process
/// profile compiles.
#[cfg(test)]
pub(crate) fn test_preparation() -> Preparation {
    let chip = crate::machine::test_chip();
    Preparation::new(
        chip,
        Arc::new(chip.admit_table(0).expect("the test chip's admit table")),
        TargetCpu::Baseline,
        true,
        MemoryChecks::DEFAULT,
        None,
        NativeSystem::NONE,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use stela_contracts::Feature;

    #[test]
    fn preparation_refusal_preserves_compiler_cause_and_instruction() {
        let pc = 0x4000;
        let source = [0x21, 0x43];
        let Err(expected) =
            block::analyze_and_plan(&source, GuestAddress(pc), &test_preparation().options())
        else {
            panic!("incomplete instruction must be refused");
        };
        let refused = test_preparation()
            .prepare(&source, pc, source.len())
            .unwrap_err();
        assert_eq!(refused.pc, pc);
        assert_eq!(refused.error, CompileError::from(expected));
        let instruction = refused.error.instruction.unwrap();
        assert_eq!(instruction.address, GuestAddress(pc));
        assert_eq!(instruction.encoding, None);
    }

    /// A description makes its preparation again, and one whose withheld set
    /// has no admit table is refused rather than trusted.
    #[test]
    fn a_description_makes_its_preparation_or_is_refused() {
        let preparation = test_preparation();
        let description = preparation.description();
        let again = Preparation::from_description(&description).unwrap();
        assert_eq!(again.description(), description);
        let split = Description {
            chip: Chip::AppleM4,
            withheld: 1 << Feature::FEAT_SME_I16I64 as u64,
            ..description
        };
        assert!(Preparation::from_description(&split).is_err());
    }

    #[test]
    #[ignore = "density harness"]
    fn density_dump() {
        use std::io::Write;
        let input = std::env::var("DENSITY_IN").unwrap();
        let output = std::env::var("DENSITY_OUT").unwrap();
        let chip = crate::machine::test_chip();
        let checks = MemoryChecks::system(true, false, true, false)
            .with_range(stela_compiler::x86_64::RangeCheck::four_level(4).one_sided(true));
        let system = NativeSystem {
            exclusive: true,
            pointer: Some(super::super::pointer::CELLS),
            self_loops: true,
            ..NativeSystem::NONE
        };
        let preparation = Preparation::new(
            chip,
            Arc::new(chip.admit_table(0).expect("admit")),
            TargetCpu::detect(),
            true,
            checks,
            None,
            system,
        );
        let mut out = std::fs::File::create(output).unwrap();
        for line in std::fs::read_to_string(input).unwrap().lines() {
            let mut fields = line.split(' ');
            let pc = u64::from_str_radix(fields.next().unwrap(), 16).unwrap();
            let weight = fields.next().unwrap();
            let hex = fields.next().unwrap();
            let bytes: Vec<u8> = (0..hex.len() / 2)
                .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap())
                .collect();
            let len = bytes.len();
            match preparation.prepare_window(&bytes, pc, len, true) {
                Some(Ok(Prepared::Block { compiled, .. })) => {
                    let map: Vec<String> = compiled
                        .native
                        .source_map
                        .iter()
                        .map(|(a, o)| format!("{:x}:{o}", a.0))
                        .collect();
                    let code: String = compiled
                        .native
                        .x86_64_code
                        .iter()
                        .map(|b| format!("{b:02x}"))
                        .collect();
                    writeln!(
                        out,
                        "{pc:x} {weight} {} {code} {}",
                        compiled.source_bytes,
                        map.join(",")
                    )
                    .unwrap();
                }
                other => writeln!(out, "{pc:x} {weight} refused {other:?}").unwrap(),
            }
        }
    }

    #[test]
    #[ignore = "density harness"]
    fn chain_dump() {
        use stela_compiler::x86_64::chain;
        let output = std::env::var("DENSITY_OUT").unwrap();
        let hex = |bytes: &[u8]| bytes.iter().map(|b| format!("{b:02x}")).collect::<String>();
        let mut text = String::new();
        let (_, body) = chain::link(0x10_0000, chain::LINK_BYTES, 5, 7, 0x20_0000).unwrap();
        text += &format!("link {}\n", hex(&body));
        let dispatcher = chain::lookup_dispatcher(0).unwrap();
        text += &format!("cell {}\n", hex(&dispatcher[..chain::PROBE_ENTRY]));
        text += &format!(
            "probe {}\n",
            hex(&dispatcher[chain::PROBE_ENTRY..chain::STAMPING_PROBE_ENTRY])
        );
        let stamped = chain::stamped_tail(
            0x10_0000,
            0x30_0000,
            9,
            0x4000,
            stela_compiler::block::ExitKind::Direct,
            5,
            7,
            0x20_0000,
            0x10_8000,
        )
        .unwrap();
        text += &format!("stamped {}\n", hex(&stamped));
        std::fs::write(output, text).unwrap();
    }
}
