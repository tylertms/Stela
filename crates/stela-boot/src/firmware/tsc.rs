//! The time-stamp counter's rate, found once before ExitBootServices: from
//! CPUID leaf 15h where Intel enumerates it, else measured against the
//! ACPI PM timer or the HPET (`stela_boot_core::tsc::settle`), as Linux's
//! `native_calibrate_tsc` and then its PIT/HPET/PM-timer calibration find
//! it. A firmware stall is the last resort, on a PC with neither timer.
use core::{
    arch::{asm, x86_64::_rdtsc},
    sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering},
    time::Duration,
};
use stela_boot_core::{
    hpet, pm_timer,
    tsc::{self as rate, Calibrated, Measured, Reference, ReferenceScale, Sample, Window},
};
use stela_runtime::pc::acpi;
use uefi::boot;

/// The time-stamp counter's rate, once found, how, and within how many
/// parts per million.
static TSC_HZ: AtomicU64 = AtomicU64::new(0);
static TSC_SOURCE: AtomicU8 = AtomicU8::new(0);
static TSC_TOLERANCE_PPM: AtomicU64 = AtomicU64::new(0);
static FOUND: AtomicBool = AtomicBool::new(false);

/// How [`tsc_hz`] found the counter's rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TscSource {
    /// CPUID leaf 15h's crystal ratio, its crystal from 15h or 16h.
    Cpuid,
    /// Measured against the ACPI PM timer.
    PmTimer,
    /// Measured against the HPET's main counter.
    Hpet,
    /// The median of [`STALLS`] firmware stalls, with no timer to measure
    /// against.
    Stall,
}

impl TscSource {
    fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::Cpuid,
            1 => Self::PmTimer,
            2 => Self::Hpet,
            _ => Self::Stall,
        }
    }
    fn name(self) -> &'static str {
        match self {
            Self::Cpuid => "cpuid",
            Self::PmTimer => "pm-timer",
            Self::Hpet => "hpet",
            Self::Stall => "stall",
        }
    }
}

/// A crystal's tolerance and leaf 16h's rounded base frequency.
const CPUID_TOLERANCE_PPM: u64 = 1_000;
/// The PM timer's 14.318 MHz chipset crystal; ACPI sets no tolerance.
const PM_TOLERANCE_PPM: u64 = 200;
/// HPET 1.0a section 2.4.1: at most 500 ppm.
const HPET_TOLERANCE_PPM: u64 = 500;
/// A stall can run short or long: the measured 4 % of one VM boot, with room.
const STALL_TOLERANCE_PPM: u64 = 50_000;
/// Stalls measured, of [`WINDOW`] each, where no timer exists.
const STALLS: usize = 5;
const WINDOW: Duration = Duration::from_nanos(rate::WINDOW_NS);

fn rdtsc() -> u64 {
    // SAFETY: RDTSC has no memory effects.
    unsafe { _rdtsc() }
}

/// The time-stamp counter's rate, found once for every user, before
/// ExitBootServices.
pub fn tsc_hz() -> u64 {
    // `FOUND` rather than a nonzero rate marks the search done, so a zero
    // rate does not search again after ExitBootServices.
    if FOUND.load(Ordering::Acquire) {
        return TSC_HZ.load(Ordering::Relaxed);
    }
    let (hz, source, tolerance) = find();
    TSC_SOURCE.store(source as u8, Ordering::Relaxed);
    TSC_TOLERANCE_PPM.store(tolerance, Ordering::Relaxed);
    TSC_HZ.store(hz, Ordering::Relaxed);
    FOUND.store(true, Ordering::Release);
    hz
}

/// Take the rate the loader found, in the engine image, whose statics start
/// empty: [`tsc_hz`] and its companions answer from it and never search,
/// which would read firmware tables that the engine's roots do not map.
pub fn adopt(hz: u64, source: TscSource, tolerance_ppm: u32) {
    TSC_SOURCE.store(source as u8, Ordering::Relaxed);
    TSC_TOLERANCE_PPM.store(u64::from(tolerance_ppm), Ordering::Relaxed);
    TSC_HZ.store(hz, Ordering::Relaxed);
    FOUND.store(true, Ordering::Release);
}

/// How [`tsc_hz`] found the rate.
pub fn tsc_source() -> TscSource {
    tsc_hz();
    TscSource::from_u8(TSC_SOURCE.load(Ordering::Relaxed))
}

/// The most [`tsc_hz`] may be off by, in parts per million: its source's
/// own error and, for a measured rate, the measurement's.
pub fn tsc_tolerance_ppm() -> u32 {
    tsc_hz();
    TSC_TOLERANCE_PPM
        .load(Ordering::Relaxed)
        .min(u64::from(u32::MAX)) as u32
}

fn find() -> (u64, TscSource, u64) {
    let measured = measure_against_timers();
    if let Some(hz) = enumerated_tsc_hz() {
        // Leaf 15h stays the rate, as Linux keeps it; a measurement only
        // reports how far the enumerated crystal is from the timers'.
        if let Some((calibrated, source)) = measured {
            crate::output::arguments(format_args!(
                "HOST TSC hz={hz} source=cpuid measured_hz={} measured_by={} differs_ppm={}\n",
                calibrated.hz,
                source.name(),
                hz.abs_diff(calibrated.hz) * 1_000_000 / hz,
            ));
        } else {
            crate::output::arguments(format_args!("HOST TSC hz={hz} source=cpuid\n"));
        }
        return (hz, TscSource::Cpuid, CPUID_TOLERANCE_PPM);
    }
    if let Some((calibrated, source)) = measured {
        let reference = match source {
            TscSource::Hpet => HPET_TOLERANCE_PPM,
            _ => PM_TOLERANCE_PPM,
        };
        crate::output::arguments(format_args!(
            "HOST TSC hz={} source={} accuracy_ppm={} cross_check_ppm={} agreed={}\n",
            calibrated.hz,
            source.name(),
            calibrated.uncertainty_ppm,
            calibrated.disagreement_ppm,
            calibrated.agreed,
        ));
        return (
            calibrated.hz,
            source,
            calibrated.uncertainty_ppm + reference,
        );
    }
    let hz = stalled_hz();
    crate::output::arguments(format_args!(
        "UNSUPPORTED host TSC calibration: no ACPI PM timer or running HPET; hz={hz} from firmware stalls, within {}%\n",
        STALL_TOLERANCE_PPM / 10_000,
    ));
    (hz, TscSource::Stall, STALL_TOLERANCE_PPM)
}

/// The median of [`STALLS`] stalls of [`WINDOW`]: firmware stalls run long
/// under SMIs and short when their own timer is coarse.
fn stalled_hz() -> u64 {
    let mut rates = [0u64; STALLS];
    for rate in &mut rates {
        let before = rdtsc();
        boot::stall(WINDOW);
        *rate = (u128::from(rdtsc() - before) * 1_000_000_000 / u128::from(rate::WINDOW_NS)) as u64;
    }
    rates.sort_unstable();
    rates[STALLS / 2]
}

/// The firmware's timers this PC has, before ExitBootServices.
struct Timers {
    pm: Option<Pm>,
    hpet: Option<Hpet>,
}

/// The rate measured against the PM timer, cross-checked by the HPET where
/// both exist, else by a second PM window; or against the HPET alone.
fn measure_against_timers() -> Option<(Calibrated, TscSource)> {
    let rsdp = crate::storage::rsdp()?;
    let timers = Timers {
        // SAFETY: the UEFI configuration table's ACPI root, before EBS;
        // `find` checks each table's length.
        pm: unsafe { acpi::find(rsdp, b"FACP") }
            .ok()
            .and_then(|table| pm_timer::Prepared::from_fadt(table).ok())
            .map(|prepared| Pm {
                port: prepared.port,
                mask: prepared.mask,
            }),
        // SAFETY: as above.
        hpet: unsafe { acpi::find(rsdp, b"HPET") }
            .ok()
            .and_then(|table| hpet::Descriptor::from_sdt(table).ok())
            .and_then(Hpet::running),
    };
    let (source, primary): (TscSource, &dyn Timer) = match (&timers.pm, &timers.hpet) {
        (Some(pm), _) => (TscSource::PmTimer, pm),
        (None, Some(hpet)) => (TscSource::Hpet, hpet),
        (None, None) => return None,
    };
    let check: &dyn Timer = match (&timers.pm, &timers.hpet) {
        (Some(_), Some(hpet)) => hpet,
        _ => primary,
    };
    let result = rate::settle(|window| match window {
        Window::Primary | Window::TieBreak => primary.measure(),
        Window::CrossCheck => check.measure(),
    });
    match result {
        Ok(calibrated) => Some((calibrated, source)),
        Err(error) => {
            crate::output::arguments(format_args!(
                "HOST TSC calibration against {} failed: {error:?}\n",
                source.name()
            ));
            None
        }
    }
}

trait Timer {
    fn reference(&self) -> Reference;
    /// One reference read, bracketed by TSC reads.
    fn sample(&self) -> Sample;
    fn measure(&self) -> Result<Measured, rate::Error> {
        rate::measure(self.reference(), || self.sample())
    }
}

/// The FADT's fixed PM timer port.
struct Pm {
    port: u16,
    mask: u32,
}

impl Pm {
    fn read(&self) -> u32 {
        let value: u32;
        // SAFETY: the FADT's fixed PM timer port, which firmware never
        // changes; a read has no effect.
        unsafe {
            asm!("in eax, dx", in("dx") self.port, out("eax") value, options(nomem, nostack, preserves_flags));
        }
        value & self.mask
    }
}

impl Timer for Pm {
    fn reference(&self) -> Reference {
        Reference {
            mask: u64::from(self.mask),
            scale: ReferenceScale::Hertz(pm_timer::HZ),
        }
    }
    /// The middle of three reads, bracketed, kept when it lies between the
    /// other two, as Linux's `acpi_pm_read_verified` reads chipsets whose
    /// latch glitches; a glitched sample's bracket is widened past the
    /// disturbance bound so that it is read again.
    fn sample(&self) -> Sample {
        let first = self.read();
        let tsc_before = rdtsc();
        let middle = self.read();
        let tsc_after = rdtsc();
        let last = self.read();
        let ordered = |a: u32, b: u32| b.wrapping_sub(a) & self.mask <= self.mask / 2;
        let verified = ordered(first, middle) && ordered(middle, last);
        Sample {
            tsc_before,
            reference: u64::from(middle),
            tsc_after: if verified { tsc_after } else { u64::MAX },
        }
    }
}

/// The HPET's main counter, read through the firmware's identity map.
struct Hpet {
    counter: *const u32,
    period_fs: u32,
}

const HPET_CAPABILITIES: u64 = 0x00;
const HPET_CONFIGURATION: u64 = 0x10;
const HPET_COUNTER: u64 = 0xf0;
const HPET_ENABLED: u32 = 1;

impl Hpet {
    /// The HPET the firmware left counting and maps; Stela neither enables
    /// nor reconfigures it before ExitBootServices.
    fn running(descriptor: hpet::Descriptor) -> Option<Self> {
        let base = descriptor.base();
        if !identity_mapped(base) {
            return None;
        }
        let register = |offset: u64| {
            // SAFETY: the HPET table's register block, which the firmware's
            // page tables map (checked above); reads have no effect.
            unsafe { ((base + offset) as *const u32).read_volatile() }
        };
        let period_fs = register(HPET_CAPABILITIES + 4);
        if register(HPET_CONFIGURATION) & HPET_ENABLED == 0
            || !(100_000..=100_000_000).contains(&period_fs)
        {
            return None;
        }
        Some(Self {
            counter: (base + HPET_COUNTER) as *const u32,
            period_fs,
        })
    }
}

impl Timer for Hpet {
    fn reference(&self) -> Reference {
        Reference {
            mask: u64::from(u32::MAX),
            scale: ReferenceScale::PeriodFemtoseconds(self.period_fs),
        }
    }
    fn sample(&self) -> Sample {
        let tsc_before = rdtsc();
        // SAFETY: the running HPET's main counter low word, mapped
        // (`Hpet::running`); a read has no effect.
        let reference = unsafe { self.counter.read_volatile() };
        let tsc_after = rdtsc();
        Sample {
            tsc_before,
            reference: u64::from(reference),
            tsc_after,
        }
    }
}

/// Whether the firmware's current page tables map `address` (UEFI maps
/// what it maps one to one). Reads its tables through that identity map.
fn identity_mapped(address: u64) -> bool {
    let (cr3, cr4): (u64, u64);
    // SAFETY: reads control registers only.
    unsafe {
        asm!("mov {}, cr3", out(reg) cr3, options(nomem, nostack, preserves_flags));
        asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack, preserves_flags));
    }
    const ADDRESS: u64 = 0x000f_ffff_ffff_f000;
    const PS: u64 = 1 << 7;
    let levels = if cr4 & (1 << 12) != 0 { 5 } else { 4 };
    let mut table = cr3 & ADDRESS;
    for level in (1..=levels).rev() {
        let index = (address >> (12 + 9 * (level - 1))) & 511;
        // SAFETY: a firmware page-table page, which UEFI maps one to one;
        // reading an entry has no effect.
        let entry = unsafe { ((table + index * 8) as *const u64).read_volatile() };
        if entry & 1 == 0 {
            return false;
        }
        if level == 1 || (level <= 3 && entry & PS != 0) {
            return true;
        }
        table = entry & ADDRESS;
    }
    false
}

/// The rate CPUID leaf 15h gives: its crystal's, times EBX over EAX. Where
/// ECX leaves the crystal out, leaf 16h's base frequency stands in for it,
/// as Linux's `native_calibrate_tsc` derives it. Linux trusts the leaf on
/// Intel processors only.
fn enumerated_tsc_hz() -> Option<u64> {
    use core::arch::x86_64::__cpuid;
    let vendor = __cpuid(0);
    let intel = (vendor.ebx, vendor.edx, vendor.ecx) == (0x756e_6547, 0x4965_6e69, 0x6c65_746e);
    if !intel || vendor.eax < 0x15 {
        return None;
    }
    let ratio = __cpuid(0x15);
    if ratio.eax == 0 || ratio.ebx == 0 {
        return None;
    }
    let crystal = match ratio.ecx {
        0 if vendor.eax >= 0x16 => {
            let base_mhz = u64::from(__cpuid(0x16).eax);
            base_mhz * 1_000_000 * u64::from(ratio.eax) / u64::from(ratio.ebx)
        }
        0 => 0,
        crystal => u64::from(crystal),
    };
    let hz = crystal * u64::from(ratio.ebx) / u64::from(ratio.eax);
    (hz != 0).then_some(hz)
}
