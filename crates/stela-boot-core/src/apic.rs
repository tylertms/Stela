//! The boot processor's local APIC: the xAPIC register page through the
//! address it is mapped at, or the x2APIC MSRs when the APIC is in x2APIC
//! mode (Intel SDM Vol. 3A chapter 11; AMD APM Vol. 2 chapter 16). Its timer
//! is measured against the TSC.
//!
//! Two tiers, chosen once per processor at bring-up ([`choose_mode`]):
//! x2APIC whenever CPUID offers it and the firmware does not opt out
//! ([`honours_x2apic_opt_out`]), else the xAPIC page. In x2APIC mode
//! every access is one RDMSR/WRMSR (no uncached load or store, no
//! instruction a hypervisor must decode), the ICR is one 64-bit write with
//! a 32-bit destination, and it has no delivery status to wait for.
use crate::msr;
use core::arch::asm;

pub mod startup;

/// IA32_APIC_BASE: x2APIC mode enabled.
pub const X2APIC_MODE: u64 = 1 << 10;
/// IA32_APIC_BASE: the APIC is enabled.
const APIC_ENABLED: u64 = 1 << 11;
/// CPUID.1:ECX: the processor offers x2APIC mode.
const CPUID_X2APIC: u32 = 1 << 21;
/// IA32_APIC_BASE: the register page's physical address.
pub const BASE_ADDRESS: u64 = 0x000f_ffff_ffff_f000;

pub const EOI: u32 = 0xb0;
pub const SPURIOUS: u32 = 0xf0;
pub const LVT_TIMER: u32 = 0x320;
pub const LVT_LINT0: u32 = 0x350;
pub const INITIAL: u32 = 0x380;
pub const CURRENT: u32 = 0x390;
pub const DIVIDE: u32 = 0x3e0;
const ID: u32 = 0x20;
const ICR_LOW: u32 = 0x300;
const ICR_HIGH: u32 = 0x310;
const ICR_PENDING: u32 = 1 << 12;
const ICR_LEVEL_ASSERT: u32 = 1 << 14;
const ICR_NMI: u32 = 4 << 8;
const ICR_INIT: u32 = 5 << 8;
const ICR_STARTUP: u32 = 6 << 8;
/// The INIT settling and STARTUP acceptance delays of the MP specification,
/// for processors that need them.
const INIT_WAIT_US: u64 = 10_000;
const SIPI_WAIT_US: u64 = 200;
/// The STARTUP acceptance delay where no INIT settling is needed.
const MODERN_SIPI_WAIT_US: u64 = 10;
const DELIVERY_WAIT_US: u64 = 10_000;
const MICROSECONDS_PER_SECOND: u64 = 1_000_000;

/// Whether this processor's family settles INIT at once, as Linux's
/// `smp_quirk_init_udelay` finds it (Intel family 6 on, AMD 0Fh on, Hygon
/// 18h on): its INIT then takes no delay and STARTUP 10 us.
fn settles_at_once() -> bool {
    use core::arch::x86_64::__cpuid;
    let vendor = __cpuid(0);
    let signature = __cpuid(1).eax;
    let base = signature >> 8 & 0xf;
    let family = if base == 0xf {
        base + (signature >> 20 & 0xff)
    } else {
        base
    };
    match (vendor.ebx, vendor.edx, vendor.ecx) {
        // "GenuineIntel", "AuthenticAMD", "HygonGenuine".
        (0x756e_6547, 0x4965_6e69, 0x6c65_746e) => family >= 6,
        (0x6874_7541, 0x6974_6e65, 0x444d_4163) => family >= 0xf,
        (0x6f67_7948, 0x6e65_476e, 0x656e_6975) => family >= 0x18,
        _ => false,
    }
}

fn init_destination(destination: u32, bsp: u32, x2: bool, tsc_hz: u64) -> Result<(), StartupError> {
    if tsc_hz == 0
        || tsc_hz
            .div_ceil(MICROSECONDS_PER_SECOND)
            .checked_mul(INIT_WAIT_US)
            .is_none()
    {
        return Err(StartupError::ClockUnavailable);
    }
    if destination == bsp {
        return Err(StartupError::BootstrapDestination);
    }
    if !x2 && destination > u32::from(u8::MAX) {
        return Err(StartupError::DestinationTooWide);
    }
    // Physical destination all-ones is broadcast, never one owned logical AP.
    if destination == if x2 { u32::MAX } else { u32::from(u8::MAX) } {
        return Err(StartupError::BroadcastDestination);
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartupError {
    ClockUnavailable,
    DestinationTooWide,
    BootstrapDestination,
    BroadcastDestination,
    InvalidVector,
    DeliveryTimeout,
}
pub const LVT_MASKED: u32 = 1 << 16;
pub const TIMER_PERIODIC: u32 = 1 << 17;
const SOFTWARE_ENABLE: u32 = 1 << 8;
const DIVIDE_BY_ONE: u32 = 0b1011;
/// The calibration runs for 1/CALIBRATION_FRACTION of a second.
const CALIBRATION_FRACTION: u64 = 100;
const PIC_MASTER_DATA: u16 = 0x21;
const PIC_SLAVE_DATA: u16 = 0xa1;

pub fn rdtsc() -> u64 {
    let (low, high): (u32, u32);
    // SAFETY: RDTSC has no side effects.
    unsafe { asm!("rdtsc", out("eax") low, out("edx") high, options(nomem, nostack)) };
    (u64::from(high) << 32) | u64::from(low)
}

/// IA32_APIC_BASE, which every x86-64 CPU has.
pub fn base_msr() -> u64 {
    // SAFETY: reading IA32_APIC_BASE has no side effects.
    unsafe { msr::APIC_BASE.read() }
}

/// The IA32_APIC_BASE value that moves an enabled xAPIC to x2APIC mode,
/// when the processor `offers` it (CPUID.1:ECX[21]): `None` when it stays
/// as it is (already x2APIC, disabled, or no x2APIC). Intel SDM Vol. 3A
/// 11.12.5: xAPIC to x2APIC is one write setting EXTD with EN set, and
/// keeps the APIC's state.
pub fn x2apic_base(base: u64, offers: bool) -> Option<u64> {
    (offers && base & APIC_ENABLED != 0 && base & X2APIC_MODE == 0).then_some(base | X2APIC_MODE)
}

/// DMAR flags bit 1, X2APIC_OPT_OUT (Intel VT-d specification 8.1).
const DMAR_X2APIC_OPT_OUT: u8 = 1 << 1;

/// Whether the boot keeps the xAPIC mode the firmware left although CPUID
/// offers x2APIC: the firmware's DMAR sets X2APIC_OPT_OUT (`dmar_flags`),
/// which firmware does when its SMM code reaches the APIC through the xAPIC
/// page, and every enabled processor's APIC ID (`apic_ids`) fits xAPIC's
/// eight bits below the broadcast ID. Linux honours the flag the same way
/// (`intel_prepare_irq_remapping`). An APIC ID of 255 or more needs x2APIC
/// whatever the firmware asks.
pub fn honours_x2apic_opt_out(dmar_flags: u8, mut apic_ids: impl Iterator<Item = u64>) -> bool {
    dmar_flags & DMAR_X2APIC_OPT_OUT != 0 && apic_ids.all(|id| id < 0xff)
}

/// [`prefer_x2apic`], or with `keep_xapic` ([`honours_x2apic_opt_out`])
/// IA32_APIC_BASE as the firmware left it.
///
/// # Safety
/// As [`prefer_x2apic`].
pub unsafe fn choose_mode(keep_xapic: bool) -> u64 {
    match keep_xapic {
        true => base_msr(),
        // SAFETY: the caller's contract.
        false => unsafe { prefer_x2apic() },
    }
}

/// Put this processor's APIC in x2APIC mode when CPUID offers it, and
/// return IA32_APIC_BASE as it then is. The xAPIC page stays the fallback
/// tier, chosen automatically where x2APIC is not offered.
///
/// # Safety
/// CPL0 with interrupts disabled, and no [`Apic`] of this processor made
/// from the earlier IA32_APIC_BASE is used afterwards: its accesses would
/// go to a page the APIC no longer decodes.
pub unsafe fn prefer_x2apic() -> u64 {
    let base = base_msr();
    let offers = core::arch::x86_64::__cpuid(1).ecx & CPUID_X2APIC != 0;
    match x2apic_base(base, offers) {
        Some(x2) => {
            // SAFETY: the caller's contract; EN is set and CPUID offers
            // x2APIC, so the transition is architectural.
            unsafe { msr::APIC_BASE.write(x2) };
            base_msr()
        }
        None => base,
    }
}

/// How one ICR command is written in each mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IcrWrite {
    /// x2APIC: one 64-bit write, the 32-bit destination in the high half.
    X2(u64),
    /// xAPIC: the high word (destination in bits 31:24), then the low.
    X { high: u32, low: u32 },
}

fn icr_write(x2: bool, destination: u32, command: u32) -> Result<IcrWrite, StartupError> {
    Ok(if x2 {
        IcrWrite::X2(u64::from(destination) << 32 | u64::from(command))
    } else {
        let destination =
            u8::try_from(destination).map_err(|_| StartupError::DestinationTooWide)?;
        IcrWrite::X {
            high: u32::from(destination) << 24,
            low: command,
        }
    })
}

pub struct Apic {
    x2: bool,
    /// Where the xAPIC register page is mapped, uncached.
    registers: u64,
}

impl Apic {
    /// The APIC that `base_msr` (IA32_APIC_BASE) describes, its xAPIC
    /// register page mapped at `registers`.
    pub fn new(base_msr: u64, registers: u64) -> Self {
        Self {
            x2: base_msr & X2APIC_MODE != 0,
            registers,
        }
    }

    pub fn is_x2apic(&self) -> bool {
        self.x2
    }

    pub fn read(&self, register: u32) -> u32 {
        if self.x2 {
            // SAFETY: an x2APIC MSR of this CPU; register `r` of the xAPIC
            // page is MSR 800h + r / 16.
            unsafe { msr::X2APIC.at(register >> 4).read() as u32 }
        } else {
            // SAFETY: the caller mapped the xAPIC page at `registers`.
            unsafe { ((self.registers + u64::from(register)) as *const u32).read_volatile() }
        }
    }

    pub fn write(&self, register: u32, value: u32) {
        if self.x2 {
            // SAFETY: an x2APIC MSR of this CPU.
            unsafe { msr::X2APIC.at(register >> 4).write(u64::from(value)) }
        } else {
            // SAFETY: as in `read`.
            unsafe { ((self.registers + u64::from(register)) as *mut u32).write_volatile(value) };
        }
    }

    pub fn eoi(&self) {
        self.write(EOI, 0);
    }

    pub fn id(&self) -> u32 {
        let value = self.read(ID);
        if self.x2 { value } else { value >> 24 }
    }

    fn wait_delivery(&self, tsc_hz: u64) -> Result<(), StartupError> {
        // x2APIC has no delivery status (Intel SDM Vol. 3A 11.12.9: the bit
        // is removed): a write is accepted when the APIC can take it.
        if self.x2 {
            return Ok(());
        }
        let started = rdtsc();
        let limit = tsc_hz
            .div_ceil(MICROSECONDS_PER_SECOND)
            .checked_mul(DELIVERY_WAIT_US)
            .filter(|ticks| *ticks != 0)
            .ok_or(StartupError::ClockUnavailable)?;
        while self.read(ICR_LOW) & ICR_PENDING != 0 {
            if rdtsc().wrapping_sub(started) >= limit {
                return Err(StartupError::DeliveryTimeout);
            }
            core::hint::spin_loop();
        }
        Ok(())
    }

    fn send(&self, destination: u32, command: u32, tsc_hz: u64) -> Result<(), StartupError> {
        self.wait_delivery(tsc_hz)?;
        self.command(destination, command)?;
        self.wait_delivery(tsc_hz)
    }

    /// Write the ICR, which the last command left idle.
    fn command(&self, destination: u32, command: u32) -> Result<(), StartupError> {
        match icr_write(self.x2, destination, command)? {
            // SAFETY: the local x2APIC ICR, with a physical destination and
            // an explicitly constructed command. A WRMSR to an x2APIC
            // register is not serializing, so without the fences the IPI
            // could reach its destination before the stores announcing its
            // work: Linux's `weak_wrmsr_fence` before `x2apic_send_IPI`.
            IcrWrite::X2(value) => unsafe {
                msr::write_fenced(msr::X2APIC.at(ICR_LOW >> 4), value);
            },
            IcrWrite::X { high, low } => {
                self.write(ICR_HIGH, high);
                self.write(ICR_LOW, low);
            }
        }
        Ok(())
    }

    pub fn validate_init(&self, destination: u32, tsc_hz: u64) -> Result<(), StartupError> {
        init_destination(destination, self.id(), self.x2, tsc_hz)
    }

    fn wait_microseconds(&self, microseconds: u64, tsc_hz: u64) {
        let started = rdtsc();
        // INIT destination validation bounds this multiplication; SIPI's wait
        // is smaller. No firmware timer or callback survives into this path.
        let ticks = tsc_hz.div_ceil(MICROSECONDS_PER_SECOND) * microseconds;
        while rdtsc().wrapping_sub(started) < ticks {
            core::hint::spin_loop();
        }
    }

    /// Directed INIT without SIPI. Intel SDM3A, "Local APIC State After an
    /// INIT Reset (Wait-for-SIPI State)": an AP receiving INIT initializes and
    /// waits for startup. No firmware idle loop or private AP stack is needed.
    /// ICR completion plus the architectural 10 ms settling interval, where
    /// the family needs it, reports delivery only, not an independently
    /// observed destination acknowledgement.
    ///
    /// # Safety
    /// Post-EBS BSP, exclusive local ICR with IF clear; the caller owns this
    /// validated enabled logical AP. Firmware code must not resume afterward;
    /// any later SIPI belongs to a separate complete owned startup sequence.
    pub unsafe fn init_only(&self, destination: u32, tsc_hz: u64) -> Result<(), StartupError> {
        self.validate_init(destination, tsc_hz)?;
        self.send(destination, ICR_INIT | ICR_LEVEL_ASSERT, tsc_hz)?;
        if !settles_at_once() {
            self.wait_microseconds(INIT_WAIT_US, tsc_hz);
        }
        Ok(())
    }

    /// Start one enabled AP at its owned SIPI page after ExitBootServices.
    /// Intel SDM Vol. 3A, multiprocessor initialization: directed INIT,
    /// 10 ms, STARTUP, 200 us, second STARTUP for integrated local APICs;
    /// no INIT delay and 10 us where the family settles at once.
    /// This reports delivery only; the caller must separately verify the AP's
    /// identity, entry acknowledgement and stopped/executor lifetime.
    ///
    /// # Safety
    /// The caller owns the destination processor after firmware shutdown. Its
    /// complete trampoline, tables, stack and argument stay pinned and visible;
    /// interrupts are disabled and no other ICR sender runs on this processor.
    pub unsafe fn startup(
        &self,
        destination: u32,
        vector: u8,
        tsc_hz: u64,
    ) -> Result<(), StartupError> {
        self.validate_init(destination, tsc_hz)?;
        if vector == 0 {
            return Err(StartupError::InvalidVector);
        }
        let (init_us, sipi_us) = if settles_at_once() {
            (0, MODERN_SIPI_WAIT_US)
        } else {
            (INIT_WAIT_US, SIPI_WAIT_US)
        };
        self.send(destination, ICR_INIT | ICR_LEVEL_ASSERT, tsc_hz)?;
        self.wait_microseconds(init_us, tsc_hz);
        self.send(destination, ICR_STARTUP | u32::from(vector), tsc_hz)?;
        self.wait_microseconds(sipi_us, tsc_hz);
        self.send(destination, ICR_STARTUP | u32::from(vector), tsc_hz)
    }

    /// Send one fixed physical doorbell, without waiting to see it delivered:
    /// each ICR access traps to a VMM, and the next command's wait finds an
    /// undelivered one. This acknowledges ICR acceptance only, never a
    /// returned native frame, stopped generation or controller effect.
    ///
    /// # Safety
    /// The destination owns this reserved vector and a live private IDT/IST.
    /// The caller exclusively owns the local ICR with interrupts disabled.
    pub unsafe fn notify(
        &self,
        destination: u32,
        vector: u8,
        tsc_hz: u64,
    ) -> Result<(), StartupError> {
        if vector < 32 || vector == u8::MAX {
            return Err(StartupError::InvalidVector);
        }
        self.wait_delivery(tsc_hz)?;
        self.command(destination, u32::from(vector))
    }

    /// Send one physical NMI, without waiting to see it delivered, as
    /// [`Self::notify`] sends a doorbell.
    ///
    /// # Safety
    /// The destination's NMI gate is live and stops or handles it. The
    /// caller exclusively owns the local ICR with interrupts disabled.
    pub unsafe fn notify_nmi(&self, destination: u32, tsc_hz: u64) -> Result<(), StartupError> {
        self.wait_delivery(tsc_hz)?;
        self.command(destination, ICR_NMI)
    }

    /// Mask the legacy PICs, enable the APIC with `spurious` as its
    /// spurious vector and LINT0 masked, and measure its timer against the
    /// TSC running at `tsc_hz`, with `timer` masked. Returns the timer's
    /// ticks per second at divide-by-one, the divider it is left at.
    pub fn start(&self, spurious: u8, timer: u8, tsc_hz: u64) -> u64 {
        // SAFETY: the legacy PIC data ports; masking every line.
        unsafe {
            asm!("out dx, al", in("dx") PIC_MASTER_DATA, in("al") 0xffu8, options(nomem, nostack));
            asm!("out dx, al", in("dx") PIC_SLAVE_DATA, in("al") 0xffu8, options(nomem, nostack));
        }
        self.write(SPURIOUS, SOFTWARE_ENABLE | u32::from(spurious));
        self.write(LVT_LINT0, LVT_MASKED);
        self.write(DIVIDE, DIVIDE_BY_ONE);
        self.write(LVT_TIMER, LVT_MASKED | u32::from(timer));
        self.write(INITIAL, u32::MAX);
        let start = rdtsc();
        let wait = tsc_hz / CALIBRATION_FRACTION;
        while rdtsc() - start < wait {
            core::hint::spin_loop();
        }
        let elapsed = u64::from(u32::MAX - self.read(CURRENT));
        let tsc = rdtsc() - start;
        self.write(INITIAL, 0);
        (u128::from(elapsed) * u128::from(tsc_hz) / u128::from(tsc.max(1))) as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The tier choice: x2APIC exactly when an enabled xAPIC's processor
    /// offers it; every other bit of IA32_APIC_BASE kept.
    #[test]
    fn x2apic_is_chosen_whenever_offered_and_xapic_stays_the_fallback() {
        let xapic = 0xfee0_0000 | APIC_ENABLED | 1 << 8;
        assert_eq!(x2apic_base(xapic, true), Some(xapic | X2APIC_MODE));
        assert_eq!(x2apic_base(xapic, false), None, "not offered");
        assert_eq!(x2apic_base(xapic | X2APIC_MODE, true), None, "already");
        assert_eq!(x2apic_base(0xfee0_0000, true), None, "disabled");
        let x2 = Apic::new(xapic | X2APIC_MODE, 0);
        let x = Apic::new(xapic, 0xfee0_0000);
        assert!(x2.is_x2apic() && !x.is_x2apic());
    }

    /// The DMAR opt-out keeps xAPIC only while every APIC ID fits it, and
    /// no other DMAR flag does.
    #[test]
    fn dmar_x2apic_opt_out_keeps_xapic_while_every_id_fits() {
        let ids = [0u64, 2, 0x10, 0xfe];
        assert!(honours_x2apic_opt_out(0b010, ids.into_iter()));
        assert!(honours_x2apic_opt_out(0b111, ids.into_iter()));
        assert!(
            !honours_x2apic_opt_out(0b101, ids.into_iter()),
            "no opt-out"
        );
        assert!(!honours_x2apic_opt_out(0, ids.into_iter()));
        assert!(
            !honours_x2apic_opt_out(0b010, [0u64, 0xff].into_iter()),
            "the broadcast ID"
        );
        assert!(
            !honours_x2apic_opt_out(0b010, [0u64, 0x100].into_iter()),
            "an ID only x2APIC reaches"
        );
    }

    /// The same ICR command in both tiers: x2APIC one write with a 32-bit
    /// destination, xAPIC the destination in ICR_HIGH[31:24] and no wider.
    #[test]
    fn icr_commands_match_in_both_modes() {
        let command = ICR_STARTUP | 0x9a;
        assert_eq!(
            icr_write(true, 0x1_0203, command),
            Ok(IcrWrite::X2(0x1_0203 << 32 | u64::from(command)))
        );
        assert_eq!(
            icr_write(false, 0x23, command),
            Ok(IcrWrite::X {
                high: 0x23 << 24,
                low: command
            })
        );
        assert_eq!(
            icr_write(false, 0x100, command),
            Err(StartupError::DestinationTooWide)
        );
        assert_eq!(
            icr_write(true, 0x23, command),
            Ok(IcrWrite::X2(0x23 << 32 | u64::from(command)))
        );
        // An NMI: delivery mode 100b, fixed physical destination, no vector.
        assert_eq!(
            icr_write(false, 0x02, ICR_NMI),
            Ok(IcrWrite::X {
                high: 0x02 << 24,
                low: 0x400
            })
        );
    }

    #[test]
    fn directed_init_excludes_bsp_and_broadcast_for_both_apic_modes() {
        for x2 in [false, true] {
            for bsp in [0, 7, 254] {
                assert_eq!(
                    init_destination(bsp, bsp, x2, 4_700_000_000),
                    Err(StartupError::BootstrapDestination)
                );
            }
            assert_eq!(init_destination(0, 7, x2, 1), Ok(()));
            assert_eq!(init_destination(254, 7, x2, 4_700_000_000), Ok(()));
            assert_eq!(
                init_destination(if x2 { u32::MAX } else { 255 }, 7, x2, 1),
                Err(StartupError::BroadcastDestination)
            );
            assert_eq!(
                init_destination(3, 7, x2, 0),
                Err(StartupError::ClockUnavailable)
            );
        }
        assert_eq!(
            init_destination(256, 7, false, 1),
            Err(StartupError::DestinationTooWide)
        );
        assert_eq!(init_destination(256, 7, true, 1), Ok(()));
        assert_eq!(init_destination(255, 7, true, 1), Ok(()));
        assert_eq!(init_destination(u32::MAX - 1, 7, true, 1), Ok(()));
    }
}
