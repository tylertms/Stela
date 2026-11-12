// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! One claimed MT7925 function taken from its registers to a managed station
//! link: `mt7925_pci_probe` identifies and boots it, `mt7925_regd_update`
//! programs its country, `mt7925_mac_init` and `mt7925_start` bring up its
//! MAC on mac80211's default channel, and `mt7925_add_interface` registers
//! its station interface, which a [`Station`] then keeps connected.
use super::{
    acpi::Sar,
    bootstrap::{
        self, Controller,
        client::{
            self, Client, Environment, LinkControlError, ManagedClient, ManagedLink, PolicyError,
        },
    },
    control,
    firmware::{
        self, LoadError, Requests,
        clc::{Environment as ClcEnvironment, Inputs as ClcInputs},
    },
    identity::{Chip, PciIdentity},
    mcu::device::Context,
    registers::MmioBar,
    ring::MmioQueue,
    station::{Recover, Station, Wifi},
};
use crate::{
    Clock, Mac,
    bus::dma::{mapping::DmaDomain, memory::PageSource},
    firmware::Provider,
    wifi::{
        network::Network,
        regulatory::database::{self, Country, Database},
        stepped::{Stepped, block_on},
    },
};
use alloc::{boxed::Box, format, string::String, sync::Arc, vec::Vec};
use core::{any::Any, fmt, ptr::NonNull, task::Poll};
use stela_driver_base::entropy::Entropy;

/// The services and platform facts the attach borrows.
pub struct Inputs {
    pub firmware: &'static dyn Provider,
    pub pages: &'static dyn PageSource,
    pub clock: Clock,
    /// Whether `owner`'s function raises an MSI the caller has routed.
    pub vector: bool,
    /// `mt76_pci_aspm_supported`: whether the function's link enables ASPM.
    pub aspm: bool,
    /// regulatory.db, whose country the network names, or the world domain
    /// `00` cfg80211 starts from when it names none.
    pub database: Database,
    pub network: Network,
    pub sar: Sar,
    /// The source of the station's handshake nonces.
    pub entropy: Arc<dyn Entropy>,
}

/// Why the function did not reach a managed link. Every owner the device may
/// still reach stays held.
#[derive(Debug)]
pub enum AttachError {
    /// Only MT7925's regulatory programming is ported.
    UnsupportedChip(PciIdentity),
    /// No regulatory database loaded, so the radio may not transmit.
    NoRegulatoryDatabase,
    /// regulatory.db holds no usable rules for the network's country.
    Country([u8; 2], database::Error),
    Firmware(LoadError),
    Bootstrap(bootstrap::Error),
    /// The firmware reports no valid station address and no entropy gives a
    /// random one.
    NoAddress,
    /// The country enables no channel the radio may start on.
    NoChannel,
    Client(client::Error),
    Policy(PolicyError),
    Link(LinkControlError),
}

impl From<bootstrap::Error> for AttachError {
    fn from(error: bootstrap::Error) -> Self {
        Self::Bootstrap(error)
    }
}

impl fmt::Display for AttachError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Firmware(error) => write!(f, "{error}"),
            Self::Bootstrap(error) => write!(f, "{error}"),
            Self::Link(error) => write!(f, "{error}"),
            Self::NoRegulatoryDatabase => f.write_str("MT7925: no regulatory database"),
            Self::Country(alpha2, error) => write!(
                f,
                "MT7925: regulatory.db country {}: {error:?}",
                alpha2.escape_ascii()
            ),
            error => write!(f, "MT7925 attach: {error:?}"),
        }
    }
}

/// A kept station connection and what limits it without stopping it.
pub struct Attached<C: 'static> {
    pub station: Wifi<C>,
    /// The firmware offers its own regulatory domain
    /// (`MT7925_REGD_SUPPORTED`). The link programs and verifies
    /// regulatory.db's country instead, so the firmware's stays unused.
    pub firmware_domain_unused: bool,
    /// The firmware reported no valid station address, so the station took
    /// a random locally administered one, as `mt76_eeprom_override` does.
    pub random_address: bool,
}

/// `mt76_eeprom_override`: the firmware's station address when it is a
/// valid unicast address, else `eth_random_addr`'s random locally
/// administered unicast address; `None` only when no entropy is available.
fn station_address(reported: Option<Mac>, entropy: &dyn Entropy) -> Option<(Mac, bool)> {
    if let Some(mac) = reported.filter(|mac| !mac.is_multicast() && mac.0 != [0; 6]) {
        return Some((mac, false));
    }
    let mut bytes = [0; 6];
    entropy.fill(&mut bytes).ok()?;
    bytes[0] = (bytes[0] & 0xfe) | 0x02;
    Some((Mac(bytes), true))
}

/// `MT792x_MCU_INIT_RETRY_COUNT`: how many times a bring-up is tried, each
/// after a WLAN subsystem reset, before the function is given up.
pub const BRING_UP_ATTEMPTS: u32 = 10;

/// Whether a failed bring-up may succeed after a WLAN subsystem reset, as
/// `mt7925_init_hardware` retries `__mt7925_init_hardware`. A function that
/// stopped answering (`bus_hung`), missing firmware or regulatory input, or
/// a refused chip never will.
fn retryable(error: &AttachError) -> bool {
    match error {
        AttachError::Bootstrap(bootstrap::Error::Control(
            control::Error::Unavailable { .. } | control::Error::Register(_),
        )) => false,
        AttachError::Bootstrap(_) => true,
        _ => false,
    }
}

/// Run `attempt` until it succeeds, at most [`BRING_UP_ATTEMPTS`] times,
/// while its failure is [`retryable`].
async fn with_retries<T>(
    mut attempt: impl AsyncFnMut() -> Result<T, AttachError>,
) -> Result<T, AttachError> {
    let mut tries = 0;
    loop {
        tries += 1;
        match attempt().await {
            Err(error) if tries < BRING_UP_ATTEMPTS && retryable(&error) => {}
            outcome => return outcome,
        }
    }
}

/// An attached function's managed link.
pub type Link<C> = ManagedLink<MmioBar, MmioQueue, C, &'static dyn PageSource>;
/// A function's controller.
type Native<C> = Controller<MmioBar, MmioQueue, C>;
/// A failed bring-up, with the controller to release after the next reset.
type Failed<C> = (AttachError, Option<Native<C>>);

/// A function brought up to a managed link.
pub struct BroughtUp<C: 'static> {
    pub link: Link<C>,
    pub interface: Context,
    pub firmware_domain_unused: bool,
    pub random_address: bool,
}

/// Everything a bring-up of one claimed MT7925 needs, kept so that the
/// function can be brought up again after a failed attempt or a chip reset.
pub struct Hardware<C: 'static> {
    pci: PciIdentity,
    base: NonNull<u8>,
    bytes: usize,
    resource: Arc<C>,
    domain: Arc<dyn DmaDomain>,
    firmware: &'static dyn Provider,
    pages: &'static dyn PageSource,
    clock: Clock,
    vector: bool,
    aspm: bool,
    alpha2: [u8; 2],
    country: Country,
    sar: Sar,
    entropy: Arc<dyn Entropy>,
    /// Controllers of failed attempts, released once a later attempt's WLAN
    /// subsystem reset has stopped their DMA.
    failed: Vec<Native<C>>,
    /// The bring-up runs from station polls: every command waits at most the
    /// runtime bound.
    polled: bool,
}

impl<C: 'static> Hardware<C> {
    /// Bring the function up, retried after a WLAN subsystem reset as
    /// `mt7925_init_hardware` does. `address` keeps a station address over
    /// a chip reset; `None` takes the firmware's.
    pub async fn bring_up(&mut self, address: Option<Mac>) -> Result<BroughtUp<C>, AttachError> {
        with_retries(async || self.attempt(address).await).await
    }

    /// Release the controllers of failed attempts after a WLAN subsystem
    /// reset under a newer controller; one that does not release stays held.
    fn release_failed(&mut self) {
        for mut controller in self.failed.drain(..) {
            // SAFETY: the newer controller's install reset the WLAN subsystem
            // (WFSYS) with WFDMA stopped and every ring base reprogrammed, so
            // the failed controller's firmware and DMA no longer reach its
            // memory, and nothing else uses it.
            if unsafe { controller.retire_after_quiesce() }.is_err() {
                core::mem::forget(controller);
            }
        }
    }

    async fn attempt(&mut self, address: Option<Mac>) -> Result<BroughtUp<C>, AttachError> {
        let owner: Arc<dyn Any> = self.resource.clone();
        // SAFETY: `attach`'s contract holds for the kept window and owner.
        let identity = unsafe {
            bootstrap::identify(
                self.pci,
                self.base,
                self.bytes,
                owner.clone(),
                self.clock,
                self.aspm,
            )
        }
        .await?;
        // Both images load before the controller claims the function.
        let prepared = Requests::new(&identity)
            .load(self.firmware)
            .map_err(AttachError::Firmware)?;
        // SAFETY: as above.
        let mut controller = unsafe {
            Controller::new(
                identity,
                self.base,
                self.bytes,
                owner,
                self.resource.clone(),
                self.clock,
            )
        }?;
        if self.polled {
            controller.bound_commands();
        }
        // SAFETY: as above, with the kept domain and page source.
        let installed =
            unsafe { controller.install(self.pages, self.domain.clone(), self.aspm) }.await;
        if controller.phase() != bootstrap::Phase::Fresh {
            // The install reset the WLAN subsystem before any ring.
            self.release_failed();
        }
        match self
            .program(controller, installed, &prepared, address)
            .await
        {
            Ok(up) => Ok(up),
            Err((error, controller)) => {
                if let Some(controller) = controller {
                    self.failed.push(controller);
                }
                Err(error)
            }
        }
    }

    #[allow(
        clippy::result_large_err,
        reason = "a failed bring-up hands back its controller for release after the next reset"
    )]
    async fn program(
        &self,
        mut controller: Native<C>,
        installed: Result<(), bootstrap::Error>,
        prepared: &firmware::Prepared<'_>,
        address: Option<Mac>,
    ) -> Result<BroughtUp<C>, Failed<C>> {
        let steps: Result<_, AttachError> = async {
            installed?;
            controller.boot(prepared).await?;
            let chip_capabilities = controller
                .capabilities()
                .and_then(|report| report.observed().chip_flags())
                .unwrap_or(0);
            // SAFETY: `resource` keeps the function's policy ownership.
            // Connac3's request carries none of Connac2's ACPI, power-node or
            // MTCL fields, and the CLC's environment is the OS's
            // `country_ie_env`, ENVIRON_ANY.
            unsafe {
                controller.execute_clc(ClcInputs {
                    alpha2: self.alpha2,
                    environment: ClcEnvironment::Any,
                    chip_capabilities,
                    power_limits_node: false,
                    power_type: 0,
                    acpi_flags: 0,
                    mtcl_configuration: 0,
                    world_mtcl_configuration: 0,
                })
            }
            .await?;
            let plan = controller.regulatory_plan(&self.country, &self.sar)?;
            let firmware_domain_unused = controller.firmware_regulatory_domain();
            // SAFETY: the plan resolves the CLC's restrictions, the country
            // and the platform's SAR, and `resource` keeps them.
            unsafe { controller.execute_remaining(plan.inputs()) }.await?;
            // SAFETY: no peer, WTBL or key user exists yet.
            unsafe { controller.initialize_mac() }.await?;
            let channel = plan.default_channel().ok_or(AttachError::NoChannel)?;
            // SAFETY: the programmed domain holds `channel`.
            unsafe { controller.start(channel.band, channel.number as u8, channel.frequency_mhz) }
                .await?;
            let (address, random_address) = match address {
                Some(address) => (address, false),
                None => station_address(
                    controller
                        .capabilities()
                        .and_then(|report| report.observed().mac()),
                    &*self.entropy,
                )
                .ok_or(AttachError::NoAddress)?,
            };
            // `mt7925_add_interface`'s first interface: BSS, OMAC and WMM 0,
            // no band yet, and the broadcast WCID `MT792x_WTBL_RESERVED - 0`.
            let context = Context {
                bss_index: 0,
                omac_index: 0,
                band_index: 0xff,
                wmm_index: 0,
                link_index: 0,
                broadcast_wcid: 19,
                address,
            };
            // SAFETY: the first interface of a fresh controller.
            let interface = unsafe { controller.register_interface(context) }.await?;
            Ok((interface, context, firmware_domain_unused, random_address))
        }
        .await;
        let (interface, context, firmware_domain_unused, random_address) = match steps {
            Ok(done) => done,
            Err(error) => return Err((error, Some(controller))),
        };
        // SAFETY: the controller and interface are this requester's, after its
        // completed bootstrap and policy; `resource` keeps their permission.
        let client = unsafe { Client::new(controller, interface) }
            .map_err(|e| (AttachError::Client(e), None))?;
        // cfg80211 applies INDOOR_ONLY only to GO/AP initiation, never to a
        // managed station following its AP (chan.c cfg80211_ir_permissive_chan),
        // so a station on an indoor-only channel joins as on Linux. The model
        // tests of the managed link run under the same environment.
        let managed = ManagedClient::new(client, self.country.clone(), Environment::Indoor)
            .map_err(|error| (AttachError::Policy(error.error), None))?;
        let link = ManagedLink::new(managed, self.pages)
            .map_err(|error| (AttachError::Link(error.error), None))?;
        Ok(BroughtUp {
            link: if self.vector {
                link.with_vector()
            } else {
                link
            },
            interface: context,
            firmware_domain_unused,
            random_address,
        })
    }
}

/// One recovery attempt: the bring-up of the reset function with the
/// station's address, the hardware handed back for the next attempt.
async fn recovery<C: 'static>(
    mut hardware: Hardware<C>,
    address: Mac,
) -> (Hardware<C>, Result<BroughtUp<C>, AttachError>) {
    let result = hardware.attempt(Some(address)).await;
    (hardware, result)
}

/// A running recovery attempt, allocated once when it starts.
type Attempt<C> = Stepped<'static, (Hardware<C>, Result<BroughtUp<C>, AttachError>)>;

/// `mt7925_mac_reset_work`'s chip reset for a lost station, run a bounded
/// step per keeper poll: each attempt resets the WLAN subsystem and brings
/// the function up again with the station's address.
pub struct Recoverer<C: 'static> {
    hardware: Option<Hardware<C>>,
    attempt: Option<Attempt<C>>,
}

impl<C: 'static> Recoverer<C> {
    fn new(mut hardware: Hardware<C>) -> Self {
        hardware.polled = true;
        Self {
            hardware: Some(hardware),
            attempt: None,
        }
    }
}

impl<C: 'static> Recover<MmioBar, MmioQueue, C, &'static dyn PageSource> for Recoverer<C> {
    fn step(&mut self, address: Mac) -> Poll<Result<(Link<C>, Context), String>> {
        if self.attempt.is_none() {
            let Some(hardware) = self.hardware.take() else {
                return Poll::Ready(Err(String::from("MT7925: the recovery lost its hardware")));
            };
            self.attempt = Some(Stepped::new(recovery(hardware, address)));
        }
        let Poll::Ready((hardware, result)) = self.attempt.as_mut().unwrap().step() else {
            return Poll::Pending;
        };
        self.attempt = None;
        self.hardware = Some(hardware);
        Poll::Ready(
            result
                .map(|up| (up.link, up.interface))
                .map_err(|error| format!("{error}")),
        )
    }
}

/// Identify, boot and program the function `pci` names, then register its
/// station interface and keep it connected to `inputs.network`.
///
/// # Safety
/// [`Controller::new`]'s and [`Controller::install`]'s contracts hold for
/// `base..base + bytes`, `resource` and `domain`, with `inputs.pages` as the
/// page source: `resource` keeps the claimed function and its uncached BAR0
/// mapping, whose interrupts are off. They hold again for each later
/// bring-up, which first resets the WLAN subsystem.
pub unsafe fn attach<C: 'static>(
    pci: PciIdentity,
    base: NonNull<u8>,
    bytes: usize,
    resource: Arc<C>,
    domain: Arc<dyn DmaDomain>,
    inputs: Inputs,
) -> Result<Attached<C>, AttachError> {
    if pci.chip() != Some(Chip::Mt7925) {
        return Err(AttachError::UnsupportedChip(pci));
    }
    let alpha2 = inputs.network.country.unwrap_or(*b"00");
    let country = inputs
        .database
        .select(alpha2)
        .map_err(|error| AttachError::Country(alpha2, error))?;
    let mut hardware = Hardware {
        pci,
        base,
        bytes,
        resource,
        domain,
        firmware: inputs.firmware,
        pages: inputs.pages,
        clock: inputs.clock,
        vector: inputs.vector,
        aspm: inputs.aspm,
        alpha2,
        country: country.clone(),
        sar: inputs.sar,
        entropy: inputs.entropy.clone(),
        failed: Vec::new(),
        polled: false,
    };
    // The attach runs the same stepped bring-up a recovery steps through,
    // to completion.
    let up = block_on(hardware.bring_up(None))?;
    Ok(Attached {
        station: Station::new(
            up.link,
            up.interface,
            country,
            inputs.network,
            inputs.entropy,
            inputs.clock,
        )
        .with_recovery(Box::new(Recoverer::new(hardware))),
        firmware_domain_unused: up.firmware_domain_unused,
        random_address: up.random_address,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use stela_driver_base::entropy::EntropyError;

    struct Fixed(u8);
    impl Entropy for Fixed {
        fn fill(&self, output: &mut [u8]) -> Result<(), EntropyError> {
            output.fill(self.0);
            Ok(())
        }
    }
    struct Empty;
    impl Entropy for Empty {
        fn fill(&self, _: &mut [u8]) -> Result<(), EntropyError> {
            Err(EntropyError::Unavailable)
        }
    }

    /// The largest a recovery attempt's state may grow, allocated once per
    /// attempt: a bring-up's awaits keep their locals across steps, and a
    /// change that makes this larger should be seen.
    /// 2584 bytes when this bound was set.
    const RECOVERY_ATTEMPT_BYTES: usize = 4 * 1024;

    #[test]
    fn a_recovery_attempt_is_one_bounded_allocation() {
        let memory = Arc::new(std::sync::Mutex::new(
            super::super::ring::tests::Memory::default(),
        ));
        let firmware: &'static dyn Provider =
            alloc::boxed::Box::leak(alloc::boxed::Box::new(firmware::tests::catalog(
                &firmware::tests::requests(Chip::Mt7925),
                Vec::new(),
                Vec::new(),
            )));
        let hardware = Hardware {
            pci: PciIdentity {
                vendor: 0x14c3,
                device: 0x7925,
                subsystem_vendor: 0,
                subsystem_device: 0,
                revision: 0,
            },
            base: NonNull::dangling(),
            bytes: 0x200000,
            resource: Arc::new(()),
            domain: Arc::new(super::super::ring::tests::Domain(memory)),
            firmware,
            pages: &super::super::ring::tests::Source,
            clock: Clock {
                ticks_per_microsecond: 1,
            },
            vector: false,
            aspm: false,
            alpha2: *b"US",
            country: crate::wifi::regulatory::database::tests::country(),
            sar: Sar::default(),
            entropy: Arc::new(Fixed(1)),
            failed: Vec::new(),
            polled: true,
        };
        // Never polled: only its size is read.
        let attempt = recovery(hardware, Mac([2, 0, 0, 0, 0, 1]));
        let bytes = core::mem::size_of_val(&attempt);
        assert!(bytes <= RECOVERY_ATTEMPT_BYTES, "{bytes} bytes");
    }

    #[test]
    fn an_invalid_or_missing_address_becomes_a_random_local_one_as_mt76_does() {
        let good = Mac([0x00, 0x0a, 0xf5, 1, 2, 3]);
        assert_eq!(
            station_address(Some(good), &Fixed(0xff)),
            Some((good, false))
        );
        for reported in [None, Some(Mac([0; 6])), Some(Mac([0x01, 0, 0, 0, 0, 1]))] {
            let (mac, random) = station_address(reported, &Fixed(0xff)).unwrap();
            assert!(random);
            assert_eq!(mac.0, [0xfe, 0xff, 0xff, 0xff, 0xff, 0xff]);
            assert!(!mac.is_multicast() && mac.0[0] & 2 != 0);
        }
        assert_eq!(station_address(None, &Empty), None);
    }

    #[test]
    fn a_failed_bring_up_is_retried_ten_times_unless_the_function_is_gone() {
        let firmware = || AttachError::Bootstrap(bootstrap::Error::FirmwareStatus(1));
        let gone = || {
            AttachError::Bootstrap(bootstrap::Error::Control(control::Error::Unavailable {
                address: 0,
            }))
        };
        // Always failing: ten attempts, as MT792x_MCU_INIT_RETRY_COUNT.
        let mut tries = 0;
        assert!(
            block_on(with_retries(async || -> Result<(), _> {
                tries += 1;
                Err(firmware())
            }))
            .is_err()
        );
        assert_eq!(tries, BRING_UP_ATTEMPTS);
        // A later attempt succeeds.
        let mut tries = 0;
        assert_eq!(
            block_on(with_retries(async || {
                tries += 1;
                if tries < 4 {
                    Err(firmware())
                } else {
                    Ok(tries)
                }
            }))
            .unwrap(),
            4
        );
        // A function that stopped answering, or missing inputs, end at once.
        for error in [gone(), AttachError::NoChannel, AttachError::NoAddress] {
            let mut error = Some(error);
            let mut tries = 0;
            assert!(
                block_on(with_retries(async || -> Result<(), _> {
                    tries += 1;
                    Err(error.take().unwrap_or_else(firmware))
                }))
                .is_err()
            );
            assert_eq!(tries, 1);
        }
    }
}
