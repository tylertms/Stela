//! The mount's own network client over the host's first network adapter:
//! the restore client's signing-server requests (TSS), the
//! connectivity check (`--network-check`) and a remote install image's
//! ranged downloads (`engine::remote`), which share it.
//!
//! UEFI's Simple Network Protocol ends with boot services, and the request
//! is made during the restore, long after ExitBootServices, so the client
//! runs the runtime's dual-stack client (`stela_runtime::net`) over the adapter's
//! after-EBS driver (`stela_drivers::net::virtio`, `stela_drivers::net::igc`
//! for an Intel I225/I226, `stela_drivers::net::e1000e` for an Intel 82574L,
//! `stela_drivers::net::igb` for supported Intel 82575–I211 copper adapters,
//! or `stela_drivers::usb::cdc` for a USB Ethernet
//! adapter). Its registers and DMA pages are identity-mapped in the
//! loader's root, so every frame moves inside that root
//! (`storage::RestoreRoot`), as block and input I/O do.
mod security;
use super::storage::RestoreRoot;
use crate::Refusal;
use crate::Result;
use crate::output;
use alloc::{format, rc::Rc, string::String, sync::Arc, vec::Vec};
use core::{
    cell::{Cell, RefCell},
    mem::ManuallyDrop,
};
use stela_contracts::refuse;
use stela_drivers::{
    Clock,
    net::{Link, LinkError, Mac},
    probe::Adapter,
};
use stela_runtime::net::{
    Backoff, HttpPost, HttpRequest, HttpTransfer, NetError, Stack, TlsUnavailable, Transfer,
    stack::DOWNLOAD_BUFFER,
};
use stela_soc::machine::AppleDevices;

/// One restore-time opaque TCP relay window. TLS stays end to end between the
/// original Apple guest process and its requested server.
pub const RELAY_BUFFER: usize = 1 << 20;

/// No adapter a native driver runs: what to connect, as the screen says it.
const NO_ADAPTER: &str = "No supported network adapter. Connect wired Ethernet: most Intel and Realtek adapters, Aquantia adapters, Broadcom's BCM5720, BCM5761 and BCM5762, or a Realtek or ASIX USB adapter.";
/// What the screen says when DHCP gives the first adapter no lease.
const NO_LEASE: &str = "No automatic network address. Connect Ethernet to a network with DHCPv4 or IPv6 router advertisements, or set an address with `stela settings --ipv4 ADDRESS/PREFIX` on another computer. Stateful DHCPv6 is unsupported.";

/// The host's network adapters and the loader root they are reached in.
/// Adapters left in it are dropped in that root.
pub struct Links {
    pub devices: Vec<Adapter>,
    pub root: u64,
}

impl Drop for Links {
    fn drop(&mut self) {
        let devices = core::mem::take(&mut self.devices);
        if !devices.is_empty() {
            in_loader_root(self.root, || drop(devices));
        }
    }
}

/// Run `release` in the loader's root, the only root that maps the host
/// devices' registers and DMA pages: dropping a device can drive it (`Igc`
/// stops its controller, a USB Bluetooth controller closes its endpoints),
/// and outside that root its first register access faults. A processor already inside the host drivers
/// (`storage::open`'s probe) is in that root and holds their lock.
pub(super) fn in_loader_root(root: u64, release: impl FnOnce()) {
    let _root = (!super::storage::host_drivers_held_here()).then(|| {
        // SAFETY: the loader root `storage::open` captured, which maps the
        // adapters; the processor's pinned record is active, as for every
        // `RootLink` call.
        unsafe { RestoreRoot::enter(root) }
    });
    release();
}

struct RootLink {
    /// Dropped only in the loader's root ([`in_loader_root`]).
    device: ManuallyDrop<Adapter>,
    root: u64,
    frames: Rc<Frames>,
}

/// The frames an adapter took and gave, for the log line of an attempt that
/// got no address: an adapter with a link that received nothing is told
/// apart from a network that never answered.
#[derive(Default)]
struct Frames {
    sent: Cell<u64>,
    received: Cell<u64>,
}

impl Frames {
    fn count(cell: &Cell<u64>) {
        cell.set(cell.get().saturating_add(1));
    }
}

impl RootLink {
    fn new(device: Adapter, root: u64) -> Self {
        Self {
            device: ManuallyDrop::new(device),
            root,
            frames: Rc::default(),
        }
    }

    /// Send `frame` inside the host drivers.
    fn send(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        self.device
            .transmit(frame)
            .inspect(|()| Frames::count(&self.frames.sent))
    }

    /// The adapter's next frame, inside the host drivers.
    fn arrived(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.device.receive().inspect(|frame| {
            if frame.is_some() {
                Frames::count(&self.frames.received);
            }
        })
    }

    /// [`Link::transmit`] for a service turn, which does not wait for the
    /// host drivers' lock: `None` while another processor is inside them.
    fn transmit_now(&mut self, frame: &[u8]) -> Option<Result<(), LinkError>> {
        // SAFETY: as in `carrier`.
        let _root = unsafe { RestoreRoot::try_enter(self.root) }?;
        Some(self.send(frame))
    }

    /// [`Link::receive`] for a service turn, as [`Self::transmit_now`].
    fn receive_now(&mut self) -> Option<Result<Option<Vec<u8>>, LinkError>> {
        // SAFETY: as in `carrier`.
        let _root = unsafe { RestoreRoot::try_enter(self.root) }?;
        Some(self.arrived())
    }
}

impl Drop for RootLink {
    fn drop(&mut self) {
        // SAFETY: taken once, here; the field is never used again.
        let device = unsafe { ManuallyDrop::take(&mut self.device) };
        in_loader_root(self.root, || drop(device));
    }
}

impl Link for RootLink {
    fn mac(&self) -> Mac {
        self.device.mac()
    }
    fn carrier(&mut self) -> bool {
        // SAFETY: the loader root `storage::open` captured maps this
        // device.
        let _root = unsafe { RestoreRoot::enter(self.root) };
        self.device.carrier()
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        // SAFETY: as in `carrier`.
        let _root = unsafe { RestoreRoot::enter(self.root) };
        self.send(frame)
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        // SAFETY: as in `carrier`.
        let _root = unsafe { RestoreRoot::enter(self.root) };
        self.arrived()
    }
    fn receive_reusing(&mut self, frame: &mut Vec<u8>) -> Result<bool, LinkError> {
        // SAFETY: as in `carrier`.
        let _root = unsafe { RestoreRoot::enter(self.root) };
        self.device.receive_reusing(frame).inspect(|&received| {
            if received {
                Frames::count(&self.frames.received);
            }
        })
    }
    fn signals(&self) -> bool {
        self.device.signals()
    }
    fn carrier_wait_us(&self) -> u64 {
        self.device.carrier_wait_us()
    }
    fn due_us(&self) -> Option<u64> {
        self.device.due_us()
    }
}

/// The PC's clock, moving the screen's running step as the stack waits.
struct Ticking(Clock);

impl stela_runtime::net::Clock for Ticking {
    fn now_us(&self) -> Result<u64, stela_runtime::net::ClockError> {
        crate::status::tick();
        self.0
            .try_now_us()
            .map_err(stela_runtime::net::ClockError::Calibration)
    }
    /// The adapter's completion signal wakes the boot processor, which runs
    /// the stack.
    fn sleep(&self, until_us: u64) -> Result<(), stela_runtime::net::ClockError> {
        let now = self
            .0
            .try_now_us()
            .map_err(stela_runtime::net::ClockError::Calibration)?;
        if until_us > now {
            super::host::sleep_for(until_us - now);
        }
        Ok(())
    }
}

pub struct HostNetwork {
    stack: Stack<RootLink, Ticking>,
    security: Result<Arc<stela_tls::ClientConfig>, TlsUnavailable>,
    /// The address and adapter, for the screen.
    description: String,
}

impl HostNetwork {
    /// Try each native adapter until DHCPv4 or IPv6 SLAAC provides a usable
    /// address. A failed adapter keeps its diagnostic while the next is tried.
    pub fn open(
        mut links: Links,
        clock: Clock,
        handoff: &crate::handoff::Handoff,
        options: &stela_boot_core::options::System,
    ) -> Result<Self> {
        let security = security::native(handoff);
        match &security {
            Ok(_) => output::arguments(format_args!(
                "NETWORK TLS certificate time: the firmware RTC carried forward on the invariant TSC\n"
            )),
            Err(unavailable) => {
                output::arguments(format_args!("NETWORK TLS unavailable: {unavailable}\n"))
            }
        }
        let mut failures = Vec::new();
        // Each adapter is owned by a `RootLink` or still by `links` on every
        // path out, so it is dropped in the loader's root.
        while !links.devices.is_empty() {
            let link = RootLink::new(links.devices.remove(0), links.root);
            let driver = link.device.driver();
            let wireless = link.device.wireless();
            let location = String::from(link.device.location());
            let frames = Rc::clone(&link.frames);
            output::arguments(format_args!(
                "NETWORK candidate driver={driver} adapter={location} mac={}\n",
                link.mac()
            ));
            crate::status::progress(&format!("adapter {location}"));
            if matches!(&*link.device, Adapter::Realtek(_)) && link.due_us().is_some() {
                crate::status::info("RTL8168H network v5: polling + bounded RX recovery");
            }
            let mut seed = [0u8; 8];
            crate::random::fill(&mut seed)
                .map_err(|_| refuse!("secure host entropy is unavailable for the network stack"))?;
            let mut stack = Stack::new(link, Ticking(clock), u64::from_le_bytes(seed))
                .map_err(|error| refuse!("host network stack: {error}", %error))?;
            if let Some(ipv4) = options.ipv4 {
                stack
                    .set_ipv4(ipv4.address, ipv4.prefix, options.ipv4_gateway)
                    .map_err(|error| refuse!("host network --ipv4 {ipv4}: {error}", %error))?;
            }
            if !options.dns.is_empty() {
                stack
                    .set_dns(&options.dns)
                    .map_err(|error| refuse!("host network --dns: {error}", %error))?;
            }
            let waiting = |seconds: u64| {
                crate::status::progress(&format!(
                    "adapter {location}: waiting for the link, {seconds} s"
                ));
            };
            match stack.automatic_waiting(waiting) {
                Ok(config) => {
                    output::arguments(format_args!("NETWORK driver={driver}\n"));
                    output::arguments(format_args!(
                        "NETWORK adapter={location} addresses={:?} dns={:?}\n",
                        config.addresses, config.dns
                    ));
                    let addresses: Vec<String> = config
                        .addresses
                        .iter()
                        .map(alloc::string::ToString::to_string)
                        .collect();
                    let description = format!("{} on adapter {location}", addresses.join(", "));
                    return Ok(Self {
                        stack,
                        description,
                        security,
                    });
                }
                Err(error) => {
                    output::arguments(format_args!(
                        "NETWORK adapter {location} failed: {error}; frames sent={} received={}\n",
                        frames.sent.get(),
                        frames.received.get()
                    ));
                    // MMIO and DMA are mapped only in the loader root. Capture
                    // the device before dropping it or leaving this root.
                    let diagnostic = {
                        let link = stack.link_mut();
                        // SAFETY: as in RootLink::receive; this live adapter
                        // retains the loader root's MMIO and DMA mappings.
                        let _root = unsafe { RestoreRoot::enter(link.root) };
                        match &*link.device {
                            Adapter::Realtek(device) => Some(device.diagnostics()),
                            _ => None,
                        }
                    };
                    failures.push(match error {
                        stela_runtime::net::NetError::NoCarrier => {
                            stela_drivers::probe::no_link(&location, wireless)
                        }
                        error => format!("{location}: no address ({error})."),
                    });
                    if let Some(diagnostic) = diagnostic {
                        output::arguments(format_args!("NETWORK {diagnostic}\n"));
                        failures.push(diagnostic);
                    }
                    // Its driver stops it, in the loader's root (`RootLink`).
                    drop(stack);
                    output::arguments(format_args!("NETWORK adapter {location} released\n"));
                }
            }
        }
        if failures.is_empty() {
            Err(refuse!("{NO_ADAPTER}", NO_ADAPTER))
        } else {
            // One line each: the screen shows every line of the refusal on
            // its own.
            Err(refuse!(
                "{NO_LEASE}\n{failures}",
                NO_LEASE,
                failures = failures.join("\n")
            ))
        }
    }

    /// The address and adapter, as the screen names them.
    pub fn describe(&self) -> &str {
        &self.description
    }

    /// Capture the native adapter state while its MMIO mappings are active.
    pub fn adapter_diagnostics(&mut self) -> Option<String> {
        let link = self.stack.link_mut();
        // SAFETY: as in RootLink::receive; this live adapter retains the
        // loader root's MMIO and DMA mappings.
        let _root = unsafe { RestoreRoot::enter(link.root) };
        match &*link.device {
            Adapter::Realtek(device) => Some(device.diagnostics()),
            _ => None,
        }
    }

    /// The connectivity check's POST (`--network-check`), waiting for its
    /// response. Redirects keep the full resulting URL and can never
    /// downgrade an HTTPS hop.
    #[cfg(feature = "diagnostics")]
    pub fn post_url(
        &mut self,
        url: &str,
        headers: &[(&str, &str)],
        content_type: &str,
        body: &[u8],
    ) -> Result<Vec<u8>, NetError> {
        let result = self
            .stack
            .post_url(url, headers, content_type, body, self.security.clone());
        match &result {
            Ok(reply) => output::arguments(format_args!(
                "NETWORK POST {url} sent={} received={}\n",
                body.len(),
                reply.len()
            )),
            Err(error) => output::arguments(format_args!("NETWORK POST {url} refused: {error}\n")),
        }
        result
    }

    /// Starts a POST without waiting ([`Self::read_post`]).
    pub fn open_post(
        &mut self,
        url: &str,
        headers: &[(&str, &str)],
        content_type: &str,
        body: &[u8],
    ) -> Result<HttpPost, NetError> {
        self.stack
            .open_post(url, headers, content_type, body, self.security.clone())
            .inspect_err(|error| {
                output::arguments(format_args!(
                    "NETWORK POST {url} refused: {error}
"
                ))
            })
    }

    /// The POST's response body once it has arrived, `None` while it is on
    /// its way.
    pub fn read_post(&mut self, post: &mut HttpPost) -> Result<Option<Vec<u8>>, NetError> {
        let result = self.stack.read_post(post);
        let url = post.url();
        match &result {
            Ok(Some(reply)) => output::arguments(format_args!(
                "NETWORK POST {url} sent={} received={}
",
                post.sent(),
                reply.len()
            )),
            Ok(None) => {}
            Err(error) => output::arguments(format_args!(
                "NETWORK POST {url} refused: {error}
"
            )),
        }
        result
    }

    pub fn close_post(&mut self, post: HttpPost) {
        self.stack.close_post(post);
    }

    pub fn check_url(&self, url: &str) -> Result<(), NetError> {
        let destination = stela_runtime::net::http::Url::parse(url).map_err(NetError::Url)?;
        if destination.scheme == stela_runtime::net::http::Scheme::Https {
            self.security
                .as_ref()
                .map_err(|error| NetError::TlsUnavailable(*error))?;
        }
        Ok(())
    }
    pub fn download(
        &mut self,
        url: &str,
        first: u64,
        last: u64,
        etag: &str,
    ) -> Result<HttpTransfer, NetError> {
        self.stack.open_http(
            url,
            HttpRequest::range(first, last, Some(etag)),
            DOWNLOAD_BUFFER,
            self.security.clone(),
        )
    }
    pub fn read_download(
        &mut self,
        transfer: &mut HttpTransfer,
        take: impl FnMut(&[u8], bool) -> Result<bool, NetError>,
    ) -> Result<bool, NetError> {
        self.stack.read_http(transfer, take)
    }
    pub fn close_download(&mut self, transfer: HttpTransfer) {
        self.stack.close_http(transfer);
    }

    pub fn close(&mut self, transfer: Transfer) {
        self.stack.close(transfer);
    }

    /// Waits up to `micros` for the adapter's next frame (`Stack::wait`).
    pub fn wait(&mut self, micros: u64) -> Result<(), NetError> {
        self.stack.wait(micros)
    }

    /// Opens one opaque TCP stream for the restore reverse proxy.
    pub fn connect(&mut self, host: &str, port: u16) -> Result<Transfer, NetError> {
        self.stack.open_raw(host, port, RELAY_BUFFER)
    }

    /// Sends caller-kept opaque bytes, returning zero under backpressure.
    pub fn relay_write(
        &mut self,
        transfer: &mut Transfer,
        bytes: &[u8],
    ) -> Result<usize, NetError> {
        self.stack.write(transfer, bytes)
    }

    /// Receives opaque bytes. `take` owns framing and reports when the remote
    /// close has completed the relay.
    pub fn relay_read(
        &mut self,
        transfer: &mut Transfer,
        take: impl FnMut(&[u8], bool) -> Result<bool, NetError>,
    ) -> Result<bool, NetError> {
        self.stack.read(transfer, take)
    }
}

/// The host network, shared by the signing transport and a remote install
/// image; only the service reaches it, which one core at a time holds, so a
/// borrow never overlaps another.
#[derive(Clone)]
pub struct Shared(pub Rc<RefCell<HostNetwork>>);

/// The restore client's way to Apple's signing service (TSS): one POST
/// at a time, moved along by each host service without waiting, so no
/// guest processor stalls while Apple answers. A transient failure is asked
/// again after a pause ([`Backoff`]), waited out across service steps.
pub struct Signing {
    network: Shared,
    /// The POST in flight and the body it sends.
    exchange: Option<(Vec<u8>, HttpPost)>,
    /// The body whose last attempt failed, and when it is tried again.
    retry: Option<(Vec<u8>, Backoff)>,
    /// The counter origin of the retry clock.
    origin: u64,
    /// Whether a request has gone to the network yet (said once on serial).
    announced: bool,
}

impl Signing {
    pub fn new(network: Shared) -> Self {
        Self {
            network,
            exchange: None,
            retry: None,
            origin: output::now(),
            announced: false,
        }
    }
}

/// After a failed attempt at `body`: keep it for a later attempt when the
/// failure is transient and attempts remain, else the failure.
fn retry_after(
    retry: &mut Option<(Vec<u8>, Backoff)>,
    body: &[u8],
    error: NetError,
    now_ms: u64,
) -> stela_runtime::restore::Result<Option<Vec<u8>>> {
    let mut backoff = match retry.take() {
        Some((sent, backoff)) if sent == body => backoff,
        _ => Backoff::new(),
    };
    let attempt = backoff.attempt();
    match backoff.failed(&error, now_ms) {
        Some(wait) => {
            output::arguments(format_args!(
                "TSS RETRY attempt={attempt} next_in_ms={wait}: {error}\n"
            ));
            *retry = Some((body.to_vec(), backoff));
            Ok(None)
        }
        None => Err(stela_runtime::restore::Error::Transport(refuse!(
            "signing request after {attempts} attempts: {error}",
            attempts = attempt as usize,
            %error
        ))),
    }
}

impl stela_runtime::restore::tss::Transport for Signing {
    fn exchange(
        &mut self,
        host: &str,
        path: &str,
        headers: &[(&str, &str)],
        content_type: &str,
        body: &[u8],
    ) -> stela_runtime::restore::Result<Option<Vec<u8>>> {
        let now_ms = output::millis_since(self.origin);
        let mut network = self.network.0.borrow_mut();
        if let Some((sent, _)) = &self.exchange
            && sent != body
        {
            let (_, post) = self.exchange.take().unwrap();
            network.close_post(post);
        }
        if let Some((sent, backoff)) = &self.retry {
            if sent != body {
                self.retry = None;
            } else if !backoff.ready(now_ms) {
                return Ok(None);
            }
        }
        let (_, post) = match &mut self.exchange {
            Some(exchange) => exchange,
            None => {
                if !self.announced {
                    self.announced = true;
                    output::arguments(format_args!(
                        "TSS NETWORK a signing request the USB does not answer goes to {host} with this machine's ECID, the restore's nonces and its component hashes\n"
                    ));
                }
                let url = format!("http://{host}{path}");
                match network.open_post(&url, headers, content_type, body) {
                    Ok(post) => self.exchange.insert((body.to_vec(), post)),
                    Err(error) => return retry_after(&mut self.retry, body, error, now_ms),
                }
            }
        };
        let result = network.read_post(post);
        if let Ok(None) = result {
            return Ok(None);
        }
        let (_, post) = self.exchange.take().unwrap();
        network.close_post(post);
        match result {
            Err(error) => retry_after(&mut self.retry, body, error, now_ms),
            answer => {
                self.retry = None;
                answer.map_err(|error| {
                    stela_runtime::restore::Error::Transport(refuse!(
                        "signing request: {error}",
                        %error
                    ))
                })
            }
        }
    }
}

/// The signing transport when the host network did not open: every request
/// is refused with the reason, so a restore stops at its first signing
/// request.
pub struct Offline(pub Refusal);

impl stela_runtime::restore::tss::Transport for Offline {
    fn exchange(
        &mut self,
        host: &str,
        path: &str,
        _headers: &[(&str, &str)],
        _content_type: &str,
        _body: &[u8],
    ) -> stela_runtime::restore::Result<Option<Vec<u8>>> {
        Err(stela_runtime::restore::Error::Transport(
            refuse!("no host network for http://{host}{path}", host, path).because(self.0.clone()),
        ))
    }
}

/// The guest's USB Ethernet adapter bridged to a host adapter of its own
/// (`--guest-network`). The adapter keeps the host adapter's station
/// address, so the guest is that adapter on the host's network and the
/// adapter needs no promiscuous mode. Frames move between engine slices.
pub struct GuestNetwork {
    link: RootLink,
    up: Option<bool>,
    /// The guest adapter's counters at the last status, for [`Self::report`].
    counters: stela_soc::machine::UsbNetworkCounters,
    /// The first link refusal, reported once.
    refused: bool,
    /// Frames read from the host adapter that the guest's receive queue
    /// had no room for yet, delivered first at the next turn.
    received: alloc::collections::VecDeque<Vec<u8>>,
    /// Frames the guest sent that found the host adapter's transmit ring
    /// full, sent first at the next turn; the guest's own queue holds the
    /// rest meanwhile.
    unsent: alloc::collections::VecDeque<Vec<u8>>,
}

/// Frames read from the host adapter in one service turn at most, outside
/// the devices' lock: the rest wait in the adapter's ring.
const RECEIVE_BURST: usize = 64;

impl GuestNetwork {
    /// The second adapter when the mount's own client (`wants_client`)
    /// keeps the first, otherwise the first.
    pub fn open(links: &mut Links, wants_client: bool) -> Result<Self> {
        let index = usize::from(wants_client);
        if links.devices.len() <= index {
            return Err(refuse!(
                "the guest network needs host network adapter {adapter} of its own; {found} found",
                adapter = index + 1,
                found = links.devices.len()
            ));
        }
        let device = links.devices.remove(index);
        output::arguments(format_args!(
            "GUEST NETWORK adapter {} mac={}\n",
            device.location(),
            device.mac()
        ));
        Ok(Self {
            link: RootLink::new(device, links.root),
            up: None,
            counters: Default::default(),
            refused: false,
            received: alloc::collections::VecDeque::new(),
            unsent: alloc::collections::VecDeque::new(),
        })
    }

    pub fn mac(&self) -> Mac {
        self.link.mac()
    }

    /// Report the guest's link state as `status` gives it.
    pub fn status(&mut self, status: Option<(bool, stela_soc::machine::UsbNetworkCounters)>) {
        let up = status.is_some_and(|(up, _)| up);
        if let Some((_, counters)) = status {
            self.counters = counters;
        }
        if self.up != Some(up) {
            self.up = Some(up);
            let counters = status.map(|(_, counters)| counters).unwrap_or_default();
            output::arguments(format_args!(
                "GUEST NETWORK link {} {counters:?}\n",
                if up { "up" } else { "down" }
            ));
        }
    }

    /// The beat report's line: the frames the guest adapter moved each way
    /// and dropped (oversized, filtered and overflowed), so a run shows
    /// whether traffic moves.
    pub fn report(&self) {
        let counters = self.counters;
        output::arguments(format_args!(
            "GUEST NETWORK frames link={} to_host={} to_guest={} drops={} oversized={} \
             filtered={} overflowed={}\n",
            match self.up {
                Some(true) => "up",
                Some(false) => "down",
                None => "unknown",
            },
            counters.to_host,
            counters.to_guest,
            counters.oversized + counters.filtered + counters.overflowed,
            counters.oversized,
            counters.filtered,
            counters.overflowed
        ));
    }

    /// Read frames from the host adapter while the guest's receive queue
    /// had `room` and nothing read earlier still waits, without the devices'
    /// lock: the adapter's MMIO and DMA can wait milliseconds on the host.
    /// While another processor is inside the host drivers the frames wait
    /// in the adapter's ring for the next turn: a turn does not wait for
    /// the drivers' lock, which can be held for longer than a PE's exit
    /// may take.
    pub fn receive(&mut self, room: bool) {
        if !room || !self.received.is_empty() {
            return;
        }
        while self.received.len() < RECEIVE_BURST {
            let Some(received) = self.link.receive_now() else {
                break;
            };
            match received {
                Ok(Some(frame)) => self.received.push_back(frame),
                Ok(None) => break,
                Err(error) => {
                    self.refuse(error.into());
                    break;
                }
            }
        }
    }

    /// Give the guest the frames read so far while its receive queue has
    /// room. The guest's queues are the only bound: the rest wait here and
    /// then in the host adapter's ring, which drops on overflow as the
    /// hardware does.
    pub fn deliver(&mut self, devices: &mut AppleDevices) {
        while !self.received.is_empty() && devices.network_room() {
            if let Some(frame) = self.received.pop_front() {
                // Without an attached adapter the frame has nowhere to go.
                let _ = devices.network_deliver_owned(frame);
            }
        }
    }

    /// The frames the guest queued, taken from its devices, for
    /// [`Self::transmit`] to send without them. None while frames a full
    /// host ring refused still wait: the guest's queue holds the rest.
    pub fn queued(&self, devices: &mut AppleDevices) -> Vec<Vec<u8>> {
        if !self.unsent.is_empty() {
            return Vec::new();
        }
        core::iter::from_fn(|| devices.network_take()).collect()
    }

    /// Send in queue order and return the last sent frame's reusable
    /// storage. A full host ring keeps that frame and the ones after it for
    /// the next turn, as Linux requeues a `NETDEV_TX_BUSY` frame, and so
    /// does another processor inside the host drivers, whose lock a turn
    /// does not wait for.
    pub fn transmit(&mut self, frames: Vec<Vec<u8>>) -> Option<Vec<u8>> {
        self.unsent.extend(frames);
        let mut reusable = None;
        while let Some(frame) = self.unsent.front() {
            match self.link.transmit_now(frame) {
                Some(Ok(())) => {}
                None | Some(Err(LinkError::Busy)) => break,
                Some(Err(error)) => self.refuse(error.into()),
            }
            reusable = self.unsent.pop_front();
        }
        reusable
    }

    fn refuse(&mut self, error: NetError) {
        if !core::mem::replace(&mut self.refused, true) {
            output::arguments(format_args!("GUEST NETWORK refused: {error}\n"));
        }
    }
}
