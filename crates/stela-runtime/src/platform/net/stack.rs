//! The interface: smoltcp over a [`Link`], configured by DHCPv4 and IPv6
//! autoconfiguration (or an authored fixture), with non-blocking
//! [`Transfer`]s that resolve, connect and move bytes as they are driven,
//! several at once, and blocking HTTP requests built on them.
mod automatic;
mod client;
pub use client::{HttpPost, HttpRequest, HttpTransfer, TlsUnavailable};
mod dad;
mod dns_tcp;
mod rdnss;
mod tls;
use super::{Clock, ClockError, Ipv4, NetError, http};
use alloc::{string::String, vec, vec::Vec};
pub use automatic::Automatic;
use core::net::Ipv6Addr;
use smoltcp::{
    iface::{Config as InterfaceConfig, Interface, SocketHandle, SocketSet},
    phy::{self, Device, DeviceCapabilities, Medium},
    socket::{dhcpv4, dns, tcp},
    time::{Duration, Instant},
    wire::{DnsQueryType, EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Cidr, Ipv6Cidr},
};
use stela_drivers::net::{Link, LinkError, MAX_FRAME};
pub use tls::SecureTransfer;
use zeroize::{Zeroize, Zeroizing};

/// An interface's IPv4 configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Config {
    pub address: Ipv4Cidr,
    pub router: Option<Ipv4>,
    pub dns: Vec<Ipv4>,
}

/// smoltcp's DHCP client asks every 10 seconds within this. A switch port
/// that runs spanning tree forwards nothing for its first 30 seconds after
/// the link comes up (IEEE 802.1D: 15 s listening, 15 s learning), and the
/// adapter's start has just dropped and raised the link, so a wait of 30
/// seconds lost every request there. Other systems' clients ask for a
/// minute or longer.
const DHCP_WAIT_US: u64 = 60_000_000;
/// smoltcp's resolver retries each server within this.
const DNS_WAIT_US: u64 = 15_000_000;
/// How long a name's address is used before it is looked up again.
const RESOLVED_US: u64 = 300_000_000;
/// A POST, from connection to the response's end.
const POST_WAIT_US: u64 = 120_000_000;
/// A transfer that receives nothing for this long has failed.
const IDLE_US: u64 = 30_000_000;
const FIRST_EPHEMERAL: u16 = 49152;
/// A POST's receive buffer, and a download's: the latter bounds the TCP
/// window, so a 30 ms path carries 4 MiB / 30 ms, about 140 MB/s, per
/// connection.
const POST_BUFFER: usize = 256 * 1024;
pub const DOWNLOAD_BUFFER: usize = 4 << 20;
const SEND_BUFFER: usize = 64 * 1024;

/// The link as smoltcp's device. A link refusal is kept, and the operation
/// polling the interface reports it. Under [`Link::receive`]'s contract a
/// receive error means the device is gone, so the operation ends there
/// rather than polling a dead link to its deadline.
struct Phy<L> {
    link: L,
    tx_frame: Vec<u8>,
    rx_frame: Vec<u8>,
    fault: Option<LinkError>,
    /// Whether the last poll took a frame from the link.
    received: bool,
    probes: Vec<dad::Probe>,
    local_ipv6: Ipv6Addr,
    approved_ipv6: Vec<Ipv6Addr>,
    rdnss: rdnss::Rdnss,
    manage_ipv6: bool,
}

struct Rx<'a>(&'a [u8]);

struct Tx<'a, L> {
    link: &'a mut L,
    frame: &'a mut Vec<u8>,
    fault: &'a mut Option<LinkError>,
    approved_ipv6: Option<&'a [Ipv6Addr]>,
}

impl phy::RxToken for Rx<'_> {
    fn consume<R, F: FnOnce(&[u8]) -> R>(self, f: F) -> R {
        f(self.0)
    }
}

impl<L: Link> phy::TxToken for Tx<'_, L> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, length: usize, f: F) -> R {
        let frame = self.frame;
        frame.clear();
        frame.resize(length, 0);
        let result = f(frame);
        // Tentative addresses stay off the wire until their DAD probe ends.
        if let Some(approved) = self.approved_ipv6
            && frame.len() >= 54
            && frame[12..14] == [0x86, 0xdd]
        {
            let source = Ipv6Addr::from(<[u8; 16]>::try_from(&frame[22..38]).unwrap());
            if !source.is_unspecified() && !approved.contains(&source) {
                return result;
            }
        }
        // A full ring drops the frame, as a full queue would; the protocol
        // retransmits it. Anything else is the link's fault.
        match self.link.transmit(frame) {
            Ok(()) | Err(LinkError::Busy) => {}
            Err(error) => {
                self.fault.get_or_insert(error);
            }
        }
        result
    }
}

impl<L: Link> Device for Phy<L> {
    type RxToken<'a>
        = Rx<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = Tx<'a, L>
    where
        Self: 'a;

    fn receive(&mut self, now: Instant) -> Option<(Rx<'_>, Tx<'_, L>)> {
        match self.link.receive_reusing(&mut self.rx_frame) {
            Ok(true) => {
                self.received = true;
                let frame = &self.rx_frame;
                for probe in &mut self.probes {
                    probe.duplicate |= dad::conflicts(frame, probe.address, probe.nonce);
                }
                self.rdnss
                    .observe(frame, self.local_ipv6, now.total_micros().max(0) as u64);
                Some((
                    Rx(frame),
                    Tx {
                        link: &mut self.link,
                        frame: &mut self.tx_frame,
                        fault: &mut self.fault,
                        approved_ipv6: self.manage_ipv6.then_some(self.approved_ipv6.as_slice()),
                    },
                ))
            }
            Ok(false) => None,
            Err(error) => {
                self.fault.get_or_insert(error);
                None
            }
        }
    }

    fn transmit(&mut self, _: Instant) -> Option<Tx<'_, L>> {
        Some(Tx {
            link: &mut self.link,
            frame: &mut self.tx_frame,
            fault: &mut self.fault,
            approved_ipv6: self.manage_ipv6.then_some(self.approved_ipv6.as_slice()),
        })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut capabilities = DeviceCapabilities::default();
        capabilities.medium = Medium::Ethernet;
        // smoltcp counts the Ethernet header in an Ethernet MTU.
        capabilities.max_transmission_unit = MAX_FRAME;
        capabilities
    }
}

/// One TCP connection carrying one request and its response. It opens
/// without waiting, as a non-blocking `connect` does: its name resolves and
/// its connection starts as the transfer is driven.
pub struct Transfer {
    endpoint: Endpoint,
    request: Vec<u8>,
    sent: usize,
    /// When the transfer fails unless something arrives first.
    idle_at: u64,
}

#[cfg(test)]
impl Transfer {
    fn socket(&self) -> SocketHandle {
        match self.endpoint {
            Endpoint::Connected(socket) => socket,
            Endpoint::Resolving(_) => panic!("the transfer is still resolving"),
        }
    }
}

#[cfg(test)]
impl<L: Link, C: Clock> Stack<L, C> {
    /// `name`'s address, driving a lookup whose wait for a server or an
    /// address family is already over.
    fn resolve_now(&mut self, name: &str) -> Result<IpAddress, NetError> {
        let mut lookup = Lookup {
            name: String::from(name),
            port: 80,
            buffer: 0,
            raw: false,
            query: None,
            tcp: None,
            ipv4_only: false,
            deadline: 0,
        };
        loop {
            if let Some(address) = self.resolve(&mut lookup)? {
                return Ok(address);
            }
            self.poll()?;
            self.wait_until(lookup.deadline)?;
        }
    }
}

enum Endpoint {
    Resolving(Lookup),
    Connected(SocketHandle),
}

/// A destination whose address is not known yet.
struct Lookup {
    name: String,
    port: u16,
    buffer: usize,
    /// An opaque stream, which no idle time ends.
    raw: bool,
    query: Option<(dns::QueryHandle, DnsQueryType)>,
    tcp: Option<alloc::boxed::Box<dns_tcp::Lookup>>,
    /// An IPv6 lookup failed, so only IPv4 is asked.
    ipv4_only: bool,
    /// When the address family, server or answer being waited for is late.
    deadline: u64,
}

pub struct Stack<L: Link, C: Clock> {
    phy: Phy<L>,
    clock: C,
    last_time: u64,
    clock_failure: Option<ClockError>,
    iface: Interface,
    sockets: SocketSet<'static>,
    /// Absent once a configuration is set by hand.
    dhcp: Option<SocketHandle>,
    dns: SocketHandle,
    config: Option<Config>,
    next_port: u16,
    /// Names looked up, their first address and when it was found.
    resolved: Vec<(String, IpAddress, u64)>,
    dns_servers: Vec<IpAddress>,
    /// DNS servers set by hand, which replace DHCP's and router
    /// advertisements'.
    fixed_dns: Option<Vec<IpAddress>>,
    conflicts: Vec<Ipv6Addr>,
    probe_nonce: u64,
}

impl<L: Link, C: Clock> Stack<L, C> {
    /// Seed TCP sequence/port choices from host cryptographic entropy.
    pub fn new(link: L, clock: C, seed: u64) -> Result<Self, NetError> {
        let mac = link.mac();
        let local_ipv6 = automatic::link_local(mac.0);
        let mut config = InterfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(mac.0)));
        config.random_seed = seed;
        config.slaac = true;
        let mut phy = Phy {
            link,
            tx_frame: Vec::new(),
            rx_frame: Vec::new(),
            fault: None,
            received: false,
            probes: Vec::new(),
            local_ipv6,
            approved_ipv6: Vec::new(),
            rdnss: Default::default(),
            manage_ipv6: true,
        };
        let initial = clock.now_us().map_err(NetError::Clock)?;
        let now = Instant::from_micros(
            i64::try_from(initial).map_err(|_| NetError::Clock(ClockError::OutOfRange(initial)))?,
        );
        let mut iface = Interface::new(config, &mut phy, now);
        iface.update_ip_addrs(|a| a.push(IpCidr::Ipv6(Ipv6Cidr::new(local_ipv6, 64))).unwrap());
        let mut sockets = SocketSet::new(Vec::new());
        let dhcp = Some(sockets.add(dhcpv4::Socket::new()));
        let dns = sockets.add(dns::Socket::new(&[], Vec::new()));
        Ok(Self {
            phy,
            clock,
            last_time: initial,
            clock_failure: None,
            iface,
            sockets,
            dhcp,
            dns,
            config: None,
            next_port: FIRST_EPHEMERAL + (seed % 8192) as u16,
            resolved: Vec::new(),
            dns_servers: Vec::new(),
            fixed_dns: None,
            conflicts: Vec::new(),
            probe_nonce: seed,
        })
    }

    pub fn link_mut(&mut self) -> &mut L {
        &mut self.phy.link
    }

    pub fn config(&self) -> Option<&Config> {
        self.config.as_ref()
    }

    /// Gives the interface `address`/`prefix` and the default router
    /// `router` by hand instead of by DHCP. IPv6 autoconfiguration goes on.
    pub fn set_ipv4(
        &mut self,
        address: core::net::Ipv4Addr,
        prefix: u8,
        router: Option<core::net::Ipv4Addr>,
    ) -> Result<(), NetError> {
        self.set_config(Config {
            address: Ipv4Cidr::new(address, prefix),
            router,
            dns: Vec::new(),
        })
    }

    /// Resolves names with `servers` in place of DHCP's and router
    /// advertisements'.
    pub fn set_dns(&mut self, servers: &[core::net::IpAddr]) -> Result<(), NetError> {
        self.fixed_dns = Some(servers.iter().map(|&server| server.into()).collect());
        self.update_dns()
    }

    /// Configures the interface by hand instead of by DHCP.
    pub fn set_config(&mut self, config: Config) -> Result<(), NetError> {
        if let Some(dhcp) = self.dhcp.take() {
            self.sockets.remove(dhcp);
        }
        self.apply(Some(config))
    }

    fn apply(&mut self, config: Option<Config>) -> Result<(), NetError> {
        let mut fit = true;
        self.iface.update_ip_addrs(|addresses| {
            addresses.retain(|a| matches!(a, IpCidr::Ipv6(_)));
            if let Some(config) = &config {
                // Reserve one slot for the lease without disturbing the
                // first link-local IPv6 address required by router discovery.
                while addresses.is_full() && addresses.len() > 1 {
                    addresses.pop();
                }
                fit = addresses.push(IpCidr::Ipv4(config.address)).is_ok();
            }
        });
        if !fit {
            return Err(NetError::Invalid("interface address capacity"));
        }
        let routes = self.iface.routes_mut();
        match config.as_ref().and_then(|config| config.router) {
            Some(router) => {
                routes
                    .add_default_ipv4_route(router)
                    .map_err(|_| NetError::Invalid("interface route capacity"))?;
            }
            None => {
                routes.remove_default_ipv4_route();
            }
        }
        self.config = config;
        self.update_dns()?;
        Ok(())
    }

    fn poison_clock(&mut self, error: ClockError) -> NetError {
        let error = *self.clock_failure.get_or_insert(error);
        for (_, socket) in self.sockets.iter_mut() {
            if let smoltcp::socket::Socket::Tcp(socket) = socket {
                socket.abort();
            }
        }
        NetError::Clock(error)
    }
    fn now(&mut self) -> Result<u64, NetError> {
        if let Some(error) = self.clock_failure {
            return Err(NetError::Clock(error));
        }
        let value = match self.clock.now_us() {
            Ok(value) => value,
            Err(error) => return Err(self.poison_clock(error)),
        };
        if value < self.last_time {
            return Err(self.poison_clock(ClockError::Regressed {
                previous: self.last_time,
                current: value,
            }));
        }
        if value > i64::MAX as u64 {
            return Err(self.poison_clock(ClockError::OutOfRange(value)));
        }
        self.last_time = value;
        Ok(value)
    }

    /// Moves every frame the link holds through the interface, then takes
    /// DHCP's news. A link refusal since the last poll is the error.
    pub fn poll(&mut self) -> Result<(), NetError> {
        let now = Instant::from_micros(self.now()? as i64);
        self.phy.received = false;
        self.iface.poll(now, &mut self.phy, &mut self.sockets);
        if let Some(dhcp) = self.dhcp {
            let news = match self.sockets.get_mut::<dhcpv4::Socket>(dhcp).poll() {
                Some(dhcpv4::Event::Configured(lease)) => Some(Some(Config {
                    address: lease.address,
                    router: lease.router,
                    dns: lease.dns_servers.iter().copied().collect(),
                })),
                Some(dhcpv4::Event::Deconfigured) => Some(None),
                None => None,
            };
            if let Some(config) = news {
                self.apply(config)?;
            }
        }
        self.probe_ipv6()?;
        self.update_dns()?;
        match self.phy.fault.take() {
            Some(error) => Err(error.into()),
            None => Ok(()),
        }
    }

    /// Waits up to `micros` for the link's next frame, when the link signals
    /// arrivals and the last poll took none: until its signal, smoltcp's next
    /// timer or the bound, whichever is first. A link that does not signal
    /// returns at once, and its caller polls.
    pub fn wait(&mut self, micros: u64) -> Result<(), NetError> {
        let deadline = self.now()?.saturating_add(micros);
        self.wait_until(deadline)
    }

    fn wait_until(&mut self, deadline: u64) -> Result<(), NetError> {
        self.now()?;
        if !self.phy.link.signals() || self.phy.received {
            return Ok(());
        }
        let now = self.now()?;
        let timer = self
            .iface
            .poll_delay(Instant::from_micros(now as i64), &self.sockets)
            .map_or(u64::MAX, |delay| now.saturating_add(delay.total_micros()));
        let dad = self
            .phy
            .probes
            .iter()
            .map(|probe| probe.until)
            .min()
            .unwrap_or(u64::MAX);
        let until = deadline
            .min(timer)
            .min(dad)
            .min(self.phy.link.due_us().unwrap_or(u64::MAX));
        if until > now
            && let Err(error) = self.clock.sleep(until)
        {
            return Err(self.poison_clock(error));
        }
        self.now()?;
        Ok(())
    }

    /// Waits for carrier as long as the link may take to bring it up
    /// ([`Link::carrier_wait_us`]), returning as soon as it is up.
    /// `waiting` hears each whole second waited so far.
    fn await_carrier(&mut self, waiting: &mut dyn FnMut(u64)) -> Result<(), NetError> {
        let start = self.now()?;
        let limit = self.phy.link.carrier_wait_us();
        let mut reported = 0;
        while !self.phy.link.carrier() {
            let elapsed = self.now()?.saturating_sub(start);
            if elapsed >= limit {
                return Err(NetError::NoCarrier);
            }
            let seconds = elapsed / 1_000_000;
            if seconds > reported {
                reported = seconds;
                waiting(seconds);
            }
        }
        Ok(())
    }

    /// Waits for carrier and, without a configuration by hand, a DHCP
    /// lease.
    pub fn dhcp(&mut self) -> Result<&Config, NetError> {
        self.await_carrier(&mut |_| {})?;
        let deadline = self.now()? + DHCP_WAIT_US;
        while self.config.is_none() {
            if self.dhcp.is_none() {
                return Err(NetError::Invalid("DHCP after a configuration by hand"));
            }
            if self.now()? >= deadline {
                return Err(NetError::Dhcp(
                    "no IPv4 lease: no DHCP server answered within 60 seconds",
                ));
            }
            self.poll()?;
            if self.config.is_none() {
                self.wait_until(deadline)?;
            }
        }
        Ok(self.config.as_ref().unwrap())
    }

    /// Advances `lookup` without waiting: its address once known, or `None`
    /// while it waits for an address family, a DNS server or the answer.
    /// An address found in the last [`RESOLVED_US`] is used again; IPv6 is
    /// asked first when it is ready, then IPv4; and a lookup that fails falls
    /// back to the last address the name had, so one lost reply does not end
    /// a long download.
    fn resolve(&mut self, lookup: &mut Lookup) -> Result<Option<IpAddress>, NetError> {
        if let Ok(address) = lookup.name.parse() {
            return Ok(Some(address));
        }
        let now = self.now()?;
        let known = self
            .resolved
            .iter()
            .position(|(name, ..)| *name == lookup.name);
        if lookup.query.is_none()
            && lookup.tcp.is_none()
            && let Some(index) = known
            && self.address_usable(self.resolved[index].1)
            && now.wrapping_sub(self.resolved[index].2) < RESOLVED_US
        {
            return Ok(Some(self.resolved[index].1));
        }
        let failure = match self.lookup(lookup, now) {
            Ok(Some(address)) => {
                match known {
                    Some(index) => self.resolved[index] = (lookup.name.clone(), address, now),
                    None => self.resolved.push((lookup.name.clone(), address, now)),
                }
                return Ok(Some(address));
            }
            Ok(None) => return Ok(None),
            Err(error @ NetError::Clock(_)) => return Err(error),
            Err(error) => error,
        };
        match known {
            Some(index) if self.address_usable(self.resolved[index].1) => {
                Ok(Some(self.resolved[index].1))
            }
            _ => Err(failure),
        }
    }

    fn lookup(&mut self, lookup: &mut Lookup, now: u64) -> Result<Option<IpAddress>, NetError> {
        if lookup.tcp.is_some() {
            return self.lookup_tcp(lookup, now);
        }
        let Some((query, kind)) = lookup.query else {
            let kind = if !lookup.ipv4_only && self.ipv6_ready() {
                DnsQueryType::Aaaa
            } else if self.config.is_some() {
                DnsQueryType::A
            } else if now >= lookup.deadline {
                return Err(NetError::Dns(alloc::format!(
                    "{}: no reachable address family",
                    lookup.name
                )));
            } else {
                return Ok(None);
            };
            if self.dns_servers.is_empty() {
                if now >= lookup.deadline {
                    return Err(NetError::Dns(String::from(
                        "no DNS server from DHCPv4 or IPv6 router advertisements",
                    )));
                }
                return Ok(None);
            }
            let query = self
                .sockets
                .get_mut::<dns::Socket>(self.dns)
                .start_query(self.iface.context(), &lookup.name, kind)
                .map_err(|error| NetError::Dns(alloc::format!("{}: {error:?}", lookup.name)))?;
            lookup.query = Some((query, kind));
            // smoltcp spends up to ten seconds on each configured server.
            // Do not cancel the whole lookup before it tries server two.
            let wait =
                DNS_WAIT_US.max(self.dns_servers.len().min(3) as u64 * 10_000_000 + 1_000_000);
            lookup.deadline = now.saturating_add(wait);
            return Ok(None);
        };
        let socket = self.sockets.get_mut::<dns::Socket>(self.dns);
        let failure =
            match socket.get_query_result(query) {
                Ok(addresses) => {
                    lookup.query = None;
                    return addresses.iter().copied().next().map(Some).ok_or_else(|| {
                        NetError::Dns(alloc::format!("{}: no address", lookup.name))
                    });
                }
                Err(dns::GetQueryResultError::Pending) if now < lookup.deadline => return Ok(None),
                Err(dns::GetQueryResultError::Pending) => {
                    socket.cancel_query(query);
                    NetError::Timeout("DNS")
                }
                Err(error) => NetError::Dns(alloc::format!("{}: {error:?}", lookup.name)),
            };
        lookup.query = None;
        if kind == DnsQueryType::Aaaa && self.config.is_some() {
            lookup.ipv4_only = true;
            return Ok(None);
        }
        self.start_dns_tcp(lookup, kind, failure, now)
    }

    /// Opens a transfer to `host` (`name` or `name:port`, port 80 by
    /// default) that sends `request` once connected, without waiting.
    /// `buffer` bounds the receive window.
    pub fn open(
        &mut self,
        host: &str,
        request: Vec<u8>,
        buffer: usize,
    ) -> Result<Transfer, NetError> {
        let mut request = Zeroizing::new(request);
        let (name, port) = automatic::authority(host)?;
        self.start(name, port, core::mem::take(&mut *request), buffer, false)
    }

    fn start(
        &mut self,
        name: &str,
        port: u16,
        request: Vec<u8>,
        buffer: usize,
        raw: bool,
    ) -> Result<Transfer, NetError> {
        let mut transfer = Transfer {
            endpoint: Endpoint::Resolving(Lookup {
                name: String::from(name),
                port,
                buffer,
                raw,
                query: None,
                tcp: None,
                ipv4_only: false,
                deadline: 0,
            }),
            request,
            sent: 0,
            idle_at: u64::MAX,
        };
        let started = self.now().and_then(|now| {
            if let Endpoint::Resolving(lookup) = &mut transfer.endpoint {
                // smoltcp spends up to ten seconds on each configured server.
                // Do not cancel the whole lookup before it tries server two.
                let wait =
                    DNS_WAIT_US.max(self.dns_servers.len().min(3) as u64 * 10_000_000 + 1_000_000);
                lookup.deadline = now.saturating_add(wait);
            }
            self.connect(&mut transfer)
        });
        if let Err(error) = started {
            self.close(transfer);
            return Err(error);
        }
        Ok(transfer)
    }

    /// The transfer's socket once its address is known and its connection
    /// has started, starting it if the address just became known.
    fn connect(&mut self, transfer: &mut Transfer) -> Result<Option<SocketHandle>, NetError> {
        let lookup = match &mut transfer.endpoint {
            Endpoint::Connected(socket) => return Ok(Some(*socket)),
            Endpoint::Resolving(lookup) => lookup,
        };
        let Some(address) = self.resolve(lookup)? else {
            if !lookup.raw {
                transfer.idle_at = lookup.deadline;
            }
            return Ok(None);
        };
        let mut socket = tcp::Socket::new(
            tcp::SocketBuffer::new(vec![0; lookup.buffer]),
            tcp::SocketBuffer::new(vec![0; SEND_BUFFER]),
        );
        socket.set_congestion_control(tcp::CongestionControl::Reno);
        socket.set_timeout((!lookup.raw).then(|| Duration::from_micros(IDLE_US)));
        let local = self.next_port;
        self.next_port = local.checked_add(1).unwrap_or(FIRST_EPHEMERAL);
        socket
            .connect(self.iface.context(), (address, lookup.port), local)
            .map_err(|_| NetError::Invalid("TCP connect"))?;
        if !lookup.raw {
            transfer.idle_at = self.now()?.saturating_add(IDLE_US);
        }
        let socket = self.sockets.add(socket);
        transfer.endpoint = Endpoint::Connected(socket);
        Ok(Some(socket))
    }

    /// Opens an unframed TCP stream. The caller supplies every application
    /// byte later through [`Self::write`] and decides when an idle connection
    /// ends. This is the transport boundary for protocols whose TLS and
    /// request framing belong to the peer rather than Stela.
    pub fn open_raw(&mut self, host: &str, port: u16, buffer: usize) -> Result<Transfer, NetError> {
        if port == 0 {
            return Err(NetError::Invalid("TCP destination port zero"));
        }
        self.start(host, port, Vec::new(), buffer, true)
    }

    /// Sends as much of one caller-kept raw payload as the stream accepts.
    /// Zero is backpressure while the connection is opening or its send window
    /// is full; the caller must retry the unchanged bytes.
    pub fn write(&mut self, transfer: &mut Transfer, bytes: &[u8]) -> Result<usize, NetError> {
        self.poll()?;
        if transfer.sent != transfer.request.len() {
            return Err(NetError::Invalid("raw write before the fixed request"));
        }
        if bytes.is_empty() {
            return Ok(0);
        }
        let now = self.now()?;
        let Some(socket) = self.connect(transfer)? else {
            return Ok(0);
        };
        let socket = self.sockets.get_mut::<tcp::Socket>(socket);
        if socket.can_send() {
            let sent = socket.send_slice(bytes).map_err(|_| NetError::Reset)?;
            if sent != 0 {
                transfer.idle_at = now.saturating_add(IDLE_US);
            }
            return Ok(sent);
        }
        match socket.state() {
            tcp::State::Closed | tcp::State::CloseWait | tcp::State::LastAck => {
                Err(NetError::Reset)
            }
            _ => Ok(0),
        }
    }

    /// Without waiting: polls the interface, sends what is left of the
    /// request, and hands what has arrived to `take` with whether the peer
    /// has closed. Returns `take`'s verdict that the response is complete.
    pub fn read(
        &mut self,
        transfer: &mut Transfer,
        take: impl FnMut(&[u8], bool) -> Result<bool, NetError>,
    ) -> Result<bool, NetError> {
        self.read_bounded(transfer, usize::MAX, take)
    }

    /// Consume at most the caller's available receive capacity, leaving every
    /// other TCP byte in its socket. This preserves TLS record backpressure.
    fn read_bounded(
        &mut self,
        transfer: &mut Transfer,
        mut capacity: usize,
        mut take: impl FnMut(&[u8], bool) -> Result<bool, NetError>,
    ) -> Result<bool, NetError> {
        self.poll()?;
        let now = self.now()?;
        let Some(socket) = self.connect(transfer)? else {
            return self.waiting(transfer, now);
        };
        let socket = self.sockets.get_mut::<tcp::Socket>(socket);
        if socket.can_send() && transfer.sent < transfer.request.len() {
            transfer.sent += socket
                .send_slice(&transfer.request[transfer.sent..])
                .map_err(|_| NetError::Reset)?;
        }
        let mut done = false;
        while socket.can_recv() && !done && capacity != 0 {
            done = socket
                .recv(|bytes| {
                    let count = bytes.len().min(capacity);
                    capacity -= count;
                    (count, take(&bytes[..count], false))
                })
                .map_err(|_| NetError::Reset)??;
            transfer.idle_at = now + IDLE_US;
        }
        if done {
            return Ok(true);
        }
        if now >= transfer.idle_at {
            return Err(NetError::Timeout("TCP transfer"));
        }
        match socket.state() {
            tcp::State::Closed if !transfer.request.is_empty() && transfer.sent == 0 => {
                Err(NetError::Reset)
            }
            tcp::State::SynSent | tcp::State::SynReceived => self.waiting(transfer, now),
            _ if !socket.may_recv() && !socket.can_recv() => take(&[], true),
            _ => self.waiting(transfer, now),
        }
    }

    fn waiting(&self, transfer: &Transfer, now: u64) -> Result<bool, NetError> {
        if now >= transfer.idle_at {
            return Err(NetError::Timeout("TCP transfer"));
        }
        Ok(false)
    }

    /// Ends `transfer`, gracefully if it completed, and frees its socket or
    /// its lookup.
    pub fn close(&mut self, mut transfer: Transfer) {
        transfer.request.zeroize();
        match transfer.endpoint {
            Endpoint::Connected(socket) => {
                self.sockets.get_mut::<tcp::Socket>(socket).abort();
                // The RST leaves on this poll; its failure loses nothing.
                let _ = self.poll();
                self.sockets.remove(socket);
            }
            Endpoint::Resolving(mut lookup) => {
                if let Some((query, _)) = lookup.query.take() {
                    self.sockets
                        .get_mut::<dns::Socket>(self.dns)
                        .cancel_query(query);
                }
                if let Some(mut lookup) = lookup.tcp.take()
                    && let Some(transfer) = lookup.transfer.take()
                {
                    self.close(*transfer);
                }
            }
        }
    }

    /// For a blocking request: configures the interface automatically
    /// first if it has no address.
    fn configure(&mut self) -> Result<(), NetError> {
        if self.config.is_none() && !self.ipv6_ready() {
            self.automatic()?;
        }
        Ok(())
    }

    /// Reads `transfer` until `take` says its response is complete.
    fn finish(
        &mut self,
        mut transfer: Transfer,
        deadline: u64,
        mut take: impl FnMut(&[u8], bool) -> Result<bool, NetError>,
    ) -> Result<(), NetError> {
        let result = (|| {
            loop {
                match self.read(&mut transfer, &mut take) {
                    Ok(true) => break Ok(()),
                    Ok(false) if self.now()? >= deadline => break Err(NetError::Timeout("HTTP")),
                    Ok(false) => self.wait_until(deadline.min(transfer.idle_at))?,
                    Err(error) => break Err(error),
                }
            }
        })();
        self.close(transfer);
        result
    }

    /// One HTTP/1.1 POST; the response body on a 2xx status.
    pub fn post(
        &mut self,
        host: &str,
        path: &str,
        headers: &[(&str, &str)],
        content_type: &str,
        body: &[u8],
    ) -> Result<Vec<u8>, NetError> {
        self.configure()?;
        let request = http::post(host, path, headers, content_type, body)?;
        let transfer = self.open(host, request, POST_BUFFER)?;
        let mut reply = Vec::new();
        let mut response = None;
        let deadline = match self.now() {
            Ok(now) => now.saturating_add(POST_WAIT_US),
            Err(error) => {
                self.close(transfer);
                return Err(error);
            }
        };
        self.finish(transfer, deadline, |bytes, closed| {
            reply.extend_from_slice(bytes);
            response = http::response(&reply, closed)?;
            Ok(response.is_some())
        })?;
        Ok(response.unwrap().body)
    }

    /// Bytes `start..=end` of `http://host/path`, handed to `body` in order
    /// as they arrive. Returns the resource's whole size.
    pub fn get_range(
        &mut self,
        host: &str,
        path: &str,
        start: u64,
        end: u64,
        mut body: impl FnMut(&[u8]),
    ) -> Result<u64, NetError> {
        self.configure()?;
        let request = http::get_range(host, path, start, end, None)?;
        let transfer = self.open(host, request, DOWNLOAD_BUFFER)?;
        let mut ranged = http::Ranged::new(start, end);
        self.finish(transfer, u64::MAX, |bytes, closed| {
            ranged.feed(bytes, closed, &mut body)
        })?;
        Ok(ranged.size.unwrap())
    }
}

#[cfg(test)]
pub(crate) mod tests;
