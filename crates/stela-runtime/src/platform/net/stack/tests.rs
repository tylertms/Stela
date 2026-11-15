mod clock;
mod dns_fallback;
mod http_flow;
#[cfg(all(not(target_os = "uefi"), feature = "native-reference-peer"))]
mod tls;
use super::*;
use alloc::{collections::VecDeque, rc::Rc};
use core::cell::{Cell, RefCell};
use stela_drivers::net::Mac;

const CLIENT: Mac = Mac([2, 0, 0, 0, 0, 1]);
const SERVER: [u8; 6] = [2, 0, 0, 0, 0, 2];
const CLIENT_IP: Ipv4 = Ipv4::new(10, 0, 2, 15);
const SERVER_IP: Ipv4 = Ipv4::new(10, 0, 2, 2);

/// Each reading advances 20 µs, so waits end.
#[derive(Clone, Default)]
pub(crate) struct TestClock(Rc<Cell<u64>>);

impl Clock for TestClock {
    fn now_us(&self) -> Result<u64, ClockError> {
        self.0.set(self.0.get() + 20);
        Ok(self.0.get())
    }
}

/// Frames from one end to the other.
#[derive(Default)]
struct Queue(VecDeque<Vec<u8>>);

struct End {
    mac: Mac,
    out: Rc<RefCell<Queue>>,
    into: Rc<RefCell<Queue>>,
}

impl Link for End {
    fn mac(&self) -> Mac {
        self.mac
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        self.out.borrow_mut().0.push_back(frame.to_vec());
        Ok(())
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        Ok(self.into.borrow_mut().0.pop_front())
    }
}

#[test]
fn transmit_reuses_storage_with_zeroed_bytes_and_the_current_frame_length() {
    use smoltcp::phy::TxToken;

    let mut lan = Lan::new(0, TestClock::default());
    let mut transmit = |length, value| {
        lan.server
            .transmit(Instant::ZERO)
            .unwrap()
            .consume(length, |frame| {
                assert!(frame.iter().all(|&byte| byte == 0));
                frame[0] = value;
                frame.as_ptr()
            })
    };
    let first = transmit(100, 3);
    assert_eq!(transmit(60, 7), first);
    assert_eq!(transmit(100, 2), first);
    for (length, value) in [(100, 3), (60, 7), (100, 2)] {
        let frame = lan.client.receive().unwrap().unwrap();
        assert_eq!(frame.len(), length);
        assert_eq!(frame[0], value);
        assert!(frame[1..].iter().all(|&byte| byte == 0));
    }
}

#[test]
fn receive_keeps_the_owned_frame_available_while_transmitting() {
    use smoltcp::phy::{RxToken, TxToken};

    let mut lan = Lan::new(0, TestClock::default());
    let frame = vec![0x5a; 60];
    let address = frame.as_ptr();
    lan.client.out.borrow_mut().0.push_back(frame);

    let (rx, tx) = lan.server.receive(Instant::ZERO).unwrap();
    tx.consume(14, |bytes| bytes.fill(0x73));
    rx.consume(|bytes| {
        assert_eq!(bytes.as_ptr(), address);
        assert_eq!(bytes, [0x5a; 60]);
    });
    assert_eq!(lan.client.receive().unwrap().unwrap(), [0x73; 14]);
    assert!(lan.server.receive(Instant::ZERO).is_none());
}

/// A smoltcp HTTP server serving `pattern(offset)` for ranges of a
/// `size`-byte resource, stepped whenever the client reads its link;
/// every `drop_every`th frame to the client is lost and every
/// `swap_every`th pair swapped.
struct Lan {
    client: End,
    server: Phy<End>,
    iface: Interface,
    sockets: SocketSet<'static>,
    listener: SocketHandle,
    clock: TestClock,
    size: u64,
    request: Vec<u8>,
    reply: Option<(Vec<u8>, u64, u64)>,
    sent: u64,
    raw: bool,
    raw_sent: usize,
    frames: usize,
    drop_every: usize,
    swap_every: usize,
    ipv6: bool,
    dns_listener: Option<SocketHandle>,
    dns_queries: Vec<u16>,
    dns_truncated: bool,
    dns_tcp_reply: Option<Vec<u8>>,
    dns_tcp_partial: bool,
}

fn pattern(offset: u64) -> u8 {
    (offset.wrapping_mul(0x9e37_79b9) >> 13) as u8
}

impl Lan {
    fn new(size: u64, clock: TestClock) -> Self {
        let (a, b) = (Rc::default(), Rc::default());
        let client = End {
            mac: CLIENT,
            out: Rc::clone(&a),
            into: Rc::clone(&b),
        };
        let mut server = Phy {
            link: End {
                mac: Mac(SERVER),
                out: b,
                into: a,
            },
            tx_frame: Vec::new(),
            rx_frame: Vec::new(),
            fault: None,
            received: false,
            probes: Vec::new(),
            local_ipv6: Ipv6Addr::UNSPECIFIED,
            approved_ipv6: Vec::new(),
            rdnss: Default::default(),
            manage_ipv6: false,
        };
        let config = InterfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(SERVER)));
        let mut iface = Interface::new(config, &mut server, Instant::ZERO);
        iface.update_ip_addrs(|a| a.push(IpCidr::new(SERVER_IP.into(), 24)).unwrap());
        let mut sockets = SocketSet::new(Vec::new());
        let mut socket = tcp::Socket::new(
            tcp::SocketBuffer::new(vec![0; 1 << 20]),
            tcp::SocketBuffer::new(vec![0; 1 << 20]),
        );
        socket.listen(80).unwrap();
        let listener = sockets.add(socket);
        Self {
            client,
            server,
            iface,
            sockets,
            listener,
            clock,
            size,
            request: Vec::new(),
            reply: None,
            sent: 0,
            raw: false,
            raw_sent: 0,
            frames: 0,
            drop_every: 0,
            swap_every: 0,
            ipv6: false,
            dns_listener: None,
            dns_queries: Vec::new(),
            dns_truncated: false,
            dns_tcp_reply: None,
            dns_tcp_partial: false,
        }
    }

    fn ipv6(size: u64, clock: TestClock) -> Self {
        use smoltcp::socket::udp;
        let mut lan = Self::new(size, clock);
        lan.ipv6 = true;
        lan.iface.update_ip_addrs(|addresses| {
            addresses.clear();
            for address in ["fe80::1", "2001:db8:1234::53"] {
                addresses
                    .push(IpCidr::new(address.parse::<Ipv6Addr>().unwrap().into(), 64))
                    .unwrap();
            }
        });
        let mut dns = udp::Socket::new(
            udp::PacketBuffer::new(vec![udp::PacketMetadata::EMPTY; 4], vec![0; 2048]),
            udp::PacketBuffer::new(vec![udp::PacketMetadata::EMPTY; 4], vec![0; 2048]),
        );
        dns.bind(53).unwrap();
        lan.dns_listener = Some(lan.sockets.add(dns));
        lan
    }

    fn step(&mut self) {
        let now = Instant::from_micros(self.clock.0.get() as i64);
        self.iface.poll(now, &mut self.server, &mut self.sockets);
        if let Some(handle) = self.dns_listener {
            let dns = self.sockets.get_mut::<smoltcp::socket::udp::Socket>(handle);
            while dns.can_recv() {
                let (query, endpoint) = dns.recv().unwrap();
                assert_eq!(u16::from_be_bytes([query[4], query[5]]), 1);
                let mut end = 12;
                while query[end] != 0 {
                    end += usize::from(query[end]) + 1;
                }
                end += 1;
                let kind = u16::from_be_bytes([query[end], query[end + 1]]);
                self.dns_queries.push(kind);
                let mut reply = query[..end + 4].to_vec();
                reply[2..4].copy_from_slice(&0x8180u16.to_be_bytes());
                reply[6..8].copy_from_slice(&1u16.to_be_bytes());
                reply[8..12].fill(0);
                if self.dns_truncated {
                    reply[2..4].copy_from_slice(&0x8380u16.to_be_bytes());
                    reply[6..8].fill(0);
                    dns.send_slice(&reply, endpoint).unwrap();
                    continue;
                }
                reply.extend_from_slice(&[0xc0, 0x0c]);
                reply.extend_from_slice(&kind.to_be_bytes());
                reply.extend_from_slice(&[0, 1, 0, 0, 0, 60]);
                assert_eq!(kind, 28, "IPv6-only link must query AAAA");
                reply.extend_from_slice(&16u16.to_be_bytes());
                reply.extend_from_slice(&"2001:db8:1234::53".parse::<Ipv6Addr>().unwrap().octets());
                dns.send_slice(&reply, endpoint).unwrap();
            }
        }
        let socket = self.sockets.get_mut::<tcp::Socket>(self.listener);
        while socket.can_recv() {
            socket
                .recv(|bytes| {
                    self.request.extend_from_slice(bytes);
                    (bytes.len(), ())
                })
                .unwrap();
        }
        if let Some(reply) = &self.dns_tcp_reply {
            if self.request.len() >= 2 {
                let length = u16::from_be_bytes([self.request[0], self.request[1]]) as usize;
                if self.request.len() >= length + 2 {
                    // Match the unpredictable query ID and split the reply to
                    // exercise both length-prefix and DNS-message framing.
                    let mut framed = (reply.len() as u16).to_be_bytes().to_vec();
                    framed.extend_from_slice(reply);
                    framed[2..4].copy_from_slice(&self.request[2..4]);
                    if socket.can_send() && self.raw_sent < framed.len() {
                        let end = if self.dns_tcp_partial {
                            (self.raw_sent + 1).min(framed.len())
                        } else {
                            framed.len()
                        };
                        self.raw_sent += socket.send_slice(&framed[self.raw_sent..end]).unwrap();
                    }
                }
            }
        } else if self.raw {
            while self.raw_sent < self.request.len() && socket.can_send() {
                let n = socket.send_slice(&self.request[self.raw_sent..]).unwrap();
                self.raw_sent += n;
            }
        } else if self.reply.is_none() && self.request.windows(4).any(|w| w == b"\r\n\r\n") {
            let text = core::str::from_utf8(&self.request).unwrap();
            let range = text.split("Range: bytes=").nth(1).unwrap();
            let range = range.split("\r\n").next().unwrap();
            let (first, last) = range.split_once('-').unwrap();
            let (first, last): (u64, u64) = (first.parse().unwrap(), last.parse().unwrap());
            let head = alloc::format!(
                "HTTP/1.1 206 Partial Content\r\nContent-Range: bytes {first}-{last}/{}\r\nContent-Length: {}\r\n\r\n",
                self.size,
                last - first + 1
            );
            self.reply = Some((head.into_bytes(), first, last + 1));
        }
        if let Some((head, first, end)) = &mut self.reply {
            if !head.is_empty() {
                let n = socket.send_slice(head).unwrap();
                head.drain(..n);
            }
            while head.is_empty() && *first + self.sent < *end && socket.can_send() {
                let at = *first + self.sent;
                let chunk: Vec<u8> = (at..(*end).min(at + 4096)).map(pattern).collect();
                self.sent += socket.send_slice(&chunk).unwrap() as u64;
            }
            if *first + self.sent == *end {
                socket.close();
            }
        }
        self.iface.poll(now, &mut self.server, &mut self.sockets);
        // Impair what the server sent.
        let mut queue = self.client.into.borrow_mut();
        let mut kept = VecDeque::new();
        while let Some(frame) = queue.0.pop_front() {
            self.frames += 1;
            if self.drop_every != 0 && self.frames.is_multiple_of(self.drop_every) {
                continue;
            }
            kept.push_back(frame);
            if self.swap_every != 0 && self.frames.is_multiple_of(self.swap_every) {
                let n = kept.len();
                if n >= 2 {
                    kept.swap(n - 1, n - 2);
                }
            }
        }
        queue.0 = kept;
    }
}

impl Link for Lan {
    fn mac(&self) -> Mac {
        CLIENT
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        if self.ipv6
            && frame.len() >= 62
            && frame[12..14] == [0x86, 0xdd]
            && frame[20] == 58
            && frame[54] == 133
        {
            self.client
                .into
                .borrow_mut()
                .0
                .push_back(automatic::tests::advertisement());
        }
        self.client.transmit(frame)
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        if self.client.into.borrow().0.is_empty() {
            self.step();
        }
        self.client.receive()
    }
}

fn stack(lan: Lan, clock: TestClock) -> Stack<Lan, TestClock> {
    let mut s = Stack::new(lan, clock, 7).unwrap();
    s.set_config(Config {
        address: Ipv4Cidr::new(CLIENT_IP, 24),
        router: None,
        dns: Vec::new(),
    })
    .unwrap();
    s
}

#[test]
fn slaac_deadline_remains_visible_without_a_socket_deadline() {
    let clock = TestClock::default();
    let mut stack = Stack::new(Lan::ipv6(8192, clock.clone()), clock.clone(), 7).unwrap();
    stack
        .set_config(Config {
            address: Ipv4Cidr::new(CLIENT_IP, 24),
            router: None,
            dns: Vec::new(),
        })
        .unwrap();
    stack.poll().unwrap();
    // The only socket is an idle DNS client. The emitted first RS still
    // owns a future retry timer even when no socket has a deadline.
    let delay = stack
        .iface
        .poll_delay(
            Instant::from_micros(clock.now_us().unwrap() as i64),
            &stack.sockets,
        )
        .expect("router discovery retry remains scheduled");
    assert!(delay > Duration::ZERO && delay <= Duration::from_secs(4));
}

#[test]
fn an_existing_ipv4_default_route_does_not_block_ipv6_router_admission() {
    let clock = TestClock::default();
    let mut stack = Stack::new(Lan::ipv6(8192, clock.clone()), clock.clone(), 7).unwrap();
    // An authored already-configured IPv4 interface must not be erased
    // by the DHCP socket's legitimate initial Deconfigured event.
    stack
        .set_config(Config {
            address: Ipv4Cidr::new(CLIENT_IP, 24),
            router: Some(SERVER_IP),
            dns: Vec::new(),
        })
        .unwrap();
    for _ in 0..1000 {
        stack.poll().unwrap();
        if stack.ipv6_ready() {
            break;
        }
        clock.0.set(clock.0.get() + 10_000);
    }
    assert!(stack.ipv6_ready());
    assert!(stack.iface.routes().get_default_ipv4_route().is_some());
    assert!(stack.iface.routes().get_default_ipv6_route().is_some());
    assert_eq!(stack.config().unwrap().router, Some(SERVER_IP));
}

#[test]
fn ipv6_only_ra_rdnss_aaaa_and_tcp_range_are_usable_without_ipv4() {
    let clock = TestClock::default();
    let lan = Lan::ipv6(8192, clock.clone());
    let mut stack = Stack::new(lan, clock, 7).unwrap();
    let configuration = stack.automatic().unwrap();
    assert!(stack.config().is_none());
    assert!(!configuration.dns.is_empty());
    let mut bytes = Vec::new();
    assert_eq!(
        stack
            .get_range("restore.test", "/source", 7, 4102, |chunk| bytes
                .extend_from_slice(chunk))
            .unwrap(),
        8192
    );
    assert_eq!(bytes, (7..4103).map(pattern).collect::<Vec<_>>());
    assert_eq!(stack.phy.link.dns_queries, vec![28]);
    assert!(
        stack
            .iface
            .ip_addrs()
            .iter()
            .all(|address| matches!(address, IpCidr::Ipv6(_)))
    );
}

#[test]
fn range_arrives_whole_through_loss_and_reordering() {
    let clock = TestClock::default();
    let mut lan = Lan::new(1 << 40, clock.clone());
    lan.drop_every = 97;
    lan.swap_every = 13;
    let mut s = stack(lan, clock);
    let start = 3 << 30;
    let length = 3 << 20;
    let mut body = Vec::new();
    let size = s
        .get_range("10.0.2.2", "/a.ipsw", start, start + length - 1, |b| {
            body.extend_from_slice(b)
        })
        .unwrap_or_else(|error| {
            panic!(
                "{error:?}; received={} server_sent={} frames={} clock={}",
                body.len(),
                s.phy.link.sent,
                s.phy.link.frames,
                s.phy.link.clock.0.get()
            )
        });
    assert_eq!(size, 1 << 40);
    assert_eq!(body.len() as u64, length);
    assert!(body.iter().zip(start..).all(|(&b, at)| b == pattern(at)));
    let request = core::str::from_utf8(&s.link_mut().request).unwrap();
    assert!(request.starts_with("GET /a.ipsw HTTP/1.1\r\nHost: 10.0.2.2\r\n"));
}

#[test]
fn a_network_without_dhcp_reports_no_ipv4_lease() {
    /// Each reading advances 100 ms.
    struct Coarse(Cell<u64>);
    impl Clock for Coarse {
        fn now_us(&self) -> Result<u64, ClockError> {
            self.0.set(self.0.get() + 100_000);
            Ok(self.0.get())
        }
    }
    let silent = End {
        mac: CLIENT,
        out: Rc::default(),
        into: Rc::default(),
    };
    let mut stack = Stack::new(silent, Coarse(Cell::new(0)), 7).unwrap();
    match stack.dhcp() {
        Err(NetError::Dhcp(why)) => assert!(why.starts_with("no IPv4 lease"), "{why}"),
        other => panic!("{other:?}"),
    }
}

/// A switch port that runs spanning tree forwards nothing for 30 seconds
/// after the link comes up: the client still asks after that, and gives up
/// only after a minute.
#[test]
fn dhcp_still_asks_after_a_spanning_tree_port_starts_forwarding() {
    /// Each reading advances 100 ms.
    #[derive(Clone)]
    struct Coarse(Rc<Cell<u64>>);
    impl Clock for Coarse {
        fn now_us(&self) -> Result<u64, ClockError> {
            self.0.set(self.0.get() + 100_000);
            Ok(self.0.get())
        }
    }
    /// Notes when each DHCP request (UDP to port 67) left.
    struct Recording(End, Coarse, Rc<RefCell<Vec<u64>>>);
    impl Link for Recording {
        fn mac(&self) -> Mac {
            self.0.mac()
        }
        fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
            if frame.len() > 38 && frame[12..14] == [0x08, 0x00] && frame[36..38] == [0, 67] {
                self.2.borrow_mut().push(self.1.0.get());
            }
            self.0.transmit(frame)
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
            self.0.receive()
        }
    }
    let clock = Coarse(Rc::default());
    let requests = Rc::new(RefCell::new(Vec::new()));
    let silent = End {
        mac: CLIENT,
        out: Rc::default(),
        into: Rc::default(),
    };
    let link = Recording(silent, clock.clone(), requests.clone());
    let mut stack = Stack::new(link, clock.clone(), 7).unwrap();
    assert!(matches!(stack.automatic(), Err(NetError::Dhcp(_))));
    assert!(clock.0.get() >= 60_000_000, "{}", clock.0.get());
    let requests = requests.borrow();
    assert!(requests.iter().any(|&at| at > 31_000_000), "{requests:?}");
}

#[test]
fn a_silent_or_absent_peer_times_out() {
    let clock = TestClock::default();
    let mut lan = Lan::new(100, clock.clone());
    lan.drop_every = 1;
    let mut s = stack(lan, clock);
    assert_eq!(
        s.get_range("10.0.2.2", "/", 0, 9, |_| ()),
        Err(NetError::Timeout("TCP transfer"))
    );
    let clock = TestClock::default();
    let mut s = stack(Lan::new(100, clock.clone()), clock);
    // Port 81 has no listener: smoltcp answers with a reset.
    assert_eq!(
        s.get_range("10.0.2.2:81", "/", 0, 9, |_| ()),
        Err(NetError::Reset)
    );
}

#[test]
fn a_signalling_link_sleeps_until_the_deadline_when_nothing_arrives() {
    /// A clock whose sleeps jump to the time asked for.
    #[derive(Clone, Default)]
    struct Sleeping(Rc<Cell<u64>>, Rc<Cell<usize>>);
    impl Clock for Sleeping {
        fn now_us(&self) -> Result<u64, ClockError> {
            Ok(self.0.get())
        }
        fn sleep(&self, until_us: u64) -> Result<(), ClockError> {
            self.1.set(self.1.get() + 1);
            self.0.set(until_us.max(self.0.get()));
            Ok(())
        }
    }
    struct Signalling(End, Option<u64>);
    impl Link for Signalling {
        fn mac(&self) -> Mac {
            self.0.mac
        }
        fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
            self.0.transmit(frame)
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
            self.0.receive()
        }
        fn signals(&self) -> bool {
            true
        }
        fn due_us(&self) -> Option<u64> {
            self.1
        }
    }
    let clock = Sleeping::default();
    let end = || End {
        mac: CLIENT,
        out: Rc::default(),
        into: Rc::default(),
    };
    let mut stack = Stack::new(Signalling(end(), None), clock.clone(), 7).unwrap();
    match stack.dhcp() {
        Err(NetError::Dhcp(why)) => assert!(why.starts_with("no IPv4 lease"), "{why}"),
        other => panic!("{other:?}"),
    }
    // DHCP's retransmissions bound each sleep, so the 60 s wait takes a
    // handful of sleeps rather than a poll per clock reading.
    assert!((1..64).contains(&clock.1.get()), "{}", clock.1.get());
    assert!(clock.0.get() >= DHCP_WAIT_US);
    // The link's own timer ends a sleep the caller would let run on.
    let due = clock.0.get() + 1_000;
    let mut stack = Stack::new(Signalling(end(), Some(due)), clock.clone(), 7).unwrap();
    stack.poll().unwrap();
    stack.wait(1_000_000).unwrap();
    assert_eq!(clock.0.get(), due);
}

#[test]
fn receive_timer_polls_a_packet_when_its_interrupt_is_lost() {
    #[derive(Clone, Default)]
    struct Sleeping(Rc<Cell<u64>>);
    impl Clock for Sleeping {
        fn now_us(&self) -> Result<u64, ClockError> {
            Ok(self.0.get())
        }
        fn sleep(&self, until: u64) -> Result<(), ClockError> {
            self.0.set(until);
            Ok(())
        }
    }
    struct SilentMsi {
        clock: Sleeping,
        delivered: bool,
    }
    impl Link for SilentMsi {
        fn mac(&self) -> Mac {
            CLIENT
        }
        fn transmit(&mut self, _: &[u8]) -> Result<(), LinkError> {
            Ok(())
        }
        fn signals(&self) -> bool {
            true
        }
        fn due_us(&self) -> Option<u64> {
            Some(self.clock.0.get() + 10_000)
        }
        fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
            if !self.delivered && self.clock.0.get() >= 5_000 {
                self.delivered = true;
                // An unrelated Ethernet frame is enough to exercise draining
                // the queue; no completion signal wakes Sleeping::sleep.
                Ok(Some(vec![0; 60]))
            } else {
                Ok(None)
            }
        }
    }
    let clock = Sleeping::default();
    let link = SilentMsi {
        clock: clock.clone(),
        delivered: false,
    };
    let mut stack = Stack::new(link, clock.clone(), 7).unwrap();
    stack.poll().unwrap();
    stack.wait(30_000_000).unwrap();
    assert_eq!(clock.0.get(), 10_000);
    stack.poll().unwrap();
    assert!(stack.link_mut().delivered);
    assert!(stack.phy.received);
}

#[test]
fn a_name_keeps_its_address_and_falls_back_to_it_when_a_lookup_fails() {
    let clock = TestClock::default();
    // No DNS server is configured, so every lookup fails.
    let mut s = stack(Lan::new(100, clock.clone()), clock);
    assert!(matches!(
        s.resolve_now("cdn.example"),
        Err(NetError::Dns(_))
    ));
    let now = s.now().unwrap();
    s.resolved
        .push((String::from("cdn.example"), SERVER_IP.into(), now));
    assert_eq!(s.resolve_now("cdn.example"), Ok(SERVER_IP.into()));
    s.resolved[0].2 = now.wrapping_sub(RESOLVED_US);
    assert_eq!(s.resolve_now("cdn.example"), Ok(SERVER_IP.into()));
    assert_eq!(s.resolve_now("10.0.2.2"), Ok(SERVER_IP.into()));
}

#[test]
fn raw_stream_keeps_partial_dynamic_writes_and_opaque_replies() {
    let clock = TestClock::default();
    let mut lan = Lan::new(0, clock.clone());
    lan.raw = true;
    let mut s = stack(lan, clock);
    let payload: Vec<u8> = (0..200_000).map(|n| (n * 37) as u8).collect();
    let mut transfer = s.open_raw("10.0.2.2", 80, 1 << 20).unwrap();
    let (mut sent, mut received) = (0, Vec::new());
    for _ in 0..1_000_000 {
        if sent < payload.len() {
            sent += s.write(&mut transfer, &payload[sent..]).unwrap();
        }
        let done = s
            .read(&mut transfer, |bytes, _| {
                received.extend_from_slice(bytes);
                Ok(received.len() == payload.len())
            })
            .unwrap();
        if done {
            break;
        }
    }
    assert_eq!(sent, payload.len());
    assert_eq!(received, payload);
    s.close(transfer);
}

#[test]
fn a_named_transfer_opens_without_waiting_and_resolves_as_it_is_driven() {
    let clock = TestClock::default();
    let mut stack = Stack::new(Lan::ipv6(8192, clock.clone()), clock.clone(), 7).unwrap();
    stack.automatic().unwrap();
    let request = http::get_range("restore.test", "/source", 0, 9, None).unwrap();
    let before = clock.0.get();
    let mut transfer = stack.open("restore.test", request, 1024).unwrap();
    // The DNS query is out, and nothing waited for its answer.
    assert!(clock.0.get() - before < 1_000);
    assert!(matches!(transfer.endpoint, Endpoint::Resolving(_)));
    let mut ranged = http::Ranged::new(0, 9);
    let mut bytes = Vec::new();
    let mut done = false;
    for _ in 0..10_000 {
        done = stack
            .read(&mut transfer, |chunk, closed| {
                ranged.feed(chunk, closed, &mut |b: &[u8]| bytes.extend_from_slice(b))
            })
            .unwrap();
        if done {
            break;
        }
    }
    assert!(done);
    assert_eq!(bytes, (0..10).map(pattern).collect::<Vec<_>>());
    assert_eq!(stack.phy.link.dns_queries, vec![28]);
    stack.close(transfer);
}
