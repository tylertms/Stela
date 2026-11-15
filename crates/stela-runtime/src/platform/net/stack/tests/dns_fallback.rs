//! A UDP refusal must not prevent a valid DNS-over-TCP response.
use super::*;
use smoltcp::socket::udp;

fn encoded(name: &str) -> Vec<u8> {
    let mut bytes = Vec::new();
    for label in name.split('.') {
        bytes.push(label.len() as u8);
        bytes.extend_from_slice(label.as_bytes());
    }
    bytes.push(0);
    bytes
}

fn reply() -> Vec<u8> {
    let mut bytes = vec![0x12, 0x34, 0x81, 0x80, 0, 1, 0, 2, 0, 0, 0, 0];
    bytes.extend_from_slice(&encoded("cdn.example"));
    bytes.extend_from_slice(&[0, 1, 0, 1]);
    // The A record comes before its alias and uses different letter case.
    bytes.extend_from_slice(&encoded("EDGE.EXAMPLE"));
    bytes.extend_from_slice(&[0, 1, 0, 1, 0, 0, 0, 60, 0, 4]);
    bytes.extend_from_slice(&SERVER_IP.octets());
    bytes.extend_from_slice(&[0xc0, 0x0c, 0, 5, 0, 1, 0, 0, 0, 60]);
    let alias = encoded("edge.example");
    bytes.extend_from_slice(&(alias.len() as u16).to_be_bytes());
    bytes.extend_from_slice(&alias);
    bytes
}

fn peer(clock: TestClock) -> Lan {
    let mut lan = Lan::new(0, clock);
    let socket = lan.sockets.get_mut::<tcp::Socket>(lan.listener);
    socket.abort();
    socket.listen(53).unwrap();
    let mut udp = udp::Socket::new(
        udp::PacketBuffer::new(vec![udp::PacketMetadata::EMPTY; 4], vec![0; 2048]),
        udp::PacketBuffer::new(vec![udp::PacketMetadata::EMPTY; 4], vec![0; 2048]),
    );
    udp.bind(53).unwrap();
    lan.dns_listener = Some(lan.sockets.add(udp));
    lan.dns_truncated = true;
    lan.dns_tcp_reply = Some(reply());
    lan.dns_tcp_partial = true;
    lan
}

#[test]
fn truncated_udp_retries_over_tcp_and_handles_split_prefix_and_cname_order() {
    let clock = TestClock::default();
    let mut stack = stack(peer(clock.clone()), clock);
    stack.set_dns(&[core::net::IpAddr::V4(SERVER_IP)]).unwrap();
    let sockets = stack.sockets.iter().count();
    assert_eq!(stack.resolve_now("cdn.example"), Ok(SERVER_IP.into()));
    assert_eq!(stack.phy.link.dns_queries, vec![1]);
    assert_eq!(
        stack.sockets.iter().count(),
        sockets,
        "DNS TCP socket leaked"
    );
}

#[test]
fn tcp_dns_checks_id_question_and_truncation_and_rejects_bad_lengths() {
    let good = reply();
    let decode =
        |bytes: &[u8], host, id| super::super::dns_tcp::answer(bytes, host, DnsQueryType::A, id);
    assert_eq!(decode(&good, "cdn.example", 0x1234), Ok(SERVER_IP.into()));
    assert!(decode(&good, "cdn.example", 0x1235).is_err());
    assert!(decode(&good, "other.example", 0x1234).is_err());
    let mut truncated = good.clone();
    truncated[2] |= 2;
    assert!(decode(&truncated, "cdn.example", 0x1234).is_err());
    for end in 0..good.len() {
        assert!(decode(&good[..end], "cdn.example", 0x1234).is_err());
    }
}

#[test]
fn cancelled_tcp_dns_releases_its_connection() {
    let clock = TestClock::default();
    let mut stack = stack(peer(clock.clone()), clock);
    stack.set_dns(&[core::net::IpAddr::V4(SERVER_IP)]).unwrap();
    let count = stack.sockets.iter().count();
    let mut transfer = stack.open_raw("cdn.example", 80, 1024).unwrap();
    for _ in 0..1000 {
        stack.read(&mut transfer, |_, _| Ok(false)).unwrap();
        if let Endpoint::Resolving(lookup) = &transfer.endpoint
            && lookup
                .tcp
                .as_ref()
                .is_some_and(|lookup| lookup.transfer.is_some())
        {
            break;
        }
    }
    assert_eq!(stack.sockets.iter().count(), count + 1);
    stack.close(transfer);
    assert_eq!(stack.sockets.iter().count(), count);
}

#[test]
fn tcp_dns_tries_the_next_server_and_times_out_without_leaking() {
    let clock = TestClock::default();
    let mut stack = stack(peer(clock.clone()), clock.clone());
    stack.set_dns(&[core::net::IpAddr::V4(SERVER_IP)]).unwrap();
    let sockets = stack.sockets.iter().count();
    let mut lookup = super::super::Lookup {
        name: String::from("cdn.example"),
        port: 80,
        buffer: 1024,
        raw: false,
        query: None,
        tcp: None,
        ipv4_only: false,
        deadline: 0,
    };
    let now = stack.now().unwrap();
    // First try an unreachable address; the second server has an answer.
    stack.dns_servers.insert(0, Ipv4::new(10, 0, 2, 99).into());
    stack
        .start_dns_tcp(&mut lookup, DnsQueryType::A, NetError::Timeout("DNS"), now)
        .unwrap();
    assert_eq!(stack.resolve(&mut lookup).unwrap(), None);
    clock.0.set(lookup.deadline);
    assert_eq!(stack.resolve(&mut lookup).unwrap(), None);
    let mut found = None;
    for _ in 0..10_000 {
        found = stack.resolve(&mut lookup).unwrap();
        if found.is_some() {
            break;
        }
        stack.poll().unwrap();
    }
    assert_eq!(found, Some(SERVER_IP.into()));
    assert_eq!(stack.sockets.iter().count(), sockets);
    stack.resolved.clear();
    stack
        .set_dns(&[
            core::net::IpAddr::V4(Ipv4::new(10, 0, 2, 99)),
            core::net::IpAddr::V4(Ipv4::new(10, 0, 2, 98)),
        ])
        .unwrap();
    // Every server times out: preserve their names and free each connection.
    let now = stack.now().unwrap();
    stack
        .start_dns_tcp(&mut lookup, DnsQueryType::A, NetError::Timeout("DNS"), now)
        .unwrap();
    stack.resolve(&mut lookup).unwrap();
    clock.0.set(lookup.deadline);
    stack.resolve(&mut lookup).unwrap();
    stack.resolve(&mut lookup).unwrap();
    clock.0.set(lookup.deadline);
    let error = stack.resolve(&mut lookup).unwrap_err();
    assert!(
        matches!(error, NetError::Dns(ref message) if message.contains("10.0.2.99") && message.contains("10.0.2.98")),
        "{error:?}"
    );
    assert_eq!(stack.sockets.iter().count(), sockets);
}

#[test]
fn udp_wait_allows_each_configured_server_its_retry_window() {
    let clock = TestClock::default();
    let mut stack = stack(peer(clock.clone()), clock.clone());
    stack
        .set_dns(&[
            core::net::IpAddr::V4(Ipv4::new(10, 0, 2, 99)),
            core::net::IpAddr::V4(SERVER_IP),
            core::net::IpAddr::V4(Ipv4::new(10, 0, 2, 98)),
        ])
        .unwrap();
    let now = clock.0.get();
    let transfer = stack.open_raw("cdn.example", 80, 1024).unwrap();
    let Endpoint::Resolving(lookup) = &transfer.endpoint else {
        panic!("lookup missing");
    };
    assert!(lookup.deadline >= now + 31_000_000);
    stack.close(transfer);
}
