//! Bounded DNS-over-TCP fallback when the UDP resolver cannot answer.
//! Keep the original host name for HTTP/TLS; never pin a CDN address.
use super::*;
use alloc::{boxed::Box, format};
use smoltcp::wire::{
    DnsFlags, DnsOpcode, DnsPacket, DnsQuestion, DnsRcode, DnsRecord, DnsRecordData, DnsRepr,
};

const SERVER_WAIT_US: u64 = 8_000_000;
const MAX_MESSAGE: usize = u16::MAX as usize;

pub(super) struct Lookup {
    servers: Vec<IpAddress>,
    next: usize,
    pub(super) transfer: Option<Box<Transfer>>,
    request: Vec<u8>,
    sent: usize,
    reply: Vec<u8>,
    kind: DnsQueryType,
    failures: String,
}

impl<L: Link, C: Clock> Stack<L, C> {
    pub(super) fn start_dns_tcp(
        &mut self,
        lookup: &mut super::Lookup,
        kind: DnsQueryType,
        failure: NetError,
        now: u64,
    ) -> Result<Option<IpAddress>, NetError> {
        let servers: Vec<_> = self
            .dns_servers
            .iter()
            .take(3)
            .copied()
            .filter(|address| self.address_usable(*address))
            .collect();
        if servers.is_empty() {
            return Err(failure);
        }
        let request = request(&lookup.name, kind, self.next_port)?;
        lookup.tcp = Some(Box::new(Lookup {
            servers,
            next: 0,
            transfer: None,
            request,
            sent: 0,
            reply: Vec::new(),
            kind,
            failures: format!("UDP {failure}"),
        }));
        lookup.deadline = now.saturating_add(SERVER_WAIT_US);
        Ok(None)
    }

    pub(super) fn lookup_tcp(
        &mut self,
        lookup: &mut super::Lookup,
        now: u64,
    ) -> Result<Option<IpAddress>, NetError> {
        let mut tcp = lookup.tcp.take().expect("active TCP DNS lookup");
        let server = tcp.servers[tcp.next];
        let result = (|| {
            if now >= lookup.deadline {
                return Err(NetError::Timeout("DNS TCP"));
            }
            if tcp.transfer.is_none() {
                // A literal IP connects without opening another DNS lookup.
                tcp.transfer = Some(Box::new(self.open_raw(&format!("{server}"), 53, 4096)?));
            }
            let transfer = tcp.transfer.as_mut().unwrap();
            if tcp.sent < tcp.request.len() {
                tcp.sent += self.write(transfer, &tcp.request[tcp.sent..])?;
            }
            let mut complete = false;
            self.read(transfer, |bytes, closed| {
                if tcp.reply.len().saturating_add(bytes.len()) > MAX_MESSAGE + 2 {
                    return Err(NetError::Invalid("DNS TCP response too large"));
                }
                tcp.reply.extend_from_slice(bytes);
                if tcp.reply.len() >= 2 {
                    let length = u16::from_be_bytes([tcp.reply[0], tcp.reply[1]]) as usize;
                    if length < 12 {
                        return Err(NetError::Invalid("DNS TCP response too short"));
                    }
                    complete = tcp.reply.len() >= length + 2;
                }
                if closed && !complete {
                    return Err(NetError::Truncated("DNS TCP response"));
                }
                Ok(complete)
            })?;
            if complete {
                let length = u16::from_be_bytes([tcp.reply[0], tcp.reply[1]]) as usize;
                answer(
                    &tcp.reply[2..length + 2],
                    &lookup.name,
                    tcp.kind,
                    u16::from_be_bytes([tcp.request[2], tcp.request[3]]),
                )
                .map(Some)
            } else {
                Ok(None)
            }
        })();
        match result {
            Ok(None) => {
                lookup.tcp = Some(tcp);
                Ok(None)
            }
            Ok(Some(address)) => {
                if let Some(transfer) = tcp.transfer.take() {
                    self.close(*transfer);
                }
                Ok(Some(address))
            }
            Err(error) => {
                if let Some(transfer) = tcp.transfer.take() {
                    self.close(*transfer);
                }
                if matches!(error, NetError::Clock(_)) {
                    return Err(error);
                }
                tcp.failures.push_str(&format!("; TCP {server}: {error}"));
                tcp.next += 1;
                if tcp.next == tcp.servers.len() {
                    return Err(NetError::Dns(format!("{}: {}", lookup.name, tcp.failures)));
                }
                tcp.sent = 0;
                tcp.reply.clear();
                lookup.deadline = now.saturating_add(SERVER_WAIT_US);
                lookup.tcp = Some(tcp);
                Ok(None)
            }
        }
    }
}

fn request(name: &str, kind: DnsQueryType, id: u16) -> Result<Vec<u8>, NetError> {
    let mut encoded = Vec::new();
    for label in name.trim_end_matches('.').split('.') {
        if label.is_empty() || label.len() > 63 || !label.is_ascii() {
            return Err(NetError::Invalid("DNS name label"));
        }
        encoded.push(label.len() as u8);
        encoded.extend_from_slice(label.as_bytes());
    }
    encoded.push(0);
    if encoded.len() > 255 {
        return Err(NetError::Invalid("DNS name too long"));
    }
    let repr = DnsRepr {
        transaction_id: id,
        flags: DnsFlags::RECURSION_DESIRED,
        opcode: DnsOpcode::Query,
        question: DnsQuestion {
            name: &encoded,
            type_: kind,
        },
    };
    let length = repr.buffer_len();
    let mut bytes = vec![0; length + 2];
    bytes[..2].copy_from_slice(&(length as u16).to_be_bytes());
    repr.emit(&mut DnsPacket::new_unchecked(&mut bytes[2..]));
    Ok(bytes)
}

fn name(packet: &DnsPacket<&[u8]>, encoded: &[u8]) -> Result<Vec<Vec<u8>>, NetError> {
    packet
        .parse_name(encoded)
        .map(|label| {
            label
                .map(|label| label.to_ascii_lowercase())
                .map_err(|_| NetError::Invalid("DNS response name"))
        })
        .collect()
}

pub(super) fn answer(
    bytes: &[u8],
    host: &str,
    kind: DnsQueryType,
    id: u16,
) -> Result<IpAddress, NetError> {
    let packet = DnsPacket::new_checked(bytes).map_err(|_| NetError::Invalid("DNS header"))?;
    if packet.transaction_id() != id
        || packet.opcode() != DnsOpcode::Query
        || !packet.flags().contains(DnsFlags::RESPONSE)
        || packet.question_count() != 1
        || packet.rcode() != DnsRcode::NoError
        || packet.flags().contains(DnsFlags::TRUNCATED)
    {
        return Err(NetError::Invalid("DNS response header"));
    }
    let (mut payload, question) =
        DnsQuestion::parse(packet.payload()).map_err(|_| NetError::Invalid("DNS question"))?;
    let expected: Vec<Vec<u8>> = host
        .trim_end_matches('.')
        .split('.')
        .map(|label| label.as_bytes().to_ascii_lowercase())
        .collect();
    if question.type_ != kind || name(&packet, question.name)? != expected {
        return Err(NetError::Invalid("DNS response question mismatch"));
    }
    let mut records = Vec::new();
    for _ in 0..packet.answer_record_count() {
        let (rest, record) =
            DnsRecord::parse(payload).map_err(|_| NetError::Invalid("DNS answer"))?;
        records.push(record);
        payload = rest;
    }
    let mut owner = expected;
    // Resolve a bounded CNAME chain regardless of answer ordering or case.
    for _ in 0..16 {
        for record in &records {
            if name(&packet, record.name)? != owner {
                continue;
            }
            match record.data {
                DnsRecordData::A(address) if kind == DnsQueryType::A => return Ok(address.into()),
                DnsRecordData::Aaaa(address) if kind == DnsQueryType::Aaaa => {
                    return Ok(address.into());
                }
                _ => {}
            }
        }
        let mut alias = None;
        for record in &records {
            if name(&packet, record.name)? == owner
                && let DnsRecordData::Cname(target) = record.data
            {
                alias = Some(name(&packet, target)?);
                break;
            }
        }
        owner = alias.ok_or_else(|| NetError::Dns(format!("{host}: no address in TCP answer")))?;
    }
    Err(NetError::Invalid("DNS CNAME chain too long or cyclic"))
}
