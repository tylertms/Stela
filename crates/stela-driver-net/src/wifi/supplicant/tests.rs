use super::*;
use alloc::vec;

fn hex(text: &str) -> Vec<u8> {
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).unwrap())
        .collect()
}

#[test]
fn passphrase_psk_matches_the_ieee_test_vectors() {
    assert_eq!(
        passphrase_psk(b"password", b"IEEE")[..],
        hex("f42c6fc52df0ebef9ebb4b90b38a5f902e83fe1b135a70e23aed762e9710a12e")
    );
    assert_eq!(
        passphrase_psk(b"ThisIsAPassword", b"ThisIsASSID")[..],
        hex("0dc0d6eb90555ed6419756b9a15ec3e3209b63df707dd508d14581f8982721af")
    );
}

#[test]
fn prf_matches_the_ieee_test_vector() {
    assert_eq!(
        prf_sha1(&[0x0b; 20], b"prefix", b"Hi There", 64)[..],
        hex(concat!(
            "bcd4c650b30b9684951829e0d75f9d54b862175ed9f00606e17d8da35402ffee",
            "75df78c3d31e0f889f012120c0862beb67753e7439ae242edb8373698356cf5a"
        ))
    );
}

#[test]
fn key_unwrap_matches_rfc_3394_and_refuses_a_wrong_kek() {
    let kek: [u8; 16] = hex("000102030405060708090a0b0c0d0e0f").try_into().unwrap();
    let wrapped = hex("1fa68b0a8112b447aef34bd8fb5a7b829d3e862371d2cfe5");
    let plain = hex("00112233445566778899aabbccddeeff");
    assert_eq!(unwrap(&kek, &wrapped).unwrap()[..], plain);
    assert_eq!(wrap(&kek, &plain), wrapped);
    assert_eq!(unwrap(&[0; 16], &wrapped).unwrap_err(), Error::Unwrap);
}

#[test]
fn cmac_matches_rfc_4493() {
    let key: [u8; 16] = hex("2b7e151628aed2a6abf7158809cf4f3c").try_into().unwrap();
    assert_eq!(cmac(&key, &[])[..], hex("bb1d6929e95937287fa37d129b756746"));
    assert_eq!(
        cmac(&key, &hex("6bc1bee22e409f96e93d7e117393172a"))[..],
        hex("070a16b46b4d4144f79bdd9dd04a287c")
    );
    let forty = hex(concat!(
        "6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51",
        "30c81c46a35ce411"
    ));
    assert_eq!(
        cmac(&key, &forty)[..],
        hex("dfa66747de9ae63030ca32611497c827")
    );
}

const STATION: Mac = Mac([0x02, 0, 0, 0, 0, 0x01]);
const AP: Mac = Mac([0x02, 0, 0, 0, 0, 0x02]);
const RSN: [u8; 22] = [
    0x30, 20, 1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac, 2, 0, 0,
];

fn suites(akm: Akm) -> Suites {
    Suites {
        akm,
        pairwise: Cipher::Ccmp128,
        group: Cipher::Ccmp128,
        management: None,
    }
}

/// The authenticator's side of the handshakes.
pub(crate) struct Authenticator {
    suites: Suites,
    akm: Akm,
    pmk: Zeroizing<[u8; 32]>,
    own: Mac,
    station: Mac,
    /// The station's RSN element, which message 2 carries.
    station_rsn: Vec<u8>,
    anonce: [u8; 32],
    replay: u64,
    ptk: Option<Ptk>,
}

impl Authenticator {
    fn new(suites: Suites) -> Self {
        Self::between(
            suites,
            Zeroizing::new([0x44; 32]),
            AP,
            STATION,
            RSN.to_vec(),
        )
    }

    /// The authenticator `own` with `pmk` for `station`, whose message 2
    /// carries `station_rsn`.
    pub(crate) fn between(
        suites: Suites,
        pmk: Zeroizing<[u8; 32]>,
        own: Mac,
        station: Mac,
        station_rsn: Vec<u8>,
    ) -> Self {
        Self {
            suites,
            akm: suites.akm,
            pmk,
            own,
            station,
            station_rsn,
            anonce: [0xa1; 32],
            replay: 0,
            ptk: None,
        }
    }

    /// The same authenticator as an AP MLD's: message 2 names the station MLD.
    pub(crate) fn naming_station_mld(mut self) -> Self {
        put_kde(&mut self.station_rsn, MAC_ADDRESS_KDE, &self.station.0);
        self
    }

    /// Message 2 also names the station on `links`.
    pub(crate) fn naming_links(mut self, links: &[Link]) -> Self {
        for link in links {
            let mut body = vec![link.link_id];
            body.extend_from_slice(&link.local.0);
            put_kde(&mut self.station_rsn, MLO_LINK_KDE, &body);
        }
        self
    }

    /// Message 3 of an AP MLD, with `links` its accepted links and `keys`
    /// theirs.
    pub(crate) fn mld_message_3(&mut self, links: &[Link], keys: &[u8]) -> Vec<u8> {
        let mut data = links[0].rsn.clone();
        put_kde(&mut data, MAC_ADDRESS_KDE, &self.own.0);
        data.extend(links.iter().flat_map(link_kde));
        data.extend_from_slice(keys);
        self.signed(PAIRWISE | ACK | MIC | INSTALL | SECURE | ENCRYPTED, &data)
    }

    pub(crate) fn message_1(&mut self) -> Vec<u8> {
        self.replay += 1;
        build(
            self.akm,
            PAIRWISE | ACK,
            self.replay,
            &self.anonce,
            &[],
            None,
        )
    }

    /// Check message 2 and derive the PTK from its SNonce.
    pub(crate) fn message_2(&mut self, frame: &[u8]) {
        let key = Frame::parse(frame).unwrap();
        assert_eq!(key.info & (PAIRWISE | MIC | ACK), PAIRWISE | MIC);
        assert_eq!(key.data, self.station_rsn);
        let ptk = Ptk::derive(
            &self.suites,
            &self.pmk,
            self.own,
            self.station,
            &self.anonce,
            &key.nonce,
        );
        key.verify(self.akm, ptk.kck()).unwrap();
        self.ptk = Some(ptk);
    }

    /// An encrypted, signed frame whose key data holds `elements`.
    fn signed(&mut self, info: u16, elements: &[u8]) -> Vec<u8> {
        let ptk = self.ptk.as_ref().unwrap();
        let mut plain = elements.to_vec();
        if !plain.len().is_multiple_of(8) || plain.len() < 16 {
            plain.push(VENDOR);
            while !plain.len().is_multiple_of(8) || plain.len() < 16 {
                plain.push(0);
            }
        }
        self.replay += 1;
        let wrapped = wrap(ptk.kek(), &plain);
        let mut frame = build(self.akm, info, self.replay, &self.anonce, &wrapped, None);
        frame[HEADER + 61..HEADER + 69].copy_from_slice(&[7, 0, 0, 0, 0, 0, 0, 0]);
        let signature = mic(self.akm, ptk.kck(), &frame);
        frame[MIC_OFFSET..MIC_OFFSET + MIC_BYTES].copy_from_slice(&signature);
        frame
    }

    pub(crate) fn message_3(&mut self, rsn: &[u8]) -> Vec<u8> {
        self.message_3_with(rsn, &[])
    }

    pub(crate) fn message_3_with(&mut self, rsn: &[u8], extra: &[u8]) -> Vec<u8> {
        let mut elements = rsn.to_vec();
        let length = self.suites.group.key_length();
        elements.extend_from_slice(&gtk_kde(
            1,
            &(0x50..0x50 + length as u8).collect::<Vec<_>>(),
        ));
        elements.extend_from_slice(extra);
        self.signed(
            PAIRWISE | ACK | MIC | INSTALL | SECURE | ENCRYPTED,
            &elements,
        )
    }

    fn group_message_1(&mut self) -> Vec<u8> {
        self.signed(ACK | MIC | SECURE | ENCRYPTED, &gtk_kde(2, &[0x66; 16]))
    }

    pub(crate) fn signed_reply(&self, frame: &[u8], info: u16) {
        let key = Frame::parse(frame).unwrap();
        assert_eq!(key.info & !VERSION, info);
        assert_eq!(key.replay, self.replay);
        key.verify(self.akm, self.ptk.as_ref().unwrap().kck())
            .unwrap();
    }
}

fn gtk_kde(index: u8, key: &[u8]) -> Vec<u8> {
    let mut kde = vec![
        VENDOR,
        6 + key.len() as u8,
        0x00,
        0x0f,
        0xac,
        GTK_KDE,
        index,
        0,
    ];
    kde.extend_from_slice(key);
    kde
}

fn supplicant(suites: Suites) -> Supplicant {
    Supplicant::new(
        suites,
        Zeroizing::new([0x44; 32]),
        STATION,
        AP,
        RSN.to_vec(),
        RSN.to_vec(),
    )
    .unwrap()
}

/// The handshake through message 2.
fn begun(suites: Suites) -> (Authenticator, Supplicant) {
    let mut ap = Authenticator::new(suites);
    let mut station = supplicant(suites);
    let step = station.receive(&ap.message_1(), &mut random).unwrap();
    ap.message_2(&step.reply.unwrap());
    (ap, station)
}

fn random() -> [u8; 32] {
    [0x5c; 32]
}

#[test]
fn four_way_and_group_handshakes_install_the_authenticators_keys() {
    for akm in [Akm::Psk, Akm::PskSha256, Akm::Sae] {
        let (mut ap, mut station) = begun(suites(akm));
        assert!(!station.established());
        let step = station.receive(&ap.message_3(&RSN), &mut random).unwrap();
        ap.signed_reply(&step.reply.unwrap(), PAIRWISE | MIC | SECURE);
        let pairwise = step.pairwise.unwrap();
        assert_eq!(pairwise.material[..], ap.ptk.as_ref().unwrap().tk()[..]);
        assert_eq!(
            (pairwise.index, pairwise.cipher, pairwise.scope),
            (0, Cipher::Ccmp128, KeyScope::Pairwise(AP))
        );
        assert_eq!(pairwise.receive, ReplaySeed::ZERO);
        let group = step.group.unwrap();
        let expected: Vec<u8> = (0x50..0x60).collect();
        assert_eq!((group.index, &group.material[..]), (1, &expected[..]));
        assert_eq!(group.scope, KeyScope::Group(AP));
        assert_eq!(
            group.receive,
            ReplaySeed::uniform(PacketNumber::new(7).unwrap())
        );
        assert!(step.integrity.is_none());
        assert!(station.established());
        let step = station.receive(&ap.group_message_1(), &mut random).unwrap();
        ap.signed_reply(&step.reply.unwrap(), MIC | SECURE);
        let group = step.group.unwrap();
        assert_eq!((group.index, &group.material[..]), (2, &[0x66; 16][..]));
        assert!(step.pairwise.is_none());
    }
}

#[test]
fn a_michael_report_counts_up_under_the_ptks_mic() {
    let (mut ap, mut station) = begun(suites(Akm::Psk));
    assert_eq!(station.michael_report(false), None, "no PTK yet");
    let step = station.receive(&ap.message_3(&RSN), &mut random).unwrap();
    assert!(step.pairwise.is_some());
    let kck = *ap.ptk.as_ref().unwrap().kck();
    for (replay, pairwise) in [(0, false), (1, true)] {
        let report = station.michael_report(pairwise).unwrap();
        let key = Frame::parse(&report).unwrap();
        assert_eq!(key.replay, replay);
        let kind = if pairwise { PAIRWISE } else { 0 };
        let info = REQUEST | ERROR | MIC | SECURE | kind;
        assert_eq!(key.info & !VERSION, info);
        key.verify(Akm::Psk, &kck).unwrap();
    }
}

#[test]
fn a_pmf_message_3_yields_its_igtk_and_a_tkip_gtk_swaps_its_michael_keys() {
    let suites = Suites {
        group: Cipher::Tkip,
        management: Some(ManagementCipher::Cmac128),
        ..suites(Akm::PskSha256)
    };
    let (mut ap, mut station) = begun(suites);
    let mut igtk = vec![VENDOR, 4 + 8 + 16, 0x00, 0x0f, 0xac, IGTK_KDE, 4, 0];
    igtk.extend_from_slice(&[9, 0, 0, 0, 0, 0]);
    igtk.extend_from_slice(&[0x77; 16]);
    let step = station
        .receive(&ap.message_3_with(&RSN, &igtk), &mut random)
        .unwrap();
    let integrity = step.integrity.unwrap();
    assert_eq!(
        (integrity.index, integrity.cipher, integrity.receive),
        (4, ManagementCipher::Cmac128, PacketNumber::new(9).unwrap())
    );
    assert_eq!(integrity.material[..], [0x77; 16]);
    let group = step.group.unwrap();
    let mut expected: Vec<u8> = (0x50..0x70).collect();
    expected[16..].rotate_left(8);
    assert_eq!(
        (group.cipher, &group.material[..]),
        (Cipher::Tkip, &expected[..])
    );

    // An IGTK index other than 4 or 5 is refused.
    let (mut ap, mut station) = begun(suites);
    igtk[6] = 6;
    assert_eq!(
        station
            .receive(&ap.message_3_with(&RSN, &igtk), &mut random)
            .unwrap_err(),
        Error::GroupKey
    );
}

#[test]
fn suites_come_from_the_pmf_selection_and_refuse_unkeyed_ciphers() {
    let new = |suites| {
        Supplicant::new(suites, Zeroizing::new([0; 32]), STATION, AP, vec![], vec![]).err()
    };
    let tkip = Suites {
        pairwise: Cipher::Tkip,
        ..suites(Akm::Psk)
    };
    let wep = Suites {
        group: Cipher::Wep104,
        ..suites(Akm::Psk)
    };
    assert_eq!(
        (new(tkip), new(wep)),
        (Some(Error::Unsupported), Some(Error::Unsupported))
    );
    assert_eq!(new(suites(Akm::Psk)), None);
    assert_eq!(Akm::from_suite(6), Some(Akm::PskSha256));
    assert_eq!(Akm::from_suite(1), None);
}

#[test]
fn replayed_forged_and_mismatched_message_3_install_nothing() {
    let (mut ap, mut station) = begun(suites(Akm::Psk));
    let mut forged = ap.message_3(&RSN);
    forged[MIC_OFFSET] ^= 1;
    assert_eq!(
        station.receive(&forged, &mut random).unwrap_err(),
        Error::Mic
    );
    let mut other = RSN;
    other[19] = 6;
    assert_eq!(
        station
            .receive(&ap.message_3(&other), &mut random)
            .unwrap_err(),
        Error::RsnElement
    );
    let message_3 = ap.message_3(&RSN);
    station.receive(&message_3, &mut random).unwrap();
    assert_eq!(
        station.receive(&message_3, &mut random).unwrap_err(),
        Error::Replay
    );
}

#[test]
fn group_keys_and_message_3_wait_for_their_handshake() {
    let mut ap = Authenticator::new(suites(Akm::Psk));
    ap.ptk = Some(Ptk::derive(
        &ap.suites,
        &ap.pmk,
        AP,
        STATION,
        &ap.anonce,
        &random(),
    ));
    let mut station = supplicant(suites(Akm::Psk));
    assert_eq!(
        station
            .receive(&ap.group_message_1(), &mut random)
            .unwrap_err(),
        Error::Sequence
    );
    assert_eq!(
        station
            .receive(&ap.message_3(&RSN), &mut random)
            .unwrap_err(),
        Error::Sequence
    );
    let mut wrong = supplicant(suites(Akm::PskSha256));
    assert_eq!(
        wrong.receive(&ap.message_1(), &mut random).unwrap_err(),
        Error::Version
    );
}

/// The setup link 0 and link 1 the AP MLD accepted, and link 2 it refused.
fn multi_link() -> MultiLink {
    let link = |link_id: u8, rsnxe: Option<Vec<u8>>, accepted| Link {
        link_id,
        local: Mac([0x02, 0x0a, 0, 0, 0, link_id]),
        ap: Mac([0x02, 0x0b, 0, 0, 0, link_id]),
        rsn: RSN.to_vec(),
        rsnxe,
        accepted,
    };
    MultiLink {
        association: 0,
        links: vec![
            link(0, None, true),
            link(1, Some(vec![244, 1, 0x20]), true),
            link(2, None, false),
        ],
    }
}

/// The MLO Link KDE naming `link`'s AP with its RSN element and RSNXE.
fn link_kde(link: &Link) -> Vec<u8> {
    let mut body = vec![link.link_id | LINK_RSNE];
    if link.rsnxe.is_some() {
        body[0] |= LINK_RSNXE;
    }
    body.extend_from_slice(&link.ap.0);
    body.extend_from_slice(&link.rsn);
    body.extend(link.rsnxe.iter().flatten());
    let mut kde = Vec::new();
    put_kde(&mut kde, MLO_LINK_KDE, &body);
    kde
}

/// An MLO GTK KDE of `link_id` with PN 3, and its MLO IGTK KDE with IPN 9.
pub(crate) fn link_keys(link_id: u8, index: u8, fill: u8) -> Vec<u8> {
    let mut gtk = vec![index | link_id << 4, 3, 0, 0, 0, 0, 0];
    gtk.extend_from_slice(&[fill; 16]);
    let mut igtk = vec![4, 0, 9, 0, 0, 0, 0, 0, link_id << 4];
    igtk.extend_from_slice(&[fill + 0x10; 16]);
    let mut kdes = Vec::new();
    put_kde(&mut kdes, MLO_GTK_KDE, &gtk);
    put_kde(&mut kdes, MLO_IGTK_KDE, &igtk);
    kdes
}

/// A multi-link handshake through message 2, which names the station MLD
/// and its address on each other requested link.
fn multi_link_begun() -> (Authenticator, Supplicant, Suites) {
    let suites = Suites {
        management: Some(ManagementCipher::Cmac128),
        ..suites(Akm::Sae)
    };
    let mut message_2 = RSN.to_vec();
    put_kde(&mut message_2, MAC_ADDRESS_KDE, &STATION.0);
    for link in &multi_link().links[1..] {
        let mut body = vec![link.link_id];
        body.extend_from_slice(&link.local.0);
        put_kde(&mut message_2, MLO_LINK_KDE, &body);
    }
    let mut ap = Authenticator::between(suites, Zeroizing::new([0x44; 32]), AP, STATION, message_2);
    let mut station = supplicant(suites).with_multi_link(multi_link()).unwrap();
    let step = station.receive(&ap.message_1(), &mut random).unwrap();
    ap.message_2(&step.reply.unwrap());
    (ap, station, suites)
}

/// Message 3 of the AP MLD: its address, then `links` and `keys`.
fn multi_link_message_3(
    ap: &mut Authenticator,
    address: Mac,
    links: &[u8],
    keys: &[u8],
) -> Vec<u8> {
    let mut data = RSN.to_vec();
    put_kde(&mut data, MAC_ADDRESS_KDE, &address.0);
    data.extend_from_slice(links);
    data.extend_from_slice(keys);
    ap.signed(PAIRWISE | ACK | MIC | INSTALL | SECURE | ENCRYPTED, &data)
}

#[test]
fn a_multi_link_handshake_names_the_mlds_and_keys_each_accepted_link() {
    let (mut ap, mut station, _) = multi_link_begun();
    let accepted = &multi_link().links[..2];
    let links: Vec<u8> = accepted.iter().flat_map(link_kde).collect();
    let keys = [link_keys(0, 1, 0x50), link_keys(1, 1, 0x60)].concat();
    let frame = multi_link_message_3(&mut ap, AP, &links, &keys);
    let step = station.receive(&frame, &mut random).unwrap();
    // Message 4 names the station MLD.
    let reply = step.reply.unwrap();
    ap.signed_reply(&reply, PAIRWISE | MIC | SECURE);
    let mut address = Vec::new();
    put_kde(&mut address, MAC_ADDRESS_KDE, &STATION.0);
    assert_eq!(Frame::parse(&reply).unwrap().data, address);
    assert_eq!(step.pairwise.unwrap().scope, KeyScope::Pairwise(AP));
    let group = step.group.unwrap();
    assert_eq!(
        (group.index, &group.material[..], group.scope, group.receive),
        (
            1,
            &[0x50; 16][..],
            KeyScope::Group(accepted[0].ap),
            ReplaySeed::uniform(PacketNumber::new(3).unwrap())
        )
    );
    assert_eq!(step.integrity.unwrap().material[..], [0x60; 16]);
    let [link] = &step.links[..] else {
        panic!("one other link");
    };
    assert_eq!(link.link_id, 1);
    assert_eq!(
        (&link.group.material[..], link.group.scope),
        (&[0x60; 16][..], KeyScope::Group(accepted[1].ap))
    );
    let integrity = link.integrity.as_ref().unwrap();
    assert_eq!(
        (integrity.index, integrity.receive, &integrity.material[..]),
        (4, PacketNumber::new(9).unwrap(), &[0x70; 16][..])
    );

    // The group key handshake rekeys each link and names the station MLD.
    let keys = [link_keys(1, 2, 0x61), link_keys(0, 2, 0x51)].concat();
    let frame = ap.signed(ACK | MIC | SECURE | ENCRYPTED, &keys);
    let step = station.receive(&frame, &mut random).unwrap();
    let reply = step.reply.unwrap();
    ap.signed_reply(&reply, MIC | SECURE);
    assert_eq!(Frame::parse(&reply).unwrap().data, address);
    assert_eq!(step.group.unwrap().material[..], [0x51; 16]);
    assert_eq!(step.links[0].group.material[..], [0x61; 16]);
}

#[test]
fn a_multi_link_message_3_misnaming_or_missing_a_link_installs_nothing() {
    let accepted = &multi_link().links[..2];
    let links: Vec<u8> = accepted.iter().flat_map(link_kde).collect();
    let keys = [link_keys(0, 1, 0x50), link_keys(1, 1, 0x60)].concat();
    let without_rsnxe = Link {
        rsnxe: None,
        ..accepted[1].clone()
    };
    let refused = [link_kde(&accepted[0]), link_kde(&multi_link().links[2])].concat();
    for (address, links, keys, error) in [
        // Another AP MLD.
        (STATION, &links[..], &keys[..], Error::MultiLink),
        // Link 1 missing, an RSNXE the AP advertises missing, a refused link.
        (AP, &link_kde(&accepted[0])[..], &keys[..], Error::MultiLink),
        (
            AP,
            &[link_kde(&accepted[0]), link_kde(&without_rsnxe)].concat()[..],
            &keys[..],
            Error::MultiLink,
        ),
        (AP, &refused[..], &keys[..], Error::MultiLink),
        // Link 1's GTK missing.
        (
            AP,
            &links[..],
            &link_keys(0, 1, 0x50)[..],
            Error::MissingGroupKey,
        ),
    ] {
        let (mut ap, mut station, _) = multi_link_begun();
        let frame = multi_link_message_3(&mut ap, address, links, keys);
        assert_eq!(station.receive(&frame, &mut random).unwrap_err(), error);
        assert!(!station.established());
    }
    // The setup link must be among the accepted ones.
    let mut unaccepted = multi_link();
    unaccepted.links[0].accepted = false;
    assert_eq!(
        supplicant(suites(Akm::Sae))
            .with_multi_link(unaccepted)
            .err(),
        Some(Error::MultiLink)
    );
}

#[test]
fn a_repeated_message_3_after_a_lost_message_4_answers_again_without_reinstalling() {
    for akm in [Akm::Psk, Akm::Sae] {
        let (mut ap, mut station) = begun(suites(akm));
        let step = station.receive(&ap.message_3(&RSN), &mut random).unwrap();
        assert!(step.pairwise.is_some() && step.group.is_some());
        // Message 4 is lost: the authenticator sends message 3 again with a
        // higher replay counter, as wpa_supplicant expects.
        let step = station.receive(&ap.message_3(&RSN), &mut random).unwrap();
        ap.signed_reply(&step.reply.unwrap(), PAIRWISE | MIC | SECURE);
        assert!(step.pairwise.is_none() && step.group.is_none() && step.integrity.is_none());
        // A forged repeat changes nothing.
        let mut forged = ap.message_3(&RSN);
        forged[MIC_OFFSET] ^= 1;
        assert_eq!(
            station.receive(&forged, &mut random).unwrap_err(),
            Error::Mic
        );
        // A new 4-way handshake (PTK rekey) still installs its keys.
        ap.anonce = [0xb2; 32];
        let step = station.receive(&ap.message_1(), &mut random).unwrap();
        ap.message_2(&step.reply.unwrap());
        let step = station.receive(&ap.message_3(&RSN), &mut random).unwrap();
        ap.signed_reply(&step.reply.unwrap(), PAIRWISE | MIC | SECURE);
        assert_eq!(
            step.pairwise.unwrap().material[..],
            ap.ptk.as_ref().unwrap().tk()[..]
        );
        // The GTK did not change: it stays installed, its counter untouched.
        assert!(step.group.is_none());
    }
}

#[test]
fn a_repeated_group_message_answers_without_reinstalling_its_key() {
    let (mut ap, mut station) = begun(suites(Akm::Psk));
    station.receive(&ap.message_3(&RSN), &mut random).unwrap();
    let step = station.receive(&ap.group_message_1(), &mut random).unwrap();
    assert_eq!(step.group.unwrap().index, 2);
    // The authenticator repeats group message 1 after a lost reply: the
    // same GTK is answered, not reinstalled (wpa_supplicant_install_gtk).
    let step = station.receive(&ap.group_message_1(), &mut random).unwrap();
    ap.signed_reply(&step.reply.unwrap(), MIC | SECURE);
    assert!(step.group.is_none());
}
