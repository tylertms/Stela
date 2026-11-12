use super::*;
use alloc::vec;

fn ap(body: &[u8]) -> Vec<u8> {
    let mut elements = vec![0, 2, b'n', b'w', 48, body.len() as u8];
    elements.extend_from_slice(body);
    elements
}

/// Version 1, group `group`, pairwise `pairwise`, AKMs `akms` and
/// capabilities `capabilities`, all 00-0F-AC suites.
fn rsn(group: u8, pairwise: &[u8], akms: &[u8], capabilities: u16) -> Vec<u8> {
    let mut body = vec![1, 0, 0, 0x0f, 0xac, group];
    for list in [pairwise, akms] {
        body.extend_from_slice(&(list.len() as u16).to_le_bytes());
        for &suite in list {
            body.extend_from_slice(&[0, 0x0f, 0xac, suite]);
        }
    }
    body.extend_from_slice(&capabilities.to_le_bytes());
    body
}

#[test]
fn a_wpa2_personal_ap_takes_psk_and_ccmp_without_protection() {
    let security = Security::select(&ap(&rsn(4, &[4, 2], &[2], 0)), true).unwrap();
    assert_eq!(
        security,
        Security {
            akm: Akm::Psk,
            group: Cipher::Ccmp128,
            management: None,
        }
    );
    assert_eq!(
        security.element(),
        [
            48, 20, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 2, 0, 0
        ]
    );
    assert_eq!(security.suites().pairwise, Cipher::Ccmp128);
    // A TKIP group stays the AP's.
    let mixed = Security::select(&ap(&rsn(2, &[4, 2], &[2], 0)), true).unwrap();
    assert_eq!(mixed.group, Cipher::Tkip);
    assert_eq!(mixed.element()[7], 2);
}

#[test]
fn a_protecting_ap_takes_psk_sha256_and_bip() {
    let security = Security::select(&ap(&rsn(4, &[4], &[2, 6], MFPC)), true).unwrap();
    assert_eq!(security.akm, Akm::PskSha256);
    assert_eq!(security.management, Some(ManagementCipher::Cmac128));
    let element = security.element();
    assert_eq!(element[19], 6);
    assert_eq!(&element[20..], &[0x80, 0]);
    // PSK alone is protected too when the AP can protect.
    let psk = Security::select(&ap(&rsn(4, &[4], &[2], MFPC | MFPR)), true).unwrap();
    assert_eq!((psk.akm, psk.management.is_some()), (Akm::Psk, true));
    // Without protection PSK-SHA256 is not the station's.
    assert_eq!(
        Security::select(&ap(&rsn(4, &[4], &[6], 0)), true),
        Err(Refusal::Akm)
    );
}

#[test]
fn a_non_default_management_cipher_follows_an_empty_pmkid_list() {
    let mut body = rsn(4, &[4], &[2], MFPC);
    body.extend_from_slice(&[0, 0, 0, 0x0f, 0xac, 12]);
    let security = Security::select(&ap(&body), true).unwrap();
    assert_eq!(security.management, Some(ManagementCipher::Gmac256));
    assert_eq!(
        &security.element()[20..],
        &[0x80, 0, 0, 0, 0, 0x0f, 0xac, 12]
    );
}

#[test]
fn absent_fields_take_the_rsn_defaults() {
    let security = Security::select(&ap(&[1, 0]), true).unwrap();
    assert_eq!((security.akm, security.group), (Akm::Psk, Cipher::Ccmp128));
}

#[test]
fn unsupported_networks_are_refused_for_their_reason() {
    for (elements, refusal) in [
        (vec![0, 1, b'x', 221, 4, 0, 0x50, 0xf2, 1], Refusal::NoRsn),
        (ap(&rsn(4, &[4], &[8], 0)), Refusal::Akm),
        (ap(&rsn(4, &[4], &[1], 0)), Refusal::Akm),
        (ap(&rsn(2, &[2], &[2], 0)), Refusal::Pairwise),
        (ap(&rsn(9, &[4], &[2], 0)), Refusal::Group),
        (ap(&rsn(4, &[4], &[2], MFPR)), Refusal::ManagementProtection),
        (
            ap(&[1, 0, 0, 0x0f, 0xac, 4, 2, 0, 0, 0x0f]),
            Refusal::Malformed,
        ),
        (ap(&[2, 0]), Refusal::Malformed),
    ] {
        assert_eq!(Security::select(&elements, true), Err(refusal));
    }
}

#[test]
fn a_passphrase_takes_sae_with_protection() {
    let offered = ap(&rsn(4, &[4], &[2, 8], MFPC));
    let security = Security::select(&offered, true).unwrap();
    assert_eq!(security.akm, Akm::Sae);
    let element = security.element();
    assert_eq!(element[19], 8);
    // MFPC, and no PMKID as sae_pmkid_in_assoc=0 leaves it out.
    assert_eq!(&element[20..], &[0x80, 0]);
    assert_eq!(element[1] as usize, element.len() - 2);
    // A raw PSK cannot run SAE: a transition network falls back to PSK.
    assert_eq!(Security::select(&offered, false).unwrap().akm, Akm::Psk);
    assert_eq!(
        Security::select(&ap(&rsn(4, &[4], &[8], MFPC)), false),
        Err(Refusal::Akm)
    );
}

#[test]
fn six_ghz_takes_sae_by_hash_to_element_alone() {
    use crate::wifi::{Band, sae::H2E_RSNXE};
    let sae = ap(&rsn(4, &[4], &[2, 8], MFPC));
    let mut h2e = sae.clone();
    h2e.extend_from_slice(&H2E_RSNXE);
    assert_eq!(
        Security::select_on(Band::Ghz6, &h2e, true).unwrap().akm,
        Akm::Sae
    );
    // Hunting and pecking, or a PSK without SAE, is refused there alone.
    assert_eq!(
        Security::select_on(Band::Ghz6, &sae, true),
        Err(Refusal::SixGhz)
    );
    assert_eq!(
        Security::select_on(Band::Ghz6, &h2e, false),
        Err(Refusal::SixGhz)
    );
    assert_eq!(
        Security::select_on(Band::Ghz5, &sae, true).unwrap().akm,
        Akm::Sae
    );
}

/// wpa_write_rsn_ie with a cached PMKSA: one PMKID after the capabilities,
/// before a non-default group management cipher.
#[test]
fn a_resumed_association_names_its_pmkid_in_the_rsn_element() {
    let sae = Security::select(&ap(&rsn(4, &[4], &[8], MFPC)), true).unwrap();
    let plain = sae.element();
    let named = sae.element_with(Some([0xab; 16]));
    assert_eq!(named[1] as usize, named.len() - 2);
    assert_eq!(named[..2], [48, plain[1] + 18]);
    assert_eq!(named[2..plain.len()], plain[2..]);
    assert_eq!(named[plain.len()..plain.len() + 2], [1, 0]);
    assert_eq!(named[plain.len() + 2..], [0xab; 16]);
    let gcmp = Security {
        management: Some(crate::wifi::security::management::ManagementCipher::Gmac256),
        ..sae
    };
    let named = gcmp.element_with(Some([0xcd; 16]));
    let end = named.len();
    assert_eq!(named[end - 22..end - 20], [1, 0]);
    assert_eq!(named[end - 20..end - 4], [0xcd; 16]);
    assert_eq!(named[end - 4..end - 1], [0, 0x0f, 0xac]);
}
