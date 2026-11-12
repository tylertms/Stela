//! The Wi-Fi network the mount's configuration names (`--wifi`, `--wifi-key`,
//! `--wifi-country`), for whichever Wi-Fi adapter joins it. The secret
//! never appears in `Debug` output.
use alloc::{string::String, vec::Vec};
use core::fmt;
use zeroize::Zeroizing;

/// The longest SSID IEEE 802.11 allows, in bytes.
pub const SSID_BYTES: usize = 32;

/// The network to join.
#[derive(Clone)]
pub struct Network {
    /// One to [`SSID_BYTES`] bytes.
    pub ssid: Vec<u8>,
    pub secret: Secret,
    /// An ISO 3166 alpha-2 country code in uppercase ASCII; none leaves the
    /// adapter firmware's default regulatory domain.
    pub country: Option<[u8; 2]>,
}

impl fmt::Debug for Network {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Network")
            .field("ssid", &String::from_utf8_lossy(&self.ssid))
            .field("secret", &self.secret)
            .field(
                "country",
                &self.country.map(|code| [code[0] as char, code[1] as char]),
            )
            .finish()
    }
}

/// A WPA2-PSK or WPA3-SAE credential, or none for an open network.
#[derive(Clone)]
pub enum Secret {
    /// Eight to 63 printable ASCII characters.
    Passphrase(Passphrase),
    /// The 256-bit pre-shared key, from 64 hex digits.
    Psk(Zeroizing<[u8; 32]>),
    /// No credential: the network is open, wpa_supplicant's `key_mgmt=NONE`.
    /// Only a BSS without privacy, RSN or WPA matches it.
    Open,
}

impl fmt::Debug for Secret {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Passphrase(_) => "Passphrase(..)",
            Self::Psk(_) => "Psk(..)",
            Self::Open => "Open",
        })
    }
}

/// A WPA passphrase of eight to 63 printable ASCII characters, cleared when
/// dropped.
#[derive(Clone)]
pub struct Passphrase {
    bytes: Zeroizing<[u8; 63]>,
    len: u8,
}

impl Passphrase {
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }
}

/// Why a key file holds no usable secret.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SecretError {
    /// Neither 8 to 63 characters nor 64 hex digits.
    Length,
    /// A character outside printable ASCII.
    NotAscii,
    /// 64 characters that are not all hex digits.
    BadHex,
}

impl fmt::Display for SecretError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Length => {
                "Wi-Fi key is neither an 8 to 63 character passphrase nor 64 hex digits"
            }
            Self::NotAscii => "Wi-Fi passphrase has a character outside printable ASCII",
            Self::BadHex => "Wi-Fi key of 64 characters is not all hex digits",
        })
    }
}

impl Secret {
    /// The secret a key file holds: a passphrase, or a PSK as 64 hex digits.
    /// One trailing `\n` or `\r\n` is dropped.
    pub fn parse(file: &[u8]) -> Result<Self, SecretError> {
        let text = file
            .strip_suffix(b"\r\n")
            .or_else(|| file.strip_suffix(b"\n"))
            .unwrap_or(file);
        if !text.iter().all(|byte| (0x20..0x7f).contains(byte)) {
            return Err(SecretError::NotAscii);
        }
        match text.len() {
            64 => {
                let mut key = Zeroizing::new([0u8; 32]);
                for (byte, pair) in key.iter_mut().zip(text.as_chunks::<2>().0) {
                    let digit = |c: u8| (c as char).to_digit(16).ok_or(SecretError::BadHex);
                    *byte = (digit(pair[0])? << 4 | digit(pair[1])?) as u8;
                }
                Ok(Self::Psk(key))
            }
            len @ 8..=63 => {
                let mut bytes = Zeroizing::new([0u8; 63]);
                bytes[..len].copy_from_slice(text);
                Ok(Self::Passphrase(Passphrase {
                    bytes,
                    len: len as u8,
                }))
            }
            _ => Err(SecretError::Length),
        }
    }
}

impl Secret {
    /// The pairwise master key for `ssid`: the PSK itself, or the
    /// passphrase's PBKDF2 derivation (IEEE 802.11-2020 J.4.1). An open
    /// network has none and keys nothing; its value is zero.
    pub fn pmk(&self, ssid: &[u8]) -> Zeroizing<[u8; 32]> {
        match self {
            Self::Passphrase(passphrase) => {
                super::supplicant::passphrase_psk(passphrase.as_bytes(), ssid)
            }
            Self::Psk(psk) => psk.clone(),
            Self::Open => Zeroizing::new([0; 32]),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_passphrase_derives_its_pmk_and_a_psk_is_its_own() {
        let pmk = Secret::parse(b"password").unwrap().pmk(b"IEEE");
        assert_eq!(pmk[..4], [0xf4, 0x2c, 0x6f, 0xc5]);
        let psk = Secret::parse(&b"0f".repeat(32)).unwrap();
        assert_eq!(*psk.pmk(b"any"), [0x0f; 32]);
    }

    #[test]
    fn a_passphrase_keeps_its_characters_without_the_line_end() {
        let Secret::Passphrase(phrase) = Secret::parse(b"correct horse\r\n").unwrap() else {
            panic!("passphrase expected");
        };
        assert_eq!(phrase.as_bytes(), b"correct horse");
    }

    #[test]
    fn sixty_four_hex_digits_are_a_psk() {
        let text = b"0f".repeat(32);
        let Secret::Psk(key) = Secret::parse(&text).unwrap() else {
            panic!("PSK expected");
        };
        assert_eq!(*key, [0x0f; 32]);
        let mut bad = text.clone();
        bad[5] = b'g';
        assert_eq!(Secret::parse(&bad).err(), Some(SecretError::BadHex));
    }

    #[test]
    fn short_long_and_control_character_keys_are_refused() {
        assert_eq!(Secret::parse(b"short").err(), Some(SecretError::Length));
        assert_eq!(Secret::parse(&[b'a'; 65]).err(), Some(SecretError::Length));
        assert_eq!(
            Secret::parse(b"tab\there!").err(),
            Some(SecretError::NotAscii)
        );
    }

    #[test]
    fn debug_output_hides_the_secret() {
        let config = Network {
            ssid: b"Home".to_vec(),
            secret: Secret::parse(b"hunter22").unwrap(),
            country: Some(*b"US"),
        };
        let shown = alloc::format!("{config:?}");
        assert!(shown.contains("Home") && !shown.contains("hunter22"));
    }
}
