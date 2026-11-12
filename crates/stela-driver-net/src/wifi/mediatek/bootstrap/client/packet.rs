// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Packet publication, RX interpretation and canonical key-state binding.
use super::*;

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    pub(super) fn submit(
        s: &mut Owned<I, Q, C>,
        bytes: &[u8],
        key: Option<KeyIdentity>,
        management: bool,
        minimum_rate: bool,
        source: &dyn PageSource,
    ) -> Result<Receipt, Error> {
        let prepared = Self::prepare_submission(s, bytes, key, management, minimum_rate)?;
        let result = Self::publish_submission(&mut s.controller, source, bytes, prepared);
        if matches!(result, Err(Error::GrantTimeout)) {
            // The peer and grant were already programmed. Keep the
            // complete owner even though this packet was never published.
            s.phase = Phase::Quarantined;
        }
        if s.controller.state().data_tx.as_ref().unwrap().is_poisoned() {
            s.phase = Phase::Quarantined;
        }
        result
    }
    pub(super) fn prepare_submission(
        s: &Owned<I, Q, C>,
        bytes: &[u8],
        key: Option<KeyIdentity>,
        management: bool,
        minimum_rate: bool,
    ) -> Result<(txwi::Config, PacketOwner<C>, Option<u64>), Error> {
        Self::validate_pmf_data(s)?;
        let p = s.profile.ok_or(Error::WrongPhase)?;
        if s.format_generation != Some(s.peer_generation) {
            return Err(Error::WrongPhase);
        }
        let h = Header::parse(bytes).map_err(Error::Frame)?;
        if h.fragmented() || h.amsdu || h.tid.is_some_and(|tid| tid >= 8) {
            return Err(Error::Unsupported);
        }
        if h.tid.is_some() && (s.phase != Phase::Associated || s.modern.is_none()) {
            return Err(Error::WrongPhase);
        }
        if let Some(m) = s.modern.as_ref() {
            s.station
                .validate_association_context(m.context())
                .map_err(Error::Station)?;
        }
        let epoch = s.station.association_epoch().ok_or(Error::WrongPhase)?;
        // mac80211 asks status for auth and (re)association requests and
        // supplicant control-port frames; mt7925 samples the rest.
        let subtype = ((h.control >> 4) & 15) as u8;
        let request = if management {
            matches!(subtype, 0 | 2 | 11)
        } else {
            minimum_rate
        };
        let now = s.controller.state().control.clock.now_us();
        let pid = s.txs.pid(h.receiver.is_multicast(), request, now);
        let index = if p.channel.band == wifi::Band::Ghz2 {
            14
        } else {
            18
        } + p.basic_rates.trailing_zeros() as u8;
        let config = txwi::Config {
            chip: s.controller.state().identity.chip,
            wcid: p.wcid,
            vif: Some(txwi::Vif {
                omac: p.interface.omac_index,
                wmm: p.interface.wmm_index,
                band: p.interface.band_index,
                // A station MLD's data leaves its MLD addresses to the
                // device's address translation.
                mld: s
                    .modern
                    .as_ref()
                    .is_some_and(|m| m.context().affiliation().is_some()),
            }),
            phy_band_one: false,
            queue: if management {
                txwi::Queue::Management
            } else {
                txwi::Queue::Data {
                    access_category: h.tid.map_or(if minimum_rate { 0 } else { 2 }, |tid| {
                        [2, 3, 3, 2, 1, 1, 0, 0][usize::from(tid)]
                    }),
                }
            },
            pid,
            frame: txwi::Frame {
                bytes: bytes.len(),
                header_bytes: h.length as u8,
                kind: if management {
                    txwi::Kind::Management
                } else {
                    txwi::Kind::Data
                },
                subtype,
                tid: h.tid.unwrap_or(0),
                multicast: h.receiver.is_multicast(),
                fragment: txwi::Fragment::None,
                sequence: txwi::Sequence::Hardware,
                minimum_rate,
            },
            protect: key.is_some(),
            no_ack: h.receiver.is_multicast(),
            // wcid->amsdu: the peer record's STA_REC_HW_AMSDU set it
            // (mt7925_mcu_sta_amsdu_tlv), and mt7925_mcu_uni_tx_ba clears it.
            hardware_amsdu: !s.amsdu_off
                && s.modern
                    .as_ref()
                    .is_some_and(|m| m.context().negotiated().max_tx_amsdu != 0),
            fixed_rate: if s.modern.is_some() && !management && !minimum_rate {
                None
            } else if s.controller.state().identity.chip == Chip::Mt7925 {
                Some(txwi::FixedRate::Connac3(index))
            } else {
                Some(txwi::FixedRate::Connac2(connac2_fixed_rate(
                    p.channel.band,
                    p.basic_rates,
                )))
            },
        };
        let context = PacketOwner {
            resource: s.controller.state().resource.clone(),
            association: epoch,
            key,
            context: s.modern.as_ref().map(|m| m.context().clone()),
            tid: h.tid,
            control_port: minimum_rate,
            pid,
            tx_ba: None,
            authentication: s.station.external_authentication().cloned(),
            management: if s.station.requires_protected_management() {
                s.management.clone()
            } else {
                None
            },
        };
        let grant = if management && s.phase == Phase::Joining {
            let roc = s.roc_deadline.ok_or(Error::GrantTimeout)?;
            Some(
                s.authentication_deadline
                    .map_or(roc, |deadline| roc.min(deadline)),
            )
        } else {
            None
        };
        Ok((config, context, grant))
    }
    pub(super) fn publish_submission(
        controller: &mut Controller<I, Q, C>,
        source: &dyn PageSource,
        bytes: &[u8],
        (config, context, grant): (txwi::Config, PacketOwner<C>, Option<u64>),
    ) -> Result<Receipt, Error> {
        let bytes = match (&config.frame.kind, context.context.as_deref()) {
            (txwi::Kind::Management, Some(association)) => Self::on_setup_link(bytes, association),
            _ => alloc::borrow::Cow::Borrowed(bytes),
        };
        let controller = controller.state_mut();
        let clock = controller.control.clock;
        controller
            .data_tx
            .as_mut()
            .unwrap()
            .submit_if(source, config, &bytes, context, || {
                grant.is_none_or(|deadline| clock.now_us() < deadline)
            })
            .map_err(|e| {
                if e == data::Error::PublicationRefused {
                    Error::GrantTimeout
                } else {
                    Error::Ring(e)
                }
            })
    }
    pub fn transmit(
        &mut self,
        epoch: u64,
        ethernet: &[u8],
        class: TxClass,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<Receipt, Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        if s.modern.is_some() {
            return Err(Error::ModernAdmissionRequired);
        }
        let ptk = s.ptk.as_mut().ok_or(Error::NoKey)?;
        let gtk = s.gtk.as_mut().ok_or(Error::NoKey)?;
        let mut keys = [ptk.slot_mut(), gtk.slot_mut()];
        let plan = s
            .station
            .prepare_msdu_refs(
                epoch,
                ethernet,
                class,
                SequenceOwner::Device,
                &mut keys,
                policy,
            )
            .map_err(Error::Data)?;
        s.station
            .validate_tx_plan_refs(&plan, &keys, policy)
            .map_err(Error::Data)?;
        let key = match plan.protection() {
            Protection::None => None,
            Protection::Firmware { key } => Some(key),
            Protection::HostIv { .. } => return Err(Error::Unsupported),
        };
        Self::submit(
            s,
            plan.plaintext_mpdu(),
            key,
            false,
            plan.control_port(),
            source,
        )
    }
    /// Native modern single-MSDU publication. Optional QoS TID uses the
    /// current WMM context and source queue mapping; firmware owns sequence,
    /// IV/PN/cipher and its checked rate set. Receipt is not an air ACK.
    ///
    /// # Safety
    /// The same-controller/permission retention and revocation contract of
    /// `configure_modern` applies. Both real canonical guards stay held through
    /// DMA preparation and producer publication; the original key,
    /// context and radio-policy owners survive all hardware/deferred users.
    pub unsafe fn transmit_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        epoch: u64,
        msdu: Msdu<'_>,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<Receipt, Error> {
        let Msdu {
            ethernet,
            class,
            tid,
        } = msdu;
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        if tid.is_some_and(|tid| Self::tx_ba_blocks_data(s, tid)) {
            return Err(Error::Busy);
        }
        let context = s
            .modern
            .as_ref()
            .ok_or(Error::WrongPhase)?
            .context()
            .clone();
        let ptk = s.ptk.as_mut().ok_or(Error::NoKey)?;
        let gtk = s.gtk.as_mut().ok_or(Error::NoKey)?;
        let mut keys = [ptk.slot_mut(), gtk.slot_mut()];
        let plan = if let Some(tid) = tid {
            s.station.prepare_qos_msdu_refs(
                epoch,
                ethernet,
                class,
                station::data::Qos {
                    context: &context,
                    tid,
                },
                &mut keys,
                policy,
            )
        } else {
            s.station.prepare_msdu_refs(
                epoch,
                ethernet,
                class,
                SequenceOwner::Device,
                &mut keys,
                policy,
            )
        }
        .map_err(Error::Data)?;
        s.station
            .validate_tx_plan_refs(&plan, &keys, policy)
            .map_err(Error::Data)?;
        let key = match plan.protection() {
            Protection::None => None,
            Protection::Firmware { key } => Some(key),
            Protection::HostIv { .. } => return Err(Error::Unsupported),
        };
        let prepared =
            Self::prepare_submission(s, plan.plaintext_mpdu(), key, false, plan.control_port())?;
        // SAFETY: covered by this function's own safety contract.
        let outcome = unsafe {
            authority.publish_association(
                admission,
                &s.station,
                if s.station.requires_protected_management() && !matches!(class, TxClass::Data) {
                    wifi::regulatory::AssociationUse::Authentication
                } else {
                    wifi::regulatory::AssociationUse::Data
                },
                |_, _| {
                    Self::publish_submission(
                        &mut s.controller,
                        source,
                        plan.plaintext_mpdu(),
                        prepared,
                    )
                },
            )
        }
        .map_err(Error::Regulatory)?;
        if s.controller.state().data_tx.as_ref().unwrap().is_poisoned() {
            s.phase = Phase::Quarantined;
        }
        outcome
    }
    /// Observe source queue progress and free packets only after both
    /// descriptor consumption and matching exact-once hardware TX-free. The
    /// returned records do not synthesize an air ACK from DMA release.
    pub fn poll_released(&mut self) -> Result<Vec<Released<PacketOwner<C>>>, Error> {
        let s = self.state_mut();
        let now = s.controller.state().control.clock.now_us();
        let q = s.controller.state_mut().data_tx.as_mut().unwrap();
        while q.poll_consumed().map_err(Error::Ring)?.is_some() {}
        let mut out = Vec::new();
        while let Some(r) = q.take_released().map_err(Error::Ring)? {
            out.push(r)
        }
        s.txs.released(&out, now);
        if s.phase == Phase::Removing && q.outstanding_tokens() == 0 && q.pending_descriptors() == 0
        {
            Self::remove(s, s.remove_ticket.ok_or(Error::WrongPhase)?)?;
        }
        Ok(out)
    }
    pub(super) fn normalized(
        s: &mut Owned<I, Q, C>,
        raw: &[u8],
        grouped: bool,
        policy: &mut impl scan::Policy,
    ) -> Result<Mpdu, Error> {
        let d = rx::decode(s.controller.state().identity.chip, raw).map_err(Error::Rx)?;
        let band = match d.channel.band {
            rx::Band::Ghz2 => wifi::Band::Ghz2,
            rx::Band::Ghz5 => wifi::Band::Ghz5,
            // Outside a scan, only the bound 6 GHz transport receives there;
            // its poll checks each frame under both held providers.
            rx::Band::Ghz6 if s.phase == Phase::Scanning || s.six_ghz.is_some() => wifi::Band::Ghz6,
            rx::Band::Ghz6 => return Err(Error::Unsupported),
        };
        let channel = policy
            .lookup_channel(band, d.channel.number)
            .ok_or(Error::WrongProfile)?;
        if !policy.allow_receive(channel) {
            return Err(Error::WrongProfile);
        }
        let header = Header::parse(d.payload).map_err(Error::Frame)?;
        if (d.fragment && !d.rejoinable_fragment()) || (!grouped && (d.ampdu || d.amsdu.is_some()))
        {
            return Err(Error::Unsupported);
        }
        if header.data_present() {
            // Each link's AP has a WCID of its own.
            let (p, ..) = Self::link_of(s, d.wcid)?;
            // mt792x_rx_get_wcid first resolves raw peer WCID, then
            // maps a non-U2M frame through that peer's VIF to its BCMC slot.
            // A copied raw BCMC number is not a validated peer lookup.
            if d.wcid != p.wcid
                || d.unicast_to_me == header.receiver.is_multicast()
                || s.format_generation != Some(s.peer_generation)
            {
                return Err(Error::WrongProfile);
            }
        }
        let mut security = SecurityStatus::default();
        if header.protected() {
            if !matches!(s.phase, Phase::Associated)
                || s.format_generation != Some(s.peer_generation)
            {
                return Err(Error::WrongPhase);
            }
            let (p, ptk, gtk) = Self::link_of(s, d.wcid)?;
            if channel != p.channel {
                return Err(Error::WrongProfile);
            }
            let owner = if header.receiver.is_multicast() {
                gtk
            } else {
                ptk
            }
            .ok_or(Error::NoKey)?;
            let key = owner.slot().installed().ok_or(Error::NoKey)?;
            // Trust comes from completed DMA + exact WCID/key
            // context. Raw security bits alone cannot manufacture generation.
            let effective_wcid = p
                .receive_slot(d.wcid, d.unicast_to_me)
                .ok_or(Error::WrongProfile)?;
            // mt7925_mac_fill_rx takes MT_CIPHER_TKIP and its NO_MIC form alike.
            if effective_wcid != key.slot()
                || d.security.key_index != key.index()
                || !match key.cipher() {
                    Cipher::Wep40 => d.security.mode == 1,
                    Cipher::Wep104 => d.security.mode == 5,
                    Cipher::Tkip => matches!(d.security.mode, 2 | 3),
                    Cipher::Ccmp128 => d.security.mode == 4,
                    Cipher::Ccmp256 => d.security.mode == 10,
                    Cipher::Gcmp128 => d.security.mode == 11,
                    Cipher::Gcmp256 => d.security.mode == 12,
                }
                || d.security.cipher_mismatch
                || d.security.cipher_length_mismatch
                || !d.security.decrypted
            {
                return Err(Error::WrongProfile);
            }
            let pn = match d.security.packet_number_be {
                Some(pn) => {
                    let value = pn
                        .into_iter()
                        .fold(0u64, |value, byte| (value << 8) | u64::from(byte));
                    PacketNumber::new(value)
                }
                None if grouped && d.amsdu.is_some_and(|a| !a.first) => None,
                // mt7925_mac_fill_rx reports no IV for WEP.
                None if key.cipher().is_wep() => None,
                None => return Err(Error::WrongProfile),
            };
            security = SecurityStatus {
                key: Some(key),
                pn,
                decrypted: true,
                integrity: if d.security.icv_error || d.security.tkip_mic_error {
                    Integrity::Failed
                } else {
                    Integrity::Verified
                },
                iv_stripped: d.security.iv_stripped && !d.fragment,
                mic_stripped: d.security.mic_stripped,
                icv_error: d.security.icv_error,
                tkip_mic_error: d.security.tkip_mic_error,
                ..SecurityStatus::default()
            };
        }
        let bytes = if d.fragment {
            d.normalize_fragment(header.length)
        } else {
            d.normalize(header.length)
        };
        let mut frame = Mpdu::new(
            bytes.map_err(Error::Rx)?,
            wifi::RadioStatus {
                band,
                channel: channel.number,
                frequency_mhz: channel.frequency_mhz,
                signal_dbm: d
                    .signal
                    .and_then(|v| v.dbm.into_iter().filter(|v| *v < 0).max()),
                chain_signal_dbm: d.signal.map_or([None; 4], |v| {
                    v.dbm.map(|x| if x < 0 { Some(x) } else { None })
                }),
                rate: wifi::RawRate {
                    layout: 0x7925,
                    words: d.rxv.unwrap_or([0; 4]),
                },
                rx_rate: d.rate,
                tsf: d.timestamp_low.map(wifi::Timestamp::Low32),
            },
            security,
        )
        .map_err(Error::Frame)?;
        frame.fcs_error = d.fcs_error;
        frame.monitor_only = d.only_monitor;
        frame.raw_status = d.base[..4].try_into().unwrap();
        if grouped {
            frame.aggregation.ampdu = if d.ampdu { d.timestamp_low } else { None };
            frame.aggregation.amsdu = d.amsdu.map(|a| wifi::Amsdu {
                index: 0,
                first: a.first,
                last: a.last,
            });
        }
        Ok(frame)
    }
}

/// `mt76_connac2_mac_tx_rate_val` on MT7921 and MT7922: the lowest basic
/// rate, through `mt76_calculate_default_rate`'s `mt76_rates` value, as the
/// TXD's fixed rate word (index, then the CCK or OFDM mode from bit 6).
fn connac2_fixed_rate(band: wifi::Band, basic_rates: u16) -> u16 {
    let offset = if band == wifi::Band::Ghz2 { 0 } else { 4 };
    let lowest = if basic_rates == 0 {
        0
    } else {
        basic_rates.trailing_zeros() as usize
    };
    let index = match lowest + offset {
        index if index < crate::wifi::mediatek::rx::LEGACY_RATES.len() => index,
        _ => offset,
    };
    let mode = u16::from(index >= 4);
    u16::from(crate::wifi::mediatek::rx::LEGACY_RATES[index]) | mode << 6
}

#[cfg(test)]
mod connac2_rate_tests {
    use super::*;

    #[test]
    fn the_lowest_basic_rate_is_the_fixed_rate_word() {
        // 1 Mbps CCK; 6 Mbps OFDM (hardware index 11); 24 Mbps (9).
        assert_eq!(connac2_fixed_rate(wifi::Band::Ghz2, 0b1111), 0);
        assert_eq!(connac2_fixed_rate(wifi::Band::Ghz5, 0x15), 11 | 1 << 6);
        assert_eq!(connac2_fixed_rate(wifi::Band::Ghz6, 0x10), 9 | 1 << 6);
        // 6 Mbps when the only basic rate is OFDM on 2.4 GHz.
        assert_eq!(connac2_fixed_rate(wifi::Band::Ghz2, 0x10), 11 | 1 << 6);
    }
}
