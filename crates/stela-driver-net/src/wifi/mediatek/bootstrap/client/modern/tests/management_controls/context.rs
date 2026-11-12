// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
//! MediaTek management context fixtures.
use super::*;

#[test]
fn deferred_envelope_guard_revalidation() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model, mut owner, state) = fixture();
    let session = attach(&mut driver, &mut owner);
    let first = received(&mut driver, &model, 9);
    assert!(matches!(
        driver.with_management(first, |_, _, _| panic!("unguarded modern OS effect")),
        Err(ClientError::ModernAdmissionRequired)
    ));
    let second = received(&mut driver, &model, 10);
    let third = received(&mut driver, &model, 11);
    let permit = admit(&mut owner, &driver, AssociationUse::Management);
    state.allowed.set(false);
    assert!(
        // SAFETY: The deferred envelope is owned by this current session.
        // Revoked OS policy must be rejected before invoking the callback or exposing
        // its borrowed body.
        unsafe {
            driver.with_management_modern::<_, _, ()>(&mut owner, permit, second, |_, _, _| {
                panic!("revoked OS effect")
            })
        }
        .is_err()
    );
    state.allowed.set(true);
    let permit = admit(&mut owner, &driver, AssociationUse::Management);
    driver.disconnect().unwrap();
    let before = model.response.lock().unwrap().client_commands.len();
    let log = model.bar.0.lock().unwrap().log.len();
    assert!(
        // SAFETY: The deferred envelope remains live after disconnect, while its
        // context is stale. Current guard/context checks must reject it before the
        // callback or any modeled register effect.
        unsafe {
            driver.with_management_modern::<_, _, ()>(&mut owner, permit, third, |_, _, _| {
                panic!("removed association effect")
            })
        }
        .is_err()
    );
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    assert_eq!(model.bar.0.lock().unwrap().log.len(), log);
    stop(&mut driver, &model);
    drop(driver);
    drop(session);
    drop(owner);
    drop(state);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
}

#[test]
fn reconnect_rejects_stale_context_and_keys() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model, mut old_owner, old_state) = fixture();
    let old_session = attach(&mut driver, &mut old_owner);
    let old_received = received(&mut driver, &model, 9);
    let old_context = driver.modern_context().unwrap().clone();
    driver.disconnect().unwrap();
    // SAFETY: old peer/key removal ACK, descriptor/RX/IRQ and deferred
    // model users drained; this observation can only be revalidated.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    let (join, beacon) = modern_inputs(&driver);
    let response_elements = beacon.elements.clone();
    driver
        .join_modern(
            join,
            beacon,
            profile(),
            NOW + 10,
            &Source,
            &mut Policy(true),
        )
        .unwrap();
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    driver.poll(NOW + 11, &Source, &mut Policy(true)).unwrap();
    driver.poll_released().unwrap();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    driver.poll(NOW + 12, &Source, &mut Policy(true)).unwrap();
    assert_eq!(finish_tx(&model)[0][..2], [0, 0]);
    driver.poll(NOW + 13, &Source, &mut Policy(true)).unwrap();
    driver.poll_released().unwrap();
    let mut body = vec![0x11, 0, 0, 0, 1, 0xc0];
    body.extend_from_slice(&response_elements);
    incoming(&model, &response(0x10, &body), None, false);
    driver.poll(NOW + 14, &Source, &mut Policy(true)).unwrap();
    let (mut owner, state) = authority(&driver, &model);
    assert!(!Arc::ptr_eq(driver.modern_context().unwrap(), &old_context));
    let permit = admit(&mut owner, &driver, AssociationUse::Configure);
    // SAFETY: The old peer was independently retired before reconnect. This fresh
    // context and its native memory are configured only under the new owner
    // and both current model guards.
    unsafe { driver.configure_modern(&mut owner, permit, NOW + 15) }.unwrap();
    for (pairwise, index, value) in [(true, 0, 3), (false, 2, 0x66)] {
        let permit = admit(&mut owner, &driver, AssociationUse::Management);
        // SAFETY: The old peer/key users were drained before reconnect. This fresh
        // association owns the new pairwise/group material, and both leases serialize
        // their synchronous model acknowledgements.
        unsafe {
            driver.install_key_modern(
                &mut owner,
                permit,
                pairwise,
                index,
                &[value; 16],
                ReplaySeed::ZERO,
            )
        }
        .unwrap();
    }
    let epoch = driver.station().association_epoch().unwrap();
    let permit = admit(&mut owner, &driver, AssociationUse::Management);
    // SAFETY: same drained requester and authenticated model OS exchange
    // as attach(). The AP kept its IGTK: the new association's handshake
    // gives it again with the AP's IPN, and it installs as mac80211 installs
    // it on every association.
    let session = unsafe {
        driver.install_management_key_modern(
            &mut owner,
            permit,
            epoch,
            4,
            &[0x55; 16],
            PacketNumber::new(100).unwrap(),
        )
    }
    .unwrap();
    assert_ne!(session.association_epoch(), old_session.association_epoch());
    assert_ne!(session.pairwise_key(), old_session.pairwise_key());
    assert_ne!(session.group_key(), old_session.group_key());
    let before = model.response.lock().unwrap().client_commands.len();
    let log = model.bar.0.lock().unwrap().log.len();
    let head = model.queues[&0xd4300].lock().unwrap().head;
    let permit = admit(&mut owner, &driver, AssociationUse::Management);
    assert!(
        // SAFETY: The old envelope remains owned alongside its old context, but the
        // new admission cannot rebrand it. Validation must reject it before a
        // callback or use of its borrowed body.
        unsafe {
            driver.with_management_modern::<_, _, ()>(
                &mut owner,
                permit,
                old_received,
                |_, _, _| panic!("old envelope cannot inherit current context/keys"),
            )
        }
        .is_err()
    );
    let permit = admit(&mut owner, &driver, AssociationUse::Management);
    assert!(
        // SAFETY: The old session is a live Arc but belongs to the previous
        // association. Current guards must reject it before source allocation, key
        // use or queue publication.
        unsafe {
            driver.transmit_management_modern(
                &mut owner,
                permit,
                &old_session,
                Transmit::SaQueryRequest([1, 2]),
                &GuardedSource(state.clone()),
            )
        }
        .is_err()
    );
    assert_eq!(state.allocations.get(), 0);
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    assert_eq!(model.bar.0.lock().unwrap().log.len(), log);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, head);
    let fresh = received(&mut driver, &model, 1);
    let permit = admit(&mut owner, &driver, AssociationUse::Management);
    assert_eq!(
        // SAFETY: This envelope belongs to the current session and live requester.
        // Both guards scope consumption, and the callback returns only the packet-
        // number scalar without keeping a borrowed frame.
        unsafe { driver.with_management_modern(&mut owner, permit, fresh, |_, _, pn| pn.value()) }
            .unwrap(),
        1
    );
    stop(&mut driver, &model);
    drop(driver);
    drop(session);
    drop(old_session);
    drop(owner);
    drop(state);
    drop(old_owner);
    drop(old_state);
    drop(old_context);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
}
