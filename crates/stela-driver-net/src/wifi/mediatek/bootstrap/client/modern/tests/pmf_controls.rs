// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
//! Firmware/DMA peers with a simulated OS PMF service.
//! These controls supply no physical-radio or production OS validation.
use super::*;
use crate::wifi::mediatek::bootstrap::client::management::{Session, Transmit};
use crate::wifi::security::PacketNumber;

type Native = Client<Bar, Q, NativeGuard>;
struct Protection {
    selected: Cell<bool>,
    authenticated: Cell<bool>,
    port: Cell<bool>,
    uses: RefCell<Vec<AssociationUse>>,
}
struct PmfState {
    hardware: Cell<bool>,
    os: Cell<bool>,
    revision: Cell<u64>,
    auth_hardware: Cell<bool>,
    native: reg::NativeContext,
    context: RefCell<Option<Arc<station::AssociationContext>>>,
    observation: Arc<station::NativeBssObservation>,
    selected: Join,
    response: Arc<Mutex<Response>>,
    _memory: Arc<Mutex<Memory>>,
    tickets: RefCell<Vec<station::Ticket>>,
    windows: RefCell<Vec<(usize, usize)>>,
}
struct ProtectedHardware(Rc<PmfState>);
struct ProtectedOs {
    state: Rc<PmfState>,
    protection: Rc<Protection>,
}
struct ProtectedHardwareGuard {
    state: Rc<PmfState>,
    start: usize,
}
struct ProtectedGuard {
    state: Rc<PmfState>,
    protection: Rc<Protection>,
}
impl Drop for ProtectedHardwareGuard {
    fn drop(&mut self) {
        assert!(!self.state.os.get());
        let end = self.state.response.lock().unwrap().client_commands.len();
        self.state.windows.borrow_mut().push((self.start, end));
        self.state.hardware.set(false);
    }
}
impl Drop for ProtectedGuard {
    fn drop(&mut self) {
        assert!(self.state.hardware.get());
        self.state.os.set(false);
    }
}
fn authentication(state: &PmfState, operation: &reg::PmfAuthenticationOperation<'_>) -> bool {
    assert!(state.hardware.get() && state.os.get());
    assert!(Arc::ptr_eq(operation.observation(), &state.observation));
    let join = operation.join();
    state.tickets.borrow_mut().push(operation.ticket());
    if let Some(bytes) = operation.transmit() {
        assert_eq!(&bytes[4..10], &AP.0);
        assert!(matches!(
            u16::from_le_bytes(bytes[..2].try_into().unwrap()),
            0 | 0xb0
        ));
        if bytes[0] == 0 {
            assert!(
                bytes
                    .windows(selected_rsn().len())
                    .any(|v| v == selected_rsn())
            );
        }
    }
    join.bssid == state.selected.bssid
        && join.channel == state.selected.channel
        && join.policy == state.selected.policy
        && join.request.elements() == state.selected.request.elements()
        && join.request.capabilities() == state.selected.request.capabilities()
        && join.request.listen_interval() == state.selected.request.listen_interval()
}
fn current(state: &PmfState, operation: &reg::AssociationOperation<'_>) -> bool {
    assert!(state.hardware.get() && state.os.get());
    let context = state.context.borrow();
    context
        .as_ref()
        .is_some_and(|context| core::ptr::eq(operation.context(), context.as_ref()))
        && operation
            .context()
            .pmf_observation()
            .is_some_and(|source| Arc::ptr_eq(source, &state.observation))
}
impl reg::HardwareLease for ProtectedHardwareGuard {
    fn context(&self) -> reg::NativeContext {
        self.state.native
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<wifi::Channel> {
        (band == CHANNEL.band && number == CHANNEL.number).then_some(CHANNEL)
    }
    fn allows(&mut self, _: reg::Operation) -> bool {
        false
    }
    fn allows_pmf_authentication(
        &mut self,
        operation: &reg::PmfAuthenticationOperation<'_>,
    ) -> bool {
        authentication(&self.state, operation) && self.state.auth_hardware.get()
    }
    fn allows_association(&mut self, operation: &reg::AssociationOperation<'_>) -> bool {
        let response = self.state.response.lock().unwrap();
        current(&self.state, operation)
            && response.regulatory_domain
            && !response.regulatory_power.is_empty()
            && operation
                .context()
                .negotiated()
                .local()
                .local_he_source_maps()
                .is_some()
    }
}
impl reg::OsLease for ProtectedGuard {
    fn revision(&self) -> u64 {
        self.state.revision.get()
    }
    fn allows(&mut self, _: reg::Operation) -> bool {
        false
    }
    fn allows_pmf_authentication(
        &mut self,
        operation: &reg::PmfAuthenticationOperation<'_>,
    ) -> bool {
        authentication(&self.state, operation) && self.protection.selected.get()
    }
    fn allows_association(&mut self, operation: &reg::AssociationOperation<'_>) -> bool {
        current(&self.state, operation)
    }
    fn allows_protected_management(&mut self, operation: &reg::AssociationOperation<'_>) -> bool {
        assert!(current(&self.state, operation));
        assert_eq!(operation.context().pmf().unwrap().rsn(), selected_rsn());
        self.protection.uses.borrow_mut().push(operation.purpose());
        self.protection.selected.get()
            && match operation.purpose() {
                AssociationUse::Configure
                | AssociationUse::Receive
                | AssociationUse::Authentication => true,
                AssociationUse::Management
                | AssociationUse::Data
                | AssociationUse::ChannelSwitch => self.protection.authenticated.get(),
            }
    }
    fn allows_protected_port(&mut self, operation: &reg::AssociationOperation<'_>) -> bool {
        assert!(current(&self.state, operation));
        assert_eq!(operation.purpose(), AssociationUse::Management);
        self.protection.authenticated.get() && self.protection.port.get()
    }
}
// SAFETY: single-threaded model requester and kept native resources;
// revocation obeys the hardware→OS guard order. This grants no physical access.
unsafe impl reg::HardwarePolicy for ProtectedHardware {
    type Lease<'a> = ProtectedHardwareGuard;
    fn acquire(&mut self) -> Option<Self::Lease<'_>> {
        assert!(!self.0.os.get());
        if self.0.hardware.replace(true) {
            return None;
        }
        let start = self.0.response.lock().unwrap().client_commands.len();
        Some(ProtectedHardwareGuard {
            state: self.0.clone(),
            start,
        })
    }
}
// SAFETY: a single modeled OS owns this original native observation,
// selected request and subsequent real current context through both phases.
unsafe impl reg::OsPolicy for ProtectedOs {
    type Lease<'a> = ProtectedGuard;
    fn acquire(&mut self, native: reg::NativeContext) -> Option<Self::Lease<'_>> {
        assert!(self.state.hardware.get());
        if native != self.state.native || self.state.os.replace(true) {
            return None;
        }
        Some(ProtectedGuard {
            state: self.state.clone(),
            protection: self.protection.clone(),
        })
    }
}
type ProtectedOwner = reg::Owner<ProtectedHardware, ProtectedOs>;
fn authority_pmf(
    observed: Arc<station::NativeBssObservation>,
    join: &Join,
    model: &Model,
) -> (ProtectedOwner, Rc<PmfState>, Rc<Protection>) {
    let state = Rc::new(PmfState {
        hardware: Cell::new(false),
        os: Cell::new(false),
        revision: Cell::new(1),
        auth_hardware: Cell::new(true),
        native: reg::NativeContext {
            device: Arc::as_ptr(&model.memory) as usize as u64,
            reset_epoch: 1,
            firmware_epoch: 1,
            policy_revision: 1,
        },
        context: RefCell::new(None),
        observation: observed,
        selected: join.clone(),
        response: model.response.clone(),
        _memory: model.memory.clone(),
        tickets: RefCell::new(Vec::new()),
        windows: RefCell::new(Vec::new()),
    });
    let protection = Rc::new(Protection {
        selected: Cell::new(true),
        authenticated: Cell::new(false),
        port: Cell::new(false),
        uses: RefCell::new(Vec::new()),
    });
    let owner = reg::Owner::new(
        ProtectedHardware(state.clone()),
        ProtectedOs {
            state: state.clone(),
            protection: protection.clone(),
        },
    )
    .unwrap();
    (owner, state, protection)
}
struct PmfSource(Rc<PmfState>);
impl crate::bus::dma::memory::PageSource for PmfSource {
    fn allocate(
        &self,
        bytes: usize,
        alignment: usize,
        attribute: crate::bus::dma::memory::CacheAttribute,
    ) -> Result<crate::bus::dma::memory::PinnedPages, crate::bus::dma::memory::MemoryError> {
        assert!(self.0.hardware.get() && self.0.os.get());
        crate::bus::dma::memory::PageSource::allocate(&Source, bytes, alignment, attribute)
    }
}
fn permit(
    owner: &mut ProtectedOwner,
    driver: &Native,
    purpose: AssociationUse,
) -> reg::AssociationAdmission {
    owner
        .admit_association(
            driver.station(),
            driver.modern_context().unwrap().clone(),
            purpose,
        )
        .unwrap()
}
fn selected_rsn() -> Vec<u8> {
    rsn(2)
}
fn rsn(akm: u8) -> Vec<u8> {
    vec![
        1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, akm, 0xc0, 0, 0, 0, 0, 15, 172,
        6,
    ]
}
fn replace_rsn(elements: &[u8], rsn: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut rest = elements;
    while !rest.is_empty() {
        let len = usize::from(rest[1]);
        if rest[0] == 48 {
            append_ie(&mut out, 48, rsn);
        } else {
            out.extend_from_slice(&rest[..2 + len]);
        }
        rest = &rest[2 + len..];
    }
    out
}
pub(in crate::wifi::mediatek::bootstrap::client) fn inputs(
    driver: &Native,
) -> (Join, scan::result::Bss) {
    inputs_akm(driver, 2)
}
/// `inputs` for another AKM, such as SAE's 8.
pub(in crate::wifi::mediatek::bootstrap::client) fn inputs_akm(
    driver: &Native,
    akm: u8,
) -> (Join, scan::result::Bss) {
    inputs_akm_cipher(driver, akm, 6)
}

pub(in crate::wifi::mediatek::bootstrap::client) fn inputs_akm_cipher(
    driver: &Native,
    akm: u8,
    management: u8,
) -> (Join, scan::result::Bss) {
    let (mut join, mut bss) = modern_inputs(driver);
    let local = driver.local_advertisement(Band::Ghz5).unwrap();
    let mut selected = rsn(akm);
    *selected.last_mut().unwrap() = management;
    join.request = AssociationRequest::new_modern_with_local_pmf(
        0x11,
        10,
        replace_rsn(join.request.elements(), &selected),
        local.capabilities().clone(),
        if local.capabilities().eht().is_some() {
            Mode::Eht
        } else {
            Mode::He
        },
        Width::Mhz80,
    )
    .unwrap();
    bss.elements = replace_rsn(&bss.elements, &selected);
    (join, bss)
}
fn observe(
    driver: &mut Native,
    model: &Model,
    bss: &scan::result::Bss,
) -> Arc<station::NativeBssObservation> {
    observe_at(driver, model, bss, 0)
}
fn observe_at(
    driver: &mut Native,
    model: &Model,
    bss: &scan::result::Bss,
    start: u64,
) -> Arc<station::NativeBssObservation> {
    driver
        .begin_scan(
            scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
            start,
            1_000_000,
            &mut Policy(true),
        )
        .unwrap();
    let mut body = bss.timestamp.to_le_bytes().to_vec();
    body.extend_from_slice(&bss.beacon_interval.to_le_bytes());
    body.extend_from_slice(&bss.capabilities.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].fill(255);
    incoming(model, &beacon, None, false);
    let events = driver.poll(start + 1, &Source, &mut Policy(true)).unwrap();
    assert!(matches!(events.as_slice(), [Event::Bss(_)]));
    let observed = driver.pmf_bss_observations().next().unwrap().clone();
    let mut done = vec![0; 52];
    done[..4].copy_from_slice(&(7u32 << 27).to_le_bytes());
    done[36] = 14;
    done[38] = 6;
    done[50] = 4;
    device_receive(&model.queues[&0xd4500], &done, true, 0);
    driver.poll(start + 2, &Source, &mut Policy(true)).unwrap();
    // SAFETY: modeled scan terminal event and all model RX/IRQ users drained.
    unsafe { driver.retire_scan_after_drain() }.unwrap();
    observed
}
fn preparing() -> (
    Native,
    Model,
    Join,
    Arc<station::NativeBssObservation>,
    ProtectedOwner,
    Rc<PmfState>,
    Rc<Protection>,
) {
    let (mut driver, model) = setup();
    let (join, bss) = inputs(&driver);
    let observed = observe(&mut driver, &model, &bss);
    let (owner, state, protection) = authority_pmf(observed.clone(), &join, &model);
    (driver, model, join, observed, owner, state, protection)
}

type Awaiting = (
    Native,
    Model,
    usize,
    ProtectedOwner,
    Rc<PmfState>,
    Rc<Protection>,
);
fn awaiting_pmf() -> Awaiting {
    awaiting_with(|_, _, _| {}).unwrap()
}
fn awaiting_with(
    alter: impl FnOnce(&mut Join, &mut scan::result::Bss, &mut Vec<u8>),
) -> Result<Awaiting, ClientError> {
    let (mut driver, model) = setup();
    let (mut join, mut bss) = inputs(&driver);
    let mut response_elements = bss.elements.clone();
    alter(&mut join, &mut bss, &mut response_elements);
    let observed = observe(&mut driver, &model, &bss);
    let (mut owner, state, protection) = authority_pmf(observed.clone(), &join, &model);
    // SAFETY: original native scan and this same modeled OS selection.
    unsafe {
        driver.join_modern_pmf(
            &mut owner,
            PmfJoin {
                join,
                observed: &observed,
            },
            profile(),
            100,
            &PmfSource(state.clone()),
            &mut Policy(true),
        )
    }?;
    finish_tx(&model);
    // SAFETY: The original scan observation, selected request and native-memory owner
    // stay held. This synchronous poll reacquires the same hardware-then-OS
    // guards after the modeled authentication TX completes.
    unsafe {
        driver.poll_pmf_authentication(
            &mut owner,
            101,
            &PmfSource(state.clone()),
            &mut Policy(true),
        )
    }
    .unwrap();
    driver.poll_released().unwrap();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    // SAFETY: The injected authentication response belongs to the live fixture RX
    // queue and original scan owner. Both model guards serialize its handling; no
    // physical radio or concurrent reset user exists.
    unsafe {
        driver.poll_pmf_authentication(
            &mut owner,
            102,
            &PmfSource(state.clone()),
            &mut Policy(true),
        )
    }
    .unwrap();
    let assoc = finish_tx(&model);
    assert!(
        assoc[0]
            .windows(selected_rsn().len())
            .any(|v| v == selected_rsn())
    );
    // SAFETY: The original owner and backing remain held after modeled
    // association TX completion. This synchronous poll checks the current
    // authentication ticket under both guards before releasing resources.
    unsafe {
        driver.poll_pmf_authentication(
            &mut owner,
            103,
            &PmfSource(state.clone()),
            &mut Policy(true),
        )
    }
    .unwrap();
    driver.poll_released().unwrap();
    let before = model.response.lock().unwrap().client_commands.len();
    let mut body = vec![0x11, 0, 0, 0, 1, 0xc0];
    body.extend_from_slice(&response_elements);
    incoming(&model, &response(0x10, &body), None, false);
    // SAFETY: The same scan owner and native backing remain live. Response elements
    // may be invalid; their validation occurs under current model guards
    // before any new association permission escapes.
    unsafe {
        driver.poll_pmf_authentication(
            &mut owner,
            104,
            &PmfSource(state.clone()),
            &mut Policy(true),
        )
    }?;
    let context = driver.modern_context().unwrap().clone();
    assert!(Arc::ptr_eq(context.pmf_observation().unwrap(), &observed));
    *state.context.borrow_mut() = Some(context);
    Ok((driver, model, before, owner, state, protection))
}

fn poll(
    driver: &mut Native,
    owner: &mut ProtectedOwner,
    state: &Rc<PmfState>,
    now: u64,
) -> Result<Vec<Event>, ClientError> {
    let admission = permit(owner, driver, AssociationUse::Receive);
    // SAFETY: modeled owner, original context and both real model guards.
    unsafe {
        driver.poll_pmf(
            owner,
            admission,
            now,
            &PmfSource(state.clone()),
            &mut Policy(true),
        )
    }
}
fn configured() -> (Native, Model, ProtectedOwner, Rc<PmfState>, Rc<Protection>) {
    let (mut driver, model, before, mut owner, state, protection) = awaiting_pmf();
    // wpa_supplicant registers for its SA Query responses.
    driver.register_action(&[8, 1]).unwrap();
    let admission = permit(&mut owner, &driver, AssociationUse::Configure);
    // SAFETY: original modeled requester and explicitly selected OS policy.
    unsafe { driver.configure_modern(&mut owner, admission, 105) }.unwrap();
    assert_eq!(driver.completed_modern_requests(), 7);
    assert_eq!(
        model.response.lock().unwrap().client_commands.len(),
        before + 7
    );
    assert!(!protection.authenticated.get());
    (driver, model, owner, state, protection)
}
fn attach(driver: &mut Native, owner: &mut ProtectedOwner) -> Arc<Session> {
    for (pairwise, index, material) in [(true, 0, 1), (false, 2, 0x44)] {
        let admission = permit(owner, driver, AssociationUse::Management);
        // SAFETY: this model OS supplied its authenticated key exchange.
        unsafe {
            driver.install_key_modern(
                owner,
                admission,
                pairwise,
                index,
                &[material; 16],
                ReplaySeed::ZERO,
            )
        }
        .unwrap();
    }
    let admission = permit(owner, driver, AssociationUse::Management);
    let epoch = driver.station().association_epoch().unwrap();
    // SAFETY: model queues and external deferred/IRQ/key users are drained.
    unsafe {
        driver.install_management_key_modern(
            owner,
            admission,
            epoch,
            4,
            &[0x55; 16],
            PacketNumber::new(100).unwrap(),
        )
    }
    .unwrap()
}

mod admission;
mod authentication_flow;
mod key_sessions;
mod management_frames;
mod port;
