//! The machine-state partition (`--nor SERIAL:GUID`): the guest's NOR flash
//! image in its first [`CAPACITY`] bytes, so NVRAM survives reboots, then the
//! SEP's roots and anti-replay store ([`sep_state`]), then, when the
//! partition holds them, the boot log ([`boot_log`]), the board devices'
//! flash ([`board_flash`]) and the PMU upcount's anchor ([`rtc_anchor`]),
//! which the service stores every [`ANCHOR_SECONDS`](super::service) and at
//! the stop. The image is loaded
//! before the bundle, which hands the kernel its newest valid bank; every
//! sector a committed program or erase changes is stored durably by the
//! host's service, the guest not waiting, and at the stop, as is the
//! pending output.
//!
//! When `--nor` names the `--storage` partition, [`split`] keeps the same
//! layout in that partition's last bytes ([`machine_state`]) and gives the
//! ANS2 model only the rest, so the guest never sees the machine state.
use crate::output;
use alloc::{boxed::Box, collections::BTreeMap, format, vec::Vec};
use stela_contracts::refuse;
use stela_drivers::block::{BlockDevice, BlockError, Shared, Submission};
use {
    stela_formats::boot_bundle::Nvram, stela_runtime::pc::board_flash, stela_runtime::pc::boot_log,
    stela_runtime::pc::machine_state, stela_runtime::pc::rtc_anchor,
    stela_soc::machine::AppleDevices, stela_soc::machine::RtcAnchor,
    stela_soc::machine::nor::persist, stela_soc::machine::sep_state, stela_soc_data::nor::CAPACITY,
    stela_soc_data::nor::Image,
};

/// Split the Apple region `partition` into the guest's ANS2 namespace and
/// the machine state in its tail. A partition too small for both, or with
/// blocks that do not divide the alignment, refuses.
pub fn split(partition: Box<dyn BlockDevice>) -> (Box<dyn BlockDevice>, Box<dyn BlockDevice>) {
    let (namespace, state, layout) =
        machine_state::split_device(partition).unwrap_or_else(|error| {
            output::fatal(format_args!(
                "the --storage partition cannot also hold the machine state: {error}"
            ))
        });
    output::arguments(format_args!(
        "MACHINE STATE in the target's last {:#x} bytes (from {:#x}); the ANS2 namespace holds \
         {:#x}\n",
        layout.state_bytes, layout.namespace_bytes, layout.namespace_bytes
    ));
    (Box::new(namespace), Box::new(state))
}

pub struct HostNor {
    device: Box<dyn BlockDevice>,
    /// Changed sectors by offset, waiting for the store in progress; a
    /// newer copy of a sector replaces the waiting one.
    waiting: BTreeMap<usize, Vec<u8>>,
    /// The store in progress: its sectors' writes run together, and they
    /// are durable before the next store begins.
    running: Option<Submission>,
    /// The PMU upcount's anchor, when the partition holds its region.
    anchor: Option<AnchorStore>,
}

/// The RTC anchor's region ([`rtc_anchor`]): the anchor this boot started
/// from and the stores of newer ones, which never wait on the device.
struct AnchorStore {
    device: Box<dyn BlockDevice>,
    loaded: Option<RtcAnchor>,
    anchors: rtc_anchor::Anchors,
    /// The newest anchor not yet stored; it replaces an older waiting one.
    waiting: Option<RtcAnchor>,
    running: Option<Submission>,
}

impl AnchorStore {
    /// The region after the board flash, when the partition holds it and
    /// its device reads; otherwise say why there is none. Without it the
    /// upcount starts at zero each boot, as before the anchor.
    fn open(disk: &Shared, block: u64, available: u64) -> Option<Self> {
        if available < rtc_anchor::PARTITION_BYTES {
            output::arguments(format_args!(
                "RTC ANCHOR off: the machine-state partition holds {available:#x} bytes; the \
                 anchor needs {:#x}; the upcount starts at zero each boot\n",
                rtc_anchor::PARTITION_BYTES
            ));
            return None;
        }
        let opened = disk
            .window(rtc_anchor::OFFSET / block, rtc_anchor::BYTES / block)
            .map_err(|error| format!("{error}"))
            .and_then(|window| {
                let mut device: Box<dyn BlockDevice> = Box::new(window);
                let read = rtc_anchor::read(&mut *device).map_err(|error| format!("{error}"))?;
                Ok((device, read))
            });
        let (device, (loaded, anchors)) = match opened {
            Ok(opened) => opened,
            Err(error) => {
                output::arguments(format_args!(
                    "RTC ANCHOR off: {error}; the upcount starts at zero each boot\n"
                ));
                return None;
            }
        };
        match loaded {
            Some(anchor) => output::arguments(format_args!(
                "RTC ANCHOR ticks={:#x} unix={}\n",
                anchor.ticks,
                anchor.unix_seconds.unwrap_or(0)
            )),
            None => output::arguments(format_args!(
                "RTC ANCHOR none: the upcount starts at zero\n"
            )),
        }
        Some(Self {
            device,
            loaded,
            anchors,
            waiting: None,
            running: None,
        })
    }

    /// Start the waiting anchor's store once the one in progress is done,
    /// the caller waiting for both when `wait`. A failed store is reported
    /// and ends the anchor's stores for this run: the next boot goes on
    /// from the last stored anchor, which with a wall time loses nothing.
    fn advance(slot: &mut Option<Self>, wait: bool) {
        let Some(store) = slot.as_mut() else {
            return;
        };
        loop {
            if let Some(running) = store.running.as_mut() {
                match running.poll(&mut *store.device) {
                    None if wait => {
                        core::hint::spin_loop();
                        continue;
                    }
                    None => return,
                    Some(Err(error)) => {
                        output::arguments(format_args!(
                            "RTC ANCHOR refusal: store: {error}; no more anchors this run\n"
                        ));
                        *slot = None;
                        return;
                    }
                    Some(Ok(_)) => store.running = None,
                }
            }
            let Some(anchor) = store.waiting.take() else {
                return;
            };
            let identity = store.device.identity();
            let request = store.anchors.request(anchor, identity.block_size);
            match Submission::new(request, identity) {
                Ok(running) => store.running = Some(running),
                Err(error) => {
                    output::arguments(format_args!(
                        "RTC ANCHOR refusal: store: {error}; no more anchors this run\n"
                    ));
                    *slot = None;
                    return;
                }
            }
        }
    }
}

impl HostNor {
    /// Load the partition's image, initializing a blank one, the bank iBoot
    /// hands off, where the SEP keeps its state, and the board devices'
    /// flash. A partition too small for the first two refuses.
    pub fn open(
        device: Box<dyn BlockDevice>,
    ) -> (Self, Image, Nvram, sep_state::Location, Option<BoardFlash>) {
        let block = u64::from(device.identity().block_size);
        let available = device.identity().blocks.saturating_mul(block);
        let needed = CAPACITY as u64 + stela_formats::machine_state::SEP_BYTES;
        if block == 0 || !(CAPACITY as u64).is_multiple_of(block) || available < needed {
            output::arguments(format_args!(
                "NOR refusal: the partition holds {available:#x} bytes in {block}-byte blocks; the NOR image and SEP state need {needed:#x}\n"
            ));
            output::fatal("the machine-state partition is too small");
        }
        let disk = Shared::new(device);
        let first = CAPACITY as u64 / block;
        let mut device: Box<dyn BlockDevice> = Box::new(
            disk.window(0, first)
                .expect("the partition holds the NOR image"),
        );
        let (image, fresh) = persist::load(&mut *device).unwrap_or_else(|error| {
            output::fatal(format_args!(
                "the NOR partition could not be loaded: {error}"
            ))
        });
        let (bank, proxy) = image
            .boot_bank()
            .expect("persist::load returns an identifiable NOR image");
        if image.current_bank().is_err() {
            output::arguments(format_args!(
                "NVRAM recovery: no valid bank; using empty boot variables; flash preserved\n"
            ));
        }
        output::arguments(format_args!("NOR persistent fresh={fresh} bank={bank}\n"));
        open_log(&disk, block, available);
        let board_flash = BoardFlash::open(&disk, block, available);
        let anchor = AnchorStore::open(&disk, block, available);
        (
            Self {
                device,
                waiting: BTreeMap::new(),
                running: None,
                anchor,
            },
            image,
            Nvram { bank, proxy },
            sep_state::Location::Partition { disk, first },
            board_flash,
        )
    }

    /// The sectors the guest changed, taken from its devices, for
    /// [`Self::store`] to write without them.
    pub fn dirty(devices: &mut AppleDevices) -> Vec<(usize, Vec<u8>)> {
        devices.nor_dirty()
    }

    /// Store the sectors the guest changed, without waiting for the
    /// device: they join the next store, which begins once the one in
    /// progress is durable. A failed store stops the machine: NVRAM the
    /// guest believes committed must not be lost silently.
    pub fn store(&mut self, dirty: Vec<(usize, Vec<u8>)>) {
        self.waiting.extend(dirty);
        self.advance(false);
        AnchorStore::advance(&mut self.anchor, false);
    }

    /// Complete every store, the caller waiting: at the stop.
    pub fn finish(&mut self) {
        self.advance(true);
        AnchorStore::advance(&mut self.anchor, true);
    }

    /// The PMU upcount's anchor this boot starts from: `None` on a new
    /// machine, one an older Stela wrote, or without the anchor's region.
    pub fn loaded_anchor(&self) -> Option<RtcAnchor> {
        self.anchor.as_ref().and_then(|store| store.loaded)
    }

    /// Store `anchor` without waiting for the device: after the store in
    /// progress, replacing a waiting one.
    pub fn keep_anchor(&mut self, anchor: RtcAnchor) {
        if let Some(store) = self.anchor.as_mut() {
            store.waiting = Some(anchor);
        }
        AnchorStore::advance(&mut self.anchor, false);
    }

    fn advance(&mut self, wait: bool) {
        loop {
            if let Some(running) = self.running.as_mut() {
                match running.poll(&mut *self.device) {
                    None if wait => {
                        core::hint::spin_loop();
                        continue;
                    }
                    None => return,
                    Some(Err(error)) => {
                        output::fatal(format_args!("a NOR sector could not be stored: {error}"));
                    }
                    Some(Ok(_)) => self.running = None,
                }
            }
            if self.waiting.is_empty() {
                return;
            }
            let identity = self.device.identity();
            let store = core::mem::take(&mut self.waiting)
                .into_iter()
                .map(|(offset, sector)| persist::request(identity, offset, sector))
                .collect::<Result<Vec<_>, _>>()
                .and_then(|requests| {
                    Submission::batch(requests, identity)
                        .map_err(|error| refuse!("NOR persistence batch: {error}", %error))
                });
            match store {
                Ok(store) => self.running = Some(store),
                Err(error) => {
                    output::fatal(format_args!("a NOR sector could not be stored: {error}"));
                }
            }
        }
    }
}

/// The board devices' flash on the machine-state partition.
pub struct BoardFlash {
    device: Box<dyn BlockDevice>,
    flash: board_flash::Flash,
}

impl BoardFlash {
    /// The region after the boot log, when the partition holds it and its
    /// device reads; otherwise say why there is none. Invalid contents read
    /// as blank: the images they held come back with the next bundle that
    /// carries them.
    fn open(disk: &Shared, block: u64, available: u64) -> Option<Self> {
        if available < board_flash::PARTITION_BYTES {
            output::arguments(format_args!(
                "BOARD FLASH off: the machine-state partition holds {available:#x} bytes; the flash needs {:#x}\n",
                board_flash::PARTITION_BYTES
            ));
            return None;
        }
        let window = disk.window(board_flash::OFFSET / block, board_flash::BYTES / block);
        let mut device: Box<dyn BlockDevice> = match window {
            Ok(window) => Box::new(window),
            Err(error) => {
                output::arguments(format_args!("BOARD FLASH off: {error}\n"));
                return None;
            }
        };
        let flash = match board_flash::Flash::read(&mut *device) {
            Ok(flash) => flash,
            Err(error @ board_flash::Error::Device(_)) => {
                output::arguments(format_args!("BOARD FLASH off: {error}\n"));
                return None;
            }
            Err(error) => {
                output::arguments(format_args!("BOARD FLASH invalid: {error}\n"));
                board_flash::Flash::default()
            }
        };
        for (name, bytes) in flash.images() {
            output::arguments(format_args!("BOARD FLASH {name} bytes={}\n", bytes.len()));
        }
        Some(Self { device, flash })
    }

    pub fn image(&self, name: &str) -> Option<&[u8]> {
        self.flash.image(name)
    }

    /// Keep the images the boot bundle carries, for the boots whose bundle
    /// does not. A failure costs those boots their devices, which refuse by
    /// name, not this one.
    pub fn keep(&mut self, images: &[(alloc::string::String, &[u8])]) {
        match self.flash.keep(&mut *self.device, images) {
            Ok(false) => {}
            Ok(true) => {
                for (name, bytes) in images {
                    output::arguments(format_args!(
                        "BOARD FLASH kept {name} bytes={}\n",
                        bytes.len()
                    ));
                }
            }
            Err(error) => output::arguments(format_args!("BOARD FLASH refusal: {error}\n")),
        }
    }
}

/// Open the boot log after the SEP's state and hand it to the output, when
/// the partition holds the region; otherwise say why there is none. Returns
/// whether the log is open.
pub(crate) fn open_log(disk: &Shared, block: u64, available: u64) -> bool {
    if available < boot_log::PARTITION_BYTES {
        output::arguments(format_args!(
            "BOOT LOG off: the machine-state partition holds {available:#x} bytes; the log needs {:#x}\n",
            boot_log::PARTITION_BYTES
        ));
        return false;
    }
    if !boot_log::OFFSET.is_multiple_of(block) || !boot_log::LOG_BYTES.is_multiple_of(block) {
        output::arguments(format_args!(
            "BOOT LOG off: {block}-byte blocks do not divide the log region\n"
        ));
        return false;
    }
    let mut device: Box<dyn BlockDevice> = Box::new(
        disk.window(boot_log::OFFSET / block, boot_log::LOG_BYTES / block)
            .expect("the partition holds the log region"),
    );
    let writer = match boot_log::Writer::open(&mut *device) {
        Ok(writer) => writer,
        Err(error @ (BlockError::Unsupported(_) | BlockError::ReadOnly)) => {
            output::arguments(format_args!("BOOT LOG off: {error}\n"));
            return false;
        }
        Err(error) => {
            output::log_lost(output::Log::Machine, error);
            return false;
        }
    };
    if let Some(reason) = writer.fresh() {
        output::arguments(format_args!("BOOT LOG fresh: {reason}\n"));
    }
    output::arguments(format_args!(
        "BOOT LOG boot={} offset={:#x} bytes={:#x} head={:#x}\n",
        writer.boots(),
        boot_log::OFFSET,
        boot_log::LOG_BYTES,
        writer.head()
    ));
    match output::add_log(writer, device, output::Log::Machine) {
        Ok(()) => true,
        Err(error @ output::Unopened::Store(_)) => {
            output::log_lost(output::Log::Machine, error);
            false
        }
        // The disk took the writes and keeps none: NVRAM will not persist
        // either. The boot goes on, so the screen shows how far it gets.
        Err(error) => {
            output::arguments(format_args!("BOOT LOG unkept: {error}\n"));
            crate::status::warn(&format!("the disk does not keep its writes: {error}"));
            false
        }
    }
}
