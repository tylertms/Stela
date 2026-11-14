//! The UEFI mount's firmware-independent logic, built for the host (its
//! tests and `stela`) and for `x86_64-unknown-uefi` (`stela.efi`).
//! `stela-boot` keeps the firmware calls and the code that needs its
//! allocator, page tables or console.
#![cfg_attr(not(test), no_std)]
extern crate alloc;

pub use stela_contracts::Refusal;

/// A refused request names the values that decided it.
pub type Result<T, E = Refusal> = core::result::Result<T, E>;

pub mod aperture;
#[cfg(target_arch = "x86_64")]
pub mod apic;
pub mod assigned_topology;
pub mod audio_check_samples;
pub mod bluetooth_dsbr;
pub mod boot_exit;
pub mod boot_progress;
pub mod bounded_spin;
pub mod bundle;
pub mod bundle_repair;
pub mod chain_check;
pub mod chosen_disk;
pub mod console_lines;
#[cfg(feature = "native-acpi")]
pub mod crs;
pub mod dmar;
pub mod doorbell;
pub mod drivers_turn;
pub mod early_fault;
pub mod entropy;
pub mod exit_port;
pub mod firmware_connect;
pub mod firmware_pages;
pub mod firmware_state;
pub mod firmware_time;
pub mod gpu_status;
pub mod guarded_stacks;
pub mod guest_timeouts;
pub mod handoff_records;
pub mod heap;
pub mod host_identity;
pub mod host_memory;
pub mod host_resources;
pub mod hpet;
pub mod interrupt_image;
pub mod ioapic;
pub mod irq_context;
pub mod log_sinks;
pub mod machine_check;
pub mod madt;
pub mod memory_budget;
pub mod memory_types;
#[cfg(target_arch = "x86_64")]
pub mod msr;
pub mod native_memory;
pub mod native_profile;
pub mod native_resources;
#[cfg(target_arch = "x86_64")]
pub mod nofault_read;
pub mod options;
pub mod page_window;
pub mod paging_controls;
pub mod panic_region;
pub mod parking;
pub mod passthrough;
pub mod pe;
pub mod pm_timer;
pub mod poll_timer;
pub mod posted_call;
pub mod power_button;
pub mod ram;
pub mod random;
pub mod record;
pub mod remapping;
#[cfg(target_arch = "x86_64")]
pub mod requester;
pub mod resident_image;
pub mod root_transition;
pub mod screen;
pub mod service_watch;
pub mod shutdown;
pub mod smi;
pub mod source_health;
pub mod staged_disk;
pub mod staged_frame;
pub mod stop_keys;
pub mod stop_record;
pub mod storage;
pub mod topology;
pub mod tsc;
pub mod usb_legacy;
#[cfg(target_arch = "x86_64")]
pub mod vtd;
pub mod work_status;
pub mod worker_bindings;
pub mod writeback_placement;
pub mod xstate;

pub mod firmware_map;
pub mod firmware_pci;

pub mod ecam;
