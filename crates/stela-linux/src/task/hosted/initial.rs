//! One original init_task on the persistent hosted service stack.
use super::*;
use crate::owner::{Owner, Platform, RawOwner};
use alloc::{boxed::Box, vec::Vec};
use stela_driver_base::bus::dma::memory::MemoryError;
use stela_driver_base::platform::work::kernel_domain::{HostedPages, HostedTicket};

type Body = unsafe extern "C" fn(*const RawOwner, *mut c_void) -> !;

struct Initial<P: Platform, const N: usize> {
    owner: Pin<Box<Owner<P>>>,
    _root: HostedPages,
    pages: HostedPages,
    _stacks: Vec<HostedPages>,
    stack_bases: Vec<*mut c_void>,
    binding: Option<Rc<HostedTaskBinding<N>>>,
    body: Body,
    context: *mut c_void,
}

unsafe extern "C" {
    fn stela_linux_kernel_claim_hosted(owner: *const RawOwner) -> i32;
    fn stela_linux_task_hosted_initial_allocate(
        owner: *const RawOwner,
        pgd: *mut c_void,
        result: *mut *mut CTask,
    ) -> i32;
    fn stela_linux_page_model_register(
        owner: *const RawOwner,
        cpu_base: *mut c_void,
        page_count: usize,
        stacks: *const *mut c_void,
        stack_count: usize,
    ) -> i32;
    fn stela_linux_task_hosted_initialize_resources(owner: *const RawOwner) -> i32;
}

/// Run the original resource sequence on the mounted hosted Initial.
/// Child creation and joined workqueue, timer and SRCU service are separate.
///
/// # Safety
/// Call inside the exact Initial's entered owner and live source scope, with
/// its model inventory and clock providers kept. Hold no transient runtime
/// loan across the original C code. A partial source attempt is permanent.
pub unsafe fn initialize_hosted_resources(
    owner: *const RawOwner,
) -> Result<(), ResourceBootstrapError> {
    // SAFETY: C authenticates the same mounted owner, source and task scope.
    match unsafe { stela_linux_task_hosted_initialize_resources(owner) } {
        0 => Ok(()),
        error => Err(ResourceBootstrapError::Native(error)),
    }
}

/// Publish original PID1 and kthreadd from the mounted hosted Initial.
/// Continue running Initial's hosted service and drive the dispatcher with
/// `step_hosted` to run the children and process their source joins.
///
/// # Safety
/// The exact Initial is entered and has completed original resources.
/// Keep the mounted source and callback graph for the life of this process.
pub unsafe fn start_hosted_threads<const N: usize>(
    owner: *const RawOwner,
    runtime: Pin<&HostedRuntime<N>>,
) -> Result<(), ResourceBootstrapError> {
    // SAFETY: forwarded exact original Initial and its retained runtime.
    unsafe { fork::start_threads(owner, runtime) }
}

const PAGE: usize = 4096;
const MAX_MODEL_PAGES: usize = 16 * 1024;

fn allocate<const N: usize>(
    runtime: &HostedRuntime<N>,
    pages: usize,
) -> Result<HostedPages, WorkError> {
    HostedPages::allocate(runtime.execution(), pages, pages * PAGE, 0).map_err(
        |error| match error {
            MemoryError::OutOfMemory => WorkError::Full,
            MemoryError::Unavailable => WorkError::Unavailable,
            _ => WorkError::corrupt(),
        },
    )
}

impl<const N: usize> HostedTaskBinding<N> {
    /// Mount this process image's actual original init_task and start it on
    /// its one persistent hosted service assignment. The body receives the
    /// exact original owner and enters it on that stack. Linux receives one
    /// kept hosted inventory with 1 to 16384 ordinary pages in a power-of-two
    /// extent and the eligible General stacks of this runtime.
    ///
    /// # Safety
    /// Use the sole archive owner thread in a fresh process. Keep this pinned
    /// runtime, the statically resident C/Rust image and all callback owners
    /// for the rest of the process, including failures after the claim. The
    /// platform services may only operate on this host model. They must
    /// refuse native mappings, IRQs, shutdown and other physical effects.
    /// The body enters the exact owner, never unwinds and never returns.
    pub unsafe fn start_initial<P: Platform + 'static>(
        runtime: Pin<&HostedRuntime<N>>,
        platform: P,
        page_count: usize,
        body: Body,
        context: *mut c_void,
    ) -> Result<HostedTicket, ResourceBootstrapError> {
        use ResourceBootstrapError::{Native, Work};
        if !page_count.is_power_of_two() || page_count > MAX_MODEL_PAGES {
            return Err(Work(WorkError::Unavailable));
        }
        let required = stack_requirement().map_err(Work)?;
        runtime.preflight_service(required).map_err(Work)?;
        let stacks = runtime.child_stack_inventory(required).map_err(Work)?;
        let mut stack_bases = Vec::new();
        stack_bases
            .try_reserve_exact(stacks.len())
            .map_err(|_| Work(WorkError::Full))?;
        for stack in &stacks {
            let base = stack
                .cpu_address(0, stack.bytes())
                .map_err(|_| Work(WorkError::corrupt()))?;
            stack_bases.push(base.as_ptr().cast());
        }
        // One private, inert model PGD. It is an actual retained mmap, with
        // no conversion to a native root, PFN or mapped-page authority.
        let root = allocate(&runtime, 1).map_err(Work)?;
        let pages = allocate(&runtime, page_count).map_err(Work)?;
        let pgd = root
            .cpu_address(0, PAGE)
            .map_err(|_| Work(WorkError::corrupt()))?;
        let owner = Owner::new(platform);
        let fork = fork::Broker::new(runtime, owner.raw(), stacks.len()).map_err(Work)?;
        let mount = Box::new(Initial {
            owner,
            _root: root,
            pages,
            _stacks: stacks,
            stack_bases,
            binding: None,
            body,
            context,
        });
        // Every original hosted source context publishes actual CPU time
        // through the scheduler's accounting rows.
        runtime.enable_ranked_scheduling().map_err(Work)?;
        // SAFETY: the actual owner/root are kept above before global claim.
        // A returned refusal leaves the archive's owner unchanged.
        let status = unsafe { stela_linux_kernel_claim_hosted(mount.owner.raw()) };
        if status != 0 {
            return Err(Native(status));
        }
        let mount = Box::leak(mount);
        let mut raw = core::ptr::null_mut();
        // SAFETY: exact same retained owner and private model PGD. There is
        // no callback/current publication while this dispatcher owns setup.
        let status = unsafe {
            stela_linux_task_hosted_initial_allocate(
                mount.owner.raw(),
                pgd.as_ptr().cast(),
                &mut raw,
            )
        };
        if status != 0 {
            return Err(Native(status));
        }
        let task = NonNull::new(raw).ok_or_else(|| Work(WorkError::corrupt()))?;
        // SAFETY: the irreversible original mount and same runtime stay kept.
        unsafe { fork.install(task) }.map_err(Work)?;
        mount.binding = Some(Self::new_record(task, runtime, Kind::Initial));
        let context = core::ptr::from_mut(mount).cast();
        // SAFETY: the published mount, binding, body and image remain kept
        // on every path after the irreversible original source claim.
        unsafe { runtime.start_service(run::<P, N>, context) }.map_err(Work)
    }
}

unsafe extern "C" fn run<P: Platform, const N: usize>(context: *mut c_void) {
    // SAFETY: start_initial retained this exact permanently mounted context.
    let mount = unsafe { &*context.cast::<Initial<P, N>>() };
    let binding = mount.binding.as_ref().unwrap_or_else(|| fatal());
    binding.bind_current().unwrap_or_else(|_| fatal());
    // SAFETY: actual checked service current. The static mount keeps this
    // context and the original C lease across every subsequent suspension.
    let _ = unsafe { binding.run_scoped(enter::<P, N>, context) };
    fatal();
}

unsafe extern "C" fn enter<P: Platform, const N: usize>(context: *mut c_void) {
    // SAFETY: the original start callback borrows this same kept mount.
    let mount = unsafe { &*context.cast::<Initial<P, N>>() };
    let owner = mount.owner.raw();
    let bytes = mount.pages.bytes();
    let cpu = mount
        .pages
        .cpu_address(0, bytes)
        .unwrap_or_else(|_| fatal());
    {
        // SAFETY: this original Initial is current. Registration retains its
        // mount's real mmap and performs no source callbacks or switches.
        let _entered = unsafe { crate::owner::Entered::new(owner) };
        // SAFETY: all extents belong to this exact permanently kept mount.
        let status = unsafe {
            stela_linux_page_model_register(
                owner,
                cpu.as_ptr().cast(),
                bytes / PAGE,
                mount.stack_bases.as_ptr(),
                mount.stack_bases.len(),
            )
        };
        if status != 0 {
            fatal();
        }
    }
    // SAFETY: the caller's permanent source body enters this exact owner.
    unsafe { (mount.body)(owner, mount.context) };
}

pub(super) fn fatal() -> ! {
    // SAFETY: a failed permanent source mount cannot release its live roots.
    unsafe { core::arch::asm!("ud2", options(noreturn)) }
}
