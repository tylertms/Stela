/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef STELA_TASK_H
#define STELA_TASK_H
#include <linux/types.h>

struct task_struct;
struct thread_info;
struct stela_linux_owner;
struct stela_linux_task;

/* Exact repr(C) platform::work::TaskContext. The callback view is borrowed;
 * primitive fields may be copied, never the view pointer. Bounds describe the
 * separately retained real Runtime stack/guard, not a new mapping authority. */
struct stela_linux_task_context {
	u64 queue_id, context_id, sequence;
	size_t stack_start, stack_end, guard_start, guard_end, stack_pointer;
};

struct stela_linux_task_rank {
	u32 policy, realtime;
	s32 effective;
	s32 nice;
};
_Static_assert(sizeof(struct stela_linux_task_rank) == 16, "Linux task rank ABI");

struct stela_linux_task_ops {
	/* Actual CPU/IRQ/RSP/current ticket validation, bounded and allocation-
	 * free. The retained binding/code outlive this attached task. */
	bool (*is_current)(void *binding, const struct stela_linux_task_context *context);
	/* Exact active-job signal; caller already proved a real same-CPU C
	 * current and retained target lifetime. No allocation/switch/final drop. */
	int (*wake_target)(void *binding, u64 expected_sequence);
	/* Immutable real Runtime reservation; never caller-controlled task flags.
	 * Used only after first waker publication and target generation validation. */
	bool (*service_reserved)(void *binding);
	/* Same-queue retained task's original PI/policy commit, or its private
	 * exit normalization. Records are copied synchronously; no C pointer is
	 * stored by Runtime. Update must succeed before C commits its fields. */
	int (*rank)(void *binding, u64 expected_sequence,
		    const struct stela_linux_task_rank *before,
		    const struct stela_linux_task_rank *after, bool head);
	/* Target concrete binding retains SAME task/wake before type erasure.
	 * Result owner is kept even on callback failure; never caller-N cast. */
	int (*retain_inactive)(void *binding, u64 sequence, void **owner);
};
_Static_assert(sizeof(struct stela_linux_task_ops) == 40, "Linux task callbacks ABI");

struct stela_linux_task_scope_ops {
	/* Retained shared Runtime borrow on this actual stack. No unwind or
	 * escape. Service idle is marked only by the original service loop.
	 * park: 0 skipped, 1 resumed; yield: 0 resumed; negative errno. */
	int (*park)(void *scope, bool service_idle);
	int (*yield)(void *scope);
	int (*reschedule)(void *scope);
	/* Read actual pending wake/due service state only at eligible process
	 * boundaries. A failed/closed provider is terminal, never false. */
	bool (*reschedule_requested)(void *scope);
    /* Genuine nonreturn native switch; no owning Rust guard/permit may remain
     * on the abandoned source frame. A returning provider is corruption. */
    void (*terminate)(void *scope);
	/* Bounded pi_lock/IF0 observation: 1 temporal inactive, 0 subscribed,
	 * negative terminal error. End consumes only this actual opaque owner
	 * after removing its actual waiter; no loan survives subsequent park. */
	int (*observe_inactive)(void *scope, void *owner, bool force_wait);
	int (*end_inactive)(void *scope, void *owner);
	/* Sample the exact current Runtime row and publish to its live source task. */
	int (*account_current)(void *scope, u64 *total_ns);
};

_Static_assert(sizeof(struct stela_linux_task_context) == 64, "TaskContext ABI");
#define STELA_TASK_OFFSET(type, member, offset) \
    _Static_assert(__builtin_offsetof(struct type, member) == offset, #type "." #member)
STELA_TASK_OFFSET(stela_linux_task_ops, is_current, 0);
STELA_TASK_OFFSET(stela_linux_task_ops, wake_target, 8);
STELA_TASK_OFFSET(stela_linux_task_ops, service_reserved, 16);
STELA_TASK_OFFSET(stela_linux_task_ops, rank, 24);
STELA_TASK_OFFSET(stela_linux_task_ops, retain_inactive, 32);
_Static_assert(sizeof(struct stela_linux_task_scope_ops) == 64, "ScopeOps ABI");
STELA_TASK_OFFSET(stela_linux_task_scope_ops, park, 0);
STELA_TASK_OFFSET(stela_linux_task_scope_ops, yield, 8);
STELA_TASK_OFFSET(stela_linux_task_scope_ops, reschedule, 16);
STELA_TASK_OFFSET(stela_linux_task_scope_ops, reschedule_requested, 24);
STELA_TASK_OFFSET(stela_linux_task_scope_ops, terminate, 32);
STELA_TASK_OFFSET(stela_linux_task_scope_ops, observe_inactive, 40);
STELA_TASK_OFFSET(stela_linux_task_scope_ops, end_inactive, 48);
STELA_TASK_OFFSET(stela_linux_task_scope_ops, account_current, 56);
#undef STELA_TASK_OFFSET

/* Private serialized Linux lease. Suspend is not exit: no callbacks or owner
 * drops run in IF=0 hooks. All functions require the actual qualified CPU. */
struct stela_linux_lease {
	const struct stela_linux_owner *owner;
	unsigned long irq_flags;
	u64 generation;
};
void stela_linux_lease_snapshot(struct stela_linux_lease *lease);
bool stela_linux_lease_matches(const struct stela_linux_lease *lease);
void stela_linux_lease_suspend(const struct stela_linux_lease *lease);
bool stela_linux_lease_resume(const struct stela_linux_lease *lease);
/* A preemption point after a delay in preemptible task context. */
void stela_linux_delay_point(void);

/* Allocation/release run in qualified process context, outside Linux entry.
 * Attach/leave/arrive/complete/detach are bounded IF=0 Runtime switch hooks.
 * Return failure before mutation; corruption after a committed switch traps. */
unsigned int stela_linux_task_catalog_capacity(void);
size_t stela_linux_task_stack_bytes(void);
size_t stela_linux_task_stack_alignment(void);
size_t stela_linux_task_stack_guard(void);
struct stela_linux_task *stela_linux_task_allocate(void);
/* Explicit retained original initial identity only. No current/attachment or
 * full-resource readiness is established. Refused after any bare allocation;
 * no rollback/reuse of a consumed initial claim is permitted. */
/* pgd is only the borrowed exact CPU view from the kept native root producer.
 * The qualified native owner is retained before any claim; this metadata call
 * is bounded/nonblocking and grants no page-table editing authority. */
int stela_linux_task_initial_allocate(const struct stela_linux_owner *owner,
	void *pgd, struct stela_linux_task **result);
#ifdef __linux__
/* The same original task, with process-image data and a kept model PGD. */
int stela_linux_task_hosted_initial_allocate(const struct stela_linux_owner *owner,
	void *pgd, struct stela_linux_task **result);
int stela_linux_task_hosted_initialize_resources(const struct stela_linux_owner *owner);
#endif
/* One genuine original Initial/Managed current + source-owned resource subset.
 * This never establishes global bootstrap/workqueue/SRCU FullReady. */
int stela_linux_require_current(const struct stela_linux_owner *owner);
struct stela_linux_task_birth;
/* Actual original allocator birth only; no generic task pointer adoption.
 * Record construction grants NO original child/current/publication readiness. */
int stela_linux_task_child_allocate(const struct stela_linux_owner *owner,
    struct stela_linux_task_birth *birth, struct stela_linux_task **result);
/* Source observations/installer, same genuine creator+actual born record only.
 * Target/generation observations establish no current or native ownership. */
int stela_linux_task_child_identity(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, void **target, u64 *generation);
void stela_linux_task_child_install_stack(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, void *base, size_t bytes);
int stela_linux_task_child_source_unwound(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, u64 generation);
struct kernel_clone_args;
/* Structural selected native kernel-thread boundary, before original clone
 * resource effects. This creates no child, native stack or publication proof. */
int stela_linux_task_clone_check(const struct kernel_clone_args *args);
/* Actual Initial record/current Service scope only. Calls original resource
 * helpers; no transient EnteredProcess may span this possibly waiting body. */
int stela_linux_task_initialize_resources(struct stela_linux_task *task,
    const struct stela_linux_owner *owner);
/* Source PID1/PID2 creation only, not execution/progress/FullReady. Actual
 * same native child broker/domain, retained heap/page/root/code/providers and
 * genuine process source context are prerequisites, not synthesized here. */
int stela_linux_task_start_threads(struct stela_linux_task *task,
    const struct stela_linux_owner *owner);
int stela_linux_require_full_kthread_current(const struct stela_linux_owner *owner);

/* Same admitted CPU/current as task services; pointer equality only. This
 * classifies an existing allocation, never obtains a reference or authorizes
 * dereferencing a foreign task pointer. Future original children are distinct. */
bool stela_linux_task_is_bare(const struct task_struct *task);
/* Consume measured time for the runqueue's locked, bound current task. */
s64 stela_linux_task_account_class(struct task_struct *current_task);
/* Same-CPU dispatcher transitions after the native current check. */
void stela_linux_rq_switch(struct task_struct *previous, struct task_struct *next);
/* Initialize the per-class queues on the canonical Initial task. */
int stela_linux_rq_init_queues(const struct stela_linux_owner *owner);
/* Exact original task-object data projection for the native measured
 * execution sink. No C calls/allocations are permitted from IRQ publication.
 * Prepared pointers must NOT be written before genuine attach accepts them.
 * Actual backing/code owner and source record remain retained independently. */
struct stela_linux_task_data;
/* Native attachment identity only; never a constructed TaskContext/current
 * or stack/mapping permission. Comes from actual private stopped token. */
struct stela_linux_task_data_identity {
    u64 queue_id, context_id, sequence;
};
_Static_assert(sizeof(struct stela_linux_task_data_identity) == 24, "native data identity ABI");
struct stela_linux_task_data_view {
    struct stela_linux_task_data *lease;
    u64 *total_ns, *system_ns;
};
_Static_assert(sizeof(struct stela_linux_task_data_view) == 24, "native task data view ABI");
/* Bounded original Initial cold metadata or genuine child creator only.
 * Not a raw task/field-address adoption API or task-ref acquisition. */
int stela_linux_task_data_prepare(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, struct stela_linux_task_data_view *view);
/* Private unsafe native boundary: exact source process, actual matching
 * StoppedPublication/IRQ-reader join (or actual NoPublication before attach)
 * held by caller. Never call from callback/IRQ/terminal successor or infer the
 * independent native receipt from any flags, errno or reference count. */
int stela_linux_task_data_release(struct stela_linux_task_data *lease,
    const struct stela_linux_owner *owner, const struct stela_linux_task_data_identity *identity);
int stela_linux_task_attach(struct stela_linux_task *task,
	const struct stela_linux_task_context *context,
	const struct stela_linux_task_ops *ops, void *binding);
int stela_linux_task_still_blocked(struct stela_linux_task *task,
	const struct stela_linux_task_context *context);
int stela_linux_task_leave(struct stela_linux_task *task,
	const struct stela_linux_task_context *context, unsigned int reason);
int stela_linux_task_arrive(struct stela_linux_task *task,
	const struct stela_linux_task_context *context, bool new_job);
int stela_linux_task_complete(struct stela_linux_task *task,
	const struct stela_linux_task_context *context);
/* Distinct terminal-only hooks: current dying frame then native-proved actual
 * off-all-stacks dispatcher successor. No resource/RCU/reaper disposal here. */
int stela_linux_task_terminal_prepare(struct stela_linux_task *task,
    const struct stela_linux_task_context *context);
int stela_linux_task_terminal_successor(struct stela_linux_task *task,
    const struct stela_linux_task_context *context);
/* Private published-child caller ONLY: actual native physical-return receipt
 * must be checked independently before this potentially waiting original
 * source reference release on another genuine process task. No dispatcher,
 * transient native process permit, raw errno or source flag can substitute. */
int stela_linux_task_reap_source(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, u64 generation);
int stela_linux_task_join_source(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, u64 generation);
/* Actual source closure/tail observation ONLY; no native physical/cancellation
 * receipt or descriptor/alias retirement authority is created by it. */
int stela_linux_task_child_tails_joined(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, u64 generation);
/* Actual source current after affine native/page cancellation or final release
 * and detach. Generation/counters are negative checks only; Rust unique
 * metadata ownership and joined producer tails remain required. */
int stela_linux_task_release_original(struct stela_linux_task *task,
    const struct stela_linux_owner *owner, u64 generation);
void __noreturn stela_linux_task_terminal_current(void);
int stela_linux_task_detach(struct stela_linux_task *task,
	const struct stela_linux_task_context *context);
void stela_linux_task_release(struct stela_linux_task *task);

int stela_linux_task_scope_begin(struct stela_linux_task *task,
	const struct stela_linux_task_scope_ops *ops, void *scope);
int stela_linux_task_scope_end(struct stela_linux_task *task);
/* Internal prepared transition hooks. The selected original mutex scheduler
 * adapter owns the sole preempt level; count==1 alone does not distinguish
 * that level from an outer RCU reader. Generic schedule enters at count0 and
 * never borrows the normal-mutex level. Non-PREEMPT_RT mutex waits use this
 * process boundary after their distinct pre-schedule worker preparation. */
int stela_linux_task_park(bool scheduler_preempt_disabled);
/* Idle wait after the original service loop finds no batch due. The actual
 * original Initial and reserved service stack must both be current. */
int stela_linux_task_park_service(void);
int stela_linux_task_yield(void);
struct task_struct *stela_linux_current(void);
/* Borrowed actual task owning this complete live-generation stack range on
 * the current real Runtime queue; NULL for unowned/closed/overflowing ranges.
 * Requires actual entered same-CPU current before any catalog/storage access.
 * Reads no storage bytes. Bounds exclude guards and non-stack metadata. This
 * proves neither an inner frame's lifetime nor arbitrary pointer ownership.
 * A timer consumer must independently retain its original storage/frame,
 * arm/task reference and real producer lifetime through cancel/drain. */
struct task_struct *stela_linux_task_stack_owner(const void *storage, size_t bytes);
struct thread_info *stela_linux_thread_info(void);
/* Narrow kernel-wait cancellation, not POSIX process/siginfo delivery. Values
 * 1/2 post actual SIGUSR1/SIGKILL bits. Target metadata is retained separately;
 * an old sequence returns -ESTALE before reading future-job state. */
int stela_linux_task_signal(struct stela_linux_task *task, u64 expected_sequence,
	unsigned int kind);
int stela_linux_task_ack_signal(unsigned int kind);
unsigned int stela_linux_task_pending_signals(void);
unsigned int stela_linux_task_waiting_io(void);
unsigned int stela_linux_task_waiting_load(void);
/* Private source-created kthread association, not a task/PID constructor.
 * Exact source create_info owner is retained before original kernel_thread;
 * fork begin/end are used only by genuine original kthreadd create_kthread. */
struct stela_linux_managed_lease;
int stela_linux_task_managed_fork_begin(const struct stela_linux_owner *,
    struct stela_linux_managed_lease *);
void stela_linux_task_managed_fork_end(const struct stela_linux_owner *,
    struct stela_linux_managed_lease *);
struct stela_linux_managed_lease *stela_linux_task_managed_birth_owner(
    struct stela_linux_task *, const struct stela_linux_owner *, u64 generation);

#endif
