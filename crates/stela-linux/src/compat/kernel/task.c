// SPDX-License-Identifier: GPL-2.0-only
/* Parts are adapted from the Linux files named below, whose notices are:
 * Copyright (C) 1991-2002  Linus Torvalds
 * Copyright (C) 1998-2024  Ingo Molnar, Red Hat
 */
/* Original Linux task metadata bound to actual retained Runtime continuations.
 * This implements identity/lease transitions, not a substitute Linux scheduler. */
#include <linux/errno.h>
#include <linux/cgroup.h>
#include <linux/cpumask.h>
#include <linux/kthread.h>
#include <stela/managed.h>
#include <linux/irqflags.h>
#include <linux/limits.h>
#include <linux/preempt.h>
#include <linux/refcount.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/task_stack.h>
#include <linux/sched/task.h>
#include <linux/sched/mm.h>
#include <linux/init_task.h>
#include <linux/sched/cputime.h>
#include <linux/sched/signal.h>
#include <linux/sched/rt.h>
#include <linux/string.h>
#include <stela/atomic_callback.h>
#include <stela/linux.h>
#include <stela/page.h>
#include <stela/task.h>
#include <stela/task_bootstrap.h>
#include <sched/fork_backend.h>
#include <stela/catalogue.h>
#include <stela/counts.h>
#include <stela/service.h>
#include <linux/mm_types.h>
#include <linux/pgtable.h>
#include <linux/blkdev.h>
/* Exact original internal headers from the same applied provider root. */
#include "rcu/rcu.h"
#include "workqueue_internal.h"
#include "../io_uring/io-wq.h"
#include "sched/sched.h"
#include <sched/affinity.h>
#include <sched/fork_capacity.h>

static_assert(CONFIG_THREAD_INFO_IN_TASK == 1);
static_assert(!IS_ENABLED(CONFIG_SMP));
static_assert(!IS_ENABLED(CONFIG_SCHED_CLASS_EXT));
static_assert(NR_CPUS == 1);
static_assert(!IS_ENABLED(CONFIG_PREEMPT_RT));
static_assert(!IS_ENABLED(CONFIG_MUTEX_SPIN_ON_OWNER));
static_assert(!IS_ENABLED(CONFIG_SCHED_PROXY_EXEC));
static_assert(!IS_ENABLED(CONFIG_DEBUG_LOCK_ALLOC));
static_assert(!IS_ENABLED(CONFIG_DEBUG_MUTEXES));
static_assert(!IS_ENABLED(CONFIG_DEBUG_WW_MUTEX_SLOWPATH));
static_assert(!IS_ENABLED(CONFIG_PREEMPT_VOLUNTARY_BUILD));
static_assert(!IS_ENABLED(CONFIG_PREEMPT_DYNAMIC));
static_assert(IS_ENABLED(CONFIG_RT_MUTEXES));
static_assert(!IS_ENABLED(CONFIG_BLOCK));
static_assert(!IS_ENABLED(CONFIG_TASK_DELAY_ACCT));
static_assert(!IS_ENABLED(CONFIG_PSI));
static_assert(!IS_ENABLED(CONFIG_SCHEDSTATS));
static_assert(sizeof(struct stela_linux_task_context) == 64);
static_assert(offsetof(struct stela_linux_task_context, sequence) == 16);
static_assert(offsetof(struct stela_linux_task_context, stack_pointer) == 56);
static_assert(offsetof(struct task_struct, thread_info) == 0);
static_assert(__alignof__(struct task_struct) >= 64);

enum allocation_kind { BARE_ALLOCATION, INITIAL_ALLOCATION, ORIGINAL_CHILD_ALLOCATION };
enum source_fork_phase { FORK_UNRESERVED, FORK_RESERVED, FORK_CLASS_SELECTED, FORK_PLACED };
static bool initial_claimed;
/* Inline affine source data lease: storage is allocated with the actual
 * record before any native attach. Prepared addresses are not publication
 * permission; only the real accepted attach acquires an original task ref. */
struct stela_linux_task_data {
    struct stela_linux_task *record;
    const struct stela_linux_owner *owner;
    struct stela_linux_task_context context;
    u64 *total_ns, *system_ns;
    u64 class_runtime_ns;
    bool prepared, claimed, released;
};
struct stela_linux_task {
	/* Only the bare metadata producer owns this embedded storage. Original
	 * task objects are retained independently, never cast to this layout. */
	struct task_struct bare_task;
	struct task_struct *linux_task;
	enum allocation_kind kind;
    u64 source_birth_generation;
    const struct stela_linux_owner *source_owner;
    struct stela_linux_task_context source_creator;
    bool source_stack_lent;
    enum source_fork_phase source_fork_phase;
    enum stela_linux_fork_class source_fork_class;
    u64 source_fork_flags;
    bool source_c_published, source_free_prepared, source_original_freed;
    bool source_unwound, source_native_returned, source_terminal_prepared;
    bool source_reap_started, source_scheduler_refs_released, source_rcu_joined;
    bool source_tail_notified;
    unsigned int source_borrows;
    struct stela_linux_task_data data;
    /* Exact source create_info scope, never inferred from entry fn/PID/name.
     * Fork scope and each actual child record retain independent lease refs. */
    struct stela_linux_managed_lease *managed_fork_owner;
    struct stela_linux_managed_lease *managed_birth_owner;
    bool source_thread_prepared, source_native_dormant, source_visible;
    bool source_activated, source_resources_copied, source_active_mm;
    int (*source_entry)(void *);
    void *source_argument;
	struct list_head allocation_node;
	struct stela_linux_task_context context;
	const struct stela_linux_task_ops *ops;
	void *binding;
	const struct stela_linux_task_scope_ops *scope_ops;
	void *scope;
	struct stela_linux_lease lease;
	int prepared_count;
	bool attached, active, suspended, prepared;
	bool io_blocked, load_blocked, finalizing;
	bool scope_boundary;
	bool rt_prepared;
    bool rt_submit_work;
	bool initial_started;
};

/* The real same-CPU Runtime owns publication. This does not authorize C entry
 * from another CPU, an ISR, or an unretained stack. The current callback checks
 * actual CPU/IRQ/RSP before any admitted task service uses mutable metadata. */
static struct stela_linux_task *active;

static void publish_current(struct stela_linux_task *previous,
			    struct stela_linux_task *next)
{
	stela_linux_rq_switch(previous ? previous->linux_task : NULL,
			      next ? next->linux_task : NULL);
	__atomic_store_n(&active, next, __ATOMIC_RELEASE);
}

/* Actual attached stack leases only. Registration/retirement are bounded IF0
 * hooks on the single qualified CPU; no native ISR can read this catalog.
 * Rust's retained binding owns each metadata pointer through actual detach.
 * A closed job remains registered until detach but never owns live storage. */
/* The work layer's MAX_CONTINUATIONS, which task.rs checks at start. */
#define STACK_TASKS 128U
static struct stela_linux_task *stack_tasks[STACK_TASKS];

/* Allocation identity is independent of attachment/stack publication. Only
 * retained wrapper allocations occur on this list; no original task pointer
 * is cast to this layout. The qualified UP process/switch boundary serializes
 * allocation, lookup and final removal. This list holds no borrowed frame. */
static LIST_HEAD(task_allocations);

static struct stela_linux_task *allocated_task(const struct task_struct *linux_task)
{
	struct stela_linux_task *task;
	list_for_each_entry(task, &task_allocations, allocation_node)
		if (task->linux_task && task->linux_task == linux_task)
			return task;
	return NULL;
}

unsigned int stela_linux_task_catalog_capacity(void)
{
	/* Ceiling only. stack_admit still requires actual attached stack leases. */
	return STACK_TASKS;
}

size_t stela_linux_task_stack_bytes(void) { return THREAD_SIZE; }
size_t stela_linux_task_stack_alignment(void) { return THREAD_ALIGN; }
size_t stela_linux_task_stack_guard(void) { return PAGE_SIZE; }

static unsigned int stack_slot(const struct stela_linux_task *task)
{
	for (unsigned int slot = 0; slot < STACK_TASKS; slot++)
		if (stack_tasks[slot] == task)
			return slot;
	return STACK_TASKS;
}

static int stack_admit(const struct stela_linux_task_context *view)
{
	int free_slot = -ENOSPC;
	for (unsigned int slot = 0; slot < STACK_TASKS; slot++) {
		const struct stela_linux_task *task = stack_tasks[slot];
		if (!task) {
			if (free_slot < 0)
				free_slot = (int)slot;
			continue;
		}
		/* Both guard and usable mapping remain exclusively retained even
		 * between jobs. Never admit a second owner of overlapping ranges. */
		if ((view->guard_start < task->context.stack_end &&
		     task->context.guard_start < view->stack_end) ||
		    (view->queue_id == task->context.queue_id &&
		     view->context_id == task->context.context_id))
			return -EINVAL;
	}
	return free_slot;
}

static bool view_valid(const struct stela_linux_task_context *view, bool live)
{
	return view && view->queue_id && view->context_id &&
	       (live ? view->sequence != 0 : view->sequence == 0) &&
	       !(view->stack_start & (PAGE_SIZE - 1UL)) &&
	       view->stack_end > view->stack_start &&
	       view->stack_end - view->stack_start == THREAD_SIZE &&
	       !(view->guard_start & (PAGE_SIZE - 1UL)) &&
	       view->guard_end == view->stack_start &&
	       view->guard_end > view->guard_start &&
	       view->guard_end - view->guard_start >= PAGE_SIZE &&
	       view->stack_pointer >= view->stack_start &&
	       view->stack_pointer < view->stack_end;
}

static bool same_stack(const struct stela_linux_task *task,
		       const struct stela_linux_task_context *view)
{
	return task->context.queue_id == view->queue_id &&
	       task->context.context_id == view->context_id &&
	       task->context.stack_start == view->stack_start &&
	       task->context.stack_end == view->stack_end &&
	       task->context.guard_start == view->guard_start &&
	       task->context.guard_end == view->guard_end;
}

static bool actual_stack(const struct stela_linux_task_context *view)
{
	unsigned long stack_pointer;
	asm volatile("mov %%rsp, %0" : "=r"(stack_pointer));
	return stack_pointer >= view->stack_start && stack_pointer < view->stack_end;
}

static bool live_here(struct stela_linux_task *task,
		      const struct stela_linux_task_context *view)
{
	if (!task)
		return false;
	/* ops/binding are immutable throughout attached lifetime and retained
	 * until real producer quiescence. Validate the actual root CPU/IRQ/RSP
	 * BEFORE reading mutable C task/context metadata. Rust's callback also
	 * defers dereferencing view until that independent validation succeeds. */
	if (!task->ops || !task->ops->is_current(task->binding, view))
		return false;
	return task->attached && task->active && view_valid(view, true) &&
	       __atomic_load_n(&active, __ATOMIC_ACQUIRE) == task &&
	       same_stack(task, view) && task->context.sequence == view->sequence &&
	       actual_stack(view);
}

static struct stela_linux_task *current_record(void)
{
	struct stela_linux_task *task = __atomic_load_n(&active, __ATOMIC_ACQUIRE);
	if (!task || !live_here(task, &task->context))
		__builtin_trap();
	return task;
}

struct thread_info *stela_linux_thread_info(void)
{
	if (!__atomic_load_n(&active, __ATOMIC_ACQUIRE))
		return &stela_thread_info;
	return &current_record()->linux_task->thread_info;
}

bool stela_linux_task_is_bare(const struct task_struct *linux_task)
{
	(void)current_record();
	(void)stela_linux_owner();
	struct stela_linux_task *task = allocated_task(linux_task);
	return task && task->kind == BARE_ALLOCATION;
}

struct task_struct *stela_linux_current(void)
{
	struct stela_linux_task *task = current_record();
	/* Even a real continuation outside its entered Linux owner is not an
	 * admitted current for original Linux services. */
	(void)stela_linux_owner();
	return task->linux_task;
}

int task_curr(const struct task_struct *task)
{
	/* The serialized Linux owner admits exactly one current continuation. */
	return task == stela_linux_current();
}

static u64 measured_runtime(struct stela_linux_task *task)
{
	struct task_struct *p = task->linux_task;
	u64 sample;

	if (!task->data.claimed || !task->scope || !task->scope_ops->account_current ||
	    task->scope_ops->account_current(task->scope, &sample))
		stela_linux_fatal("current task runtime sampling failed");
	if (stela_linux_kernel_hosted()) {
		/* The sole hosted CPU keeps the original task data reference.
		 * A bounded scheduler sample charges only its actual running stack.
		 * Keep writes here with the exact current record, with no Rust writer. */
		if (READ_ONCE(p->se.sum_exec_runtime) > sample || READ_ONCE(p->stime) > sample)
			stela_linux_fatal("hosted task runtime regressed");
		WRITE_ONCE(p->stime, sample);
		WRITE_ONCE(p->se.sum_exec_runtime, sample);
	} else if (READ_ONCE(p->se.sum_exec_runtime) < sample || READ_ONCE(p->stime) < sample) {
		stela_linux_fatal("current task runtime publication failed");
	}
	return sample;
}

unsigned long long task_sched_runtime(struct task_struct *p)
{
	struct stela_linux_task *task = current_record();
	(void)stela_linux_owner();

	/* Only current accrues CPU time on this single-CPU Linux owner.
	 * Each switch publishes the previous task's outgoing sample. */
	if (p != task->linux_task)
		return READ_ONCE(p->se.sum_exec_runtime);
	if (task->kind == BARE_ALLOCATION) {
		stela_linux_unsupported("task runtime without a measured native context");
		return READ_ONCE(p->se.sum_exec_runtime);
	}

	return measured_runtime(task);
}

s64 stela_linux_task_account_class(struct task_struct *current_task)
{
	struct stela_linux_task *task = current_record();
	const struct stela_linux_owner *owner = stela_linux_owner();

	if (!preempt_count() || current_task != task->linux_task ||
	    task->kind == BARE_ALLOCATION || task->data.owner != owner)
		stela_linux_fatal("class accounting without the current native runqueue task");

	/* The native sampler already publishes sum_exec_runtime.
	 * Class callbacks consume the measured increment since their last update;
	 * an ordinary task_sched_runtime() observation only refreshes the total.
	 * The runqueue transition must update the outgoing class before changing it. */
	u64 total = measured_runtime(task);
	u64 previous = task->data.class_runtime_ns;
	if (total < previous || total - previous > S64_MAX)
		stela_linux_fatal("invalid native class runtime increment");
	task->data.class_runtime_ns = total;
	return (s64)(total - previous);
}

struct task_struct *stela_linux_task_stack_owner(const void *storage, size_t bytes)
{
	/* Establish real CPU/current/RSP/owner BEFORE any mutable catalog read.
	 * Merely supplying a pointer or being inside a mapped stack is no grant. */
	struct stela_linux_task *caller = current_record();
	(void)stela_linux_owner();
	unsigned long begin = (unsigned long)storage;
	if (!bytes || bytes > ULONG_MAX - begin)
		return NULL;
	unsigned long end = begin + bytes;
	for (unsigned int slot = 0; slot < STACK_TASKS; slot++) {
		struct stela_linux_task *task = stack_tasks[slot];
		if (!task || !task->attached || !task->context.sequence ||
		    task->context.queue_id != caller->context.queue_id)
			continue;
		if (begin >= task->context.stack_start && end <= task->context.stack_end)
			return task->linux_task;
	}
	return NULL;
}

/* Only admitted same-CPU C code and IF=0 switch hooks access these counters.
 * They describe actual committed sleepers, not token nesting or all Linux rq
 * statistics. A native CPU-idle/accounting consumer remains unbound. */
static unsigned int waiting_io, waiting_load;

static void block_account(struct stela_linux_task *task, unsigned int state)
{
	if (task->io_blocked || task->load_blocked || task->linux_task->is_blocked)
		__builtin_trap();
	bool io = task->linux_task->in_iowait;
	/* Pinned sched/core.c block_task: idle/frozen sleepers do not add load. */
	bool load = (state & TASK_UNINTERRUPTIBLE) &&
		!(state & TASK_NOLOAD) && !(state & TASK_FROZEN);
	if ((io && waiting_io == UINT_MAX) || (load && waiting_load == UINT_MAX))
		__builtin_trap();
	task->io_blocked = io;
	task->load_blocked = load;
	task->linux_task->sched_contributes_to_load = load;
	waiting_io += io;
	waiting_load += load;
	task->linux_task->is_blocked = 1;
	task->linux_task->on_rq = 0;
}

static void wake_account(struct stela_linux_task *task)
{
	if ((task->io_blocked && !waiting_io) || (task->load_blocked && !waiting_load))
		__builtin_trap();
	waiting_io -= task->io_blocked;
	waiting_load -= task->load_blocked;
	task->io_blocked = false;
	task->load_blocked = false;
	task->linux_task->is_blocked = 0;
}

static bool quiet_except_usage_flags(const struct stela_linux_task *task, unsigned long allowed)
{
	/* Linux's sigisemptyset() takes a mutable set: test a copy. */
	sigset_t pending = task->linux_task->pending.signal;

	return !task->linux_task->thread_info.preempt_count &&
	       !(task->linux_task->thread_info.flags & ~allowed) &&
	       READ_ONCE(task->linux_task->__state) == TASK_RUNNING &&
	       task->linux_task->flags == PF_KTHREAD && !task->linux_task->blocked_on &&
	       !task->linux_task->blocked_donor && !task->linux_task->wake_q.next &&
	       !task->linux_task->in_iowait && !task->linux_task->is_blocked &&
	       !task->rt_prepared && !task->linux_task->pi_blocked_on &&
	       !task->linux_task->pi_top_task && RB_EMPTY_ROOT(&task->linux_task->pi_waiters.rb_root) &&
	       task->linux_task->prio == task->linux_task->normal_prio &&
	       !task->io_blocked && !task->load_blocked &&
	       sigisemptyset(&pending) &&
	       list_empty(&task->linux_task->pending.list) &&
	       refcount_read(&task->linux_task->stack_refcount) == 1;
}

static bool quiet_except_usage(const struct stela_linux_task *task)
{
	return quiet_except_usage_flags(task, 0);
}

static bool quiet(const struct stela_linux_task *task)
{
	if (task->kind != BARE_ALLOCATION)
		return false; /* Original initial/children never use bare retirement. */
	return quiet_except_usage(task) && refcount_read(&task->linux_task->usage) == 1;
}

/* Every task's signal_struct: no task belongs to a process (see pid.c). */
static struct signal_struct no_process;

struct stela_linux_task *stela_linux_task_allocate(void)
{
    /* Canonical tasks have actual original factories; never allocate a bare
     * replacement before or after the initial source claim. */
	if (stela_linux_kernel_source_active() || initial_claimed ||
	    stela_linux_kernel_prepare_resource()) {
		return NULL;
	}
	struct stela_linux_task *task = stela_linux_heap_alloc(sizeof(*task), __alignof__(*task));
	if (!task)
		return NULL;
	task->linux_task = &task->bare_task;
	task->kind = BARE_ALLOCATION;
	task->linux_task->thread_info.task = task->linux_task;
	task->linux_task->flags = PF_KTHREAD;
	task->linux_task->signal = &no_process;
	/* Equal-priority normal kernel continuations; zero prio would falsely
	 * describe an RT task when a later separately admitted profile reads it. */
	task->linux_task->policy = SCHED_NORMAL;
	task->linux_task->prio = DEFAULT_PRIO;
	task->linux_task->static_prio = DEFAULT_PRIO;
	task->linux_task->normal_prio = DEFAULT_PRIO;
	task->linux_task->pi_waiters = RB_ROOT_CACHED;
	INIT_LIST_HEAD(&task->linux_task->pending.list);
	sigemptyset(&task->linux_task->pending.signal);
	refcount_set(&task->linux_task->usage, 1);
	refcount_set(&task->linux_task->stack_refcount, 1);
	raw_spin_lock_init(&task->linux_task->pi_lock);
	raw_spin_lock_init(&task->linux_task->blocked_lock);
	task->lease.irq_flags = ARCH_IRQ_ENABLED;
	list_add_tail(&task->allocation_node, &task_allocations);
	return task;
}

/* No general task-pointer adoption API exists. The sole initial producer
 * names the actual pinned original object itself. It retains both this record
 * and init_task forever on any uncertain/partial bootstrap. */
static int initial_allocate(const struct stela_linux_owner *owner,
	void *pgd, struct stela_linux_task **result)
{
	if (!owner || !stela_linux_kernel_source_active() ||
	    !stela_linux_kernel_owner_allowed(owner) || stela_linux_kernel_prepare_resource())
		return -EPERM;
	if (!result || !pgd || ((unsigned long)pgd & (PAGE_SIZE - 1UL)))
		return -EINVAL;
	if (initial_claimed)
		return -EALREADY;
	if (__atomic_load_n(&active, __ATOMIC_ACQUIRE) || !list_empty(&task_allocations) ||
	    init_mm.pgd || init_task.stack)
		return -EBUSY;
	struct stela_linux_task *task = stela_linux_heap_alloc(sizeof(*task), __alignof__(*task));
	if (!task)
		return -ENOMEM;
	task->linux_task = &init_task;
	task->kind = INITIAL_ALLOCATION;
	task->lease.irq_flags = ARCH_IRQ_ENABLED;
	/* Rust retains the qualified OriginalKernelRoot BEFORE this call. This
	 * pointer is its exact original root CPU view, not a CR3/mapping grant.
	 * The admitted kernel-thread path never edits page tables through it. */
	init_mm.pgd = pgd;
	initial_claimed = true;
	list_add_tail(&task->allocation_node, &task_allocations);
	*result = task;
	return 0;
}

int stela_linux_task_initial_allocate(const struct stela_linux_owner *owner,
	void *pgd, struct stela_linux_task **result)
{
	if (!stela_linux_kernel_catalogue_active())
		return -EPERM;
	return initial_allocate(owner, pgd, result);
}

#ifdef __linux__
int stela_linux_task_hosted_initial_allocate(const struct stela_linux_owner *owner,
	void *pgd, struct stela_linux_task **result)
{
	if (!stela_linux_kernel_hosted())
		return -EPERM;
	int error = initial_allocate(owner, pgd, result);
	if (error)
		return error;
	/* This actual process image keeps init_task's C storage. The C record
	 * owns the original task reference after attach. The native Rust data
	 * publisher and its IRQ writer are never exposed to this model. */
	struct stela_linux_task_data_view view;
	return stela_linux_task_data_prepare(*result, owner, &view);
}
#endif
/* Actual source birth -> private metadata only. Native stack/page/rank/record
 * publication and original copy_process resources remain separate obligations. */
int stela_linux_task_child_allocate(const struct stela_linux_owner *owner,
    struct stela_linux_task_birth *birth, struct stela_linux_task **result)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (!birth || !result || !stela_linux_quiescent())
        return -EINVAL;
    struct stela_linux_task *record = stela_linux_heap_alloc(sizeof(*record), __alignof__(*record));
    if (!record)
        return -ENOMEM;
    struct stela_linux_managed_lease *managed = current_record()->managed_fork_owner;
    /* Reserve the actual existing producer reference BEFORE birth_take's
     * irreversible transfer. No failure after a genuine birth is consumed. */
    if (managed && !stela_linux_managed_lease_get(managed, owner)) {
        stela_linux_heap_free(record, sizeof(*record), __alignof__(*record));
        return -ENOMEM;
    }
    struct task_struct *original = NULL;
    u64 generation = 0;
    error = stela_linux_task_birth_take(birth, &original, &generation);
    if (error) {
        if (managed)
            stela_linux_managed_lease_put(managed, owner);
        stela_linux_heap_free(record, sizeof(*record), __alignof__(*record));
        return error;
    }
    /* No fallible callback/postcheck after consuming this actual C birth. */
    if (!original || !generation || allocated_task(original))
        __builtin_trap();
    record->linux_task = original;
    record->kind = ORIGINAL_CHILD_ALLOCATION;
    record->source_birth_generation = generation;
    record->source_owner = owner;
    record->source_creator = current_record()->context;
    record->managed_birth_owner = managed;
    record->lease.irq_flags = ARCH_IRQ_ENABLED;
    list_add_tail(&record->allocation_node, &task_allocations);
    *result = record;
    return 0;
}

/* Only existing actual source-born records can be observed/installed. This
 * is not a constructor and never accepts a caller task pointer as authority. */
static struct stela_linux_task *source_child_record(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner)
{
    if (stela_linux_require_current(owner))
        return NULL;
    struct stela_linux_task *creator = current_record();
    struct stela_linux_task *record;
    list_for_each_entry(record, &task_allocations, allocation_node) {
        if (record != requested)
            continue;
        const struct stela_linux_task_context *a = &record->source_creator;
        const struct stela_linux_task_context *b = &creator->context;
        if (record->kind != ORIGINAL_CHILD_ALLOCATION || record->source_owner != owner ||
            !record->source_birth_generation ||
            a->queue_id != b->queue_id || a->context_id != b->context_id ||
            !a->sequence || a->sequence != b->sequence)
            return NULL;
        return record;
    }
    return NULL;
}
/* The source original create_kthread owns this exact synchronous scope.
 * Genuine source current and the retained opaque lease are both required;
 * neither a task pointer, callback address nor a label creates association. */
int stela_linux_task_managed_fork_begin(const struct stela_linux_owner *owner,
    struct stela_linux_managed_lease *lease)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (!stela_linux_quiescent() || !lease || current != kthreadd_task)
        return -EPERM;
    struct stela_linux_task *record = current_record();
    if (record->managed_fork_owner)
        return -EBUSY;
    if (!stela_linux_managed_lease_get(lease, owner))
        return -ESTALE;
    /* No fallible action follows the retained scope publication. */
    record->managed_fork_owner = lease;
    return 0;
}
void stela_linux_task_managed_fork_end(const struct stela_linux_owner *owner,
    struct stela_linux_managed_lease *lease)
{
    if (stela_linux_require_current(owner) || !stela_linux_quiescent() ||
        current != kthreadd_task || !lease)
        __builtin_trap();
    struct stela_linux_task *record = current_record();
    if (record->managed_fork_owner != lease)
        __builtin_trap();
    record->managed_fork_owner = NULL;
    stela_linux_managed_lease_put(lease, owner);
}
struct stela_linux_managed_lease *stela_linux_task_managed_birth_owner(
    struct stela_linux_task *requested, const struct stela_linux_owner *owner,
    u64 generation)
{
    struct stela_linux_task *record = source_child_record(requested, owner);
    if (!record || !generation || record->source_birth_generation != generation)
        __builtin_trap();
    /* Borrowed existing source-constructor ownership; NULL only for an actual
     * unmanaged birth. Caller must retain it before any callback/suspension. */
    return record->managed_birth_owner;
}

int stela_linux_task_child_identity(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, void **target, u64 *generation)
{
    struct stela_linux_task *record = source_child_record(requested, owner);
    if (!record)
        return -EPERM;
    if (!target || !generation || !record->linux_task)
        return -EINVAL;
    *target = record->linux_task;
    *generation = record->source_birth_generation;
    return 0;
}
void stela_linux_task_child_install_stack(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, void *base, size_t bytes)
{
    struct stela_linux_task *record = source_child_record(requested, owner);
    if (!record || !stela_linux_quiescent() || record->source_stack_lent ||
        record->attached || !record->linux_task || record->linux_task->stack || !base ||
        bytes != THREAD_SIZE || ((unsigned long)base & (THREAD_ALIGN - 1UL)))
        __builtin_trap();
    /* Called INSIDE accepted actual PreparedChildStackLease::lend. That owner
     * pins real pages/mapping and retains them before this synchronous install.
     * No allocation/callback/wait/postcheck can conceal accepted source refs. */
    record->linux_task->stack = base;
    record->source_stack_lent = true;
}

/* Source hooks are invoked by actual original allocator/unwind/RCU code.
 * Generation only selects an ALREADY owned actual source birth; no pointer
 * adoption, task/current construction or no-publication proof follows from it. */
/* A source callback may notify native code while original unwind/RCU still
 * owns this C frame. Final metadata release must see zero borrows; no observer
 * can free a record merely because it sees source_freed or source_unwound. */
static void source_borrow(struct stela_linux_task *record)
{
    if (record->source_borrows == UINT_MAX)
        __builtin_trap();
    record->source_borrows++;
}
static void source_return(struct stela_linux_task *record)
{
    if (!record->source_borrows)
        __builtin_trap();
    record->source_borrows--;
    /* Actual final C source-tail availability, one notification only. Source
     * free/abort notices may precede this release; consumers never timer-poll
     * a metadata counter. Notification itself only records/wakes and cannot
     * invoke source cleanup or final release inline. No record use follows. */
    bool closed = record->source_original_freed &&
        (record->source_c_published ?
            (record->source_native_returned && record->source_scheduler_refs_released && record->source_rcu_joined) :
            record->source_unwound);
    if (!record->source_borrows && closed && !record->source_tail_notified) {
        record->source_tail_notified = true;
        stela_linux_task_native_source_tail(record, record->source_owner,
            record->source_birth_generation);
    }
}
/* The inline lease is located by actual record-list membership BEFORE a
 * caller pointer is dereferenced. No task pointer/field address constructor. */
static struct stela_linux_task *data_record(struct stela_linux_task_data *lease)
{
    struct stela_linux_task *record;
    list_for_each_entry(record, &task_allocations, allocation_node)
        if (&record->data == lease)
            return record;
    return NULL;
}
int stela_linux_task_data_prepare(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, struct stela_linux_task_data_view *view)
{
    if (!owner || !view || !stela_linux_kernel_source_active() ||
        !stela_linux_kernel_owner_allowed(owner) || stela_linux_kernel_prepare_resource())
        return -EPERM;
    struct stela_linux_task *record;
    list_for_each_entry(record, &task_allocations, allocation_node) {
        if (record != requested)
            continue;
        if (record->attached || record->data.prepared || record->kind == BARE_ALLOCATION ||
            !record->linux_task || record->source_original_freed)
            return -EBUSY;
        if (record->kind == INITIAL_ALLOCATION) {
            if (record->linux_task != &init_task || record->initial_started ||
                __atomic_load_n(&active, __ATOMIC_ACQUIRE))
                return -EPERM;
        } else if (!source_child_record(record, owner)) {
            return -EPERM;
        }
        struct task_struct *p = record->linux_task;
        if (!IS_ALIGNED((unsigned long)&p->se.sum_exec_runtime, sizeof(u64)) ||
            !IS_ALIGNED((unsigned long)&p->stime, sizeof(u64)))
            return -EOPNOTSUPP;
        /* Pure original-object projection, still NO task ref, metric writer,
         * current identity, native scheduling or source publication grant. */
        record->data.record = record;
        record->data.owner = owner;
        record->data.total_ns = &p->se.sum_exec_runtime;
        record->data.system_ns = &p->stime;
        record->data.prepared = true;
        view->lease = &record->data;
        view->total_ns = record->data.total_ns;
        view->system_ns = record->data.system_ns;
        return 0;
    }
    return -ESRCH;
}
static bool data_attachable(const struct stela_linux_task *record)
{
    if (record->kind == BARE_ALLOCATION)
        return true;
    const struct stela_linux_task_data *data = &record->data;
    return data->prepared && !data->claimed && !data->released &&
        data->record == record && data->owner &&
        stela_linux_kernel_owner_allowed(data->owner) &&
        (record->kind == INITIAL_ALLOCATION || record->source_owner == data->owner) &&
        data->total_ns == &record->linux_task->se.sum_exec_runtime &&
        data->system_ns == &record->linux_task->stime;
}
static void data_attached(struct stela_linux_task *record,
    const struct stela_linux_task_context *view)
{
    if (record->kind == BARE_ALLOCATION)
        return;
    /* All validation and stack admission completed before this nofail tail.
     * Real source publication/current attachment is accepted; the original
     * get_task_struct pins the actual slab object independently of metadata. */
    get_task_struct(record->linux_task);
    record->data.context = *view;
    record->data.class_runtime_ns = 0;
    record->data.claimed = true;
}
int stela_linux_task_data_release(struct stela_linux_task_data *lease,
    const struct stela_linux_owner *owner, const struct stela_linux_task_data_identity *view)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (!stela_linux_quiescent())
        return -EBUSY;
    struct stela_linux_task *record = data_record(lease);
    if (!record || !lease->prepared || lease->record != record ||
        lease->owner != owner || lease->released)
        return -EINVAL;
    /* Permanent Initial has no implemented joined kernel shutdown. Never
     * pretend its reference is releasable because this callback exists. */
    if (record->kind != ORIGINAL_CHILD_ALLOCATION)
        return -EPERM;
    if (!lease->claimed) {
        /* No native attachment/writer ever accepted. Actual source reverse
         * unwind and original object free must still precede lease disposal.
         * The private Rust caller ALSO owns genuine native NoPublication. */
        if (record->attached || record->source_c_published ||
            !record->source_unwound || !record->source_original_freed)
            return -EAGAIN;
        lease->released = true;
        return 0;
    }
    const struct stela_linux_task_context *a = &lease->context;
    if (!view || a->queue_id != view->queue_id || a->context_id != view->context_id ||
        !a->sequence || a->sequence != view->sequence ||
        record->context.queue_id != a->queue_id ||
        record->context.context_id != a->context_id ||
        record->context.sequence || /* actual source terminal record is closed */
        !record->source_terminal_prepared || !record->source_native_returned ||
        record->source_original_freed || !record->linux_task)
        return -EPERM;
    /* The private caller must additionally own the exact native stopped
     * publication/IRQ-reader join token. Metadata flags are only rejection
     * checks; this function cannot construct that independent receipt.
     * Release in genuine source process BEFORE source_freed/join_source. */
    struct task_struct *p = record->linux_task;
    source_borrow(record);
    lease->claimed = false;
    lease->released = true;
    put_task_struct(p);
    /* May close the original object and invoke source-free notifications.
     * Record remains kept through this genuine C tail; no use follows it. */
    source_return(record);
    return 0;
}
static struct stela_linux_task *source_generation(u64 generation)
{
    struct stela_linux_task *record;
    list_for_each_entry(record, &task_allocations, allocation_node)
        if (record->kind == ORIGINAL_CHILD_ALLOCATION &&
            record->source_birth_generation == generation)
            return record;
    return NULL;
}
u64 stela_linux_task_free_prepare(struct task_struct *original)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    if (stela_linux_require_current(owner))
        __builtin_trap();
    struct stela_linux_task *record = allocated_task(original);
    if (!record)
        return 0; /* genuine ctor failed BEFORE native metadata birth */
    if (record->kind != ORIGINAL_CHILD_ALLOCATION || record->source_owner != owner ||
        record->source_free_prepared || record->source_original_freed ||
        (record->source_c_published && !record->source_native_returned))
        __builtin_trap();
    record->source_free_prepared = true;
    return record->source_birth_generation;
}
void stela_linux_task_source_freed(u64 generation)
{
    if (!generation)
        return;
    const struct stela_linux_owner *owner = stela_linux_owner();
    if (stela_linux_require_current(owner))
        __builtin_trap();
    struct stela_linux_task *record = source_generation(generation);
    if (!record || record->source_owner != owner || !record->source_free_prepared ||
        record->source_original_freed)
        __builtin_trap();
    /* Actual original kmem_cache_free returned. NEVER dereference that task
     * again. Native binding/physical pin/code and this record stay retained. */
    source_borrow(record);
    record->linux_task = NULL;
    record->source_original_freed = true;
    /* Required native producer only records pending source retirement here:
     * no allocation/drop/wait or inline original callback in RCU/free tail. */
    stela_linux_task_native_source_freed(record, owner, generation);
    source_return(record);
}
int stela_linux_task_child_source_unwound(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, u64 generation)
{
    struct stela_linux_task *record = source_child_record(requested, owner);
    if (!record || record->source_birth_generation != generation)
        return -EPERM;
    /* This predicate records actual source return+barrier; page lease cancel
     * independently demands every real descriptor reference is zero. */
    return record->source_unwound && record->source_original_freed &&
        !record->source_c_published ? 0 : -EAGAIN;
}
void stela_linux_task_fork_abort(u64 generation)
{
    if (!generation)
        return;
    const struct stela_linux_owner *owner = stela_linux_owner();
    if (stela_linux_require_current(owner) || !stela_linux_quiescent())
        __builtin_trap();
    struct stela_linux_task *record = source_generation(generation);
    if (!record)
        return; /* actual source birth was never taken by a native ctor */
    if (source_child_record(record, owner) != record || record->source_c_published ||
        record->attached || record->source_unwound)
        __builtin_trap();
    /* Actual original reverse-unwind reached this hook, outside its locks.
     * Join its genuine delayed stack/task callbacks; never poll a page bit or
     * call an old manual WQ retry. If progress is unavailable we retain/wait. */
    source_borrow(record);
    rcu_barrier();
    if (!record->source_original_freed)
        __builtin_trap();
    record->source_unwound = true;
    /* Same creator performs exact native/page cancellation under fresh local
     * permit. Retained/unknown disposition must NOT return to original errno. */
    stela_linux_task_native_abort(record, owner, generation);
    source_return(record);
}

int stela_linux_task_thread_prepare(struct task_struct *original,
    const struct kernel_clone_args *args)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    struct stela_linux_task *record = allocated_task(original);
    if (!record || source_child_record(record, owner) != record ||
        !record->source_stack_lent || record->source_thread_prepared ||
        record->source_c_published || !args || !args->kthread || !args->fn)
        return -EINVAL;
    /* Exact native typed producer validates/stores real callback ownership;
     * this remains prepublication and its errors use genuine source unwind. */
    int error = stela_linux_task_native_thread_prepare(record, owner, args->fn, args->fn_arg);
    if (error)
        return error;
    record->source_entry = args->fn;
    record->source_argument = args->fn_arg;
    record->source_thread_prepared = true;
    return 0;
}
static struct stela_linux_task_rank source_fork_rank(struct task_struct *task)
{
    return (struct stela_linux_task_rank) {
        .policy = task->policy, .realtime = task->rt_priority,
        .effective = task->prio, .nice = task_nice(task),
    };
}

static bool source_fork_cpu_ready(const struct task_struct *p)
{
    /* boot_cpu_init() publishes these masks. A fully published singleton
     * cannot form an asymmetric domain. Check the actual retained state. */
    const struct cpumask *singleton = cpumask_of(0);
    return smp_processor_id() == 0 && task_cpu(p) == 0 &&
        cpumask_equal(cpu_possible_mask, singleton) &&
        cpumask_equal(cpu_present_mask, singleton) &&
        cpumask_equal(cpu_online_mask, singleton) &&
        cpumask_equal(cpu_active_mask, singleton) &&
        p->cpus_ptr == &p->cpus_mask && !p->user_cpus_ptr &&
        p->nr_cpus_allowed == 1 && cpumask_equal(p->cpus_ptr, singleton) &&
        !sched_asym_cpucap_active() && list_empty(&asym_cap_list);
}

static struct stela_linux_task *source_fork_candidate(struct task_struct *p,
    const struct stela_linux_owner *owner)
{
    struct stela_linux_task *record = allocated_task(p);
    if (!record || source_child_record(record, owner) != record ||
        !record->source_stack_lent || record->source_c_published ||
        READ_ONCE(p->__state) != TASK_NEW || p->prio != p->normal_prio ||
        p->sched_class)
        return NULL;
    return record;
}

int stela_linux_sched_fork_admit(u64 clone_flags, struct task_struct *p)
{
    const u64 supported = CLONE_VM | CLONE_UNTRACED | CLONE_FS | CLONE_FILES;
    if ((clone_flags & ~supported) || !(p->flags & PF_KTHREAD))
        return -EOPNOTSUPP;

    const struct stela_linux_owner *owner = stela_linux_owner();
    struct stela_linux_task *record = source_fork_candidate(p, owner);
    if (!record || record->source_fork_phase != FORK_UNRESERVED)
        return -EINVAL;
    if (!source_fork_cpu_ready(p))
        return -ENODEV;

    struct stela_linux_task_rank rank = source_fork_rank(p);
    int error = stela_linux_task_native_reserve_rank(record, owner, &rank);
    if (error)
        return error;
    record->source_fork_flags = clone_flags;
    record->source_fork_phase = FORK_RESERVED;
    return 0;
}

void stela_linux_sched_fork_select(struct task_struct *p, enum stela_linux_fork_class kind)
{
    struct stela_linux_task *record = source_fork_candidate(p, stela_linux_owner());
    if (!record || record->source_fork_phase != FORK_RESERVED ||
        !((kind == STELA_LINUX_FORK_FAIR && p->policy == SCHED_NORMAL) ||
          (kind == STELA_LINUX_FORK_RT && p->policy == SCHED_FIFO)))
        stela_linux_fatal("invalid original fork class selection");

    /* Policy and rank select the native class. The Linux runqueue class
     * table has no owner in this profile. */
    record->source_fork_class = kind;
    record->source_fork_phase = FORK_CLASS_SELECTED;
}

int stela_linux_sched_fork_place(struct task_struct *p, struct kernel_clone_args *args)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    struct stela_linux_task *record = source_fork_candidate(p, owner);
    if (!record || record->source_fork_phase != FORK_CLASS_SELECTED ||
        !args || !args->kthread || args->flags != record->source_fork_flags ||
        !irqs_disabled() || !preempt_count())
        return -EINVAL;
    if (!source_fork_cpu_ready(p))
        return -ENODEV;

    /* The checked reserve call is idempotent for the same rank and row.
     * The source keeps its child pi_lock and the native loan is bounded. */
    struct stela_linux_task_rank rank = source_fork_rank(p);
    int error = stela_linux_task_native_reserve_rank(record, owner, &rank);
    if (error)
        return error;
    if (record->source_fork_class == STELA_LINUX_FORK_FAIR)
        stela_linux_sched_fork_fair_capacity(p);
    record->source_fork_phase = FORK_PLACED;
    return 0;
}

void stela_linux_task_fork_publish(struct task_struct *original)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    struct stela_linux_task *record = allocated_task(original);
    if (!record || source_child_record(record, owner) != record ||
        !record->source_thread_prepared || !record->source_stack_lent ||
        record->source_fork_phase != FORK_PLACED ||
        record->source_native_dormant || record->source_c_published ||
        record->source_original_freed || READ_ONCE(original->__state) != TASK_NEW)
        __builtin_trap();
    struct stela_linux_task_rank rank = source_fork_rank(original);
    /* This required producer accepts the actual source rank in the SAME
     * Dormant transaction. It cannot return on uncertain native publication;
     * no original bad_fork error path can reclaim an accepted job/context. */
    stela_linux_task_native_publish(record, owner, &rank);
    record->source_native_dormant = true;
}
void stela_linux_task_fork_visible(struct task_struct *original)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    struct stela_linux_task *record = allocated_task(original);
    if (!record || source_child_record(record, owner) != record ||
        !record->source_native_dormant || record->source_visible ||
        !(original->flags & PF_KTHREAD) || original->mm || original->active_mm ||
        !original->cred || !original->real_cred || !original->signal ||
        !original->sighand || !original->exec_state || !original->fs ||
        !original->files || !original->nsproxy || !original->thread_pid ||
        !original->parent || !original->real_parent ||
        original->thread_info.task != original ||
        !refcount_read(&original->usage) || !refcount_read(&original->stack_refcount))
        __builtin_trap();
    /* Called only AFTER actual original source tasklist/PID/parent resource
     * publication. These fields validate its effects; they do not construct
     * source ownership from a pointer snapshot or grant a native current. */
    record->source_resources_copied = true;
    record->source_c_published = true;
    record->source_visible = true;
}
void stela_linux_task_wake_new(struct task_struct *original)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    struct stela_linux_task *record = allocated_task(original);
    if (!record || source_child_record(record, owner) != record ||
        !record->source_visible || !record->source_native_dormant ||
        record->source_activated || READ_ONCE(original->__state) != TASK_NEW)
        __builtin_trap();
    /* Exact actual native Dormant owner and pre-reserved order/capacity.
     * No allocation/wait, metadata lock borrow or alternate worker launch. */
    WRITE_ONCE(original->__state, TASK_RUNNING);
    stela_linux_task_native_activate(record, owner);
    record->source_activated = true;
}
void __noreturn stela_linux_task_run_original(void *context)
{
    struct stela_linux_task *record = current_record();
    if (record != context || record->kind != ORIGINAL_CHILD_ALLOCATION ||
        !record->source_activated || !record->source_entry || !record->scope)
        __builtin_trap();
    /* Explicit C lease owns entry; no owning Rust Entered guard is stranded
     * on a frame abandoned by the genuine nonreturn native terminal switch. */
    stela_linux_enter(record->source_owner);
    if (stela_linux_require_current(record->source_owner))
        __builtin_trap();
    calculate_sigpending();
    int result = record->source_entry(record->source_argument);
    /* Same original kernel-thread return effect as x86 ret_from_fork.
     * Original kthread() usually calls kthread_exit itself and never returns. */
    do_exit(result);
}

/* A non-terminal scheduling observation for the permanent original initial
 * object. Extra actual C refs are legal and are never reclamation authority.
 * Bare retirement predicates below remain separate and unchanged. */
static bool initial_quiet_flags(const struct stela_linux_task *task, unsigned long allowed)
{
	const struct task_struct *p = task->linux_task;
	return task->kind == INITIAL_ALLOCATION && p == &init_task && task->initial_started &&
	       !p->thread_info.preempt_count && !(p->thread_info.flags & ~allowed) &&
	       READ_ONCE(p->__state) == TASK_RUNNING && (p->flags & PF_KTHREAD) &&
	       !(p->flags & PF_EXITING) && !p->blocked_on && !p->blocked_donor &&
	       !p->wake_q.next && !p->in_iowait && !p->is_blocked &&
	       !task->rt_prepared && !p->pi_blocked_on && !p->pi_top_task &&
	       RB_EMPTY_ROOT(&p->pi_waiters.rb_root) && p->prio == p->normal_prio &&
	       !task->io_blocked && !task->load_blocked &&
	       refcount_read(&p->usage) > 0 && refcount_read(&p->stack_refcount) > 0;
}
static unsigned long source_async_flags(const struct stela_linux_task *task)
{
    /* Genuine original signal/task-work notification may be inherited or
     * posted by kthread_stop. Keeping these source bits is not normalization;
     * original calculate_sigpending/task-work/kthread code owns consumption. */
    return task->kind == ORIGINAL_CHILD_ALLOCATION ?
        (_TIF_SIGPENDING | _TIF_NOTIFY_SIGNAL) : 0;
}
static bool child_scope_quiet_flags(const struct stela_linux_task *record, unsigned long allowed)
{
    const struct task_struct *p = record->linux_task;
    return record->kind == ORIGINAL_CHILD_ALLOCATION && record->source_c_published &&
        record->source_resources_copied && record->source_activated && p &&
        !record->source_original_freed && !(p->flags & PF_EXITING) &&
        (p->flags & PF_KTHREAD) && !p->thread_info.preempt_count &&
        !(p->thread_info.flags & ~(allowed | source_async_flags(record))) &&
        READ_ONCE(p->__state) == TASK_RUNNING &&
        !record->prepared && !record->suspended && !record->io_blocked && !record->load_blocked &&
        !p->blocked_on && !p->blocked_donor && !p->wake_q.next && !p->is_blocked &&
        !record->rt_prepared && !p->pi_blocked_on && !p->pi_top_task &&
        RB_EMPTY_ROOT(&p->pi_waiters.rb_root) && p->prio == p->normal_prio &&
        refcount_read(&p->usage) > 0 && refcount_read(&p->stack_refcount) > 0;
}
static bool child_cold(const struct stela_linux_task *record,
    const struct stela_linux_task_context *view)
{
    const struct task_struct *p = record->linux_task;
    return view && record->kind == ORIGINAL_CHILD_ALLOCATION && record->source_c_published &&
        record->source_resources_copied && record->source_activated && p &&
        record->source_stack_lent && !record->source_original_freed &&
        !record->source_active_mm && !p->mm && !p->active_mm &&
        p->stack == (void *)view->stack_start && p->thread_info.task == p &&
        p->thread_info.preempt_count == FORK_PREEMPT_COUNT &&
        (p->flags & PF_KTHREAD) && !(p->flags & PF_EXITING) &&
        READ_ONCE(p->__state) == TASK_RUNNING &&
        refcount_read(&p->usage) > 0 && refcount_read(&p->stack_refcount) > 0;
}

static bool scope_quiet_flags(const struct stela_linux_task *task, unsigned long allowed)
{
	if (task->kind == INITIAL_ALLOCATION)
		return initial_quiet_flags(task, allowed);
	if (task->kind == ORIGINAL_CHILD_ALLOCATION) {
		return child_scope_quiet_flags(task, allowed);
	}
	return quiet_except_usage_flags(task, allowed) &&
	       refcount_read(&task->linux_task->usage) == 1;
}
static bool initial_cold(const struct stela_linux_task *task)
{
	const struct task_struct *p = task->linux_task;
	return task->kind == INITIAL_ALLOCATION && p == &init_task && !task->initial_started &&
	       p->thread_info.task == p && p->thread_info.preempt_count == INIT_PREEMPT_COUNT && !p->stack &&
	       !p->thread_info.flags && p->flags == PF_KTHREAD && p->active_mm == &init_mm &&
	       !p->mm && p->cred && p->real_cred && p->cred == p->real_cred &&
	       p->signal && p->sighand && p->exec_state && p->fs && p->files && p->nsproxy &&
	       p->thread_pid && p->parent == p && p->real_parent == p &&
	       refcount_read(&p->usage) == 2 && refcount_read(&p->stack_refcount) == 1;
}

int stela_linux_task_attach(struct stela_linux_task *task,
	const struct stela_linux_task_context *view,
	const struct stela_linux_task_ops *ops, void *binding)
{
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
	if (!task || task->attached || !data_attachable(task) ||
	    !(task->kind == INITIAL_ALLOCATION ? initial_cold(task) :
          task->kind == BARE_ALLOCATION ? quiet(task) : child_cold(task, view)) ||
	    !view_valid(view, true) ||
	    !actual_stack(view) || !ops || !ops->is_current || !ops->wake_target ||
	    !ops->service_reserved || !binding ||
	    __atomic_load_n(&active, __ATOMIC_ACQUIRE) || lease.owner ||
	    lease.irq_flags != ARCH_IRQ_ENABLED || stela_thread_info.preempt_count ||
	    stela_thread_info.flags)
		return -EINVAL;
	int slot = stack_admit(view);
	if (slot < 0)
		return slot;
	/* Initial attach precedes publication of the first real WakeHandle. The
	 * Runtime caller already validated CPU/IRQ/current/stack and retains us. */
	if (task->kind == INITIAL_ALLOCATION) {
		/* Consume only the exact original initial startup reference once,
		 * after actual CPU/RSP/stack/ownerless IF0 attachment is admitted.
		 * No arbitrary counter normalization or original resource init occurs. */
		task->linux_task->thread_info.preempt_count -= INIT_PREEMPT_COUNT;
		task->initial_started = true;
	} else if (task->kind == ORIGINAL_CHILD_ALLOCATION) {
        /* Own and consume only the exact original child startup preempt ref;
         * active_mm is a real retained borrowed original init_mm, never a user
         * mm/page-table permission. Its final lazy ref belongs to the reaper. */
        mmgrab_lazy_tlb(&init_mm);
        task->linux_task->active_mm = &init_mm;
        task->source_active_mm = true;
        task->linux_task->thread_info.preempt_count -= FORK_PREEMPT_COUNT;
    }
	task->context = *view;
	task->linux_task->stack = (void *)view->stack_start;
	*(unsigned long *)task->linux_task->stack = STACK_END_MAGIC;
	task->ops = ops;
	task->binding = binding;
	task->attached = true;
	task->active = true;
	task->linux_task->on_cpu = 1;
	task->linux_task->on_rq = 1;
	stack_tasks[(unsigned int)slot] = task;
	data_attached(task, view);
	publish_current(NULL, task);
	return 0;
}

/* Source-owned current/resources, not global all-subsystems boot Ready.
 * Initial identity is established only by the exact original-object/native
 * root/stack producer above. A future original child must be admitted by its
 * actual copy_process publication producer, never pointer/counter adoption. */
int stela_linux_require_current(const struct stela_linux_owner *owner)
{
	if (!owner)
		return -EINVAL;
	if (!stela_linux_kernel_source_active() || !stela_linux_kernel_owner_allowed(owner))
		return -EPERM;
	struct stela_linux_task *task = __atomic_load_n(&active, __ATOMIC_ACQUIRE);
	if (!task || !live_here(task, &task->context))
		return -EAGAIN;
	if (stela_linux_owner() != owner)
		return -EPERM;
    if (task->kind == ORIGINAL_CHILD_ALLOCATION) {
        /* Actual source-construction/publication provenance persists through
         * lawful exit_files/fs/nsproxy cleanup. Required operation objects are
         * retained by those original helpers; no repeated nonNULL snapshot is
         * substituted for this source identity. Native terminal is distinct. */
        return task->source_owner == owner && task->source_resources_copied &&
            task->source_visible && task->source_activated && !task->source_original_freed
            ? 0 : -EAGAIN;
    }
	if (task->kind != INITIAL_ALLOCATION || task->linux_task != &init_task || !task->initial_started)
		return -EAGAIN;
	const struct task_struct *p = task->linux_task;
	/* These are corruption checks AFTER genuine source identity, not a
	 * constructor or resource-ref/namespace/subsystem readiness receipt. */
	if (!p->cred || !p->real_cred || !p->signal || !p->sighand || !p->exec_state ||
	    !p->fs || !p->files || !p->nsproxy || !p->thread_pid || p->mm ||
	    p->active_mm != &init_mm || !init_mm.pgd || p->stack != (void *)task->context.stack_start)
		return -EIO;
	return 0;
}
int stela_linux_task_initialize_resources(struct stela_linux_task *task,
    const struct stela_linux_owner *owner)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    struct stela_linux_task *actual = current_record();
    /* Validate real current before comparing caller token; never adopt a
     * supplied task pointer. No second/bare/original child can exist yet. */
    if (task != actual || actual->kind != INITIAL_ALLOCATION ||
        actual->linux_task != &init_task || !actual->scope ||
        actual->suspended || actual->scope_boundary || actual->prepared ||
        !list_is_singular(&task_allocations) || !stela_linux_quiescent())
        return -EPERM;
    return stela_linux_task_bootstrap_resources(owner);
}

#ifdef __linux__
int stela_linux_task_hosted_initialize_resources(const struct stela_linux_owner *owner)
{
    if (!stela_linux_kernel_hosted())
        return -EPERM;
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    return stela_linux_task_initialize_resources(current_record(), owner);
}
#endif

int stela_linux_task_start_threads(struct stela_linux_task *task,
    const struct stela_linux_owner *owner)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (task != current_record() || task->kind != INITIAL_ALLOCATION ||
        task->linux_task != &init_task || !task->scope || task->suspended ||
        task->scope_boundary || task->prepared || !stela_linux_quiescent())
        return -EPERM;
    return stela_linux_task_bootstrap_threads(owner);
}

/* The actual native task runtime supports original kernel_thread creation,
 * not Linux userspace address spaces, userspace TIDs/TLS or clone namespaces.
 * Keep the original copy_process resource helpers/unwind authoritative after
 * this check. No errno from here certifies a later native publication absent. */
int stela_linux_task_clone_check(const struct kernel_clone_args *args)
{
    int error = stela_linux_require_current(stela_linux_owner());
    if (error)
        return error;
    if (!stela_linux_quiescent() ||
        !stela_linux_task_resource_roots_complete(stela_linux_owner()) ||
        rcu_scheduler_active == RCU_SCHEDULER_INACTIVE)
        return -EAGAIN;
    const u64 flags = CLONE_VM | CLONE_UNTRACED | CLONE_FS | CLONE_FILES;
    if (!args || !args->kthread || !args->fn || args->io_thread ||
        args->user_worker || args->no_files || args->idle ||
        (args->flags & ~flags) || !(args->flags & CLONE_VM) ||
        !(args->flags & CLONE_UNTRACED) ||
        (args->exit_signal && args->exit_signal != SIGCHLD) ||
        args->pidfd || args->child_tid || args->parent_tid ||
        args->stack || args->stack_size || args->tls || args->set_tid ||
        args->set_tid_size || args->cgroup || args->cgrp || args->cset || args->kill_seq)
        return -EOPNOTSUPP;
    if (current->flags & PF_EXITING)
        return -ESRCH;
    if (current->mm || current->active_mm != &init_mm)
        return -EIO;
    return 0;
}

int stela_linux_kthread_current_started(void);
int stela_linux_require_full_kthread_current(const struct stela_linux_owner *owner)
{
	int error = stela_linux_require_current(owner);
	if (error)
		return error;
	/* PID0 PF_KTHREAD is not original kthread() private initialization. */
	if (current_record()->kind == INITIAL_ALLOCATION)
		return -EAGAIN;
	return stela_linux_kthread_current_started();
}

int stela_linux_task_scope_begin(struct stela_linux_task *task,
	const struct stela_linux_task_scope_ops *ops, void *scope)
{
	if (!task || !live_here(task, &task->context) || task->scope ||
	    task->prepared || task->suspended || !scope_quiet_flags(task, 0) ||
	    !ops || !ops->park || !ops->yield || !ops->reschedule ||
	    !ops->reschedule_requested || !ops->terminate || !scope)
		return -EINVAL;
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
	if (lease.owner || lease.irq_flags != ARCH_IRQ_ENABLED)
		return -EBUSY;
	task->scope_ops = ops;
	task->scope = scope;
	return 0;
}

int stela_linux_task_scope_end(struct stela_linux_task *task)
{
	if (stela_linux_atomic_callback_active() || !task ||
	    !live_here(task, &task->context) || !task->scope || task->scope_boundary ||
	    task->prepared || task->suspended ||
	    !scope_quiet_flags(task, _TIF_NEED_RESCHED))
		return -EBUSY;
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
	if (lease.owner || lease.irq_flags != ARCH_IRQ_ENABLED)
		return -EBUSY;
	for (;;) {
		bool forced = test_tsk_thread_flag(task->linux_task, TIF_NEED_RESCHED);
		clear_tsk_thread_flag(task->linux_task, TIF_NEED_RESCHED);
		bool requested = task->scope_ops->reschedule_requested(task->scope);
		if (!forced && !requested)
			break;
		{
			/* The body exited its owner and drained RCU/pages already. Its local
			 * owner no longer exists: retain only this actual pinned Runtime scope
			 * and a private outside-owner preparation marker through the switch. */
			task->scope_boundary = true;
			int result = task->scope_ops->reschedule(task->scope);
			if (result || !live_here(task, &task->context) ||
			    !task->scope_boundary || task->prepared || task->suspended ||
			    !scope_quiet_flags(task, _TIF_NEED_RESCHED) || !stela_linux_lease_matches(&lease))
				__builtin_trap();
			task->scope_boundary = false;
		}
		/* A peer can post a new scheduler request during the ownerless switch.
		 * Consume it only through another real boundary, never a silent clear. */
		if (!test_tsk_thread_flag(task->linux_task, TIF_NEED_RESCHED))
			break;
	}
	task->scope = NULL;
	task->scope_ops = NULL;
	return 0;
}

/* Process-context PREPARE. The caller proves why count==1 belongs exclusively
 * to a known scheduler level; this internal function does not infer an RCU
 * reader's legality from the numeric count. RCU/BH/IRQ suspension is forbidden. */
static int prepare(struct stela_linux_task *task, int count)
{
	if (stela_linux_atomic_callback_active() || task->scope_boundary ||
	    !live_here(task, &task->context) || !task->scope || task->prepared ||
	    task->suspended || preempt_count() != count || irqs_disabled())
		return -EBUSY;
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
	if (!lease.owner || lease.irq_flags != ARCH_IRQ_ENABLED)
		return -EBUSY;
	if (count)
		preempt_count_sub(count);
	/* Owed irq_work runs before the task stops, as its self-interrupt
	 * would have before schedule(). */
	stela_linux_irq_work_run_owed();
	stela_linux_printk_flush();
	stela_linux_pages_finish();
	if (!stela_linux_quiescent() || !stela_linux_lease_matches(&lease))
		__builtin_trap();
	if (count)
		preempt_count_add(count);
	task->lease = lease;
	task->prepared_count = count;
	task->prepared = true;
	return 0;
}

static bool prepared_here(struct stela_linux_task *task)
{
	return !stela_linux_atomic_callback_active() &&
	       task->prepared && task->scope && !task->suspended &&
	       task->linux_task->thread_info.preempt_count == task->prepared_count &&
	       !(task->linux_task->thread_info.flags &
             ~(_TIF_SIGPENDING | _TIF_NEED_RESCHED | source_async_flags(task))) &&
	       task->lease.owner && task->lease.irq_flags == ARCH_IRQ_ENABLED &&
	       stela_linux_lease_matches(&task->lease);
}

static bool supported_block_state(const struct stela_linux_task *task, unsigned int state)
{
	/* Preserve the existing ordinary wait masks, plus the exact idle state.
	 * TASK_NOLOAD alone is not an admitted sleeping state. */
	if (state == TASK_PARKED) {
		return task->kind == ORIGINAL_CHILD_ALLOCATION &&
		       stela_linux_kthread_current_started() == 0;
	}
	return state == TASK_IDLE ||
	       !(state & ~(unsigned int)(TASK_NORMAL | TASK_WAKEKILL));
}

int stela_linux_task_still_blocked(struct stela_linux_task *task,
	const struct stela_linux_task_context *view)
{
	if (!live_here(task, view) || !prepared_here(task))
		return -EBUSY;
	unsigned int state = READ_ONCE(task->linux_task->__state);
	if (!supported_block_state(task, state))
		return -EOPNOTSUPP;
	return state != TASK_RUNNING;
}

int stela_linux_task_leave(struct stela_linux_task *task,
	const struct stela_linux_task_context *view, unsigned int reason)
{
	/* The private delivery scope cannot be defeated by rebalancing public
	 * preempt/IRQ state. Refuse before task accounting or lease mutation. */
	if (stela_linux_atomic_callback_active())
		return -EBUSY;
	if (!live_here(task, view) || reason > 2)
		return -EINVAL;
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
	if (lease.owner) {
		if (!prepared_here(task) ||
		    (reason != 1 && (task->prepared_count || READ_ONCE(task->linux_task->__state))) ||
		    (reason == 1 && !supported_block_state(task, READ_ONCE(task->linux_task->__state))))
			return -EBUSY;
	} else if (task->scope_boundary) {
		if (reason != 2 || !task->scope || task->prepared || task->suspended ||
		    !scope_quiet_flags(task, 0) || lease.irq_flags != ARCH_IRQ_ENABLED)
			return -EBUSY;
	} else if (reason || task->scope || task->prepared || !scope_quiet_flags(task, 0) ||
		   lease.irq_flags != ARCH_IRQ_ENABLED) {
		return -EBUSY;
	}
	/* Account the actual outgoing model stack before committing the switch.
	 * Its bounded sampler holds no owner loan across the transition. */
	if (task->kind != BARE_ALLOCATION && stela_linux_kernel_hosted())
		measured_runtime(task);
	/* Every refusal above is mutation-free. No callbacks, allocator, final
	 * release or Rust owner borrow crosses this committed IF=0 transition. */
	task->lease = lease;
	if (reason == 1)
		block_account(task, READ_ONCE(task->linux_task->__state));
    /* Actual native voluntary switch is now committed. These are original
     * source scheduler counters/observations, never final-retirement proofs. */
    task->linux_task->nvcsw++;
    task->linux_task->on_rq = reason == 1 ? 0 : 1;
	task->linux_task->on_cpu = 0;
	if (lease.owner)
		rcu_note_context_switch(false);
	stela_linux_lease_suspend(&lease);
	task->active = false;
	task->suspended = true;
	publish_current(task, NULL);
	return 0;
}

int stela_linux_task_arrive(struct stela_linux_task *task,
	const struct stela_linux_task_context *view, bool new_job)
{
	if (!task || !task->attached || task->active || !view_valid(view, true) ||
	    !same_stack(task, view) || !actual_stack(view) ||
	    __atomic_load_n(&active, __ATOMIC_ACQUIRE) ||
	    stela_thread_info.preempt_count || stela_thread_info.flags ||
	    !task->ops->is_current(task->binding, view))
		return -EINVAL;
	if (new_job) {
        if (task->kind == ORIGINAL_CHILD_ALLOCATION)
            return -EINVAL;
		if (task->context.sequence || task->suspended || task->scope ||
		    task->prepared || !quiet(task) || task->lease.owner)
			return -EBUSY;
	} else if (!task->suspended || task->context.sequence != view->sequence) {
		return -EINVAL;
	}
	if (!stela_linux_lease_resume(&task->lease))
		return -EBUSY;
	/* Also resolves a real Runtime spurious/IRQ pending wake which never
	 * entered C wake_up_process. Actual !SMP/!PROXY switch exclusion owns
	 * this target before current publication; no Linux ISR/task can race. */
	WRITE_ONCE(task->linux_task->blocked_on, NULL);
	wake_account(task);
	task->context = *view;
	task->linux_task->on_cpu = 1;
	task->linux_task->on_rq = 1;
	task->suspended = false;
	task->active = true;
	/* A genuine resume makes this sleeper runnable. Original lock/wait code
	 * must still recheck its own condition; this never grants lock ownership. */
	WRITE_ONCE(task->linux_task->__state, TASK_RUNNING);
	publish_current(NULL, task);
	return 0;
}

void __noreturn stela_linux_task_terminal_current(void)
{
    struct stela_linux_task *record = current_record();
    /* Resources may legitimately have been dismantled by original do_exit.
     * This authenticates terminal closure, never new ordinary task admission. */
    if (record->kind != ORIGINAL_CHILD_ALLOCATION || !record->scope ||
        !record->scope_ops || !record->scope_ops->terminate)
        __builtin_trap();
    record->scope_ops->terminate(record->scope);
    __builtin_trap();
}
int stela_linux_task_terminal_prepare(struct stela_linux_task *record,
    const struct stela_linux_task_context *view)
{
    if (!live_here(record, view) || record->kind != ORIGINAL_CHILD_ALLOCATION ||
        !record->source_c_published || !record->source_activated ||
        record->source_original_freed || record->source_terminal_prepared ||
        record->source_native_returned || !record->scope || record->scope_boundary ||
        record->prepared || record->suspended || record->rt_prepared ||
        stela_linux_atomic_callback_active() || stela_linux_rcu_callback_active())
        return -EINVAL;
    struct task_struct *p = record->linux_task;
    if (!p || !(p->flags & PF_EXITING) || READ_ONCE(p->__state) != TASK_DEAD ||
        p->thread_info.preempt_count != 1 || p->mm ||
        !record->source_active_mm || p->active_mm != &init_mm ||
        record->io_blocked || record->load_blocked || p->is_blocked ||
        p->blocked_on || p->blocked_donor || p->pi_blocked_on || p->pi_top_task ||
        !RB_EMPTY_ROOT(&p->pi_waiters.rb_root) || p->wake_q.next)
        return -EBUSY;
    struct stela_linux_lease lease;
    stela_linux_lease_snapshot(&lease);
    if (lease.owner != record->source_owner || !lease.generation ||
        lease.irq_flags != ARCH_IRQ_ENABLED)
        return -EPERM;
    if (stela_linux_kernel_hosted())
        measured_runtime(record);
    /* Every refusal precedes mutation. Exact C lease suspension replaces the
     * normal returning owner-exit tail; no skipped owning Rust guard exists. */
    record->lease = lease;
    p->nvcsw++;
    rcu_note_context_switch(false);
    stela_linux_lease_suspend(&lease);
    record->source_terminal_prepared = true;
    record->active = false;
    record->suspended = false;
    publish_current(record, NULL);
    return 0;
}
int stela_linux_task_terminal_successor(struct stela_linux_task *record,
    const struct stela_linux_task_context *view)
{
    /* Caller is the same actual retained binding; native Core proved real
     * physical dispatcher RSP outside ALL Runtime stacks at IF0 first. These
     * saved bounds are negative identity checks, never a forged process loan. */
    if (!record || record->kind != ORIGINAL_CHILD_ALLOCATION ||
        !record->source_terminal_prepared || record->source_native_returned ||
        record->active || record->suspended || !record->attached ||
        !view_valid(view, true) || !same_stack(record, view) ||
        record->context.sequence != view->sequence ||
        __atomic_load_n(&active, __ATOMIC_ACQUIRE) ||
        stela_thread_info.preempt_count || stela_thread_info.flags)
        return -EINVAL;
    struct stela_linux_lease empty;
    stela_linux_lease_snapshot(&empty);
    if (empty.owner || empty.generation || empty.irq_flags != ARCH_IRQ_ENABLED)
        return -EBUSY;
    struct task_struct *p = record->linux_task;
    if (!p || !(p->flags & PF_EXITING) || READ_ONCE(p->__state) != TASK_DEAD)
        return -EINVAL;
    p->on_cpu = 0;
    p->on_rq = 0;
    record->scope = NULL;
    record->scope_ops = NULL;
    record->source_native_returned = true;
    record->context.sequence = 0;
    /* Native ticket is closed by Core AFTER this hook returns. Only enqueue
     * retained notification here; a real original supervisor later performs
     * lazy-MM/source stack/task/RCU scheduler-ref release. */
    stela_linux_task_native_terminal_returned(record, record->source_owner,
        record->source_birth_generation);
    return 0;
}

/* Called ONLY by the private Rust published-child owner after its exact
 * native physically_returned receipt succeeds. This function does not turn a
 * C flag or source pointer into that native receipt. The caller runs on a
 * different actual original process task, with no transient EnteredProcess
 * or native queue borrow spanning these original source disposal helpers. */
int stela_linux_task_reap_source(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, u64 generation)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (!stela_linux_quiescent())
        return -EBUSY;
    struct stela_linux_task *record = source_generation(generation);
    if (!record || record != requested || record->source_owner != owner ||
        record->kind != ORIGINAL_CHILD_ALLOCATION)
        return -ESTALE;
    if (record == current_record() || record->source_borrows ||
        !record->source_native_returned || !record->source_terminal_prepared ||
        record->source_reap_started || record->source_original_freed ||
        record->active || record->suspended || record->scope || record->prepared)
        return -EBUSY;
    struct task_struct *p = record->linux_task;
    if (!p || !(p->flags & PF_EXITING) || READ_ONCE(p->__state) != TASK_DEAD ||
        p->on_cpu || p->on_rq || !record->source_active_mm || p->mm ||
        p->active_mm != &init_mm || refcount_read(&p->rcu_users) < 1)
        return -EINVAL;
    /* Every refusal is above. The native scheduler already joined its exact
     * job/class accounting as part of the checked physical successor receipt;
     * this task was NEVER enqueued in CFS/RT, so task_dead_fair's CFS rq access
     * is not owned here. Preserve the remaining original source reference
     * order from finish_task_switch, on a real process rather than dispatcher. */
    source_borrow(record);
    record->source_reap_started = true;
    record->source_active_mm = false;
    p->active_mm = NULL;
    mmdrop_lazy_tlb_sched(&init_mm);
    sched_ext_dead(p);
    cgroup_task_dead(p);
    put_task_stack(p);
    put_task_struct_rcu_user(p);
    /* p may already be freed by the final source callback. Never inspect it
     * after the original final ref helper. record stays under this borrow. */
    record->source_scheduler_refs_released = true;
    source_return(record);
    return 0;
}

int stela_linux_task_join_source(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, u64 generation)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (!stela_linux_quiescent())
        return -EBUSY;
    struct stela_linux_task *record = source_generation(generation);
    if (!record || record != requested || record->source_owner != owner)
        return -ESTALE;
    if (!record->source_native_returned || !record->source_scheduler_refs_released ||
        record->source_borrows || record == current_record())
        return -EBUSY;
    if (record->source_rcu_joined)
        return 0;
    source_borrow(record);
    /* External original get_task_struct refs may legitimately delay the real
     * destructor. A barrier alone is not a zero-ref/final-source receipt. */
    rcu_barrier();
    bool freed = record->source_original_freed;
    if (freed)
        record->source_rcu_joined = true;
    source_return(record);
    return freed ? 0 : -EAGAIN;
}

int stela_linux_task_child_tails_joined(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, u64 generation)
{
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (!stela_linux_quiescent())
        return -EBUSY;
    struct stela_linux_task *record = source_generation(generation);
    if (!record || record != requested || record->source_owner != owner)
        return -ESTALE;
    bool closed = record->source_original_freed &&
        (record->source_c_published ?
            (record->source_native_returned && record->source_scheduler_refs_released && record->source_rcu_joined) :
            record->source_unwound);
    return !record->source_borrows && closed && record->source_tail_notified ? 0 : -EAGAIN;
}

int stela_linux_task_complete(struct stela_linux_task *task,
	const struct stela_linux_task_context *view)
{
	if (!live_here(task, view) || task->scope || task->prepared || task->suspended ||
	    !quiet(task))
		return -EBUSY;
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
	if (lease.owner || lease.irq_flags != ARCH_IRQ_ENABLED)
		return -EBUSY;
	/* Runtime reuses this physical stack for a new NORMAL job. No PI donor
	 * or waiter survives quiet(), so exit can withdraw its base FIFO policy. */
	if (task->linux_task->policy != SCHED_NORMAL || task_nice(task->linux_task)) {
		struct stela_linux_task_rank before = {
			.policy = task->linux_task->policy,
			.realtime = task->linux_task->rt_priority,
			.effective = task->linux_task->prio,
			.nice = task_nice(task->linux_task),
		};
		struct stela_linux_task_rank after = {.policy = SCHED_NORMAL, .effective = DEFAULT_PRIO};
		if (!task->ops->rank || task->ops->rank(task->binding, task->context.sequence, &before, &after, false))
			return -EBUSY;
		task->linux_task->policy = SCHED_NORMAL;
		task->linux_task->rt_priority = 0;
		task->linux_task->static_prio = DEFAULT_PRIO;
		task->linux_task->normal_prio = DEFAULT_PRIO;
		task->linux_task->prio = DEFAULT_PRIO;
	}
	task->lease = lease;
	task->context.sequence = 0;
	task->linux_task->on_cpu = 0;
	task->linux_task->on_rq = 0;
	task->active = false;
	publish_current(task, NULL);
	return 0;
}

int stela_linux_task_detach(struct stela_linux_task *task,
	const struct stela_linux_task_context *view)
{
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
    bool source_closed = task && task->kind == ORIGINAL_CHILD_ALLOCATION &&
        task->source_native_returned && task->source_scheduler_refs_released &&
        task->source_rcu_joined && task->source_original_freed && !task->linux_task &&
        !task->source_borrows;
	if (!task || !task->attached || task->active || task->suspended || task->scope ||
	    task->prepared || task->context.sequence ||
        (task->kind == ORIGINAL_CHILD_ALLOCATION ? !source_closed : !quiet(task)) ||
	    !view_valid(view, false) || !same_stack(task, view) ||
	    __atomic_load_n(&active, __ATOMIC_ACQUIRE) || lease.owner ||
	    lease.irq_flags != ARCH_IRQ_ENABLED || stela_thread_info.preempt_count ||
	    stela_thread_info.flags)
		return -EBUSY;
	unsigned int slot = stack_slot(task);
	if (slot == STACK_TASKS)
		__builtin_trap();
	/* The unsafe Runtime caller established real producer/waiter quiescence.
	 * Reference counts cannot prove arbitrary held-mutex or foreign lifetimes. */
	stack_tasks[slot] = NULL;
	task->attached = false;
	task->binding = NULL;
	task->ops = NULL;
	return 0;
}

int stela_linux_task_release_original(struct stela_linux_task *requested,
    const struct stela_linux_owner *owner, u64 generation)
{
    /* Caller consumes private affine native/page completed-release or cancel
     * ownership after ALL callbacks/bindings/producers and source references
     * independently joined. This predicate cannot manufacture that proof. */
    int error = stela_linux_require_current(owner);
    if (error)
        return error;
    if (!stela_linux_quiescent())
        return -EBUSY;
    struct stela_linux_task *task = source_generation(generation);
    if (!task || task != requested || task->source_owner != owner)
        return -ESTALE;
    if (task->attached || task->active || task->suspended || task->scope ||
        (task->data.prepared && !task->data.released) ||
        task->prepared || task->context.sequence || task->source_borrows ||
        task->linux_task || !task->source_original_freed ||
        (task->source_c_published ?
            !(task->source_native_returned && task->source_scheduler_refs_released && task->source_rcu_joined) :
            !task->source_unwound) || stack_slot(task) != STACK_TASKS)
        return -EBUSY;
    list_del_init(&task->allocation_node);
    struct stela_linux_managed_lease *managed = task->managed_birth_owner;
    task->managed_birth_owner = NULL;
    if (managed) {
        /* ALL existing source/native/observer/data/RCU joins above independently
         * completed. This bounded record+wake is no callback/task completion
         * substitute and cannot invoke cleanup inline. Root ref remains kept. */
        stela_linux_managed_native_retired(managed, owner);
        stela_linux_managed_lease_put(managed, owner);
    }
    stela_linux_heap_free(task, sizeof(*task), __alignof__(*task));
    return 0;
}

static void free_bare_record(struct task_struct *linux_task);
void stela_linux_task_release(struct stela_linux_task *task)
{
	if (task && task->kind != BARE_ALLOCATION) {
		__builtin_trap(); /* original source destructor has a separate owner */
	}
	if (!task || task->attached || task->active || task->suspended || task->scope ||
	    task->prepared || !quiet(task))
		__builtin_trap();
	if (stack_slot(task) != STACK_TASKS)
		__builtin_trap();
	/* Consume the actual retained baseline only after independent detach and
	 * unique Rust metadata ownership; extra public C refs must already end. */
	task->finalizing = true;
	if (!refcount_dec_and_test(&task->linux_task->usage))
		__builtin_trap();
	free_bare_record(task->linux_task);
}

static void free_bare_record(struct task_struct *linux_task)
{
	struct stela_linux_task *task = allocated_task(linux_task);
	if (!task || task->kind != BARE_ALLOCATION || !task->finalizing || task->attached || task->active || task->suspended || task->scope ||
	    task->prepared || task->context.sequence || !quiet_except_usage(task) ||
	    refcount_read(&linux_task->usage))
		__builtin_trap();
	list_del_init(&task->allocation_node);
	stela_linux_heap_free(task, sizeof(*task), __alignof__(*task));
}

/* Original fork.c owns the sole exported __put_task_struct and its actual
 * RCU callback. Historical bare fixture storage is never adopted as a genuine
 * original allocation; canonical claim forbids creating it in production. */

/* Mechanically retained applied sched/core.c worker effects. Original WQ
 * pools/rescuers/managers own concurrency; native task scheduling does not
 * replace them with a compatibility pool or opportunistic retry. */
static inline void native_submit_work(struct task_struct *tsk)
{
	static DEFINE_WAIT_OVERRIDE_MAP(sched_map, LD_WAIT_CONFIG);
	unsigned int task_flags;

	/*
	 * Establish LD_WAIT_CONFIG context to ensure none of the code called
	 * will use a blocking primitive -- which would lead to recursion.
	 */
	lock_map_acquire_try(&sched_map);

	task_flags = tsk->flags;
	/*
	 * If a worker goes to sleep, notify and ask workqueue whether it
	 * wants to wake up a task to maintain concurrency.
	 */
	if (task_flags & PF_WQ_WORKER)
		wq_worker_sleeping(tsk);
	else if (task_flags & PF_IO_WORKER)
		io_wq_worker_sleeping(tsk);

	/*
	 * spinlock and rwlock must not flush block requests.  This will
	 * deadlock if the callback attempts to acquire a lock which is
	 * already acquired.
	 */
	WARN_ON_ONCE(current->__state & TASK_RTLOCK_WAIT);

	/*
	 * If we are going to sleep and we have plugged IO queued,
	 * make sure to submit it to avoid deadlocks.
	 */
	blk_flush_plug(tsk->plug, true);

	lock_map_release(&sched_map);
}

static void native_update_worker(struct task_struct *tsk)
{
	if (tsk->flags & (PF_WQ_WORKER | PF_IO_WORKER)) {
		if (tsk->flags & PF_WQ_WORKER)
			wq_worker_running(tsk);
		else
			io_wq_worker_running(tsk);
	}
}

static int finish_prepared(struct stela_linux_task *task, int result)
{
	if (!live_here(task, &task->context) || !prepared_here(task))
		__builtin_trap();
	task->prepared = false;
	task->prepared_count = 0;
	/* The task runs again, perhaps after a long idle: its jiffies are now. */
	stela_linux_jiffies_refresh();
	return result;
}

/* Original schedule still crosses its scheduler when a signal/genuine native
 * wake prevents blocking. Returning directly would starve the real service
 * under persistent TASK_INTERRUPTIBLE signals (notably strict min sleeps).
 * The scoped native yield physically returns to dispatcher/new step. */
static int runnable_checkpoint(struct stela_linux_task *task)
{
    if (!prepared_here(task))
        __builtin_trap();
    int count = task->prepared_count;
    /* This private ref was authenticated by prepare's explicit scheduler
     * boundary, never guessed from an arbitrary reader's numeric nesting. */
    if (count) {
        preempt_count_sub(count);
        task->prepared_count = 0;
    }
    clear_task_blocked_on(task->linux_task, NULL);
    wake_account(task);
    WRITE_ONCE(task->linux_task->__state, TASK_RUNNING);
    int result = task->scope_ops->yield(task->scope);
    if (result)
        __builtin_trap();
    if (count) {
        preempt_count_add(count);
        task->prepared_count = count;
    }
    return 0; /* runnable checkpoint, not an actual blocked wait */
}

static int task_park(struct stela_linux_task *task, bool scheduler_preempt_disabled,
		     bool service_idle)
{
	/* As Linux's sched_submit_work, before the switch is prepared. */
	if (!task->rt_prepared)
		native_submit_work(task->linux_task);
	int error = prepare(task, scheduler_preempt_disabled ? 1 : 0);
	if (error) {
		if (!task->rt_prepared)
			native_update_worker(task->linux_task);
		return error;
	}
	/* Same selected scheduler effect as try_to_block_task: a signal posted
	 * by a process PREPARE retirement callback must not be lost. Native IRQ
	 * views only publish generic pending wake, never mutate Linux sigsets. */
	smp_mb();
    int result;
	if (signal_pending_state(READ_ONCE(task->linux_task->__state), task->linux_task)) {
        /* Preserve original signal->RUNNING semantics, then genuine native
         * scheduling checkpoint rather than a skipped-park C busy loop. */
        result = runnable_checkpoint(task);
    } else {
        result = task->scope_ops->park(task->scope, service_idle);
        if (result > 1)
            __builtin_trap();
        /* An actual native pending wake can legitimately skip block without
         * entering the C wake helper. It still owes the source scheduler
         * checkpoint, and makes the current source task runnable as on arrive. */
        if (result == 0)
            result = runnable_checkpoint(task);
    }
	result = finish_prepared(task, result);
	if (!task->rt_prepared)
		native_update_worker(task->linux_task);
	return result;
}

int stela_linux_task_park(bool scheduler_preempt_disabled)
{
	return task_park(current_record(), scheduler_preempt_disabled, false);
}

/* Selected !SMP original wait contract on the actual retained native UP
 * runtime. This replaces neither task birth/current nor the original state
 * machine. No rq flag/counter is accepted as a physical-switch receipt. */
unsigned long wait_task_inactive(struct task_struct *p, unsigned int match_state)
{
    struct stela_linux_task *caller = current_record();
    (void)stela_linux_owner();
    if (stela_linux_atomic_callback_active() || preempt_count() || irqs_disabled() ||
        !caller->scope || caller->scope_boundary || caller->prepared ||
        caller->suspended || !caller->scope_ops ||
        !caller->scope_ops->observe_inactive || !caller->scope_ops->end_inactive ||
        READ_ONCE(caller->linux_task->__state) != TASK_RUNNING)
        __builtin_trap();
    if (!match_state)
        return 0; /* original mask0 cannot match either original source state */
    /* p is already borrowed by the original kthread owner. Membership and
     * positive usage only reject foreign/dead inputs; they never adopt p. */
    struct stela_linux_task *target = allocated_task(p);
    if (!target || target == caller || !refcount_read(&p->usage) ||
        !target->attached || !target->context.sequence || !target->ops ||
        !target->ops->retain_inactive)
        __builtin_trap();
    u64 sequence = target->context.sequence;
    /* This is a record LIFETIME pin, not a data/RefCell/entered-process loan.
     * Original task usage independently keeps p through the physical wait. */
    source_borrow(target);
    get_task_struct(p);
    unsigned long cookie = 0;
    for (;;) {
        /* A mismatched original state returns0 even if the native assignment
         * has since closed. This preliminary check only refuses success; the
         * definitive matched check remains below under the same original lock. */
        unsigned long early_flags;
        raw_spin_lock_irqsave(&p->pi_lock, early_flags);
        bool matches = (READ_ONCE(p->__state) & match_state) ||
            (READ_ONCE(p->saved_state) & match_state);
        raw_spin_unlock_irqrestore(&p->pi_lock, early_flags);
        if (!matches)
            break;
        void *owner = NULL;
        int error = target->ops->retain_inactive(target->binding, sequence, &owner);
        if (error || !owner)
            __builtin_trap(); /* actual C refs + returned opaque error owner kept */
        unsigned long flags;
        raw_spin_lock_irqsave(&p->pi_lock, flags);
        unsigned int state = READ_ONCE(p->__state);
        unsigned int saved = READ_ONCE(p->saved_state);
        int match = (state & match_state) ? 1 : ((saved & match_state) ? -1 : 0);
        int observed = 0;
        if (match) {
            /* Source lock and genuine IF exclusion stay held THROUGH native
             * same-assignment/physical/pending recheck and original cookie. */
            bool force_wait = match < 0 || READ_ONCE(p->on_cpu) || READ_ONCE(p->on_rq);
            observed = caller->scope_ops->observe_inactive(caller->scope, owner, force_wait);
            if (observed == 1)
                cookie = READ_ONCE(p->nvcsw) | (1UL << (sizeof(unsigned long) * 8 - 1));
        }
        raw_spin_unlock_irqrestore(&p->pi_lock, flags);
        if (observed < 0 || observed > 1)
            __builtin_trap(); /* whole opaque owner retains precise native cause */
        if (!match || observed == 1) {
            if (caller->scope_ops->end_inactive(caller->scope, owner))
                __builtin_trap();
            break;
        }
        /* Already armed. A wake before state assignment remains pending; the
         * genuine park final check skips/checkpoints instead of losing it.
         * No pi_lock/queue/Task/native loan/IF guard spans this physical wait. */
        set_current_state(TASK_UNINTERRUPTIBLE);
        error = stela_linux_task_park(false);
        if (error)
            __builtin_trap(); /* actual wait frame/error + whole owner kept */
        __set_current_state(TASK_RUNNING);
        if (caller->scope_ops->end_inactive(caller->scope, owner))
            __builtin_trap();
    }
    put_task_struct(p);
    source_return(target); /* final record use; possible tail notification only */
    return cookie; /* mismatch0 or one temporal nvcsw|MSB observation, never death */
}

int stela_linux_task_yield(void)
{
	struct stela_linux_task *task = current_record();
	int error = prepare(task, 0);
	if (error)
		return error;
	int result = task->scope_ops->yield(task->scope);
	if (result > 0)
		__builtin_trap();
	return finish_prepared(task, result);
}

int __cond_resched(void)
{
	struct stela_linux_task *task = current_record();
	(void)stela_linux_owner();
	/* A worker calls this after each work item, with its function still set. */
	stela_linux_count_work();
	/* Legal nonblocking no-op in critical context, before provider/drain or
	 * flag effects. The private guard survives public counter rebalance. */
	if (stela_linux_atomic_callback_active() || preempt_count() || irqs_disabled())
		return 0;
	/* irq_work the item raised, which the worker owes before its next. */
	stela_linux_irq_work_run_owed();
	if (!task->scope || task->scope_boundary || task->prepared || task->suspended ||
	    READ_ONCE(task->linux_task->__state) != TASK_RUNNING)
		__builtin_trap();
	bool forced = test_tsk_thread_flag(task->linux_task, TIF_NEED_RESCHED);
	clear_tsk_thread_flag(task->linux_task, TIF_NEED_RESCHED);
	bool requested = task->scope_ops->reschedule_requested(task->scope);
	if (!forced && !requested)
		return 0;
	if (prepare(task, 0))
		__builtin_trap();
	int result = task->scope_ops->reschedule(task->scope);
	if (result || finish_prepared(task, result))
		__builtin_trap();
	return 1;
}

/* A delay in preemptible task context is a preemption point, as it is in a
 * preemptible kernel: the dispatcher's caller (a GPU pump past its budget, or
 * with a core waiting for its lock) may end its step here and resume the task
 * on a later step. Spinlocks, RCU readers and disabled interrupts keep
 * preempt_count or the IRQ flag set, so they never reach a switch; a task
 * between set_current_state and schedule keeps its delay plain. Unlike
 * __cond_resched this never traps: what it cannot switch stays a delay. */
void stela_linux_delay_point(void)
{
	if (!__atomic_load_n(&active, __ATOMIC_ACQUIRE) || !stela_linux_quiescent())
		return;
	struct stela_linux_task *task = current_record();
	if (!task->scope || task->scope_boundary || task->prepared || task->suspended ||
	    READ_ONCE(task->linux_task->__state) != TASK_RUNNING)
		return;
	/* prepare's own refusals, checked first, so a request the query
	 * consumes is followed by its switch. */
	struct stela_linux_lease lease;
	stela_linux_lease_snapshot(&lease);
	if (!live_here(task, &task->context) || !lease.owner ||
	    lease.irq_flags != ARCH_IRQ_ENABLED)
		return;
	if (!task->scope_ops->reschedule_requested(task->scope))
		return;
	/* Not reached after the checks above; were it, the request stays for
	 * the dispatcher's next step, which inspects its domain itself. */
	if (prepare(task, 0))
		return;
	(void)finish_prepared(task, task->scope_ops->reschedule(task->scope));
}

void yield(void)
{
	__set_current_state(TASK_RUNNING);
	schedule();
}

/* The whole original mutex uses precisely this scheduler boundary. Invalid
 * outer atomic/read-side nesting is a contract violation, not busy polling. */
void schedule_preempt_disabled(void)
{
    /* Preserve original source preempt ownership: allow real sched_submit_work
     * before the native wait, then return with exactly the caller's one ref. */
    if (preempt_count() != 1)
        __builtin_trap();
    sched_preempt_enable_no_resched();
    schedule();
    preempt_disable();
}

/* The selected UP source may request a real scheduler checkpoint. */
void resched_cpu(int cpu)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    if (cpu != 0 || stela_linux_require_current(owner))
        stela_linux_fatal("reschedule without genuine UP current");
    set_tsk_thread_flag(current, TIF_NEED_RESCHED);
}

/* Generic process waits (including original rcuwait) enter at preempt0.
 * They never borrow the normal-mutex scheduler level or infer an RT wait.
 * A running caller yields fairly; a sleeping caller uses the same real
 * PREPARE/drain, signal recheck and atomic predicate/park transition. */
static int schedule_current(bool service_idle)
{
	struct stela_linux_task *record = current_record();
	const struct stela_linux_owner *owner = stela_linux_owner();
	struct task_struct *task = record->linux_task;
	unsigned int state = READ_ONCE(task->__state);
	if (service_idle && (record->kind != INITIAL_ALLOCATION ||
	    state != TASK_UNINTERRUPTIBLE ||
	    stela_linux_require_current(owner) ||
	    !record->ops->service_reserved(record->binding)))
		return -EPERM;
	bool admitted_state = state == TASK_RUNNING || state == TASK_INTERRUPTIBLE ||
		state == TASK_UNINTERRUPTIBLE || state == TASK_KILLABLE || state == TASK_IDLE ||
        (state == TASK_PARKED && record->kind == ORIGINAL_CHILD_ALLOCATION &&
         stela_linux_kthread_current_started() == 0);
	if (stela_linux_atomic_callback_active() || preempt_count() || irqs_disabled() ||
	    !admitted_state ||
	    (task->flags & (PF_KTHREAD | PF_IO_WORKER)) != PF_KTHREAD || task->plug)
		__builtin_trap();
	int result = state == TASK_RUNNING ? stela_linux_task_yield() :
		task_park(record, false, service_idle);
	if (result < 0)
		return result;
    if (state == TASK_RUNNING && !record->rt_prepared)
        native_update_worker(task);
	/* A pending immutable Runtime wake can skip the actual switch without
	 * a C wake_up_process/arrive hook. Scheduler return is still runnable. */
	__set_current_state(TASK_RUNNING);
	return 0;
}

void schedule(void)
{
	if (schedule_current(false))
		__builtin_trap();
}

int stela_linux_task_park_service(void)
{
	return schedule_current(true);
}

/* Original RT-mutex slow paths submit work before publishing their waiter.
 * Creating another worker after pi_blocked_on is visible could recurse into
 * a second RT wait by this task, so the generic park skips that work here. */
void rt_mutex_pre_schedule(void)
{
    struct stela_linux_task *task = current_record();
    if (task->rt_prepared || task->linux_task->sched_rt_mutex || task->linux_task->pi_blocked_on)
        __builtin_trap();
    task->linux_task->sched_rt_mutex = 1;
    native_submit_work(task->linux_task);
    task->rt_submit_work = true;
    task->rt_prepared = true;
}

void rt_mutex_post_schedule(void)
{
    struct stela_linux_task *task = current_record();
    if (!task->rt_prepared || !task->rt_submit_work ||
        !task->linux_task->sched_rt_mutex || task->linux_task->pi_blocked_on)
        __builtin_trap();
    native_update_worker(task->linux_task);
    task->rt_prepared = false;
    task->rt_submit_work = false;
    task->linux_task->sched_rt_mutex = 0;
}

void rt_mutex_schedule(void)
{
    struct stela_linux_task *task = current_record();
    if (!task->rt_prepared || !task->linux_task->sched_rt_mutex)
        __builtin_trap();
    schedule();
}

/* Original futex context expressly skips worker/plug submission and update.
 * Keep that distinction; it is not an alias of normal RT mutex preparation. */
void rt_mutex_futex_pre_schedule(void)
{
    struct stela_linux_task *task = current_record();
    if (task->rt_prepared || task->linux_task->sched_rt_mutex ||
        task->linux_task->flags & (PF_WQ_WORKER | PF_IO_WORKER) || task->linux_task->plug)
        __builtin_trap();
    task->linux_task->sched_rt_mutex = 1;
    task->rt_submit_work = false;
    task->rt_prepared = true;
}
void rt_mutex_futex_post_schedule(void)
{
    struct stela_linux_task *task = current_record();
    if (!task->rt_prepared || task->rt_submit_work ||
        !task->linux_task->sched_rt_mutex || task->linux_task->pi_blocked_on)
        __builtin_trap();
    task->rt_prepared = false;
    task->linux_task->sched_rt_mutex = 0;
}

/* Selected native UP affinity has one eligible CPU and no migration path.
 * The caller borrows a live source task; catalog membership alone does not
 * acquire a task reference. Preserve the original pi_lock publication. */
static bool up_affinity_target(struct task_struct *p)
{
	const struct stela_linux_owner *owner = stela_linux_owner();
	if (stela_linux_require_current(owner) || stela_linux_atomic_callback_active())
		return false;
	struct stela_linux_task *target = allocated_task(p);
	return target && target->attached && target->context.sequence &&
	       refcount_read(&p->usage) && (p->flags & PF_KTHREAD) && !p->mm &&
	       (target->kind == INITIAL_ALLOCATION ||
		(target->kind == ORIGINAL_CHILD_ALLOCATION && target->source_owner == owner &&
		 target->source_visible && target->source_activated && !target->source_original_freed));
}

static int update_up_affinity(struct task_struct *p, const struct cpumask *mask)
{
	if (!mask || !cpumask_test_cpu(0, cpu_online_mask) || !cpumask_equal(mask, cpumask_of(0)))
		return -EINVAL;
	/* User affinity and temporary migration masks are outside this source
	 * profile. Refuse them before mutating any original scheduler fields. */
	if (p->user_cpus_ptr || p->cpus_ptr != &p->cpus_mask)
		return -EOPNOTSUPP;
	struct affinity_context context = { .new_mask = mask };
	set_cpus_allowed_common(p, &context);
	return 0;
}

int set_cpus_allowed_ptr(struct task_struct *p, const struct cpumask *mask)
{
	/* Authenticate current before touching a caller-supplied target lock. */
	if (!up_affinity_target(p))
		return -EPERM;
	guard(raw_spinlock_irqsave)(&p->pi_lock);
	return update_up_affinity(p, mask);
}

void set_cpus_allowed_force(struct task_struct *p, const struct cpumask *mask)
{
	/* Original kthread binds enter with the target pi_lock already held. */
	if (!irqs_disabled() || !preempt_count() || !up_affinity_target(p) ||
	    update_up_affinity(p, mask))
		stela_linux_fatal("unsupported forced UP affinity");
}

static struct stela_linux_task *priority_target(struct task_struct *p)
{
	(void)current_record();
	for (unsigned int slot = 0; slot < STACK_TASKS; slot++) {
		struct stela_linux_task *task = stack_tasks[slot];
		if (task && task->linux_task == p && task->attached && task->context.sequence)
			return task;
	}
	/* No caller-controlled task or an old job may change the queue's ranks. */
	__builtin_trap();
}

static int update_priority(struct stela_linux_task *task, unsigned int policy,
			   unsigned int realtime, int priority, int nice, bool head)
{
	struct task_struct *p = task->linux_task;
	struct stela_linux_task_rank before = {
		.policy = p->policy, .realtime = p->rt_priority, .effective = p->prio,
		.nice = task_nice(p),
	};
	struct stela_linux_task_rank after = {
		.policy = policy, .realtime = realtime, .effective = priority, .nice = nice,
	};
	if (before.policy == after.policy && before.realtime == after.realtime &&
	    before.effective == after.effective && before.nice == after.nice)
		return 0;
	if (!task->ops->rank)
		return -EOPNOTSUPP;
	return task->ops->rank(task->binding, task->context.sequence, &before, &after, head);
}

/* NORMAL workqueue priority remains nice-based; it never acquires the
 * strict real-time policy. Original PI may still boost its effective rank. */
void set_user_nice(struct task_struct *p, long nice)
{
	struct stela_linux_task *task = priority_target(p);
	if (nice < MIN_NICE || nice > MAX_NICE || nice == task_nice(p))
		return;
	guard(raw_spinlock_irqsave)(&p->pi_lock);
	int normal = p->policy == SCHED_FIFO ? p->normal_prio : NICE_TO_PRIO((int)nice);
	int priority = p->pi_top_task ? min(normal, p->pi_top_task->prio) : normal;
	int error = update_priority(task, p->policy, p->rt_priority, priority, (int)nice, false);
	if (error) {
		stela_linux_unsupported("NORMAL nice scheduling needs the retained kernel service domain");
		if (error != -EOPNOTSUPP)
			__builtin_trap();
		return;
	}
	p->static_prio = NICE_TO_PRIO((int)nice);
    /* The native rank/weight commit above owns scheduling. Keep genuine
     * source task-local load metadata coherent for original fork/averages;
     * this pure nofail leaf does not run CFS's unowned rq reweight path. */
    stela_linux_sched_load_weight(p);
	p->normal_prio = normal;
	p->prio = priority;
}

extern unsigned int sysctl_sched_base_slice;

int sched_setscheduler_nocheck(struct task_struct *p, int policy,
			       const struct sched_param *param)
{
	(void)stela_linux_owner();
	if (!param || param->sched_priority < 0 || param->sched_priority >= MAX_RT_PRIO)
		return -EINVAL;
	if (stela_linux_atomic_callback_active() || irqs_disabled())
		return -EPERM;

	struct stela_linux_task *task = priority_target(p);
	{
		guard(raw_spinlock_irqsave)(&p->pi_lock);
		bool reset = p->sched_reset_on_fork;
		if (policy < 0) {
			policy = (int)p->policy;
		} else {
			reset = (policy & SCHED_RESET_ON_FORK) != 0;
			policy &= ~SCHED_RESET_ON_FORK;
		}
		if (policy != SCHED_NORMAL && policy != SCHED_FIFO)
			return -EOPNOTSUPP;
		unsigned int realtime = (unsigned int)param->sched_priority;
		if ((policy == SCHED_FIFO) != (realtime != 0))
			return -EINVAL;
		int nice = task_nice(p);
		int normal = realtime ? MAX_RT_PRIO - 1 - (int)realtime : NICE_TO_PRIO(nice);
		int priority = p->pi_top_task ? min(normal, p->pi_top_task->prio) : normal;
		bool head = p->prio < priority && priority < MAX_RT_PRIO;
		int error = update_priority(task, (unsigned int)policy, realtime, priority, nice, head);
		if (error)
			return error;
		p->sched_reset_on_fork = reset;
		p->policy = (unsigned int)policy;
		p->rt_priority = realtime;
		p->normal_prio = normal;
		p->prio = priority;
		if (policy == SCHED_FIFO) {
			p->timer_slack_ns = 0;
		} else {
			if (!p->timer_slack_ns)
				p->timer_slack_ns = p->default_timer_slack_ns;
			if (p->se.custom_slice && p->se.slice)
				p->se.slice = clamp_t(u64, p->se.slice, NSEC_PER_MSEC / 10,
						     NSEC_PER_MSEC * 100);
			else {
				p->se.custom_slice = 0;
				p->se.slice = sysctl_sched_base_slice;
			}
		}
		stela_linux_sched_load_weight(p);
	}
	/* The original helper reorders a blocked waiter and propagates PI after
	 * the rank and task metadata commit. It takes pi_lock itself. */
	rt_mutex_adjust_pi(p);
	return 0;
}

void sched_set_fifo(struct task_struct *p)
{
	const struct sched_param param = {.sched_priority = MAX_RT_PRIO / 2};
	int error = sched_setscheduler_nocheck(p, SCHED_FIFO, &param);
	if (error == -EOPNOTSUPP)
		stela_linux_unsupported("FIFO scheduling needs the retained kernel service domain");
	else if (error)
		__builtin_trap();
}

/* Called by the original RT-mutex owner/waiter trees with p->pi_lock held.
 * Linux retains the donor/waiter; Runtime stores only its primitive rank.
 * Queue and original task fields commit together before that lease ends. */
void rt_mutex_setprio(struct task_struct *p, struct task_struct *pi_task)
{
	struct stela_linux_task *task = priority_target(p);
	int priority = pi_task ? min(p->normal_prio, pi_task->prio) : p->normal_prio;
	bool head = p->prio < priority && priority < MAX_RT_PRIO;
	if (update_priority(task, p->policy, p->rt_priority, priority, task_nice(p), head))
		__builtin_trap();
	p->pi_top_task = pi_task;
	p->prio = priority;
}

static int wake_state(struct task_struct *linux_task, unsigned int state)
{
	(void)current_record();
	(void)stela_linux_owner();
	/* Original API borrowed lifetime: a task ref, live waiter/lock scope or
	 * other real producer lease retains this actual job. A numeric extra
	 * reference is not universal ownership proof. The caller supplies that
	 * lifetime; positive usage only rejects an already-dead object. */
	struct stela_linux_task *task = allocated_task(linux_task);
	if (!task || !refcount_read(&task->linux_task->usage))
		__builtin_trap();
	guard(raw_spinlock_irqsave)(&linux_task->pi_lock);
	/* Full wake barrier pairs with original set_current_state before the
	 * genuine predicate/park. No preemptive Linux ISR can enter this port. */
	smp_mb();
	if (!(READ_ONCE(linux_task->__state) & state))
		return 0;
	/* A newly published runnable child can be woken before its first
	 * dispatch. Only a matching sleep state needs an attached wake target. */
	if (!task->attached || !task->context.sequence || !task->ops)
		__builtin_trap();
	/* Publish the generation-safe signal before changing C state. Failure
	 * under a held live ref is corruption and must not lose a wake. */
	if (task->ops->wake_target(task->binding, task->context.sequence))
		__builtin_trap();
	clear_task_blocked_on(linux_task, NULL);
	wake_account(task);
	linux_task->on_rq = 1;
	WRITE_ONCE(linux_task->__state, TASK_RUNNING);
	return 1;
}

int wake_up_process(struct task_struct *task)
{
	return wake_state(task, TASK_NORMAL);
}

int wake_up_state(struct task_struct *task, unsigned int state)
{
	return wake_state(task, state);
}

/* Linux's wake core (kernel/sched/core.c). Its flags only hint at the CPU
 * that runs the woken task. */
int try_to_wake_up(struct task_struct *task, unsigned int state, int wake_flags)
{
	(void)wake_flags;
	return wake_state(task, state);
}

int default_wake_function(struct wait_queue_entry *curr, unsigned mode, int wake_flags, void *key)
{
	(void)key;
	return try_to_wake_up(curr->private, mode, wake_flags);
}

void io_schedule(void)
{
	int token = io_schedule_prepare();
	schedule();
	io_schedule_finish(token);
}


static int wait_signal(unsigned int kind)
{
	switch (kind) {
	case 1: return SIGUSR1;
	case 2: return SIGKILL;
	default: return 0;
	}
}

int stela_linux_task_signal(struct stela_linux_task *task, u64 expected_sequence,
	unsigned int kind)
{
	(void)stela_linux_current();
	/* Metadata is separately retained by TargetTask. Inspect only generation
	 * until it matches; stale handles cannot inspect a future job's state. */
	if (!expected_sequence || task->context.sequence != expected_sequence)
		return -ESTALE;
	int signal_number = wait_signal(kind);
	if (!signal_number)
		return -EINVAL;
	if (!task->attached)
		return -ESTALE;
	/* Wait cancellation requires a live body, never its returned owner. */
	if (!task->scope || task->scope_boundary)
		return -EBUSY;
	/* Reserved Timer/IRQ service shutdown is explicit stop/drain/join, not
	 * ordinary wait cancellation. Refuse before refs, signal bits or wakes. */
	if (task->ops->service_reserved(task->binding))
		return -EOPNOTSUPP;
	get_task_struct(task->linux_task);
	sigaddset(&task->linux_task->pending.signal, signal_number);
	set_tsk_thread_flag(task->linux_task, TIF_SIGPENDING);
	unsigned int state = TASK_INTERRUPTIBLE | (kind == 2 ? TASK_WAKEKILL : 0U);
	(void)wake_up_state(task->linux_task, state);
	put_task_struct(task->linux_task);
	return 0;
}

int stela_linux_task_ack_signal(unsigned int kind)
{
	struct task_struct *task = stela_linux_current();
	int signal_number = wait_signal(kind);
	if (!signal_number)
		return -EINVAL;
	sigdelset(&task->pending.signal, signal_number);
	if (sigisemptyset(&task->pending.signal))
		clear_tsk_thread_flag(task, TIF_SIGPENDING);
	return 0;
}

unsigned int stela_linux_task_pending_signals(void)
{
	struct task_struct *task = stela_linux_current();
	return (sigismember(&task->pending.signal, SIGUSR1) ? 1U : 0U) |
	       (sigismember(&task->pending.signal, SIGKILL) ? 2U : 0U);
}

unsigned int stela_linux_task_waiting_io(void)
{
	(void)stela_linux_current();
	return waiting_io;
}

unsigned int stela_linux_task_waiting_load(void)
{
	(void)stela_linux_current();
	return waiting_load;
}
