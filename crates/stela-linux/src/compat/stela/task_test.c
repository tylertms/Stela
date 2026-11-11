// SPDX-License-Identifier: GPL-2.0-or-later
/* Callable mechanics fixtures. These use real Runtime stacks/current tasks;
 * the separately retained Rust test CPU is a hosted model, not native entry. */
#include <linux/bottom_half.h>
#include <linux/errno.h>
#include <linux/irqflags.h>
#include <linux/preempt.h>
#include <linux/rcupdate.h>
#include <linux/rcuwait.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/task_stack.h>
#include <linux/string.h>
#include <stela/linux.h>
#include <stela/atomic_callback.h>
#include <stela/task.h>
#include <stela/test.h>

void stela_linux_task_test_taskless(void);
void stela_linux_task_test_unprepared_begin(void);
void stela_linux_task_test_unprepared_end(void);
void stela_linux_task_test_foreground(void *context);
void stela_linux_task_test_worker(void *context);
void stela_linux_task_test_schedule_worker(void *context);
void stela_linux_task_test_pending_schedule(void *context);
void stela_linux_task_test_service_wait(void *context);
void stela_linux_task_test_service_cancel(void *context);
void stela_linux_task_test_clock_service(void *context);
void stela_linux_task_test_runnable_boundary(void *context);
void stela_linux_task_test_closed_stack(void);
void stela_linux_atomic_callback_fixture(void);
void stela_linux_atomic_callback_taskless_fixture(void);

static struct task_struct *foreground_task;
static bool worker_done;
static bool schedule_worker_ran;
static bool schedule_worker_done;
static struct task_struct *schedule_worker;
static unsigned int refused_schedule_state;
static struct rcuwait *published_wait;
static bool wait_condition, wait_worker_done, idle_worker_done;
static unsigned int wait_payload;
static struct task_struct *reserved_service_task;
static struct task_struct *scope_end_target;
static void *closed_stack;

struct service_call {
	void *context;
	void (*cancel_and_wake)(void *context);
};
struct clock_service_call {
	void *context;
	bool (*boundary)(void *context);
};

static void taskless_stack_owner(void)
{
	(void)stela_linux_task_stack_owner(NULL, 1);
}

static void taskless_current(void)
{
	(void)READ_ONCE(current->flags);
}

void stela_linux_task_test_taskless(void)
{
	assert(stela_test_traps(taskless_current));
	assert(stela_test_traps(taskless_stack_owner));
	stela_linux_atomic_callback_taskless_fixture();
}

static const struct stela_linux_services unprepared_services;
static const struct stela_linux_owner unprepared_owner = {.services = &unprepared_services};

void stela_linux_task_test_unprepared_begin(void)
{
	stela_linux_enter(&unprepared_owner);
	assert(stela_linux_quiescent());
	assert(current->flags == PF_KTHREAD);
}

void stela_linux_task_test_unprepared_end(void)
{
	assert(stela_linux_owner() == &unprepared_owner);
	assert(stela_linux_quiescent());
	assert(current->flags == PF_KTHREAD);
	stela_linux_exit();
}

/* A real pending signal, as a signal sender leaves it. */
static void post_signal(void)
{
	sigaddset(&current->pending.signal, SIGUSR1);
	set_tsk_thread_flag(current, TIF_SIGPENDING);
}

static void check_current(void)
{
	struct task_struct *task = current;
	unsigned long pointer;
	asm volatile("mov %%rsp, %0" : "=r"(pointer));
	assert(task_thread_info(task) == current_thread_info());
	assert(task->thread_info.task == task);
	assert(task->flags == PF_KTHREAD);
	assert(task->stack && pointer >= (unsigned long)task->stack);
	assert(pointer - (unsigned long)task->stack < THREAD_SIZE);
	assert(!task_stack_end_corrupted(task));
	assert(refcount_read(&task->stack_refcount) == 1);
	unsigned long local = 0;
	assert(stela_linux_task_stack_owner(&local, sizeof(local)) == task);
	assert(!stela_linux_task_stack_owner(&local, 0));
	assert(!stela_linux_task_stack_owner((void *)(~0UL - 3), 8));
	assert(!stela_linux_task_stack_owner((void *)((unsigned long)task->stack - 1), 2));
	assert(!stela_linux_task_stack_owner((void *)((unsigned long)task->stack + THREAD_SIZE - 1), 2));
	assert(!stela_linux_task_stack_owner(task, sizeof(*task)));
}

static void unsupported_schedule_state(void)
{
	set_current_state(refused_schedule_state);
	schedule();
}

static void original_rcuwait(void)
{
	/* This complete original wait lives on the actual foreground stack;
	 * its frame and both real tasks stay retained through original finish
	 * and the final RCU lifetime boundary. No replacement waiter/task. */
	struct rcuwait wait;
	rcuwait_init(&wait);
	assert(!rcuwait_active(&wait) && !rcuwait_wake_up(&wait));
	assert(stela_linux_task_stack_owner(&wait, sizeof(wait)) == current);
	published_wait = &wait;
	assert(wake_up_process(schedule_worker) == 1);
	long error = rcuwait_wait_event(&wait, READ_ONCE(wait_condition), TASK_UNINTERRUPTIBLE);
	assert(!error && wait_worker_done && READ_ONCE(wait_payload) == 0x12345678U);
	assert(!rcuwait_active(&wait) && !rcuwait_wake_up(&wait));
	assert(current == foreground_task && READ_ONCE(current->__state) == TASK_RUNNING);
	assert(!stela_linux_task_waiting_io() && !stela_linux_task_waiting_load());
	/* An already-satisfied condition and a pending signal both execute
	 * original finish, clearing the published pointer. */
	assert(!rcuwait_wait_event(&wait, true, TASK_UNINTERRUPTIBLE));
	assert(!rcuwait_active(&wait) && current->__state == TASK_RUNNING);
	post_signal();
	error = rcuwait_wait_event(&wait, false, TASK_INTERRUPTIBLE);
	assert(error == -EINTR && !rcuwait_active(&wait));
	assert(current->__state == TASK_RUNNING && stela_linux_task_pending_signals() == 1);
	assert(!stela_linux_task_ack_signal(1));
	published_wait = NULL;
	printf("Linux rcuwait: original wait/wake/finish, atomic delivery and signals passed\n");
}

static void original_idle_wait(void)
{
	/* The peer itself is already parked in exact TASK_IDLE on its actual
	 * retained Runtime frame. Waking it must not add a load-accounted sleeper. */
	assert(schedule_worker->__state == TASK_IDLE && schedule_worker->is_blocked);
	assert(!schedule_worker->sched_contributes_to_load);
	assert(!stela_linux_task_waiting_io() && !stela_linux_task_waiting_load());
	assert(wake_up_process(schedule_worker) == 1);
	set_current_state(TASK_IDLE);
	schedule();
	assert(idle_worker_done && current == foreground_task);
	assert(current->__state == TASK_RUNNING && !current->is_blocked);
	assert(!stela_linux_task_waiting_io() && !stela_linux_task_waiting_load());
	assert(stela_linux_quiescent() && !preempt_count() && !irqs_disabled());
}

void stela_linux_task_test_foreground(void *context)
{
	(void)context;
	const struct stela_linux_services services = {};
	const struct stela_linux_owner owner = {.services = &services};
	stela_linux_enter(&owner);
	check_current();
	stela_linux_atomic_callback_fixture();
	foreground_task = current;
	assert(stela_linux_quiescent());
	preempt_disable();
	assert(stela_linux_task_yield() == -EBUSY);
	assert(preempt_count() == 1 && current == foreground_task);
	preempt_enable();
	local_bh_disable();
	int bh_count = preempt_count();
	assert(stela_linux_task_yield() == -EBUSY);
	assert(preempt_count() == bh_count);
	local_bh_enable();
	local_irq_disable();
	assert(stela_linux_task_yield() == -EBUSY);
	assert(irqs_disabled());
	local_irq_enable();
	rcu_read_lock();
	assert(stela_linux_task_yield() == -EBUSY);
	assert(preempt_count() == 1);
	rcu_read_unlock();
	/* A TASK_RUNNING park is still a genuine scheduling checkpoint: the
	 * queued worker runs there and wakes this task before it blocks. */
	assert(!worker_done);
	assert(!stela_linux_task_park(false));
	assert(worker_done && current == foreground_task);
	/* That early wake is kept, so the next park skips blocking. This one
	 * known preempt level is this fixture's scheduler level, never an outer
	 * RCU/BH/IRQ critical section. Whole-original-mutex callers of
	 * schedule_preempt_disabled are exercised by the separate mutex suite. */
	set_current_state(TASK_UNINTERRUPTIBLE);
	preempt_disable();
	assert(!stela_linux_task_park(true));
	assert(preempt_count() == 1);
	assert(current == foreground_task && current->__state == TASK_RUNNING);
	assert(stela_linux_owner() == &owner);
	preempt_enable();
	assert(!stela_linux_task_yield());
	/* A signal posted after the caller's condition check: the real pending
	 * bit must cancel this interruptible park. */
	set_current_state(TASK_INTERRUPTIBLE);
	post_signal();
	schedule();
	assert(current == foreground_task && current->__state == TASK_RUNNING);
	assert(stela_linux_task_pending_signals() == 1);
	assert(!stela_linux_task_ack_signal(1));
	/* A separate real worker is now sleeping at generic preempt0. Wake its
	 * original task, then block here until its original wake returns us. */
	assert(schedule_worker && !schedule_worker_done);
	assert(wake_up_process(schedule_worker) == 1);
	/* A no-op RUNNING schedule would leave this peer checkpoint unset. */
	schedule();
	assert(schedule_worker_ran && !schedule_worker_done);
	assert(current == foreground_task && current->__state == TASK_RUNNING);
	assert(wake_up_process(schedule_worker) == 1);
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule();
	assert(schedule_worker_done);
	assert(current == foreground_task && current->__state == TASK_RUNNING);
	assert(!preempt_count() && !irqs_disabled() && stela_linux_owner() == &owner);
	/* TASK_RUNNING schedule is an actual cooperative yield, not a skip. */
	schedule();
	original_rcuwait();
	original_idle_wait();
	refused_schedule_state = TASK_WAKEKILL;
	assert(stela_test_traps(unsupported_schedule_state));
	refused_schedule_state = TASK_NORMAL;
	assert(stela_test_traps(unsupported_schedule_state));
	refused_schedule_state = TASK_NOLOAD;
	assert(stela_test_traps(unsupported_schedule_state));
	refused_schedule_state = TASK_IDLE | TASK_WAKEKILL;
	assert(stela_test_traps(unsupported_schedule_state));
	check_current();
	assert(stela_linux_quiescent());
	/* Scope-end must consume this real request through an actual dispatch. */
	set_tsk_thread_flag(current, TIF_NEED_RESCHED);
	stela_linux_exit();
}

void stela_linux_task_test_schedule_worker(void *context)
{
	(void)context;
	stela_linux_enter(&stela_linux_bare);
	check_current();
	schedule_worker = current;
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule();
	assert(foreground_task->__state == TASK_RUNNING);
	schedule_worker_ran = true;
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule();
	assert(current == schedule_worker && current->__state == TASK_RUNNING);
	assert(!preempt_count() && !irqs_disabled());
	assert(stela_linux_owner() == &stela_linux_bare);
	schedule_worker_done = true;
	assert(wake_up_process(foreground_task) == 1);
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule();
	assert(published_wait && rcu_access_pointer(published_wait->task) == foreground_task);
	assert(stela_linux_task_stack_owner(published_wait, sizeof(*published_wait)) == foreground_task);
	assert(!wait_condition && !wait_worker_done);
	struct stela_linux_atomic_callback delivery;
	stela_linux_atomic_callback_begin(&delivery);
	assert(stela_linux_task_stack_owner(published_wait, sizeof(*published_wait)) == foreground_task);
	WRITE_ONCE(wait_payload, 0x12345678U);
	WRITE_ONCE(wait_condition, true);
	assert(rcuwait_wake_up(published_wait) == 1);
	assert(!rcuwait_wake_up(published_wait));
	assert(rcuwait_active(published_wait));
	stela_linux_atomic_callback_end(&delivery);
	wait_worker_done = true;
	set_current_state(TASK_IDLE);
	schedule();
	assert(current == schedule_worker && current->__state == TASK_RUNNING);
	assert(foreground_task->__state == TASK_IDLE && foreground_task->is_blocked);
	assert(!foreground_task->sched_contributes_to_load);
	assert(!stela_linux_task_waiting_io() && !stela_linux_task_waiting_load());
	idle_worker_done = true;
	assert(wake_up_process(foreground_task) == 1);
	assert(wake_up_process(foreground_task) == 0);
	stela_linux_exit();
}

void stela_linux_task_test_worker(void *context)
{
	(void)context;
	const struct stela_linux_services services = {};
	const struct stela_linux_owner owner = {.services = &services};
	stela_linux_enter(&owner);
	check_current();
	assert(current != foreground_task);
	closed_stack = current->stack;
	assert(!preempt_count() && !irqs_disabled());
	assert(stela_linux_owner() == &owner);
	worker_done = true;
	stela_linux_exit();
}

void stela_linux_task_test_pending_schedule(void *context)
{
	(void)context;
	stela_linux_enter(&stela_linux_bare);
	struct task_struct *task = current;
	/* Rust published this actual job's immutable wake before C admission.
	 * No C task wake changes __state, and no arrive hook runs on a skip. */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule();
	assert(current == task && READ_ONCE(task->__state) == TASK_RUNNING);
	assert(stela_linux_quiescent() && !task->is_blocked);
	assert(!stela_linux_task_waiting_io() && !stela_linux_task_waiting_load());
	stela_linux_exit();
}

void stela_linux_task_test_service_wait(void *context)
{
	(void)context;
	stela_linux_enter(&stela_linux_bare);
	check_current();
	reserved_service_task = current;
	assert(!stela_linux_task_pending_signals());
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule();
	assert(current == reserved_service_task && current->__state == TASK_RUNNING);
	assert(!stela_linux_task_pending_signals() && refcount_read(&current->usage) == 1);
	reserved_service_task = NULL;
	stela_linux_exit();
}

void stela_linux_task_test_service_cancel(void *context)
{
	const struct service_call *call = context;
	stela_linux_enter(&stela_linux_bare);
	assert(reserved_service_task && reserved_service_task != current);
	assert(reserved_service_task->__state == TASK_UNINTERRUPTIBLE);
	assert(refcount_read(&reserved_service_task->usage) == 1);
	call->cancel_and_wake(call->context);
	/* Both cancellation classes were refused before pending/ref/wake effects.
	 * The permitted immutable generic wake alone does not mutate C state. */
	sigset_t pending = reserved_service_task->pending.signal;
	assert(sigisemptyset(&pending));
	assert(!test_tsk_thread_flag(reserved_service_task, TIF_SIGPENDING));
	assert(reserved_service_task->__state == TASK_UNINTERRUPTIBLE);
	assert(refcount_read(&reserved_service_task->usage) == 1);
	stela_linux_exit();
}

void stela_linux_task_test_clock_service(void *context)
{
	const struct clock_service_call *call = context;
	stela_linux_enter(&stela_linux_bare);
	check_current();
	while (!call->boundary(call->context)) {
		assert(stela_linux_quiescent());
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule();
		check_current();
	}
	if (scope_end_target) {
		sigset_t pending = scope_end_target->pending.signal;
		assert(sigisemptyset(&pending));
		assert(!test_tsk_thread_flag(scope_end_target, TIF_SIGPENDING));
		assert(refcount_read(&scope_end_target->usage) == 1);
		set_tsk_thread_flag(scope_end_target, TIF_NEED_RESCHED);
		scope_end_target = NULL;
	}
	assert(stela_linux_quiescent());
	stela_linux_exit();
}

struct runnable_boundary_call {
	void *context;
	u32 mode;
	void (*make_due)(void *context);
	u64 (*reads)(void *context);
	u64 (*ticks)(void *context);
};

void stela_linux_task_test_runnable_boundary(void *context)
{
	const struct runnable_boundary_call *call = context;
	stela_linux_enter(&stela_linux_bare);
	check_current();
	foreground_task = current;
	call->make_due(call->context);
	assert(!call->ticks(call->context));
	if (call->mode == 1) {
		u64 reads = call->reads(call->context);
		preempt_disable();
		assert(!__cond_resched());
		preempt_enable();
		local_irq_disable();
		assert(!__cond_resched());
		local_irq_enable();
		struct stela_linux_atomic_callback scope;
		stela_linux_atomic_callback_begin(&scope);
		assert(!__cond_resched());
		stela_linux_atomic_callback_end(&scope);
		assert(call->reads(call->context) == reads);
		/* The due service tick, then an explicit reschedule request. */
		assert(__cond_resched() == 1);
		assert(!test_tsk_thread_flag(current, TIF_NEED_RESCHED));
		set_tsk_thread_flag(current, TIF_NEED_RESCHED);
		assert(__cond_resched() == 1);
		assert(!test_tsk_thread_flag(current, TIF_NEED_RESCHED));
		assert(call->ticks(call->context) == 1);
		assert(current->__state == TASK_RUNNING && stela_linux_quiescent());
	}
	if (call->mode == 3) {
		/* A delay point never switches or queries where it cannot. */
		u64 reads = call->reads(call->context);
		preempt_disable();
		stela_linux_delay_point();
		preempt_enable();
		local_irq_disable();
		stela_linux_delay_point();
		local_irq_enable();
		set_current_state(TASK_UNINTERRUPTIBLE);
		stela_linux_delay_point();
		__set_current_state(TASK_RUNNING);
		assert(call->reads(call->context) == reads);
		assert(!call->ticks(call->context));
		/* In preemptible context it switches as cond_resched does. */
		stela_linux_delay_point();
		assert(call->ticks(call->context) == 1);
		assert(current->__state == TASK_RUNNING && stela_linux_quiescent());
	}
	if (call->mode == 2)
		scope_end_target = current;
	/* mode2 performs no cooperative switch in its body: scope-end must
	 * inspect overdue work independently after this actual owner exits. */
	stela_linux_exit();
}

void stela_linux_task_test_closed_stack(void)
{
	stela_linux_enter(&stela_linux_bare);
	assert(closed_stack && !stela_linux_task_stack_owner(closed_stack, 1));
	/* Same foreground backing is now a genuine new job. Range ownership is
	 * current again; this does not dereference or resurrect any old frame. */
	unsigned long local = 0;
	assert(stela_linux_task_stack_owner(&local, sizeof(local)) == current);
	stela_linux_exit();
}
