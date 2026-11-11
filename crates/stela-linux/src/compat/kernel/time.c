// SPDX-License-Identifier: GPL-2.0-or-later
/* Nominal timekeeping is data. Physical minima use a separately retained
 * actual source, and sleep through the genuine service/scheduler IRQ path. */
#include <linux/delay.h>
#include <linux/limits.h>
#include <linux/hrtimer.h>
#include <linux/sched/signal.h>
#include <stela/task.h>
#include <stela/time.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/preempt.h>
#include <linux/timekeeping.h>
#include <stela/linux.h>

/* Minimum admission is independent of full subsystem readiness. Authenticating
 * current preserves original Initial/Managed provenance even in atomic scope. */
struct task_struct *stela_linux_minimum_task(const struct stela_linux_owner *owner)
{
    if (stela_linux_owner() != owner || stela_linux_require_current(owner))
        stela_linux_fatal("physical minimum without actual original current");
    return current;
}

static void atomic_minimum_us(u64 minimum)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    stela_linux_minimum_task(owner);
    if (!owner->services->minimum_delay_us)
        stela_linux_fatal("missing actual atomic minimum provider");
    owner->services->minimum_delay_us(owner->context, minimum);
    stela_linux_delay_point();
}

static const struct stela_linux_owner *sleep_context(unsigned int state,
    enum stela_linux_minimum_wake wake, u64 quantum_us)
{
    const struct stela_linux_owner *owner = stela_linux_owner();
    stela_linux_minimum_task(owner);
    bool admitted = state == TASK_RUNNING || state == TASK_INTERRUPTIBLE ||
        state == TASK_UNINTERRUPTIBLE || state == TASK_KILLABLE || state == TASK_IDLE;
    if (!stela_linux_quiescent() || !admitted || !quantum_us ||
        (unsigned int)wake > (unsigned int)STELA_MINIMUM_SIGNAL)
        stela_linux_fatal("invalid sleepable physical minimum context");
    return owner;
}

u64 stela_linux_minimum_sleep(u64 minimum_us, unsigned int state,
                            enum stela_linux_minimum_wake wake, u64 quantum_us)
{
    const struct stela_linux_owner *owner = sleep_context(state, wake, quantum_us);
    if (!owner->services->minimum_deadline_us)
        stela_linux_fatal("missing actual relative minimum provider");
    struct stela_linux_minimum_deadline *deadline =
        owner->services->minimum_deadline_us(owner->context, minimum_us);
    if (!deadline || !deadline->ready || !deadline->wake_ns ||
        !deadline->remaining_quanta_ceil || !deadline->release)
        stela_linux_fatal("invalid actual minimum deadline producer");
    u64 remaining = 0;
    bool scheduled = false;
    for (;;) {
        __set_current_state(state);
        if (deadline->ready(deadline))
            break;
        ktime_t hint = deadline->wake_ns(deadline);
        if (hint <= ktime_get()) {
            if (deadline->ready(deadline))
                break;
            stela_linux_fatal("physical minimum source/scheduler clock mismatch");
        }
        /* The genuine task scheduler checkpoint must physically return a
         * runnable/signal-pending continuation to its Runtime dispatcher.
         * This source depends on that producer; a skipped native park cannot
         * substitute a full-duration userspace/counter polling loop. */
        int result = schedule_hrtimeout(&hint, HRTIMER_MODE_ABS);
        scheduled = true;
        /* schedule_hrtimeout has synchronously canceled its exact timer before
         * this source check/release. A numeric expiry never grants success. */
        if (deadline->ready(deadline))
            break;
        if (result != 0 && result != -EINTR)
            stela_linux_fatal("physical minimum scheduler failure");
        if (result == -EINTR && (wake == STELA_MINIMUM_EARLY ||
            (wake == STELA_MINIMUM_SIGNAL && signal_pending(current)))) {
            remaining = deadline->remaining_quanta_ceil(deadline, quantum_us);
            /* The source can reach ready between the first check and its
             * remaining-counter sample. Zero still needs genuine ready. */
            if (!remaining && !deadline->ready(deadline))
                stela_linux_fatal("physical minimum early wake lost original counter target");
            u64 original_quanta = minimum_us / quantum_us + (minimum_us % quantum_us != 0);
            if (remaining > original_quanta)
                stela_linux_fatal("physical minimum remainder exceeds original request");
            break;
        }
    }
    __set_current_state(TASK_RUNNING);
    deadline->release(deadline);
    /* A sleep whose minimum passed before its first check still reaches the
     * scheduler, as Linux's always does. Otherwise a loop of short sleeps
     * polling a device never lets other tasks or the dispatcher run. */
    if (!scheduled)
        schedule();
    return remaining;
}

static u64 jiffy_minimum_us(u64 ticks)
{
    _Static_assert(USEC_PER_SEC % HZ == 0, "relative physical jiffy conversion");
    const u64 quantum = USEC_PER_SEC / HZ;
    if (!quantum || ticks > U64_MAX / quantum)
        stela_linux_fatal("physical jiffy minimum conversion overflow");
    return ticks * quantum;
}

/* Match the original rounded jiffy request, retaining ONE physical target
 * through every explicit wake rather than restarting a fresh relative delay. */
void msleep(unsigned int msecs)
{
    u64 ticks = msecs_to_jiffies(msecs);
    if (ticks)
        stela_linux_minimum_sleep(jiffy_minimum_us(ticks),
                                 TASK_UNINTERRUPTIBLE, STELA_MINIMUM_STRICT, USEC_PER_SEC / HZ);
}

unsigned long msleep_interruptible(unsigned int msecs)
{
    u64 ticks = msecs_to_jiffies(msecs);
    if (!ticks)
        return 0;
    if (signal_pending(current))
        return jiffies_to_msecs(ticks);
    u64 remaining = stela_linux_minimum_sleep(jiffy_minimum_us(ticks),
        TASK_INTERRUPTIBLE, STELA_MINIMUM_SIGNAL, USEC_PER_SEC / HZ);
    return jiffies_to_msecs(remaining);
}

void usleep_range_state(unsigned long min, unsigned long max, unsigned int state)
{
    /* Earliest allowed wake is a valid coalescing choice. The source has no
     * physical upper/UTC guarantee; Linux scheduler lateness is unbounded. */
    sleep_context(state, STELA_MINIMUM_STRICT, 1);
    WARN_ON_ONCE(max < min);
    stela_linux_minimum_sleep(min, state, STELA_MINIMUM_STRICT, 1);
}

void __udelay(unsigned long usecs)
{
    atomic_minimum_us(usecs);
}

void __ndelay(unsigned long nsecs)
{
    u64 usecs = (u64)nsecs / NSEC_PER_USEC + ((u64)nsecs % NSEC_PER_USEC != 0);
    atomic_minimum_us(usecs);
}

/* Exact conservative ceil(xloops * USEC_PER_SEC / 2^32), without an
 * overflowing 64-bit multiply or a saturating/zero-length success. */
void __const_udelay(unsigned long xloops)
{
    u64 whole = ((u64)xloops >> 32) * USEC_PER_SEC;
    u64 part = ((u64)xloops & U32_MAX) * USEC_PER_SEC;
    u64 usecs = whole + (part >> 32) + ((part & U32_MAX) != 0);
    atomic_minimum_us(usecs);
}

unsigned long __msecs_to_jiffies(const unsigned int m)
{
	return m > jiffies_to_msecs(MAX_JIFFY_OFFSET) ? MAX_JIFFY_OFFSET : _msecs_to_jiffies(m);
}

u64 nsecs_to_jiffies64(u64 n)
{
	return div_u64(n, NSEC_PER_SEC / HZ);
}

unsigned long nsecs_to_jiffies(u64 n)
{
	return (unsigned long)nsecs_to_jiffies64(n);
}

struct timespec64 ns_to_timespec64(s64 nsec)
{
	s32 remainder;
	s64 seconds = div_s64_rem(nsec, NSEC_PER_SEC, &remainder);

	if (remainder < 0) {
		seconds--;
		remainder += NSEC_PER_SEC;
	}
	return (struct timespec64){.tv_sec = seconds, .tv_nsec = remainder};
}

unsigned long __usecs_to_jiffies(const unsigned int u)
{
	return u > jiffies_to_usecs(MAX_JIFFY_OFFSET) ? MAX_JIFFY_OFFSET : _usecs_to_jiffies(u);
}
