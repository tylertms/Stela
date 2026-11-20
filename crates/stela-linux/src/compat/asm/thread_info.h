/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_STELA_THREAD_INFO_H
#define _ASM_STELA_THREAD_INFO_H
/* Actual Runtime-bound Linux tasks use embedded metadata. Taskless leaf
 * services use independent explicit metadata and acquire no current task. */
#include <asm-generic/thread_info_tif.h>
#include <asm/page.h>

/* Exact admitted Linux usable stack size; Runtime retains the real backing. */
/* 64 KiB, four times x86-64 Linux's 16 KiB: Linux's own stacks hold only
 * Linux, while these also carry the host's log, storage and USB paths a
 * driver's printk reaches. amdgpu's probe on a Navi 48, at IP discovery,
 * overran 16 KiB there, and the overrun stopped the boot with no report.
 * stela_boot_core::native_resources::LINUX_TASK_STACK_BYTES is the same. */
#define THREAD_SIZE_ORDER 4
#define THREAD_SIZE (PAGE_SIZE << THREAD_SIZE_ORDER)

struct task_struct;

struct thread_info {
	struct task_struct *task;
	unsigned long flags;
	int preempt_count;
};

/* Original initial-task metadata starts with Linux's real initialization
 * preemption reference. This does not install current or enter a Runtime.
 * INIT_PREEMPT_COUNT is supplied by the pinned linux/preempt.h at expansion. */
#define INIT_THREAD_INFO(initial_task) { \
    .task = &(initial_task), \
    .flags = 0, \
    .preempt_count = INIT_PREEMPT_COUNT, \
}

extern struct thread_info stela_thread_info;
struct thread_info *stela_linux_thread_info(void);
/* Override Linux's CONFIG_THREAD_INFO_IN_TASK default at the arch boundary so
 * explicit taskless leaf bookkeeping never fabricates current. */
#undef current_thread_info
#define current_thread_info() stela_linux_thread_info()
#endif
