/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef STELA_LINUX_H
#define STELA_LINUX_H
/* Stela's kernel-API binding. Linux code runs only between stela_linux_enter
 * and stela_linux_exit, for one owner at a time: entering waits until no other
 * owner runs Linux code, and Linux code never enters again. That is the
 * uniprocessor, non-preemptible execution generated/autoconf.h configures.
 * Both boundaries require balanced preemption/BH/read-side state and enabled
 * simulated IRQs. Callbacks and critical sections cannot escape the borrowed
 * owner; exit invokes pending RCU callbacks before returning that owner.
 *
 * The owner supplies the services that depend on its device and platform.
 * Linux names no device for most of them (msleep, ktime_get, printk), so they
 * belong to whoever runs the code; device-scoped services, such as register
 * access, are bound by the adapter that owns the device. A NULL log or warn
 * discards those reports; a required service reached
 * without its callback, or outside enter and exit, traps. */
#include <linux/compiler_attributes.h>
#include <linux/types.h>

struct stela_linux_minimum_deadline;
struct stela_linux_services {
	/* Monotonic nanoseconds. */
	s64 (*now_ns)(void *context);
	/* Actual source-bound atomic minimum. Failure is terminal and retains
	 * every original owner; this void ABI must never return false success. */
	void (*minimum_delay_us)(void *context, u64 minimum);
	/* One owned relative physical deadline. No borrowed Rust execution or
	 * process scope spans any following C scheduler wait. See stela/time.h. */
	struct stela_linux_minimum_deadline *(*minimum_deadline_us)(void *context, u64 minimum);
	/* Real same-Environment RaisedWork notification after queue mutation.
	 * Bounded/IRQ-safe; post-arm failure is terminal with real queue retained. */
	void (*timer_changed)(void *context);
	/* One formatted kernel message at its printk level (LOGLEVEL_*). */
	void (*log)(void *context, int level, const char *message);
	/* A WARN that fired, with its condition text when Linux records one. */
	void (*warn)(void *context, const char *file, int line, const char *condition);
	/* An operation Stela does not provide yet; the caller then continues
	 * with that service's documented degraded result. An owner without this
	 * service traps instead. Called for every occurrence. */
	void (*unsupported)(void *context, const char *operation);
	/* Infallible bytes from an already seeded, retained CSPRNG. Kernel
	 * registration must obtain that real capability before admitting callers;
	 * a non-NULL callback alone is not seed/readiness evidence. It neither
	 * allocates, waits, switches owner/task nor changes the incoming atomic/IRQ
	 * state. The caller retains actual IRQ-safe non-migrating serialization;
	 * this table is not a native execution/interrupt grant. */
	void (*random_bytes)(void *context, void *bytes, size_t size);
	/* Power the machine off, without waiting for clean exits if force. */
	void (*shutdown)(void *context, bool force);
	/* Map page-aligned device memory uncached, or return NULL; unmapped by
	 * the address map_device returned. */
	void *(*map_device)(void *context, u64 physical, size_t size,
			    bool write_combining);
	void (*unmap_device)(void *context, void *address);
	/* Map count write-back pages contiguously, or return NULL. */
	void *(*map_pages)(void *context, const u64 *physical, size_t count);
	void (*unmap_pages)(void *context, void *address, size_t count);
	/* Action is already linked before enable. Initialize *unexposed=false;
	 * set true only with actual no-publication plus exact owner/event cleanup
	 * proof. An errno/disabled bit never implies that classification. */
	int (*irq_enable)(void *context, unsigned int irq, bool *unexposed);
	/* Reversible mask only; queued old delivery remains owned. */
	int (*irq_disable)(void *context, unsigned int irq);
	/* Exact action/root/CPU/service join after C action-thread joining.
	 * A negative result must not return through free_irq/reclaim dev_id. */
	int (*irq_retire)(void *context, unsigned int irq);
	/* Copy between the running client's memory and Linux's, returning the
	 * bytes left uncopied at a fault. */
	unsigned long (*copy_from_user)(void *context, void *to, const void *from,
					unsigned long size);
	unsigned long (*copy_to_user)(void *context, void *to, const void *from,
				      unsigned long size);
	/* Install or remove a client's page table entries (see stela/file.h);
	 * prot holds x86 page table bits, and map_user returns a negative errno. */
	int (*map_user)(void *context, void *space, unsigned long address, unsigned long pfn,
			u64 prot);
	/* Zero confirms removal and stale-translation retirement. */
	int (*unmap_user)(void *context, void *space, unsigned long address, size_t size);
	/* Calibrated x86 TSC rate for BogoMIPS metadata, or zero if unavailable. */
	u64 (*calibrated_tsc_hz)(void *context);
	int (*copy_from_kernel_nofault)(void *context, void *to, const void *from, size_t size);
	int (*available_page_frames)(void *context, size_t start, size_t pages, size_t *available);
	/* Convert a same-owner monotonic sample to realtime. Modes 0/1/2 are
	 * fine, coarse with multigrain floor, and fine updating that floor.
	 * Only mode 1 takes a nonzero tick_ns. False leaves outputs unchanged. */
	bool (*realtime_at)(void *context, s64 now_ns, u32 mode, u32 tick_ns,
			    s64 *seconds, u32 *nanoseconds);
	/* Mix auxiliary input into the already seeded owner stream. */
	bool (*mix_random)(void *context, const void *bytes, size_t size);
	/* Actual elapsed time since this Linux boot, including any suspend.
	 * Bounded read from the retained owner. False leaves *now unchanged. */
	bool (*boottime_ns)(void *context, s64 *now);
	/* A terminal stop the owner names with message, after log reported it.
	 * It should not return; stela_linux_fatal traps when it does or when the
	 * owner has none. */
	void (*fatal)(void *context, const char *message);
};

struct stela_linux_owner {
	const struct stela_linux_services *services;
	void *context;
};

/* Private x86-64 C ABI. All producers and consumers rebuild from this header;
 * Rust mirrors this exact table and checks every x86-64 field offset. */
_Static_assert(sizeof(struct stela_linux_services) == 216, "Linux services ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, now_ns) == 0, "Linux now_ns ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, minimum_delay_us) == 8, "Linux minimum_delay_us ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, minimum_deadline_us) == 16, "Linux minimum_deadline_us ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, timer_changed) == 24, "Linux timer_changed ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, log) == 32, "Linux log ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, warn) == 40, "Linux warn ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, unsupported) == 48, "Linux unsupported ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, random_bytes) == 56, "Linux random_bytes ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, shutdown) == 64, "Linux shutdown ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, map_device) == 72, "Linux map_device ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, unmap_device) == 80, "Linux unmap_device ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, map_pages) == 88, "Linux map_pages ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, unmap_pages) == 96, "Linux unmap_pages ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, irq_enable) == 104, "Linux irq_enable ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, irq_disable) == 112, "Linux irq_disable ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, irq_retire) == 120, "Linux irq_retire ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, copy_from_user) == 128, "Linux copy_from_user ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, copy_to_user) == 136, "Linux copy_to_user ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, map_user) == 144, "Linux map_user ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, unmap_user) == 152, "Linux unmap_user ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, calibrated_tsc_hz) == 160, "Linux calibrated_tsc_hz ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, copy_from_kernel_nofault) == 168, "Linux nofault read ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, available_page_frames) == 176, "Linux page availability ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, realtime_at) == 184, "Linux realtime ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, mix_random) == 192, "Linux random input ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, boottime_ns) == 200, "Linux boottime ABI");
_Static_assert(__builtin_offsetof(struct stela_linux_services, fatal) == 208, "Linux fatal ABI");
_Static_assert(sizeof(struct stela_linux_owner) == 16, "Linux owner ABI");

/* An owner for Linux code that needs none of these services. */
extern const struct stela_linux_owner stela_linux_bare;

void stela_linux_enter(const struct stela_linux_owner *owner);
/* One acquisition attempt; false leaves an already-running owner untouched.
 * This is also safe when a platform callback tries to retire another owner. */
bool stela_linux_try_enter(const struct stela_linux_owner *owner);
void stela_linux_exit(void);
/* Checked quiescence in the serialized context, not a native IRQ witness. */
bool stela_linux_quiescent(void);
/* Runs irq_work a workqueue worker raised in process context, from a
 * quiescent task (its cond_resched after the item, or its park). */
void stela_linux_irq_work_run_owed(void);
/* Explicit original rcu_barrier on a retained source process; it may sleep.
 * Joins already queued callbacks, not future descendants. Caller first stops
 * producers and joins each last-callback lifetime. Never a scheduler/exit hook. */
/* Entered serialized-owner observation only. Private RCU delivery remains
 * active despite balanced public preempt/IRQ counters. No task/root grant. */
bool stela_linux_rcu_callback_active(void);
void stela_linux_rcu_finish(void);
void stela_linux_printk_flush(void);
/* The owner running Linux code; traps outside enter and exit. */
const struct stela_linux_owner *stela_linux_owner(void);
/* Report an operation Stela does not provide yet to the running owner. */
void stela_linux_unsupported(const char *operation);
/* Emit an emergency report through the entered owner and give it the stop,
 * then trap if that returns; this does not implement Linux panic machinery. */
__noreturn void stela_linux_fatal(const char *message);

/* Platform memory behind Linux's allocators: Stela's global heap in its
 * images, libc in hosted test programs. Zeroed and aligned to align, a power
 * of two accepted by the platform layout, or NULL when exhausted; freed with the
 * same size and alignment. */
void *stela_linux_heap_alloc(size_t size, size_t align);
void stela_linux_heap_free(void *object, size_t size, size_t align);
/* Permanent access to the SAME canonical heap/free identity, without waiting,
 * reclaim, growth or fallback. Query first; absent/in-progress capability is
 * unsupported, while an installed allocator's NULL is ordinary exhaustion or
 * contention. Nonzero valid layouts require the actual retained owner/context.
 * Registration supplies no native ISR entry, page/PFN or DMA authority. */
bool stela_linux_heap_try_available(void);
void *stela_linux_heap_try_alloc(size_t size, size_t align);
/* Actual retained native image DATA only; no pointer read or heap/free grant.
 * Requires permanent Rust image registration before the dependent lifecycle is
 * admitted. Missing/in-progress registration traps instead of returning false.
 * The registration is independent of per-call Linux execution owners. The
 * admission owner must cover every complete borrowed constant name (including
 * NUL), with stable true classification through release. A negative free query
 * requires separate heap ownership (except canonical NULL/zero-size no-op
 * values); negative duplication may copy a valid
 * mutable/stack string. A negative result never proves an unknown pointer is
 * heap. */
bool stela_linux_is_rodata(const void *address);
#endif
