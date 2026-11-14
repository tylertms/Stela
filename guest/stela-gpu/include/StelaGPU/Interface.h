/*
 * The StelaGPU user client's interface, shared by the kext and the Metal
 * bundle.
 *
 * A task opens one user client per Metal device (`IOServiceOpen` with
 * kStelaGPUUserClientType), which opens its gate connection, and maps the
 * gate page (`IOConnectMapMemory64` with kStelaGPUGateMemory). Calls go from
 * the task straight to the page: a 64-bit store of a call record's address
 * to its STELA_GATE_CALL register runs the call. The kernel is entered only
 * to wait for GPU completions.
 */
#pragma once

enum {
	/* The `IOServiceOpen` type. */
	kStelaGPUUserClientType = 0,
	/* The `clientMemoryForType` type of the connection's gate page. */
	kStelaGPUGateMemory = 0,
};

enum {
	/*
	 * Async: scalar input the completion count the caller has seen. Once
	 * the count differs, the kext sends the async result kIOReturnSuccess
	 * with one argument, the count now. One notification is armed per
	 * client; arming again replaces it.
	 */
	kStelaGPUNotifyCompletion = 0,
	/*
	 * Scalar input a page-aligned address and length of the task's memory,
	 * which the kext wires; scalar output the memory's handle, which goes in
	 * a buffer or heap call through the gate, or in Release. The memory
	 * stays wired, whatever becomes of the task's mapping, until the GPU no
	 * longer reaches it.
	 */
	kStelaGPUImportMemory = 1,
	kStelaGPUSelectorCount,
};

/*
 * The second `IOServiceOpen` type, root's alone, which opens no gate: the
 * sandbox patcher's (guest/stela-gpu/sandbox, kext/sandbox_cache.cpp).
 */
enum {
	kStelaGPUSandboxClientType = 1,
};

enum {
	/*
	 * Structure input: a little-endian u32 path length, the path of a WebKit
	 * sandbox cache, then the cache's new bytes, which keep its header and
	 * change only its compiled rules.
	 */
	kStelaSandboxReplaceCache = 0,
	/*
	 * Structure input: a WebKit sandbox cache's path; scalar output its
	 * length; structure output its bytes. Its data vault is closed to reads
	 * by any process, root's included.
	 */
	kStelaSandboxReadCache = 1,
	kStelaSandboxSelectorCount,
};
