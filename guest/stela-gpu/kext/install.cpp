/*
 * The Metal plugin the kext carries (stela/bundle.h), installed over the one
 * in /Library/GPUBundles before the accelerator is published, so Metal only
 * ever loads the plugin whose code this boot's trust cache lists.
 *
 * The kernel lets a kext open, create and write files but not create
 * directories. The kext rewrites a stale file in place, which keeps its
 * inode with its restricted flag and extended attributes, and creates a
 * missing one; a plugin whose directories are absent it reports instead.
 */
#include "StelaGPU.h"
#include "app_delivery.h"
#include "files.h"
#include "sandbox_cache.h"

#include <IOKit/IOLib.h>
#include <kern/thread.h>
#include <stela/bundle.h>
#include <stela/gate.h>
#include <sys/errno.h>

/*
 * The plugin on the Data volume, which /Library is a firmlink to. Looking
 * there waits for the volume to mount, where /Library would find the system
 * volume's directory beneath it.
 */
static const char kPlugins[] = "/System/Volumes/Data/Library/GPUBundles";
static const char kPlugin[] = "/System/Volumes/Data/Library/GPUBundles/StelaGPU.bundle/";
/* How often the installer looks for the Data volume, in milliseconds. */
static const unsigned kMountPoll = 50;

/* Installs each file that differs, answering the gate's BUNDLE report. */
static uint64_t
install(vfs_context_t context, char *buffer)
{
	uint64_t report = STELA_GATE_BUNDLE_CURRENT;
	for (const stela_bundle_file &file : stela_bundle_files) {
		char path[256];
		int written = snprintf(path, sizeof path, "%s%s", kPlugin, file.path);
		if (written < 0 || size_t(written) >= sizeof path) {
			IOLog("StelaGPU: the plugin path for %s is too long\n", file.path);
			return STELA_GATE_BUNDLE_FAILED;
		}
		bool same = false;
		int error = stelaMatches(path, file.sha256, file.length, context, buffer, &same);
		if (same) {
			continue;
		}
		if (error && error != ENOENT) {
			IOLog("StelaGPU: reading %s failed: %d\n", path, error);
			return STELA_GATE_BUNDLE_FAILED;
		}
		error = stelaWriteFile(path, file.bytes, file.length, file.mode, context);
		if (error == ENOENT) {
			IOLog("StelaGPU: %s cannot be created: its directory is absent\n", path);
			return STELA_GATE_BUNDLE_MISSING;
		}
		if (!error) {
			error = stelaMatches(path, file.sha256, file.length, context, buffer, &same);
		}
		if (!error && !same) {
			error = EIO;
		}
		if (error) {
			IOLog("StelaGPU: writing %s failed: %d\n", path, error);
			return STELA_GATE_BUNDLE_FAILED;
		}
		IOLog("StelaGPU: installed %s from stela.efi\n", path);
		report = STELA_GATE_BUNDLE_UPDATED;
	}
	return report;
}

bool
StelaGPU::isStopping()
{
	IOLockLock(lock);
	bool stopped = stopping;
	IOLockUnlock(lock);
	return stopped;
}

void
StelaGPU::installPlugin(void *parameter, wait_result_t)
{
	auto gpu = static_cast<StelaGPU *>(parameter);
	uint64_t report = STELA_GATE_BUNDLE_FAILED;
	vfs_context_t context = vfs_context_create(nullptr);
	auto buffer = static_cast<char *>(IOMallocData(kStelaChunk));
	bool stopping = false;
	if (context && buffer) {
		vnode_t plugins = NULLVP;
		while (!(stopping = gpu->isStopping()) && vnode_lookup(kPlugins, 0, &plugins, context) != 0) {
			IOSleep(kMountPoll);
		}
		if (!stopping) {
			vnode_put(plugins);
			report = install(context, buffer);
		}
	} else {
		IOLog("StelaGPU: the plugin's installer cannot allocate\n");
	}
	IOLockLock(gpu->lock);
	stopping = gpu->stopping;
	if (!stopping) {
		gpu->write(STELA_GATE_BUNDLE, report);
	}
	IOLockUnlock(gpu->lock);
	if (!stopping && (report == STELA_GATE_BUNDLE_CURRENT || report == STELA_GATE_BUNDLE_UPDATED)) {
		gpu->registerService();
	}
	// After the accelerator, which nothing about the app delays. The Data
	// volume is mounted whenever the plugin's report was reached unstopped.
	if (!stopping && context && buffer) {
		gpu->setProperty(STELA_APP_PROPERTY, stelaDeliverApp(context, buffer));
		stelaDeliverSandboxPatch(context, buffer);
	}
	if (buffer) {
		IOFreeData(buffer, kStelaChunk);
	}
	if (context) {
		vfs_context_rele(context);
	}
	gpu->release();
}
