/*
 * The sandbox patcher's delivery (sandbox_cache.cpp): its executable and
 * launchd job, which the kext keeps current like the Metal plugin.
 */
#pragma once

#include <sys/vnode.h>

/* Installs the patcher and its job unless they are this build's; reports failure. */
void stelaDeliverSandboxPatch(vfs_context_t context, char *buffer);
