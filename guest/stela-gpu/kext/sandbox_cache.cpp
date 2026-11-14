/*
 * WebKit's GPU and web content processes run under sandboxes that may read
 * /Library/GPUBundles but not map code from it, so without help they have no
 * Metal device and Safari draws pages wrong. WebKit compiles each sandbox
 * once into a cache file, `<user cache>/com.apple.WebKit.<process>.Sandbox/
 * CompiledSandbox`, which it trusts by its header alone: the parameters and
 * profile text it was compiled from. The guest's sandbox patcher
 * (guest/stela-gpu/sandbox) recompiles that very profile with one rule more,
 * mapping the plugin's executable, and asks here, as root, to swap the
 * compiled rules in. The cache's directory is a data vault no process may
 * write, so the kext writes it, and only that: a file already there, under
 * that path, whose header the new bytes repeat exactly.
 *
 * The kext also keeps the patcher and its launchd job installed.
 */
#include "StelaGPU.h"
#include "files.h"
#include "sandbox_cache.h"

#include <IOKit/IOLib.h>
#include <StelaGPU/Interface.h>
#include <stela/sandbox_patch.h>
#include <sys/errno.h>

/* Exported to kexts, but declared only for the kernel's own builds. */
extern "C" vfs_context_t vfs_context_kernel(void);

OSDefineMetaClassAndStructors(StelaSandboxClient, IOUserClient);

/* Larger than any compiled WebKit sandbox (about 100 KiB in macOS 27). */
static const unsigned kMaxCache = 2 << 20;
/* WebKit's CachedSandboxHeader is 92 bytes; anything far off is not one. */
static const unsigned long kMinFixed = 20;
static const unsigned long kMaxFixed = 256;
/* CachedSandboxHeader's fields, from its start. */
static const unsigned long kHeaderSizeAt = 8;
static const unsigned long kBuiltinSizeAt = 12;
static const unsigned long kDataSizeAt = 16;

static const IOExternalMethodDispatch kMethods[kStelaSandboxSelectorCount] = {
	{
		.function = &StelaSandboxClient::replaceCache,
		.checkScalarInputCount = 0,
		.checkStructureInputSize = kIOUCVariableStructureSize,
		.checkScalarOutputCount = 0,
		.checkStructureOutputSize = 0,
	},
	{
		.function = &StelaSandboxClient::readCache,
		.checkScalarInputCount = 0,
		.checkStructureInputSize = kIOUCVariableStructureSize,
		.checkScalarOutputCount = 1,
		.checkStructureOutputSize = kIOUCVariableStructureSize,
	},
};

bool
StelaSandboxClient::initWithTask(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties)
{
	if (type != kStelaGPUSandboxClientType
	    || clientHasPrivilege(owningTask, kIOClientPrivilegeAdministrator) != kIOReturnSuccess) {
		return false;
	}
	return IOUserClient::initWithTask(owningTask, securityID, type, properties);
}

IOReturn
StelaSandboxClient::clientClose()
{
	terminate();
	return kIOReturnSuccess;
}

IOReturn
StelaSandboxClient::externalMethod(uint32_t selector, IOExternalMethodArguments *arguments,
    IOExternalMethodDispatch *, OSObject *, void *)
{
	if (selector >= kStelaSandboxSelectorCount) {
		return kIOReturnBadArgument;
	}
	IOExternalMethodDispatch dispatch = kMethods[selector];
	return IOUserClient::externalMethod(selector, arguments, &dispatch, this, nullptr);
}

static uint32_t
le32(const unsigned char *bytes, unsigned long at)
{
	return uint32_t(bytes[at]) | uint32_t(bytes[at + 1]) << 8 | uint32_t(bytes[at + 2]) << 16
	    | uint32_t(bytes[at + 3]) << 24;
}

/*
 * A compiled WebKit sandbox without a builtin: its fixed header's length and
 * its header text's, or false.
 */
static bool
parseCache(const unsigned char *bytes, unsigned long length, unsigned long *fixed, unsigned long *text)
{
	if (length < kMinFixed || le32(bytes, kBuiltinSizeAt) != 0xffffffffu) {
		return false;
	}
	unsigned long header = le32(bytes, kHeaderSizeAt);
	unsigned long data = le32(bytes, kDataSizeAt);
	if (header > length || data > length - header) {
		return false;
	}
	*fixed = length - header - data;
	*text = header;
	return *fixed >= kMinFixed && *fixed <= kMaxFixed;
}

static bool
isSegment(char c)
{
	return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '+'
	    || c == '-';
}

/* Consumes `literal` at `*at`. */
static bool
take(const char **at, const char *end, const char *literal)
{
	const char *cursor = *at;
	for (; *literal; literal++, cursor++) {
		if (cursor == end || *cursor != *literal) {
			return false;
		}
	}
	*at = cursor;
	return true;
}

/* Consumes one path component of at most `longest` segment characters. */
static bool
segment(const char **at, const char *end, long longest)
{
	const char *start = *at;
	while (*at != end && isSegment(**at) && *at - start < longest) {
		(*at)++;
	}
	return *at != start && *at != end && **at == '/';
}

static bool
isLetter(char c)
{
	return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

/*
 * Only `/private/var/folders/<a>/<b>/C/com.apple.WebKit.<process>.Sandbox/
 * CompiledSandbox`, the user caches' layout: `<a>` and `<b>` of segment
 * characters, `<process>` of letters joined by single dots.
 */
static bool
isCachePath(const char *path, unsigned long length)
{
	static const char kSuffix[] = ".Sandbox/CompiledSandbox";
	const long suffixLength = sizeof kSuffix - 1;
	const char *at = path, *end = path + length;
	if (!take(&at, end, "/private/var/folders/") || !segment(&at, end, 8) || !take(&at, end, "/")
	    || !segment(&at, end, 64) || !take(&at, end, "/C/com.apple.WebKit.")) {
		return false;
	}
	if (end - at <= suffixLength) {
		return false;
	}
	const char *nameEnd = end - suffixLength;
	const char *suffix = nameEnd;
	if (!take(&suffix, end, kSuffix) || suffix != end) {
		return false;
	}
	if (!isLetter(*at) || !isLetter(nameEnd[-1])) {
		return false;
	}
	for (const char *c = at; c != nameEnd; c++) {
		if (!isLetter(*c) && !(*c == '.' && isLetter(c[1]))) {
			return false;
		}
	}
	return true;
}

IOReturn
StelaSandboxClient::replaceCache(OSObject *, void *, IOExternalMethodArguments *arguments)
{
	unsigned long size = arguments->structureInputDescriptor ? arguments->structureInputDescriptor->getLength()
	                                                         : arguments->structureInputSize;
	if (size < 4 || size > kMaxCache + 1024) {
		return kIOReturnBadArgument;
	}
	auto input = static_cast<unsigned char *>(IOMallocData(size));
	auto existing = static_cast<char *>(IOMallocData(kMaxCache));
	auto chunk = static_cast<char *>(IOMallocData(kStelaChunk));
	// The kernel's own context: one made from this thread would carry the
	// caller's credentials, which the data vault refuses.
	vfs_context_t context = vfs_context_kernel();
	IOReturn result = kIOReturnNoMemory;
	char path[256];
	if (!input || !existing || !chunk || !context) {
		goto done;
	}
	if (arguments->structureInputDescriptor) {
		IOMemoryDescriptor *in = arguments->structureInputDescriptor;
		if (in->prepare(kIODirectionOut) != kIOReturnSuccess) {
			result = kIOReturnBadArgument;
			goto done;
		}
		unsigned long got = in->readBytes(0, input, size);
		in->complete(kIODirectionOut);
		if (got != size) {
			result = kIOReturnBadArgument;
			goto done;
		}
	} else {
		memcpy(input, arguments->structureInput, size);
	}
	{
		unsigned long pathLength = le32(input, 0);
		if (pathLength == 0 || pathLength >= sizeof path || pathLength > size - 4
		    || !isCachePath(reinterpret_cast<const char *>(input + 4), pathLength)) {
			result = kIOReturnBadArgument;
			goto done;
		}
		memcpy(path, input + 4, pathLength);
		path[pathLength] = 0;
		const unsigned char *bytes = input + 4 + pathLength;
		unsigned long length = size - 4 - pathLength;
		int had = 0;
		int error = stelaReadFile(path, existing, int(kMaxCache), &had, context);
		if (error) {
			IOLog("StelaGPU: sandbox cache %s does not read: %d\n", path, error);
			result = kIOReturnNotFound;
			goto done;
		}
		auto old = reinterpret_cast<const unsigned char *>(existing);
		unsigned long oldFixed = 0, oldText = 0, newFixed = 0, newText = 0;
		// The new bytes keep the cache's fixed header but for its data size,
		// and its header text: what WebKit checks before applying them.
		if (!parseCache(old, unsigned(had), &oldFixed, &oldText) || !parseCache(bytes, length, &newFixed, &newText)
		    || oldFixed != newFixed || oldText != newText || memcmp(old, bytes, kDataSizeAt) != 0
		    || memcmp(old + kDataSizeAt + 4, bytes + kDataSizeAt + 4, oldFixed - kDataSizeAt - 4) != 0
		    || memcmp(old + oldFixed, bytes + newFixed, oldText) != 0) {
			IOLog("StelaGPU: refused new sandbox rules for %s: they change its header\n", path);
			result = kIOReturnNotPermitted;
			goto done;
		}
		error = stelaWriteFile(path, bytes, length, 0644, context);
		bool same = false;
		unsigned char digest[SHA256_DIGEST_LENGTH];
		SHA256_CTX hash;
		SHA256_Init(&hash);
		SHA256_Update(&hash, bytes, length);
		SHA256_Final(digest, &hash);
		if (!error) {
			error = stelaMatches(path, digest, length, context, chunk, &same);
		}
		if (error || !same) {
			IOLog("StelaGPU: writing sandbox cache %s failed: %d\n", path, error ? error : EIO);
			result = kIOReturnIOError;
			goto done;
		}
		IOLog("StelaGPU: patched sandbox cache %s\n", path);
		result = kIOReturnSuccess;
	}
done:
	if (chunk) {
		IOFreeData(chunk, kStelaChunk);
	}
	if (existing) {
		IOFreeData(existing, kMaxCache);
	}
	if (input) {
		IOFreeData(input, size);
	}
	return result;
}

/*
 * A data vault is closed to reads as well as writes. Structure input: the
 * cache's path, as for replaceCache; scalar output: its length; structure
 * output: its bytes, when the caller's buffer holds them all.
 */
IOReturn
StelaSandboxClient::readCache(OSObject *, void *, IOExternalMethodArguments *arguments)
{
	char path[256];
	unsigned long pathLength = arguments->structureInputSize;
	if (arguments->structureInputDescriptor || pathLength == 0 || pathLength >= sizeof path
	    || !isCachePath(static_cast<const char *>(arguments->structureInput), pathLength)) {
		return kIOReturnBadArgument;
	}
	memcpy(path, arguments->structureInput, pathLength);
	path[pathLength] = 0;
	auto existing = static_cast<char *>(IOMallocData(kMaxCache));
	// The kernel's own context: one made from this thread would carry the
	// caller's credentials, which the data vault refuses.
	vfs_context_t context = vfs_context_kernel();
	IOReturn result = kIOReturnNoMemory;
	if (existing && context) {
		int length = 0;
		int error = stelaReadFile(path, existing, int(kMaxCache), &length, context);
		if (error) {
			static bool reported = false;
			if (error != ENOENT && !reported) {
				reported = true;
				IOLog("StelaGPU: sandbox cache %s does not read: %d\n", path, error);
			}
			result = error == ENOENT ? kIOReturnNotFound : kIOReturnIOError;
		} else {
			arguments->scalarOutput[0] = uint64_t(length);
			IOMemoryDescriptor *out = arguments->structureOutputDescriptor;
			unsigned long room = out ? out->getLength() : arguments->structureOutputSize;
			if (room < unsigned(length)) {
				result = kIOReturnNoSpace;
			} else if (out) {
				result = out->prepare(kIODirectionIn);
				if (result == kIOReturnSuccess) {
					result = out->writeBytes(0, existing, unsigned(length)) == unsigned(length)
					    ? kIOReturnSuccess
					    : kIOReturnIOError;
					out->complete(kIODirectionIn);
				}
				arguments->structureOutputDescriptorSize = unsigned(length);
			} else {
				memcpy(arguments->structureOutput, existing, unsigned(length));
				arguments->structureOutputSize = unsigned(length);
				result = kIOReturnSuccess;
			}
		}
	}
	if (existing) {
		IOFreeData(existing, kMaxCache);
	}
	return result;
}

/* Writes `length` bytes to `path` unless it holds them already, and checks them. */
static int
stage(const char *path, const unsigned char *bytes, unsigned long length, const unsigned char sha256[32], unsigned mode,
    vfs_context_t context, char *buffer)
{
	bool same = false;
	int error = stelaMatches(path, sha256, length, context, buffer, &same);
	if (same) {
		return 0;
	}
	if (error && error != ENOENT) {
		return error;
	}
	error = stelaWriteFile(path, bytes, length, mode, context);
	if (!error) {
		error = stelaMatches(path, sha256, length, context, buffer, &same);
	}
	return !error && !same ? EIO : error;
}

void
stelaDeliverSandboxPatch(vfs_context_t context, char *buffer)
{
	int error = stage(STELA_SANDBOX_PATCH_PATH, stela_sandbox_patch, sizeof stela_sandbox_patch,
	    stela_sandbox_patch_sha256, 0755, context, buffer);
	if (!error) {
		error = stage(STELA_SANDBOX_PATCH_JOB_PATH, stela_sandbox_patch_job, sizeof stela_sandbox_patch_job,
		    stela_sandbox_patch_job_sha256, 0644, context, buffer);
	}
	if (error) {
		IOLog("StelaGPU: installing the sandbox patcher failed: %d\n", error);
	}
}

IOReturn
StelaGPU::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
    IOUserClient **handler)
{
	if (type != kStelaGPUSandboxClientType) {
		return IOAccelerator::newUserClient(owningTask, securityID, type, properties, handler);
	}
	StelaSandboxClient *client = OSTypeAlloc(StelaSandboxClient);
	if (!client) {
		return kIOReturnNoMemory;
	}
	if (!client->initWithTask(owningTask, securityID, type, properties)) {
		client->release();
		return kIOReturnNotPrivileged;
	}
	if (!client->attach(this)) {
		client->release();
		return kIOReturnError;
	}
	if (!client->start(this)) {
		client->detach(this);
		client->release();
		return kIOReturnError;
	}
	*handler = client;
	return kIOReturnSuccess;
}
