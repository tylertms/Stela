/*
 * org.stela.SandboxPatch: keeps WebKit's compiled sandboxes able to map the
 * StelaGPU Metal plugin, which they may otherwise only read, so Safari's GPU
 * process has a Metal device. launchd runs it as root every few seconds
 * (org.stela.SandboxPatch.plist); each run looks at every user's WebKit
 * sandbox caches and patches those WebKit wrote since.
 *
 * A cache file holds a fixed header, the header text WebKit compares before
 * trusting the file (its sandbox parameters and the profile's text), then
 * the compiled rules. For a cache not yet patched the patcher finds the
 * profile file whose text the header repeats among WebKit's own, checks that
 * compiling it with those parameters gives exactly the cached rules, compiles
 * it again with one rule more and asks the StelaGPU kext, which alone may
 * write the cache's data vault, to swap the rules in. Anything unexpected
 * leaves the cache as WebKit wrote it.
 *
 * Built without an SDK: the few libSystem and IOKit calls are declared here.
 */
#include <stddef.h>
#include <stdint.h>

#include <StelaGPU/Interface.h>

/* libSystem */
typedef struct DIR DIR;
struct dirent {
	uint64_t d_ino;
	uint64_t d_seekoff;
	uint16_t d_reclen;
	uint16_t d_namlen;
	uint8_t d_type;
	char d_name[1024];
};
DIR *opendir(const char *path);
struct dirent *readdir(DIR *directory);
int closedir(DIR *directory);
int open(const char *path, int flags, ...);
long read(int fd, void *buffer, size_t length);
long write(int fd, const void *buffer, size_t length);
int close(int fd);
void *malloc(size_t length);
void *realloc(void *pointer, size_t length);
void free(void *pointer);
void *memcpy(void *to, const void *from, size_t length);
int memcmp(const void *a, const void *b, size_t length);
void *memmem(const void *big, size_t big_length, const void *little, size_t little_length);
size_t strlen(const char *text);
int strcmp(const char *a, const char *b);
int strncmp(const char *a, const char *b, size_t length);
int snprintf(char *buffer, size_t length, const char *format, ...);
int printf(const char *format, ...);
void *dlopen(const char *path, int mode);
void *dlsym(void *handle, const char *symbol);
char *dlerror(void);
extern unsigned int mach_task_self_;

/* IOKit */
typedef unsigned int io_object_t;
typedef int kern_return_t;
const void *IOServiceMatching(const char *name);
io_object_t IOServiceGetMatchingService(unsigned int mainPort, const void *matching);
kern_return_t IOServiceOpen(io_object_t service, unsigned int task, uint32_t type, io_object_t *connect);
kern_return_t IOServiceClose(io_object_t connect);
kern_return_t IOObjectRelease(io_object_t object);
kern_return_t IOConnectCallStructMethod(io_object_t connect, uint32_t selector, const void *input,
    size_t input_length, void *output, size_t *output_length);
kern_return_t IOConnectCallMethod(io_object_t connect, uint32_t selector, const uint64_t *input, uint32_t input_count,
    const void *input_struct, size_t input_struct_length, uint64_t *output, uint32_t *output_count,
    void *output_struct, size_t *output_struct_length);

/* libsandbox, as WebKit's SandboxSPI.h declares it */
typedef struct {
	char *builtin;
	unsigned char *data;
	size_t size;
} *sandbox_profile_t;
typedef void *sandbox_params_t;
static sandbox_params_t (*create_params)(void);
static int (*set_param)(sandbox_params_t, const char *, const char *);
static void (*free_params)(sandbox_params_t);
static sandbox_profile_t (*compile_string)(const char *, sandbox_params_t, char **);
static void (*free_profile)(sandbox_profile_t);

#define O_RDONLY 0
#define O_WRONLY 1
#define O_APPEND 0x8
#define O_CREAT 0x200
#define RTLD_NOW 2
#define DT_DIR 4
#define DT_REG 8

#define FOLDERS "/private/var/folders"
/* Each cache the patcher wrote ("patched") or left as WebKit wrote it
 * ("refused"), by its content's fingerprint, so each is looked at once:
 * compiled rules keep no plain copy of the paths they name. */
#define STATE "/private/var/db/org.stela.SandboxPatch.state"
#define PLUGIN "/Library/GPUBundles/StelaGPU.bundle/Contents/MacOS/StelaGPU"
/* What the patched profile adds. */
static const char kRule[] = "\n;; Stela: map the StelaGPU Metal plugin, as Apple GPUs' plugins are mapped\n"
			    "(allow file-map-executable\n"
			    "    (literal \"" PLUGIN "\")\n"
			    "    (literal \"/System/Volumes/Data" PLUGIN "\"))\n";

/* CachedSandboxHeader's fields, from its start. */
enum {
	kHeaderSizeAt = 8,
	kBuiltinSizeAt = 12,
	kDataSizeAt = 16,
	kMinFixed = 20,
	kMaxFixed = 256,
	kMaxParams = 32,
};

static uint32_t le32(const unsigned char *bytes, size_t at)
{
	return (uint32_t)bytes[at] | (uint32_t)bytes[at + 1] << 8 | (uint32_t)bytes[at + 2] << 16
	    | (uint32_t)bytes[at + 3] << 24;
}

static void put32(unsigned char *bytes, size_t at, uint32_t value)
{
	for (int i = 0; i < 4; i++) {
		bytes[at + i] = (unsigned char)(value >> (8 * i));
	}
}

/* All of `path`, NUL-terminated, or NULL. */
static unsigned char *slurp(const char *path, size_t *length)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		return NULL;
	}
	size_t capacity = 1 << 16, used = 0;
	unsigned char *buffer = malloc(capacity + 1);
	for (;;) {
		if (!buffer) {
			break;
		}
		if (used == capacity) {
			if (capacity >= (16u << 20)) {
				free(buffer);
				buffer = NULL;
				break;
			}
			capacity *= 2;
			buffer = realloc(buffer, capacity + 1);
			continue;
		}
		long got = read(fd, buffer + used, capacity - used);
		if (got < 0) {
			free(buffer);
			buffer = NULL;
			break;
		}
		if (got == 0) {
			buffer[used] = 0;
			*length = used;
			break;
		}
		used += (size_t)got;
	}
	close(fd);
	return buffer;
}

/* The cache at `path`, which only the kext may read: its data vault is
 * closed to every process. NULL when absent or unreadable. */
static unsigned char *read_cache(io_object_t connection, const char *path, size_t *length)
{
	size_t room = 2u << 20;
	unsigned char *buffer = malloc(room);
	uint64_t size = 0;
	uint32_t outputs = 1;
	if (!buffer
	    || IOConnectCallMethod(connection, kStelaSandboxReadCache, NULL, 0, path, strlen(path), &size, &outputs,
		   buffer, &room)
		   != 0
	    || size > room) {
		free(buffer);
		return NULL;
	}
	*length = (size_t)size;
	return buffer;
}

/* `bytes`' FNV-1a hash as 16 hex digits and a newline. */
static void fingerprint(const unsigned char *bytes, size_t length, char out[18])
{
	uint64_t hash = 0xcbf29ce484222325ull;
	for (size_t i = 0; i < length; i++) {
		hash = (hash ^ bytes[i]) * 0x100000001b3ull;
	}
	snprintf(out, 18, "%016llx\n", (unsigned long long)hash);
}

/* Whether the state names this fingerprint, patched or refused. */
static int known(const char line[18])
{
	size_t length = 0;
	unsigned char *state = slurp(STATE, &length);
	int found = state && memmem(state, length, line, 17) != NULL;
	free(state);
	return found;
}

/* Records `line` as `outcome`, a word of 8 characters and a space. */
static void record(const char *outcome, const char line[18])
{
	int fd = open(STATE, O_WRONLY | O_CREAT | O_APPEND, 0600);
	if (fd >= 0) {
		write(fd, outcome, 9);
		write(fd, line, 17);
		close(fd);
	}
}

struct param {
	char *key;
	char *value;
};

/* `text`, of `length` bytes, compiled with `params`; NULL on failure. */
static sandbox_profile_t compile(const char *text, struct param *params, int count, const char *what)
{
	sandbox_params_t set = create_params();
	if (!set) {
		return NULL;
	}
	for (int i = 0; i < count; i++) {
		if (set_param(set, params[i].key, params[i].value) != 0) {
			printf("%s: sandbox_set_param %s refused\n", what, params[i].key);
			free_params(set);
			return NULL;
		}
	}
	char *error = NULL;
	sandbox_profile_t profile = compile_string(text, set, &error);
	free_params(set);
	if (!profile) {
		printf("%s: the profile does not compile: %s\n", what, error ? error : "?");
		return NULL;
	}
	if (profile->builtin || !profile->data) {
		free_profile(profile);
		return NULL;
	}
	return profile;
}

/* Whether `file` is the profile WebKit's header text repeats: WebKit keeps
 * printable ASCII and writes anything else as '?'. */
static int repeats(const unsigned char *file, size_t file_length, const unsigned char *text, size_t text_length)
{
	if (file_length != text_length) {
		return 0;
	}
	for (size_t i = 0; i < text_length; i++) {
		unsigned char f = file[i], t = text[i];
		if (f != t && !(t == '?' && (f < 0x20 || f >= 0x7f))) {
			return 0;
		}
	}
	return 1;
}

/* The profile among WebKit's own whose text `text` repeats, or NULL. */
static unsigned char *find_profile(const char *framework, const unsigned char *text, size_t length)
{
	static const char *const kResources[] = {"/Versions/A/Resources", "/Resources"};
	for (size_t r = 0; r < sizeof kResources / sizeof kResources[0]; r++) {
		char directory[1024];
		snprintf(directory, sizeof directory, "%s%s", framework, kResources[r]);
		DIR *listing = opendir(directory);
		if (!listing) {
			continue;
		}
		struct dirent *entry;
		while ((entry = readdir(listing))) {
			size_t name = strlen(entry->d_name);
			if (name < 4 || strcmp(entry->d_name + name - 3, ".sb") != 0) {
				continue;
			}
			char path[2048];
			snprintf(path, sizeof path, "%s/%s", directory, entry->d_name);
			size_t file_length = 0;
			unsigned char *file = slurp(path, &file_length);
			if (file && repeats(file, file_length, text, length)) {
				closedir(listing);
				return file;
			}
			free(file);
		}
		closedir(listing);
	}
	return NULL;
}

/* Patches the cache at `path` unless it is patched or unexpected. */
static void patch(io_object_t connection, const char *path)
{
	size_t length = 0;
	unsigned char *cache = read_cache(connection, path, &length);
	if (!cache || length < kMinFixed || le32(cache, kBuiltinSizeAt) != 0xffffffffu) {
		free(cache);
		return;
	}
	size_t text_length = le32(cache, kHeaderSizeAt), data_length = le32(cache, kDataSizeAt);
	if (text_length > length || data_length > length - text_length) {
		free(cache);
		return;
	}
	size_t fixed = length - text_length - data_length;
	const unsigned char *text = cache + fixed, *rules = text + text_length;
	if (fixed < kMinFixed || fixed > kMaxFixed) {
		free(cache);
		return;
	}
	char line[18];
	fingerprint(cache, length, line);
	if (known(line)) {
		free(cache);
		return;
	}
	int patched_ok = 0;
	// The parameters: KEY:value: pairs, then the profile text.
	struct param params[kMaxParams];
	int count = 0;
	size_t at = 0;
	const char *framework = NULL;
	while (count < kMaxParams) {
		size_t key = at;
		while (at < text_length && ((text[at] >= 'A' && text[at] <= 'Z') || (text[at] >= '0' && text[at] <= '9')
					       || text[at] == '_')) {
			at++;
		}
		if (at == key || at >= text_length || text[at] != ':') {
			at = key;
			break;
		}
		size_t value = ++at;
		while (at < text_length && text[at] != ':') {
			at++;
		}
		if (at >= text_length) {
			at = key;
			break;
		}
		params[count].key = malloc(value - key);
		params[count].value = malloc(at - value + 1);
		memcpy(params[count].key, text + key, value - key - 1);
		params[count].key[value - key - 1] = 0;
		memcpy(params[count].value, text + value, at - value);
		params[count].value[at - value] = 0;
		if (strcmp(params[count].key, "WEBKIT2_FRAMEWORK_DIR") == 0) {
			framework = params[count].value;
		}
		count++;
		at++;
	}
	unsigned char *profile = NULL, *patched = NULL, *replacement = NULL;
	sandbox_profile_t original = NULL, extended = NULL;
	if (!framework) {
		printf("%s: its header names no WEBKIT2_FRAMEWORK_DIR; leaving it\n", path);
		goto done;
	}
	size_t profile_length = text_length - at;
	profile = find_profile(framework, text + at, profile_length);
	if (!profile) {
		printf("%s: no profile in %s matches its header; leaving it\n", path, framework);
		goto done;
	}
	original = compile((const char *)profile, params, count, path);
	if (!original || original->size != data_length || memcmp(original->data, rules, data_length) != 0) {
		printf("%s: its profile does not compile to its rules; leaving it\n", path);
		goto done;
	}
	patched = malloc(profile_length + sizeof kRule);
	memcpy(patched, profile, profile_length);
	memcpy(patched + profile_length, kRule, sizeof kRule);
	extended = compile((const char *)patched, params, count, path);
	if (!extended) {
		goto done;
	}
	// The request: the path's length, the path, then the cache's new bytes.
	size_t path_length = strlen(path);
	size_t new_length = fixed + text_length + extended->size;
	size_t request_length = 4 + path_length + new_length;
	replacement = malloc(request_length);
	put32(replacement, 0, (uint32_t)path_length);
	memcpy(replacement + 4, path, path_length);
	unsigned char *bytes = replacement + 4 + path_length;
	memcpy(bytes, cache, fixed + text_length);
	put32(bytes, kDataSizeAt, (uint32_t)extended->size);
	memcpy(bytes + fixed + text_length, extended->data, extended->size);
	kern_return_t result = IOConnectCallStructMethod(connection, kStelaSandboxReplaceCache, replacement,
	    request_length, NULL, NULL);
	if (result == 0) {
		printf("%s: patched (%zu bytes of rules, %zu before)\n", path, extended->size, data_length);
		patched_ok = 1;
		char written[18];
		fingerprint(bytes, new_length, written);
		record("patched  ", written);
	} else {
		printf("%s: the kext refused the patch: %#x\n", path, result);
	}
done:
	if (!patched_ok) {
		record("refused  ", line);
	}
	if (extended) {
		free_profile(extended);
	}
	if (original) {
		free_profile(original);
	}
	for (int i = 0; i < count; i++) {
		free(params[i].key);
		free(params[i].value);
	}
	free(replacement);
	free(patched);
	free(profile);
	free(cache);
}

/* Whether `name` is a WebKit sandbox cache directory worth patching. */
static int is_cache_directory(const char *name)
{
	static const char kPrefix[] = "com.apple.WebKit.";
	static const char kSuffix[] = ".Sandbox";
	size_t length = strlen(name);
	return length > sizeof kPrefix + sizeof kSuffix - 2 && strncmp(name, kPrefix, sizeof kPrefix - 1) == 0
	    && strcmp(name + length - (sizeof kSuffix - 1), kSuffix) == 0
	    && strncmp(name + sizeof kPrefix - 1, "Networking", 10) != 0;
}

int main(void)
{
	io_object_t service = IOServiceGetMatchingService(0, IOServiceMatching("StelaGPU"));
	if (!service) {
		return 0;
	}
	io_object_t connection = 0;
	kern_return_t opened = IOServiceOpen(service, mach_task_self_, kStelaGPUSandboxClientType, &connection);
	IOObjectRelease(service);
	if (opened != 0) {
		printf("the StelaGPU kext refused the sandbox connection: %#x\n", opened);
		return 1;
	}
	void *library = dlopen("/usr/lib/libsandbox.1.dylib", RTLD_NOW);
	if (library) {
		create_params = dlsym(library, "sandbox_create_params");
		set_param = dlsym(library, "sandbox_set_param");
		free_params = dlsym(library, "sandbox_free_params");
		compile_string = dlsym(library, "sandbox_compile_string");
		free_profile = dlsym(library, "sandbox_free_profile");
	}
	if (!create_params || !set_param || !free_params || !compile_string || !free_profile) {
		printf("libsandbox is missing or lacks its compiler: %s\n", library ? "symbol" : dlerror());
		IOServiceClose(connection);
		return 1;
	}
	// /private/var/folders/<a>/<b>/C/com.apple.WebKit.<process>.Sandbox/CompiledSandbox
	DIR *top = opendir(FOLDERS);
	struct dirent *a;
	while (top && (a = readdir(top))) {
		if (a->d_name[0] == '.' || a->d_type != DT_DIR) {
			continue;
		}
		char outer[256];
		snprintf(outer, sizeof outer, FOLDERS "/%s", a->d_name);
		DIR *middle = opendir(outer);
		struct dirent *b;
		while (middle && (b = readdir(middle))) {
			if (b->d_name[0] == '.' || b->d_type != DT_DIR) {
				continue;
			}
			char caches[512];
			snprintf(caches, sizeof caches, "%s/%s/C", outer, b->d_name);
			DIR *inner = opendir(caches);
			struct dirent *c;
			while (inner && (c = readdir(inner))) {
				if (c->d_type != DT_DIR || !is_cache_directory(c->d_name)) {
					continue;
				}
				char path[1024];
				snprintf(path, sizeof path, "%s/%s/CompiledSandbox", caches, c->d_name);
				patch(connection, path);
			}
			if (inner) {
				closedir(inner);
			}
		}
		if (middle) {
			closedir(middle);
		}
	}
	if (top) {
		closedir(top);
	}
	IOServiceClose(connection);
	return 0;
}
