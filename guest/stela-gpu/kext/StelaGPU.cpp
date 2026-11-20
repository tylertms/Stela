#include "StelaGPU.h"
#include "host_record.h"

#include <IOKit/IOLib.h>
#include <StelaGPU/Interface.h>
#include <stela/gate.h>
#include <sys/proc.h>

OSDefineMetaClassAndStructors(StelaGPU, IOAccelerator);

bool
StelaGPU::start(IOService *provider)
{
	if (!IOAccelerator::start(provider)) {
		return false;
	}
	if (!attachGate(provider)) {
		stop(provider);
		return false;
	}
	/*
	 * AppleCLPC reads this from every accelerator it finds and panics without
	 * it. In milliwatts of the SoC's package budget, which this device spends
	 * none of.
	 */
	setProperty("GetGPUMaxPower", 0ULL, 32);
	publishHost(provider);
	// Metal loads the plugin the accelerator names, so the accelerator is
	// published once the plugin is the one this boot trusts.
	retain();
	thread_t installer;
	if (kernel_thread_start(&StelaGPU::installPlugin, this, &installer) != KERN_SUCCESS) {
		release();
		IOLog("StelaGPU: cannot start the plugin's installer\n");
		stop(provider);
		return false;
	}
	thread_deallocate(installer);
	return true;
}

/* Adds one record entry to the OSDictionary `context`. */
static void
addHostEntry(void *context, const char *key, unsigned long keyLength, int isNumber, unsigned long long number,
    const char *text, unsigned long textLength)
{
	char name[STELA_HOST_KEY_BYTES + 1];
	char value[STELA_HOST_TEXT_BYTES + 1];
	memcpy(name, key, keyLength);
	name[keyLength] = 0;
	OSObject *object = nullptr;
	if (isNumber) {
		object = OSNumber::withNumber(number, 64);
	} else {
		memcpy(value, text, textLength);
		value[textLength] = 0;
		object = OSString::withCString(value);
	}
	if (object) {
		static_cast<OSDictionary *>(context)->setObject(name, object);
		object->release();
	}
}

void
StelaGPU::publishHost(IOService *provider)
{
	auto record = OSDynamicCast(OSData, provider->getProperty(STELA_HOST_PROPERTY));
	if (!record) {
		IOLog("StelaGPU: this stela.efi describes no host PC\n");
		return;
	}
	OSDictionary *host = OSDictionary::withCapacity(24);
	if (!host) {
		return;
	}
	int entries = stela_host_parse(static_cast<const char *>(record->getBytesNoCopy()), record->getLength(),
	    addHostEntry, host);
	if (entries < 0) {
		IOLog("StelaGPU: the host PC's description is in an unknown format\n");
	} else {
		setProperty("StelaHost", host);
	}
	host->release();
}

bool
StelaGPU::attachGate(IOService *provider)
{
	window = provider->getDeviceMemoryWithIndex(0);
	if (!window || window->getLength() != STELA_GATE_BYTES) {
		IOLog("StelaGPU: the gate window is not %llu bytes\n", STELA_GATE_BYTES);
		window = nullptr;
		return false;
	}
	window->retain();
	IODeviceMemory *page = IODeviceMemory::withSubRange(window, 0, STELA_GATE_PAGE);
	if (page) {
		control = page->map(kIOMapInhibitCache);
		page->release();
	}
	if (!control) {
		IOLog("StelaGPU: cannot map the gate's control page\n");
		return false;
	}
	if (read(STELA_GATE_COUNT) != STELA_GATE_CONNECTIONS) {
		IOLog("StelaGPU: the gate has %llu connections, not %llu\n",
		    read(STELA_GATE_COUNT), STELA_GATE_CONNECTIONS);
		return false;
	}
	lock = IOLockAlloc();
	workLoop = IOWorkLoop::workLoop();
	if (!lock || !workLoop) {
		return false;
	}
	// Diagnostics only: without the table nothing is logged.
	reports = static_cast<stela_plugin_reports *>(IOMallocZeroData(sizeof(*reports)));
	interrupt = IOInterruptEventSource::interruptEventSource(this, &StelaGPU::interruptOccurred, provider, 0);
	if (!interrupt || workLoop->addEventSource(interrupt) != kIOReturnSuccess) {
		IOLog("StelaGPU: cannot take the gate's interrupt\n");
		OSSafeReleaseNULL(interrupt);
		return false;
	}
	interrupt->enable();
	return true;
}

void
StelaGPU::stop(IOService *provider)
{
	if (lock) {
		IOLockLock(lock);
		stopping = true;
		IOLockUnlock(lock);
		// Only the host's retirement tokens permit unwiring imported pages.
		retireMemory();
		IOLockLock(lock);
		if (imports != 0 && !retainedForDma) {
			retainedForDma = true;
			retain();
			IOLog("StelaGPU: stopped with %llu wired imports; retaining the provider until native DMA retirement is available\n", imports);
		}
		IOLockUnlock(lock);
	}
	if (interrupt) {
		interrupt->disable();
		workLoop->removeEventSource(interrupt);
		OSSafeReleaseNULL(interrupt);
	}
	IOAccelerator::stop(provider);
}

void
StelaGPU::free()
{
	OSSafeReleaseNULL(interrupt);
	OSSafeReleaseNULL(workLoop);
	OSSafeReleaseNULL(control);
	OSSafeReleaseNULL(window);
	if (reports) {
		IOFreeData(reports, sizeof(*reports));
		reports = nullptr;
	}
	if (lock) {
		IOLockFree(lock);
		lock = nullptr;
	}
	IOAccelerator::free();
}

void
StelaGPU::reportOnce(const char *event)
{
	if (!lock || !reports) {
		return;
	}
	char process[64];
	proc_selfname(process, sizeof process);
	char line[STELA_PLUGIN_REPORT_BYTES];
	if (!stela_plugin_report_line(line, process, event)) {
		return;
	}
	IOLockLock(lock);
	switch (stela_plugin_report_admit(reports, line)) {
	case STELA_PLUGIN_REPORT_LOG:
		IOLog("StelaGPU: %s\n", line);
		break;
	case STELA_PLUGIN_REPORT_FULL:
		IOLog("StelaGPU: %u open reports logged; later ones are dropped\n", STELA_PLUGIN_REPORT_LINES);
		break;
	case STELA_PLUGIN_REPORT_SEEN:
	case STELA_PLUGIN_REPORT_DROP:
		break;
	}
	IOLockUnlock(lock);
}

/*
 * Whether Metal reached the accelerator at all, and what the open came to:
 * a user client that opened its gate connection, or why none did.
 */
IOReturn
StelaGPU::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
    IOUserClient **handler)
{
	if (type == kStelaGPUSandboxClientType) {
		return newSandboxClient(owningTask, securityID, type, properties, handler);
	}
	IOReturn result = IOAccelerator::newUserClient(owningTask, securityID, type, properties, handler);
	char event[64];
	if (result == kIOReturnSuccess) {
		snprintf(event, sizeof event, "opened the accelerator (type %u)", unsigned(type));
	} else {
		snprintf(event, sizeof event, "accelerator open (type %u) refused: 0x%x", unsigned(type),
		    unsigned(result));
	}
	reportOnce(event);
	return result;
}

IOReturn
StelaGPU::setProperties(OSObject *properties)
{
	auto dictionary = OSDynamicCast(OSDictionary, properties);
	if (!dictionary || dictionary->getCount() != 1) {
		return kIOReturnUnsupported;
	}
	auto event = OSDynamicCast(OSString, dictionary->getObject(kStelaGPUPluginReportKey));
	if (!event) {
		return kIOReturnUnsupported;
	}
	char text[STELA_PLUGIN_REPORT_BYTES + 16];
	snprintf(text, sizeof text, "plugin: %s", event->getCStringNoCopy());
	reportOnce(text);
	// Nothing is stored: the property is a message, not state.
	return kIOReturnSuccess;
}

uint64_t
StelaGPU::openGate()
{
	IOLockLock(lock);
	if (stopping) {
		IOLockUnlock(lock);
		return 0;
	}
	write(STELA_GATE_OPEN, 1);
	uint64_t page = read(STELA_GATE_OPENED);
	IOLockUnlock(lock);
	return page;
}

void
StelaGPU::closeGate(uint64_t page)
{
	IOLockLock(lock);
	write(STELA_GATE_CLOSE, page);
	IOLockUnlock(lock);
}

IODeviceMemory *
StelaGPU::copyGateMemory(uint64_t page)
{
	return IODeviceMemory::withSubRange(window, page * STELA_GATE_PAGE, STELA_GATE_PAGE);
}

/* The wired descriptor, or null with why prepare refused it in refused. */
static IOMemoryDescriptor *
wireMemory(task_t task, mach_vm_address_t address, mach_vm_size_t length, IODirection direction,
	   IOReturn *refused)
{
	IOMemoryDescriptor *memory = IOMemoryDescriptor::withAddressRange(address, length, direction, task);
	if (!memory) {
		*refused = kIOReturnNoMemory;
		return nullptr;
	}
	*refused = memory->prepare();
	if (*refused != kIOReturnSuccess) {
		memory->release();
		return nullptr;
	}
	return memory;
}

/*
 * The prepared descriptor of imported memory stands for its token: only the
 * gate sees a token, and it answers only those the kext gave it.
 */
uint64_t
StelaGPU::importMemory(uint64_t page, task_t task, mach_vm_address_t address, mach_vm_size_t length)
{
	IOReturn refused;
	IOMemoryDescriptor *memory = wireMemory(task, address, length, kIODirectionOutIn, &refused);
	if (!memory) {
		// A read-only mapping refuses that with a protection failure, and
		// wires for the GPU's reads alone.
		memory = wireMemory(task, address, length, kIODirectionOut, &refused);
	}
	if (!memory) {
		IOLog("StelaGPU: wiring %llx bytes at %llx refused: %x\n", length, address, refused);
		return 0;
	}
	IOLockLock(lock);
	if (stopping || imports == ~uint64_t(0)) {
		bool stopped = stopping;
		IOLockUnlock(lock);
		IOLog("StelaGPU: import of %llx bytes at %llx refused: %s\n", length, address,
		    stopped ? "the GPU is stopping" : "the import count is exhausted");
		memory->complete();
		memory->release();
		return 0;
	}
	for (IOByteCount offset = 0; offset < length;) {
		IOByteCount bytes = 0;
		addr64_t physical = memory->getPhysicalSegment(offset, &bytes, kIOMemoryMapperNone);
		write(STELA_GATE_IMPORT_ADDRESS, physical);
		write(STELA_GATE_IMPORT_BYTES, bytes);
		offset += bytes;
	}
	write(STELA_GATE_IMPORT_TOKEN, reinterpret_cast<uint64_t>(memory));
	write(STELA_GATE_IMPORT, page);
	uint64_t handle = read(STELA_GATE_IMPORTED);
	if (handle != 0) {
		imports++;
	}
	IOLockUnlock(lock);
	if (handle == 0) {
		memory->complete();
		memory->release();
	}
	return handle;
}

void
StelaGPU::retireMemory()
{
	for (;;) {
		IOLockLock(lock);
		write(STELA_GATE_RETIRE, 1);
		uint64_t token = read(STELA_GATE_RETIRED);
		if (token != 0) {
			// The host answers only tokens of successful imports, once each.
			assert(imports != 0);
			imports--;
		}
		IOLockUnlock(lock);
		if (token == 0) {
			return;
		}
		auto memory = reinterpret_cast<IOMemoryDescriptor *>(token);
		memory->complete();
		memory->release();
	}
}

uint64_t
StelaGPU::completions() const
{
	return __atomic_load_n(&completionCount, __ATOMIC_ACQUIRE);
}

/*
 * The gate holds its level interrupt until acknowledged. Acknowledging
 * before counting means work completing after the count raises it again.
 */
void
StelaGPU::interruptOccurred(OSObject *owner, IOInterruptEventSource *, int)
{
	auto gpu = static_cast<StelaGPU *>(owner);
	gpu->write(STELA_GATE_ACK, 1);
	gpu->retireMemory();
	uint64_t now = __atomic_add_fetch(&gpu->completionCount, 1, __ATOMIC_ACQ_REL);
	gpu->applyToClients([](IOService *client, void *context) {
		if (auto user = OSDynamicCast(StelaGPUUserClient, client)) {
			user->notify(*static_cast<uint64_t *>(context));
		}
	}, &now);
}

uint64_t
StelaGPU::read(uint64_t reg) const
{
	return *reinterpret_cast<volatile uint64_t *>(control->getVirtualAddress() + reg);
}

void
StelaGPU::write(uint64_t reg, uint64_t value)
{
	*reinterpret_cast<volatile uint64_t *>(control->getVirtualAddress() + reg) = value;
}
