// SPDX-License-Identifier: GPL-2.0-or-later
/* A physical function as a Linux pci_dev. Stela's PCI owner enumerates and
 * places functions, so this only presents one: configuration space and DMA go
 * through its owner's callbacks, as an x86 host bridge's pci_ops and IOMMU's
 * dma_map_ops would. x86 caches snoop DMA, so no operation syncs. */
#include <linux/dma-map-ops.h>
#include <linux/pci.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <stela/catalogue.h>
#include <stela/linux.h>
#include <stela/pci.h>

struct host_function {
	struct stela_linux_pci_function function;
	const struct stela_linux_owner *owner;
	struct pci_bus bus;
	struct pci_dev dev;
	struct host_function *quarantined_next;
};

static u64 quarantined_functions;
static struct host_function *quarantined_hosts;

u64 stela_linux_pci_quarantined_functions(void)
{
	return __atomic_load_n(&quarantined_functions, __ATOMIC_RELAXED);
}

static const struct stela_linux_pci_function *of_bus(const struct pci_bus *bus)
{
	return bus->sysdata;
}

static const struct stela_linux_pci_function *of_device(struct device *dev)
{
	return of_bus(to_pci_dev(dev)->bus);
}

static int read_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *value)
{
	const struct stela_linux_pci_function *function = of_bus(bus);

	if (devfn != function->devfn)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return function->read_config(function->context, where, size, value);
}

static int write_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 value)
{
	const struct stela_linux_pci_function *function = of_bus(bus);

	if (devfn != function->devfn)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return function->write_config(function->context, where, size, value);
}

static struct pci_ops config_ops = {.read = read_config, .write = write_config};

static dma_addr_t map_phys(struct device *dev, phys_addr_t physical, size_t size,
			   enum dma_data_direction direction, unsigned long attributes)
{
	const struct stela_linux_pci_function *function = of_device(dev);

	return function->dma_map(function->context, physical, size, direction,
				 attributes & DMA_ATTR_MMIO);
}

static void unmap_phys(struct device *dev, dma_addr_t address, size_t size,
		       enum dma_data_direction direction, unsigned long attributes)
{
	const struct stela_linux_pci_function *function = of_device(dev);

	(void)attributes;
	function->dma_unmap(function->context, address, size, direction);
}

static void unmap_scatter(struct device *dev, struct scatterlist *list, int entries,
			  enum dma_data_direction direction, unsigned long attributes)
{
	struct scatterlist *entry;
	int i;

	for_each_sg(list, entry, entries, i)
		unmap_phys(dev, sg_dma_address(entry), sg_dma_len(entry), direction, attributes);
}

static int map_scatter(struct device *dev, struct scatterlist *list, int entries,
		       enum dma_data_direction direction, unsigned long attributes)
{
	struct scatterlist *entry;
	int i;

	for_each_sg(list, entry, entries, i) {
		dma_addr_t address = map_phys(dev, sg_phys(entry), entry->length, direction,
					      attributes);

		if (address == DMA_MAPPING_ERROR) {
			unmap_scatter(dev, list, i, direction, attributes);
			return -EIO;
		}
		sg_dma_address(entry) = address;
		sg_dma_len(entry) = entry->length;
	}
	return entries;
}

static void *alloc_coherent(struct device *dev, size_t size, dma_addr_t *address, gfp_t gfp,
			    unsigned long attributes)
{
	unsigned int order = (unsigned int)get_order(size);
	struct page *page = alloc_pages(gfp | __GFP_ZERO, order);

	if (!page)
		return NULL;
	*address = map_phys(dev, page_to_phys(page), size, DMA_BIDIRECTIONAL, attributes);
	if (*address == DMA_MAPPING_ERROR) {
		__free_pages(page, order);
		return NULL;
	}
	return page_address(page);
}

static void free_coherent(struct device *dev, size_t size, void *cpu, dma_addr_t address,
			  unsigned long attributes)
{
	unmap_phys(dev, address, size, DMA_BIDIRECTIONAL, attributes);
	__free_pages(virt_to_page(cpu), (unsigned int)get_order(size));
}

/* Linux dma_alloc_pages, which dma_alloc_noncoherent uses: pages the device
 * reaches through the same mapping a coherent allocation gets. x86 caches
 * snoop DMA, so a noncoherent buffer needs no more than the coherent path.
 * Without this every PCI function's dma_alloc_noncoherent returns NULL, which
 * nouveau's nvkm_firmware_ctor (NVKM_FIRMWARE_IMG_DMA) then hands to
 * sg_init_one before it checks, and virt_to_page traps on. */
static struct page *alloc_noncoherent_pages(struct device *dev, size_t size,
					    dma_addr_t *address, enum dma_data_direction direction,
					    gfp_t gfp)
{
	unsigned int order = (unsigned int)get_order(size);
	struct page *page = alloc_pages(gfp | __GFP_ZERO, order);

	if (!page)
		return NULL;
	*address = map_phys(dev, page_to_phys(page), size, direction, 0);
	if (*address == DMA_MAPPING_ERROR) {
		__free_pages(page, order);
		return NULL;
	}
	return page;
}

static void free_noncoherent_pages(struct device *dev, size_t size, struct page *page,
				   dma_addr_t address, enum dma_data_direction direction)
{
	unmap_phys(dev, address, size, direction, 0);
	__free_pages(page, (unsigned int)get_order(size));
}

/* The device reaches whatever its owner hands it: its addresses, not physical
 * memory, are what a mask must cover. */
static int mask_supported(struct device *dev, u64 mask)
{
	return mask >= of_device(dev)->dma_limit;
}

static const struct dma_map_ops mapping_ops = {
	.map_phys = map_phys,
	.unmap_phys = unmap_phys,
	.map_sg = map_scatter,
	.unmap_sg = unmap_scatter,
	.alloc = alloc_coherent,
	.free = free_coherent,
	.alloc_pages_op = alloc_noncoherent_pages,
	.free_pages = free_noncoherent_pages,
	.dma_supported = mask_supported,
};

int pci_enable_msi(struct pci_dev *dev)
{
	const struct stela_linux_pci_function *function = of_bus(dev->bus);
	int error = function->enable_msi(function->context);

	dev->msi_enabled = !error;
	return error;
}

void pci_disable_msi(struct pci_dev *dev)
{
	const struct stela_linux_pci_function *function = of_bus(dev->bus);

	if (dev->msi_enabled)
		function->disable_msi(function->context);
	dev->msi_enabled = 0;
}

static void release(struct device *dev)
{
	struct host_function *host = container_of(to_pci_dev(dev), struct host_function, dev);
	if (stela_linux_owner() != host->owner || !stela_linux_quiescent()) {
		/* The serialized owner admits only one writer. Preserve the original
		 * C allocation and every callback/root in its context; final Rust
		 * destruction and native deallocation need process context. */
		host->quarantined_next = quarantined_hosts;
		quarantined_hosts = host;
		u64 count = __atomic_load_n(&quarantined_functions, __ATOMIC_RELAXED);
		if (count != ~(u64)0)
			__atomic_store_n(&quarantined_functions, count + 1, __ATOMIC_RELAXED);
		return;
	}
	if (host->function.release_context)
		host->function.release_context(host->function.context);
	kfree(host);
}

struct pci_dev *stela_linux_pci_attach(const struct stela_linux_pci_function *function, int *status)
{
	if (stela_linux_kernel_hosted()) {
		if (status)
			*status = -EPERM;
		return NULL;
	}
	int error = -ENOMEM;
	struct host_function *host = kzalloc_obj(*host);
	if (status)
		*status = error;
	if (!host)
		return NULL;
	host->function = *function;
	host->owner = stela_linux_owner();
	host->bus.sysdata = &host->function;
	host->bus.ops = &config_ops;
	host->bus.number = (unsigned char)function->bus;
	struct pci_dev *dev = &host->dev;
	u32 class_revision;

	dev->bus = &host->bus;
	dev->devfn = function->devfn;
	dev->irq = function->irq;
	dev->dma_mask = DMA_BIT_MASK(32);
	dev->current_state = PCI_D0;
	dev->rom_base_reg = PCI_ROM_ADDRESS;
    error = pci_read_config_word(dev, PCI_VENDOR_ID, &dev->vendor);
    if (!error)
        error = pci_read_config_word(dev, PCI_DEVICE_ID, &dev->device);
    if (!error)
        error = pci_read_config_word(dev, PCI_SUBSYSTEM_VENDOR_ID, &dev->subsystem_vendor);
    if (!error)
        error = pci_read_config_word(dev, PCI_SUBSYSTEM_ID, &dev->subsystem_device);
    if (!error)
        error = pci_read_config_byte(dev, PCI_HEADER_TYPE, &dev->hdr_type);
    if (!error)
        error = pci_read_config_dword(dev, PCI_CLASS_REVISION, &class_revision);
    if (!error)
        error = stela_linux_pci_find_capability(dev, PCI_CAP_ID_EXP, &dev->pcie_cap);
    if (error) {
        /* No original device_initialize/publication or context transfer has
         * occurred. Rust retains exact State/Host/authority on this refusal. */
        if (status)
            *status = pcibios_err_to_errno(error);
        kfree(host);
        return NULL;
    }
    dev->revision = (u8)class_revision;
    dev->class = class_revision >> 8;
	for (int i = 0; i < PCI_STD_NUM_BARS; i++) {
		const struct stela_linux_pci_region *region = &function->regions[i];

		if (!region->size)
			continue;
		dev->resource[i].start = region->start;
		dev->resource[i].end = region->start + region->size - 1;
		dev->resource[i].flags = region->flags;
	}
	dev->dev.dma_mask = &dev->dma_mask;
	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	dev->dev.release = release;
	set_dma_ops(&dev->dev, &mapping_ops);
	device_initialize(&dev->dev);
	error = dev_set_name(&dev->dev, "%04x:%02x:%02x.%d", 0, function->bus,
                         PCI_SLOT(function->devfn), PCI_FUNC(function->devfn));
	if (!error && function->enroll)
		error = function->enroll(function->context, &dev->dev);
	if (!error)
		error = stela_linux_pci_publish_device(dev);
	if (error) {
		/* Refusal never takes the caller's host-context ownership. Any exact
         * source record already enrolled retires through the real final put. */
		host->function.release_context = NULL;
		put_device(&dev->dev);
		if (status)
			*status = error;
		return NULL;
	}
	if (status)
		*status = 0;
	return dev;
}

#ifdef STELA_LINUX_PCI_HOST_TEST
#include <linux/bottom_half.h>
#include <stela/page_test.h>
#include <stela/test.h>

#define BASE 0xc0000000ULL
#define BUS_OFFSET 0x100000000ULL

static u8 config[256];
static unsigned int mapped, fail_map_after = ~0U;
static int msi_error, config_error;
static bool on_message;
static unsigned int context_releases;

static void test_release_context(void *context)
{
	assert(context == config && stela_linux_quiescent());
	context_releases++;
}

static int test_read(void *context, int where, int size, u32 *value)
{
	assert(context == config);
	if (config_error)
		return config_error;
	*value = 0;
	memcpy(value, &config[where], (size_t)size);
	return 0;
}

static int test_write(void *context, int where, int size, u32 value)
{
	memcpy(&config[where], &value, (size_t)size);
	return context == config ? 0 : PCIBIOS_SET_FAILED;
}

static bool last_pageless;

static dma_addr_t test_map(void *context, phys_addr_t physical, size_t size,
			   enum dma_data_direction direction, bool pageless)
{
	(void)context;
	(void)size;
	(void)direction;
	last_pageless = pageless;
	if (!fail_map_after)
		return DMA_MAPPING_ERROR;
	fail_map_after--;
	mapped++;
	return physical + BUS_OFFSET;
}

static void test_unmap(void *context, dma_addr_t address, size_t size,
		       enum dma_data_direction direction)
{
	(void)context;
	(void)address;
	(void)size;
	(void)direction;
	mapped--;
}

static int test_enable_msi(void *context)
{
	(void)context;
	on_message = !msi_error;
	return msi_error;
}

static void test_disable_msi(void *context)
{
	(void)context;
	on_message = false;
}

int main(void)
{
	stela_linux_enter(&stela_linux_bare);
	stela_test_register_pages();
	memcpy(&config[PCI_VENDOR_ID], "\xde\x10\x07\x2d", 4);
	memcpy(&config[PCI_CLASS_REVISION], "\xa1\x00\x00\x03", 4);
	memcpy(&config[PCI_SUBSYSTEM_VENDOR_ID], "\x43\x10\x21\x87", 4);
	struct stela_linux_pci_function function = {
		.context = config, .read_config = test_read, .write_config = test_write,
		.dma_map = test_map, .dma_unmap = test_unmap,
		.enable_msi = test_enable_msi, .disable_msi = test_disable_msi,
		.dma_limit = DMA_BIT_MASK(40), .bus = 3, .devfn = PCI_DEVFN(1, 0), .irq = 9,
		.release_context = test_release_context,
	};
	function.regions[0] = (struct stela_linux_pci_region){BASE, 0x1000000, IORESOURCE_MEM};
    int refused_status = 0;
    config_error = PCIBIOS_SET_FAILED;
    assert(!stela_linux_pci_attach(&function, &refused_status));
    assert(refused_status == -EIO && !context_releases);
    config_error = 0;
	struct pci_dev *dev = stela_linux_pci_attach(&function, NULL);

	assert(dev && !stela_linux_pci_bound(dev));

	assert(dev && dev->vendor == 0x10de && dev->device == 0x2d07);
	assert(dev->subsystem_vendor == 0x1043 && dev->subsystem_device == 0x8721);
	assert(dev->class == 0x030000 && dev->revision == 0xa1 && dev->irq == 9);
	assert(pci_resource_start(dev, 0) == BASE && pci_resource_len(dev, 0) == 0x1000000);
	assert(!pci_resource_len(dev, 1));
	assert(!strcmp(dev_name(&dev->dev), "0000:03:01.0"));

	u32 value;
	struct pci_dev other = {.bus = dev->bus, .devfn = PCI_DEVFN(2, 0)};

	assert(pci_read_config_dword(&other, 0, &value) == PCIBIOS_DEVICE_NOT_FOUND);

	/* A mask must cover what the owner hands out. */
	assert(dma_set_mask(&dev->dev, DMA_BIT_MASK(39)) == -EIO);
	assert(!dma_set_mask(&dev->dev, DMA_BIT_MASK(47)));

	dma_addr_t address = dma_map_phys(&dev->dev, 0x2000, 0x100, DMA_TO_DEVICE, 0);

	assert(address == 0x2000 + BUS_OFFSET && mapped == 1 && !last_pageless);
	dma_unmap_phys(&dev->dev, address, 0x100, DMA_TO_DEVICE, 0);
	assert(!mapped);
	address = dma_map_phys(&dev->dev, 0x2000, 0x100, DMA_TO_DEVICE, DMA_ATTR_MMIO);
	assert(last_pageless);
	dma_unmap_phys(&dev->dev, address, 0x100, DMA_TO_DEVICE, DMA_ATTR_MMIO);

	/* A failure part way through releases what it mapped. */
	static struct scatterlist list[2];
	struct page *page = alloc_page(GFP_KERNEL);

	assert(page);
	sg_init_table(list, 2);
	sg_set_page(&list[0], page, 0x10, 0);
	sg_set_page(&list[1], page, 0x10, 0x10);
	struct sg_table table = {.sgl = list, .orig_nents = 2};

	fail_map_after = 1;
	assert(dma_map_sgtable(&dev->dev, &table, DMA_BIDIRECTIONAL, 0) == -EIO && !mapped);
	fail_map_after = ~0U;
	assert(!dma_map_sgtable(&dev->dev, &table, DMA_BIDIRECTIONAL, 0));
	assert(table.nents == 2 && mapped == 2);
	dma_unmap_sgtable(&dev->dev, &table, DMA_BIDIRECTIONAL, 0);
	assert(!mapped);

	dma_addr_t coherent;
	void *cpu = dma_alloc_coherent(&dev->dev, PAGE_SIZE, &coherent, GFP_KERNEL);

	assert(cpu && mapped == 1 && !*(u8 *)cpu && coherent > BUS_OFFSET);
	dma_free_coherent(&dev->dev, PAGE_SIZE, cpu, coherent);
	assert(!mapped);

	msi_error = -ENOSPC;
	assert(pci_enable_msi(dev) == -ENOSPC && !dev->msi_enabled);
	msi_error = 0;
	assert(!pci_enable_msi(dev) && on_message && dev->msi_enabled && dev->irq == 9);
	pci_disable_msi(dev);
	assert(!on_message && !dev->msi_enabled);

	/* Removal owns only registration's reference. An escaped pci_dev keeps
	 * the copied callback table and its original context until its last put. */
	assert(pci_dev_get(dev) == dev);
	stela_linux_pci_remove_device(dev);
	assert(!context_releases);
	assert(!pci_read_config_dword(dev, PCI_VENDOR_ID, &value) && value == 0x2d0710de);
	assert(!pci_write_config_dword(dev, PCI_COMMAND, PCI_COMMAND_MASTER));
	assert(!pci_read_config_dword(dev, PCI_COMMAND, &value) && value == PCI_COMMAND_MASTER);
	pci_dev_put(dev);
	assert(context_releases == 1);
	/* A legacy caller can still retain ownership without a release callback. */
	function.release_context = NULL;
	dev = stela_linux_pci_attach(&function, NULL);
	assert(dev);
	stela_linux_pci_remove_device(dev);
	assert(context_releases == 1);
	/* An atomic final put cannot destroy the host or callback context. */
	function.release_context = test_release_context;
	dev = stela_linux_pci_attach(&function, NULL);
	assert(dev);
	u64 before = stela_linux_pci_quarantined_functions();
	local_bh_disable();
	stela_linux_pci_remove_device(dev);
	local_bh_enable();
	assert(context_releases == 1 && stela_linux_pci_quarantined_functions() == before + 1);
	/* Quiescence under another platform does not authorize destruction of
	 * this function's callback roots through that platform's services. */
	dev = stela_linux_pci_attach(&function, NULL);
	assert(dev && pci_dev_get(dev) == dev);
	stela_linux_pci_remove_device(dev);
	const struct stela_linux_services other_services = {0};
	const struct stela_linux_owner other_owner = {.services = &other_services};
	stela_linux_exit();
	stela_linux_enter(&other_owner);
	pci_dev_put(dev);
	assert(context_releases == 1 && stela_linux_pci_quarantined_functions() == before + 2);
	stela_linux_exit();
	printf("Linux host functions passed\n");
	return 0;
}
#endif
