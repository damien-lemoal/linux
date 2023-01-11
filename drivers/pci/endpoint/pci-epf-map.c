// SPDX-License-Identifier: GPL-2.0
/*
 * PCI Endpoint *Function* Address Space Management
 *
 * Copyright (C) 2019 SiFive
 */

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

int pci_epf_map_alloc_region(struct pci_epf_map *map, struct pci_epf *epf,
			     const struct pci_epc_features *features)
{
	phys_addr_t phys_base;
	void __iomem *virt_base;
	size_t align, size;

	if (map->pci.phys_base)
		return -EALREADY;

	align = (features && features->align) ? features->align : PAGE_SIZE;
	size = (map->size < align) ? (align << 1) : map->size;

	virt_base = pci_epc_mem_alloc_addr(epf->epc, &phys_base, size);
	if (!virt_base)
		return -ENOMEM;

	map->epf = epf;
	map->align = align;
	map->pci.size = size;
	map->pci.virt_base = virt_base;
	map->pci.phys_base = phys_base;
	return 0;
}
EXPORT_SYMBOL_GPL(pci_epf_map_alloc_region);

void pci_epf_map_free_region(struct pci_epf_map *map)
{
	if (!map->pci.phys_base)
		return;

	pci_epc_mem_free_addr(map->epf->epc, map->pci.phys_base,
			      map->pci.virt_base, map->pci.size);
	map->pci.phys_base = 0;
}
EXPORT_SYMBOL_GPL(pci_epf_map_free_region);

int pci_epf_map_enable(struct pci_epf_map *map)
{
	struct pci_epf *epf = map->epf;
	int ret;

	if (!map->pci.phys_base)
		return -ENOMEM;

	if (map->pci.phys_addr)
		return -EALREADY;

	map->host.phys_base = map->host.phys_addr;
	if (map->align > PAGE_SIZE)
		map->host.phys_base &= ~(map->align-1);

	map->host.phys_end = map->host.phys_base + map->pci.size - 1;

	map->offset = map->host.phys_addr - map->host.phys_base;
	if (map->offset + map->size > map->pci.size)
		return -ERANGE;

	ret = pci_epc_map_addr(epf->epc, epf->func_no, epf->vfunc_no,
			       map->pci.phys_base, map->host.phys_base,
			       map->pci.size);
	if (ret)
		return ret;

	map->pci.virt_addr = map->pci.virt_base + map->offset;
	map->pci.phys_addr = map->pci.phys_base + map->offset;

	return 0;
}
EXPORT_SYMBOL_GPL(pci_epf_map_enable);

void pci_epf_map_disable(struct pci_epf_map *map)
{
	if (!map->pci.phys_addr)
		return;

	pci_epc_unmap_addr(map->epf->epc, map->epf->func_no,
			   map->epf->vfunc_no, map->pci.phys_base);
	map->pci.phys_addr = 0;
}
EXPORT_SYMBOL_GPL(pci_epf_map_disable);

int pci_epf_map(struct pci_epf_map *map, struct pci_epf *epf,
		const struct pci_epc_features *features)
{
	int ret;

	ret = pci_epf_map_alloc_region(map, epf, features);
	if (ret) {
		dev_err(&epf->dev, "Failed to allocate address map\n");
		return ret;
	}

	ret = pci_epf_map_enable(map);
	if (ret) {
		dev_err(&epf->dev, "Failed to enable address map\n");
		pci_epf_map_free_region(map);
		return ret;
	}


	return 0;
}
EXPORT_SYMBOL_GPL(pci_epf_map);

void pci_epf_unmap(struct pci_epf_map *map)
{
	pci_epf_map_disable(map);
	pci_epf_map_free_region(map);
	memset(map, 0, sizeof(*map));
}
EXPORT_SYMBOL_GPL(pci_epf_unmap);

int pci_epf_map_check_fit(struct pci_epf_map *map, u64 addr, u64 end)
{
	return addr >= map->host.phys_base && end <= map->host.phys_end;
}
EXPORT_SYMBOL_GPL(pci_epf_map_check_fit);

void __iomem *pci_epf_map_get(struct pci_epf_map *map, u64 addr)
{
	return addr - map->host.phys_base + map->pci.virt_base;
}
EXPORT_SYMBOL_GPL(pci_epf_map_get);

