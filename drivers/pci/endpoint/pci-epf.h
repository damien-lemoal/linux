/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Endpoint *Function* (EPF) internal header file
 */

#ifndef PCI_EPF_H
#define PCI_EPF_H

#include <linux/pci-epf.h>

struct config_group *pci_epf_type_add_cfs(struct pci_epf *epf,
					  struct config_group *group);

#endif /* PCI_EPF_H */
