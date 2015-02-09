/*
 * dyplo-of.c
 *
 * Dyplo loadable kernel module.
 *
 * (C) Copyright 2013,2014 Topic Embedded Products B.V. (http://www.topic.nl).
 * All rights reserved.
 *
 * This file is part of kernel-module-dyplo.
 * kernel-module-dyplo is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * kernel-module-dyplo is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with <product name>.  If not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA or see <http://www.gnu.org/licenses/>.
 *
 * You can contact Topic by electronic mail via info@topic.nl or via
 * paper mail at the following address: Postbus 440, 5680 AK Best, The Netherlands.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/slab.h>
#include "dyplo-core.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Topic Embedded Products <www.topic.nl>");
MODULE_DESCRIPTION("Driver for Topic Dyplo PCIe device");

#define PCI_DEVICE_ID_TOPIC_BOARD		0x7022

#ifndef PCI_VENDOR_ID_ALTERA
#	define PCI_VENDOR_ID_ALTERA		0x1172
#endif

static const struct pci_device_id dyplo_pci_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_TOPIC_BOARD)},
	{PCI_DEVICE(PCI_VENDOR_ID_ALTERA, PCI_DEVICE_ID_TOPIC_BOARD)},
	{ /* End: all zeroes */ }
};

static const char dyplo_pci_name[] = "dyplo-pci";

static int dyplo_pci_probe(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	struct device *device = &pdev->dev;
	struct dyplo_dev *dev;
	int rc;

	dev_info(device, "%s\n", __func__);

	dev = devm_kzalloc(device, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	pci_set_drvdata(pdev, dev);

	rc = pcim_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev,
			"pcim_enable_device() failed. Aborting.\n");
		return rc;
	}

	/* resource configuration */

	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM)) {
		dev_err(device,
			"Incorrect BAR configuration. Aborting.\n");
		return -ENODEV;
	}

	rc = pcim_iomap_regions(pdev, BIT(0), dyplo_pci_name);
	if (rc) {
		dev_err(device,
			"pcim_iomap_regions() failed. Aborting.\n");
		return rc;
	}
	dev->base = pcim_iomap_table(pdev)[0];
	dev->mem = devm_kzalloc(device, sizeof(*dev->mem), GFP_KERNEL);
	if (!dev->mem)
		return -ENOMEM;
	dev->mem->start = pci_resource_start(pdev, 0);
	dev->mem->end = pci_resource_end(pdev, 0);
	dev->mem->flags = IORESOURCE_MEM;

	pci_set_master(pdev);

	/* Set up a single MSI interrupt */
	if (pci_enable_msi(pdev)) {
		dev_err(device,
			"Failed to enable MSI interrupts. Aborting.\n");
		return -ENODEV;
	}

	dev->irq = pdev->irq;
	
	if (pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
		dev_err(device, "Failed to set DMA mask. Aborting.\n");
		return -ENODEV;
	}

	return dyplo_core_probe(device, dev);
}

static void dyplo_pci_remove(struct pci_dev *pdev)
{
	struct device *device = &pdev->dev;
	struct dyplo_dev *dev = pci_get_drvdata(pdev);

	dyplo_core_remove(device, dev);
}

MODULE_DEVICE_TABLE(pci, dyplo_pci_ids);

static struct pci_driver dyplo_pci_driver = {
	.name = dyplo_pci_name,
	.id_table = dyplo_pci_ids,
	.probe = dyplo_pci_probe,
	.remove = dyplo_pci_remove,
};

module_pci_driver(dyplo_pci_driver);
