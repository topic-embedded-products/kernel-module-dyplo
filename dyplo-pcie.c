/*
 * dyplo-pcie.c
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
#include <linux/slab.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0)
# include <linux/pci-aspm.h>
#endif
#include "dyplo-core.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Topic Embedded Products <www.topic.nl>");
MODULE_DESCRIPTION("Driver for Topic Dyplo PCIe device");

#define PCI_DEVICE_ID_TOPIC_BOARD		0x7024
#define PCI_DEVICE_ID_TOPIC_BOARD_DRM		0x7025

#ifndef PCI_VENDOR_ID_ALTERA
#	define PCI_VENDOR_ID_ALTERA		0x1172
#endif

#define DYPLO_CONTROL_BAR 0
#define DYPLO_PCIE_BAR 1

#define AXIBAR2PCIEBAR_0U	0x208
#define AXIBAR2PCIEBAR_0L	0x20C

#define DYPLO_PCI_TYPE_WITHOUT_DRM	0
#define DYPLO_PCI_TYPE_WITH_DRM		1

#define DYPLO_PCIE_DRM_OFFSET		0x10000
#define DYPLO_PCIE_DRM_SIZE		0x10000

static const struct pci_device_id dyplo_pci_ids[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_TOPIC_BOARD),
		.driver_data = DYPLO_PCI_TYPE_WITHOUT_DRM,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_ALTERA, PCI_DEVICE_ID_TOPIC_BOARD),
		.driver_data = DYPLO_PCI_TYPE_WITHOUT_DRM,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_TOPIC_BOARD_DRM),
		.driver_data = DYPLO_PCI_TYPE_WITH_DRM,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_ALTERA, PCI_DEVICE_ID_TOPIC_BOARD_DRM),
		.driver_data = DYPLO_PCI_TYPE_WITH_DRM,
	},
	{ /* End: all zeroes */ }
};

/* Encapsulate dyplo_dev to add some information specific for PCIe */
struct dyplo_drm;

struct dyplo_dev_pci {
	struct dyplo_dev dyplo_dev;
	struct dyplo_drm *drm;
};

static const char dyplo_pci_name[] = "dyplo-pci";

static void dyplo_pci_write_bar_reg(void __iomem *base, unsigned int reg, u32 data)
{
	iowrite32(data, ((__iomem u8*)base) + reg);
}

static u32 dyplo_pci_read_bar_reg(void __iomem *base, unsigned int reg)
{
	return ioread32(((__iomem u8*)base) + reg);
}

static void dyplo_pci_bar_initialize(struct device *device, void __iomem *regs)
{
	u32 reg = dyplo_pci_read_bar_reg(regs, 0x144);

	/* Output some diagnostic link information */
	dev_info(device, "Link %s x%u %s\n",
		(reg & 1) ? "5GT/s" : "2.5GT/s", /* BIT0 = link speed */
		1 << ((reg >> 1) & 0x03),	/* BIT1..2 = number of lanes (1, 2, 4, 8) */
		(reg & BIT(11)) ? "UP" : "DOWN"); /* Uh, I don't really expect to see "down" here */

	/* We use a very simple translation: All 32-bits to address 0 */
	dyplo_pci_write_bar_reg(regs, AXIBAR2PCIEBAR_0U, 0);
	dyplo_pci_write_bar_reg(regs, AXIBAR2PCIEBAR_0L, 0);
}

/* DRM driver handling (begin) */
struct dyplo_drm {
	dev_t devt;
	u32 __iomem *base;
	phys_addr_t mem_start;
	struct cdev cdev_drm;
};

static int dyplo_drm_open(struct inode *inode, struct file *filp)
{
	int status = 0;
	struct dyplo_drm *drm;

	drm = container_of(inode->i_cdev, struct dyplo_drm, cdev_drm);
	filp->private_data = drm; /* for other methods */

	return status;
}

static int dyplo_drm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dyplo_drm *drm = filp->private_data;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return vm_iomap_memory(vma, drm->mem_start, DYPLO_PCIE_DRM_SIZE);
}

static const struct file_operations dyplo_drm_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.mmap = dyplo_drm_mmap,
	.open = dyplo_drm_open,
};

static int dyplo_drm_probe(
	struct dyplo_dev_pci *pci_dev, phys_addr_t mem_start, u32 __iomem *base)
{
	struct device *device = pci_dev->dyplo_dev.device;
	struct device *char_device;
	int retval;

	pci_dev->drm = devm_kzalloc(device, sizeof(*pci_dev->drm), GFP_KERNEL);
	if (!pci_dev->drm)
		return -ENOMEM;

	pci_dev->drm->base = base;
	pci_dev->drm->mem_start = mem_start;

	/* Create /dev/dyplo.. devices */
	retval = alloc_chrdev_region(&pci_dev->drm->devt, 0, 1, "dyplo");
	if (retval < 0) {
		dev_err(device, "alloc_chrdev_region(drm) failed\n");
		return retval;
	}

	cdev_init(&pci_dev->drm->cdev_drm, &dyplo_drm_fops);
	pci_dev->drm->cdev_drm.owner = THIS_MODULE;
	retval = cdev_add(&pci_dev->drm->cdev_drm, pci_dev->drm->devt, 1);
	if (retval) {
		dev_err(device, "cdev_add(drm) failed\n");
		goto failed_device_create;
	}

	char_device = device_create(pci_dev->dyplo_dev.class, device,
			pci_dev->drm->devt, pci_dev, "dyplo-drm");
	if (IS_ERR(char_device)) {
		dev_err(device, "unable to create dyplo-drm device\n");
		retval = PTR_ERR(char_device);
		goto failed_device_create;
	}

	return 0;

failed_device_create:
	unregister_chrdev_region(pci_dev->drm->devt, 1);
	return retval;
}

static void dyplo_drm_remove(struct dyplo_dev_pci *pci_dev)
{
	device_destroy(pci_dev->dyplo_dev.class, pci_dev->drm->devt);
	unregister_chrdev_region(pci_dev->drm->devt, 1);
}
/* DRM driver handling (end) */


static int dyplo_pci_probe(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	struct device *device = &pdev->dev;
	struct dyplo_dev_pci *pci_dev;
	struct dyplo_dev *dev;
	void __iomem *pcie_regs;
	int rc;

	dev_dbg(device, "%s\n", __func__);

	pci_dev = devm_kzalloc(device, sizeof(*pci_dev), GFP_KERNEL);
	if (!pci_dev)
		return -ENOMEM;
	pci_set_drvdata(pdev, pci_dev);
	dev = &pci_dev->dyplo_dev;

	rc = pcim_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev,
			"pcim_enable_device() failed. Aborting.\n");
		return rc;
	}

	/* resource configuration */
	if (!(pci_resource_flags(pdev, DYPLO_CONTROL_BAR) & IORESOURCE_MEM)) {
		dev_err(device,
			"Incorrect BAR configuration. Aborting.\n");
		return -ENODEV;
	}

	rc = pcim_iomap_regions(pdev,
		BIT(DYPLO_CONTROL_BAR) | BIT(DYPLO_PCIE_BAR), dyplo_pci_name);
	if (rc) {
		dev_err(device,
			"pcim_iomap_regions() failed. Aborting.\n");
		return rc;
	}
	dev->base = pcim_iomap_table(pdev)[DYPLO_CONTROL_BAR];
	dev->mem = devm_kzalloc(device, sizeof(*dev->mem), GFP_KERNEL);
	if (!dev->mem)
		return -ENOMEM;
	dev->mem->start = pci_resource_start(pdev, DYPLO_CONTROL_BAR);
	dev->mem->end = pci_resource_end(pdev, DYPLO_CONTROL_BAR);
	dev->mem->flags = IORESOURCE_MEM;

	pcie_regs = pcim_iomap_table(pdev)[DYPLO_PCIE_BAR];
	dyplo_pci_bar_initialize(device, pcie_regs);

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

	rc = dyplo_core_probe(device, dev);
	if (rc < 0)
		return rc;

	if (ent->driver_data == DYPLO_PCI_TYPE_WITH_DRM) {
		rc = dyplo_drm_probe(pci_dev,
			pci_resource_start(pdev, DYPLO_PCIE_BAR) +
				DYPLO_PCIE_DRM_OFFSET,
			(u32 __iomem *)((u8 __iomem *)pcie_regs +
				DYPLO_PCIE_DRM_OFFSET));
		if (rc)
			dev_err(device, "Failed to initialize DRM: %d\n", rc);
	}

	return 0;
}

static void dyplo_pci_remove(struct pci_dev *pdev)
{
	struct device *device = &pdev->dev;
	struct dyplo_dev_pci *pci_dev = pci_get_drvdata(pdev);

	if (pci_dev->drm)
		dyplo_drm_remove(pci_dev);

	dyplo_core_remove(device, &pci_dev->dyplo_dev);
}

MODULE_DEVICE_TABLE(pci, dyplo_pci_ids);

static struct pci_driver dyplo_pci_driver = {
	.name = dyplo_pci_name,
	.id_table = dyplo_pci_ids,
	.probe = dyplo_pci_probe,
	.remove = dyplo_pci_remove,
};

module_pci_driver(dyplo_pci_driver);
