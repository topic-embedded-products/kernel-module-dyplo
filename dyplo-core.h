/*
 * dyplo-core.h
 *
 * Dyplo loadable kernel module.
 *
 * (C) Copyright 2013-2015 Topic Embedded Products B.V. (http://www.topic.nl).
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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/wait.h>

#define ICAP_NOT_AVAILABLE	((u8)-1)

struct dyplo_dev; /* forward */

struct dyplo_config_dev
{
	struct dyplo_dev* parent; /* Owner of this struct */
	u32 __iomem *base;
	u32 __iomem *control_base;
	mode_t open_mode; /* Only FMODE_READ and FMODE_WRITE */
	irqreturn_t(*isr)(struct dyplo_dev *dev, struct dyplo_config_dev *cfg_dev); /* IRQ handler, if any */
	void* private_data; /* Extra information for sub-device */
};

struct dyplo_dev
{
	struct device *device;
	struct cdev cdev_control;
	struct cdev cdev_config;
	dev_t devt;
	dev_t devt_last;
	struct class *class;
	struct semaphore fop_sem;
	struct resource *mem;
	u32 __iomem *base;
	int irq;
	int number_of_config_devices;
	unsigned int stream_id_width;
	struct dyplo_config_dev *config_devices;
	u8 count_fifo_write_devices;
	u8 count_fifo_read_devices;
	u8 number_of_dma_devices;
	u8 icap_device_index;
};

int dyplo_core_remove(struct device *device, struct dyplo_dev *dev);

int dyplo_core_probe(struct device *device, struct dyplo_dev *dev);
