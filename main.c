/*
 * main.c
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "dyplo.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Topic Embedded Products <www.topic.nl>");

/* When defined, copies data directly to/from user space instead of
 * bouncing via an intermediate kernel buffer. */
/* #define ALLOW_DIRECT_USER_IOMEM_TRANSFERS */

static const char DRIVER_CLASS_NAME[] = "dyplo";
static const char DRIVER_CONTROL_NAME[] = "dyploctl";
static const char DRIVER_CONFIG_NAME[] = "dyplocfg%d";
static const char DRIVER_FIFO_CLASS_NAME[] = "dyplo-fifo";
static const char DRIVER_FIFO_WRITE_NAME[] = "dyplow%d";
static const char DRIVER_FIFO_READ_NAME[] = "dyplor%d";

/* How to do IO. We rarely need any memory barriers, so add a "quick"
 * version that skips the memory barriers. */
#define ioread32_quick	__raw_readl
#define iowrite32_quick	__raw_writel

struct dyplo_dev; /* forward */

struct dyplo_config_dev
{
	struct dyplo_dev* parent;
	u32 __iomem *base;
	u32 __iomem *control_base;
	mode_t open_mode; /* Only FMODE_READ and FMODE_WRITE */
};

struct dyplo_fifo_dev
{
	struct dyplo_config_dev* config_parent;
	wait_queue_head_t fifo_wait_queue; /* So the IRQ handler can notify waiting threads */
	int index;
	unsigned int words_transfered;
	unsigned int poll_treshold;
	bool is_open;
};

struct dyplo_dev
{
	struct cdev cdev_control;
	struct cdev cdev_config;
	struct cdev cdev_fifo_write;
	struct cdev cdev_fifo_read;
	dev_t devt;
	struct class *class;
	struct semaphore fop_sem;
	struct resource *mem;
	u32 __iomem *base;
	int irq;
	int number_of_config_devices;
	struct dyplo_config_dev *config_devices;
	int number_of_fifo_write_devices;
	struct dyplo_fifo_dev *fifo_write_devices;
	/* fifo_read_devices actually points to
	 * fifo_write_devices+number_of_fifo_write_devices */
	int number_of_fifo_read_devices;
	struct dyplo_fifo_dev *fifo_read_devices;
	/* Need to know wich device is the CPU-PL interface */
	struct dyplo_config_dev* fifo_config_device;
};

union dyplo_route_item_u {
	unsigned int route;
	struct dyplo_route_item_t route_item;
};

/* Relative offset of the configuration node in memory map */
static unsigned int dyplo_get_config_mem_offset(const struct dyplo_config_dev *cfg_dev)
{
	return ((char*)cfg_dev->base - (char*)cfg_dev->parent->base);
}
/* 0-based index of the config node */
static unsigned int dyplo_get_config_index(const struct dyplo_config_dev *cfg_dev)
{
	return (((char*)cfg_dev->base - (char*)cfg_dev->parent->base) / DYPLO_CONFIG_SIZE) - 1;
}

static bool dyplo_is_cpu_node(const struct dyplo_config_dev *cfg_dev)
{
	u32 device_id = ioread32_quick(cfg_dev->control_base + (DYPLO_REG_ID>>2));
	return
		(device_id & DYPLO_REG_ID_MASK_VENDOR_PRODUCT) == DYPLO_REG_ID_PRODUCT_TOPIC_CPU;
}
static int dyplo_number_of_input_queues(const struct dyplo_config_dev *cfg_dev)
{
	return ioread32_quick(cfg_dev->control_base + (DYPLO_REG_CPU_FIFO_WRITE_COUNT>>2));
}
static int dyplo_number_of_output_queues(const struct dyplo_config_dev *cfg_dev)
{
	return ioread32_quick(cfg_dev->control_base + (DYPLO_REG_CPU_FIFO_READ_COUNT>>2));
}

static int dyplo_ctl_open(struct inode *inode, struct file *filp)
{
	int status = 0;
	struct dyplo_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct dyplo_dev, cdev_control);
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	filp->private_data = dev; /* for other methods */
	up(&dev->fop_sem);
	return status;
}

static int dyplo_ctl_release(struct inode *inode, struct file *filp)
{
	//struct dyplo_dev *dev = filp->private_data;
	return 0;
}

static ssize_t dyplo_ctl_write (struct file *filp, const char __user *buf, size_t count,
	loff_t *f_pos)
{
	int status;
	struct dyplo_dev *dev = filp->private_data;
	u32 __iomem *mapped_memory = dev->base;
	size_t offset;

	/* EOF when past our area */
	if (*f_pos >= DYPLO_CONFIG_SIZE)
		return 0;

	if (count < 4) /* Do not allow read or write below word size */
		return -EINVAL;

	offset = ((size_t)*f_pos) & ~0x03; /* Align to word size */
	count &= ~0x03;
	if ((offset + count) > DYPLO_CONFIG_SIZE)
		count = DYPLO_CONFIG_SIZE - offset;

	if (copy_from_user(mapped_memory + (offset >> 2), buf, count))
	{
		status = -EFAULT;
	}
	else
	{
		status = count;
		*f_pos = offset + count;
	}

	return status;
}

static ssize_t dyplo_ctl_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	int status;
	struct dyplo_dev *dev = filp->private_data;
	u32 __iomem *mapped_memory = dev->base;
	size_t offset;

	/* EOF when past our area */
	if (*f_pos >= DYPLO_CONFIG_SIZE)
		return 0;

	offset = ((size_t)*f_pos) & ~0x03; /* Align to word size */
	count &= ~0x03;
	if ((offset + count) > DYPLO_CONFIG_SIZE)
		count = DYPLO_CONFIG_SIZE - offset;

	if (copy_to_user(buf, mapped_memory + (offset >> 2), count))
	{
		status = -EFAULT;
	}
	else
	{
		status = count;
		*f_pos = offset + count;
	}

	return status;
}

loff_t dyplo_ctl_llseek(struct file *filp, loff_t off, int whence)
{
    loff_t newpos;

    switch(whence) {
      case 0: /* SEEK_SET */
        newpos = off;
        break;

      case 1: /* SEEK_CUR */
        newpos = filp->f_pos + off;
        break;

      case 2: /* SEEK_END */
        newpos = DYPLO_CONFIG_SIZE + off;
        break;

      default: /* can't happen */
        return -EINVAL;
    }
    if (newpos < 0) return -EINVAL;
    if (newpos > DYPLO_CONFIG_SIZE) return -EINVAL;
    filp->f_pos = newpos;
    return newpos;
}


static int dyplo_ctl_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dyplo_dev *dev = filp->private_data;

	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long physical = dev->mem->start + off;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	if (vsize > (DYPLO_CONFIG_SIZE - off))
		return -EINVAL; /*  spans too high */
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, physical >> PAGE_SHIFT, vsize, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static void dyplo_ctl_route_remove_dst(struct dyplo_dev *dev, u32 route)
{
	int ctl_index;
	int queue_index;
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		int __iomem *ctl_route_base_out =
			dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		{
			if (ctl_route_base_out[queue_index] == route) {
				pr_debug("removed route %d,%d->%d,%d\n", ctl_index, queue_index, (route>>5)-1, route&0x1f);
				ctl_route_base_out[queue_index] = 0;
			}
		}
	}
}

static int dyplo_ctl_route_add(struct dyplo_dev *dev, struct dyplo_route_item_t route)
{
	int __iomem* dst_control_addr;
	u32 dst_route;

	pr_debug("%s %d,%d->%d,%d\n", __func__,
		route.srcNode, route.srcFifo, route.dstNode, route.dstFifo);
	if ((route.srcNode >= dev->number_of_config_devices) ||
	    (route.dstNode >= dev->number_of_config_devices))
	{
		pr_debug("%s: Invalid source or destination\n", __func__);
	    return -EINVAL;
	}
	dst_route = ((route.dstNode+1) << 5) | route.dstFifo;
	dyplo_ctl_route_remove_dst(dev, dst_route);
	/* Setup route. The PL assumes that "0" is the control node, hence
	 * the "+1" in config node indices */
	dst_control_addr =
		dev->config_devices[route.srcNode].control_base +
		(DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2) +
		route.srcFifo;
	pr_debug("%s (%d) @ %p: %x\n", __func__, route.srcNode,
		dst_control_addr, dst_route);
	*dst_control_addr = dst_route;
	return 0;
}

static int dyplo_ctl_route_add_from_user(struct dyplo_dev *dev, const struct dyplo_route_t __user *uroutes)
{
	int status = 0;
	struct dyplo_route_t routes;
	if (copy_from_user(&routes, uroutes, sizeof(routes)))
		return -EFAULT;
	while (routes.n_routes--)
	{
		union dyplo_route_item_u u;
		status = get_user(u.route, (unsigned int*)routes.proutes);
		if (status)
			break;
		status = dyplo_ctl_route_add(dev, u.route_item);
		if (status)
			break;
		++routes.proutes;
	}
	return status;
}

static int dyplo_ctl_route_get_from_user(struct dyplo_dev *dev, struct dyplo_route_t __user *uroutes)
{
	int status = 0;
	int nr = 0;
	int ctl_index;
	int queue_index;
	struct dyplo_route_t routes;
	if (copy_from_user(&routes, uroutes, sizeof(routes)))
		return -EFAULT;
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		int __iomem *ctl_route_base =
			dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		{
			unsigned int route = ctl_route_base[queue_index];
			if (route)
			{
				int src_ctl_index = route >> 5;
				if (src_ctl_index > 0)
				{
					int src_index = route & 0x1F;
					if (nr >= routes.n_routes)
						return nr; /* No room for more, quit */
					route = (ctl_index << 24) | (queue_index << 16) | ((src_ctl_index-1) << 8) | (src_index);
					pr_debug("%s: cfg=%d 0x%x @ %p\n", __func__, ctl_index, route, ctl_route_base + queue_index);
					status = put_user(route, (unsigned int*)routes.proutes + nr);
					if (status)
						return status;
					++nr;
				}
			}
		}
	}
	return status ? status : nr; /* Return number of items found */
}

static int dyplo_ctl_route_delete(struct dyplo_dev *dev, int ctl_index_to_delete)
{
	int queue_index;
	int ctl_index;
	const int match = (ctl_index_to_delete+1) << 5;
	const int number_of_fifos =
		dyplo_number_of_output_queues(&dev->config_devices[ctl_index_to_delete]);
	int __iomem *ctl_route_base_out = dev->config_devices[ctl_index_to_delete].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
	for (queue_index = 0; queue_index < number_of_fifos; ++queue_index) {
		ctl_route_base_out[queue_index] = 0;
	}
	for (ctl_index = 0; ctl_index < ctl_index_to_delete; ++ctl_index)
	{
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		int __iomem *ctl_route_base_out =
			dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		{
			if ((ctl_route_base_out[queue_index] & 0xFFE0) == match) {
				ctl_route_base_out[queue_index] = 0;
			}
		}
	}
	for (ctl_index = ctl_index_to_delete+1; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		int __iomem *ctl_route_base_out =
			dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		{
			if ((ctl_route_base_out[queue_index] & 0xFFE0) == match) {
				ctl_route_base_out[queue_index] = 0;
			}
		}
	}
	return 0;
}

static int dyplo_ctl_route_clear(struct dyplo_dev *dev)
{
	int ctl_index;
	int queue_index;
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		int __iomem *ctl_route_base;
		/* Remove outgoing routes */
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		ctl_route_base = dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		{
			ctl_route_base[queue_index] = 0;
		}
	}
	return 0;
}

static long dyplo_ctl_ioctl_impl(struct dyplo_dev *dev, unsigned int cmd, unsigned long arg)
{
	int status;

	/* pr_debug("%s(%x, %lx)\n", __func__, cmd, arg); */
	if (_IOC_TYPE(cmd) != DYPLO_IOC_MAGIC)
		return -ENOTTY;

	/* Verify read/write access to user memory early on */
	if (_IOC_DIR(cmd) & _IOC_READ) 	{
		/* IOC and VERIFY use different perspectives, hence the "WRITE" and "READ" confusion */
		if (unlikely(!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))))
			return -EFAULT;
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (unlikely(!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))))
			return -EFAULT;
	}

	switch (_IOC_NR(cmd))
	{
		case DYPLO_IOC_ROUTE_CLEAR: /* Remove all routes */
			status = dyplo_ctl_route_clear(dev);
			break;
		case DYPLO_IOC_ROUTE_SET: /* Set routes. */
			status = dyplo_ctl_route_add_from_user(dev, (struct dyplo_route_t __user *)arg);
			break;
		case DYPLO_IOC_ROUTE_GET: /* Get routes. */
			status = dyplo_ctl_route_get_from_user(dev, (struct dyplo_route_t __user *)arg);
			break;
		case DYPLO_IOC_ROUTE_TELL: /* Tell route: Adds a single route entry */
		{
			union dyplo_route_item_u u;
			u.route = arg;
			status = dyplo_ctl_route_add(dev, u.route_item);
			break;
		}
		case DYPLO_IOC_ROUTE_DELETE: /* Remove routes to a node */
			status = dyplo_ctl_route_delete(dev, arg);
			break;
		case DYPLO_IOC_BACKPLANE_STATUS:
			status = *(dev->base + (DYPLO_REG_BACKPLANE_ENABLE_STATUS>>2)) >> 1;
			break;
		case DYPLO_IOC_BACKPLANE_ENABLE:
			*(dev->base + (DYPLO_REG_BACKPLANE_ENABLE_SET>>2)) = (arg << 1);
			/* Read back the register to assure that the transaction is complete */
			status = *(dev->base + (DYPLO_REG_BACKPLANE_ENABLE_STATUS>>2)) >> 1;
			break;
		case DYPLO_IOC_BACKPLANE_DISABLE:
			*(dev->base + (DYPLO_REG_BACKPLANE_ENABLE_CLR>>2)) = (arg << 1);
			/* Read back the register to assure that the transaction is complete */
			status = *(dev->base + (DYPLO_REG_BACKPLANE_ENABLE_STATUS>>2)) >> 1;
			break;
		default:
			printk(KERN_WARNING "DYPLO ioctl unknown command: %d (arg=0x%lx).\n", _IOC_NR(cmd), arg);
			status = -ENOTTY;
	}

	return status;
}

static long dyplo_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_dev *dev = filp->private_data;
	if (unlikely(dev == NULL))
		return -ENODEV;
	return dyplo_ctl_ioctl_impl(dev, cmd, arg);
}

static struct file_operations dyplo_ctl_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_ctl_read,
	.write = dyplo_ctl_write,
	.llseek = dyplo_ctl_llseek,
	.mmap = dyplo_ctl_mmap,
	.unlocked_ioctl = dyplo_ctl_ioctl,
	.open = dyplo_ctl_open,
	.release = dyplo_ctl_release,
};

static int dyplo_cfg_open(struct inode *inode, struct file *filp)
{
	struct dyplo_dev *dev =  container_of(inode->i_cdev, struct dyplo_dev, cdev_config);
	int index = iminor(inode) - 1;
	struct dyplo_config_dev *cfg_dev = &dev->config_devices[index];
	int status = 0;
	mode_t rw_mode = filp->f_mode & (FMODE_READ | FMODE_WRITE);

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	/* Allow only one open, or one R and one W */

	if (rw_mode & cfg_dev->open_mode) {
		status = -EBUSY;
		goto exit_open;
	}
	cfg_dev->open_mode |= rw_mode; /* Set in-use bits */
	filp->private_data = cfg_dev; /* for other methods */
exit_open:
	up(&dev->fop_sem);
	return status;
}

static int dyplo_cfg_release(struct inode *inode, struct file *filp)
{
	struct dyplo_config_dev *cfg_dev = filp->private_data;
	struct dyplo_dev *dev = cfg_dev->parent;

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	cfg_dev->open_mode &= ~filp->f_mode; /* Clear in use bits */
	up(&dev->fop_sem);
	return 0;
}

static ssize_t dyplo_cfg_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	int status;
	struct dyplo_config_dev *cfg_dev = filp->private_data;
	u32 __iomem *mapped_memory = cfg_dev->base;
	size_t offset;

	/* EOF when past our area */
	if (*f_pos >= DYPLO_CONFIG_SIZE)
		return 0;

	offset = ((size_t)*f_pos) & ~0x03; /* Align to word size */
	count &= ~0x03;
	if ((offset + count) > DYPLO_CONFIG_SIZE)
		count = DYPLO_CONFIG_SIZE - offset;

	if (unlikely(copy_to_user(buf, mapped_memory + (offset >> 2), count)))
	{
		status = -EFAULT;
	}
	else
	{
		status = count;
		*f_pos = offset + count;
	}

	return status;
}

static ssize_t dyplo_cfg_write (struct file *filp, const char __user *buf, size_t count,
	loff_t *f_pos)
{
	int status;
	struct dyplo_config_dev *cfg_dev = filp->private_data;
	u32 __iomem *mapped_memory = cfg_dev->base;
	size_t offset;

	/* EOF when past our area */
	if (*f_pos >= DYPLO_CONFIG_SIZE)
		return 0;

	if (count < 4) /* Do not allow read or write below word size */
		return -EINVAL;

	offset = ((size_t)*f_pos) & ~0x03; /* Align to word size */
	count &= ~0x03;
	if ((offset + count) > DYPLO_CONFIG_SIZE)
		count = DYPLO_CONFIG_SIZE - offset;

	if (unlikely(copy_from_user(mapped_memory + (offset >> 2), buf, count)))
	{
		status = -EFAULT;
	}
	else
	{
		status = count;
		*f_pos = offset + count;
	}

	return status;
}

loff_t dyplo_cfg_llseek(struct file *filp, loff_t off, int whence)
{
    loff_t newpos;

    switch(whence) {
      case 0: /* SEEK_SET */
        newpos = off;
        break;

      case 1: /* SEEK_CUR */
        newpos = filp->f_pos + off;
        break;

      case 2: /* SEEK_END */
        newpos = DYPLO_CONFIG_SIZE + off;
        break;

      default: /* can't happen */
        return -EINVAL;
    }
    if (newpos < 0) return -EINVAL;
    if (newpos > DYPLO_CONFIG_SIZE) return -EINVAL;
    filp->f_pos = newpos;
    return newpos;
}

static int dyplo_cfg_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dyplo_config_dev *cfg_dev = filp->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long physical =
		cfg_dev->parent->mem->start +
		dyplo_get_config_mem_offset(cfg_dev) +
		off;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	if (vsize > (DYPLO_CONFIG_SIZE - off))
		return -EINVAL; /*  spans too high */
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, physical >> PAGE_SHIFT, vsize, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static long dyplo_cfg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_config_dev *cfg_dev = filp->private_data;
	int status;

	if (unlikely(cfg_dev == NULL))
		return -ENODEV;
	if (_IOC_TYPE(cmd) != DYPLO_IOC_MAGIC)
		return -ENOTTY;

	switch (_IOC_NR(cmd))
	{
		case DYPLO_IOC_BACKPLANE_STATUS:
			{
				int index = dyplo_get_config_index(cfg_dev);
				status = *(cfg_dev->parent->base + (DYPLO_REG_BACKPLANE_ENABLE_STATUS>>2)) >> 1;
				status &= (1 << index);
			}
			break;
		case DYPLO_IOC_BACKPLANE_ENABLE:
			{
				int index = dyplo_get_config_index(cfg_dev);
				*(cfg_dev->parent->base + (DYPLO_REG_BACKPLANE_ENABLE_SET>>2)) = (1 << (index+1));
				/* Read back the register to assure that the transaction is complete */
				status = *(cfg_dev->parent->base + (DYPLO_REG_BACKPLANE_ENABLE_STATUS>>2)) >> 1;
			}
			break;
		case DYPLO_IOC_BACKPLANE_DISABLE:
			{
				int index = dyplo_get_config_index(cfg_dev);
				*(cfg_dev->parent->base + (DYPLO_REG_BACKPLANE_ENABLE_CLR>>2)) = (1 << (index+1));
				/* Read back the register to assure that the transaction is complete */
				status = *(cfg_dev->parent->base + (DYPLO_REG_BACKPLANE_ENABLE_STATUS>>2)) >> 1;
			}
			break;
		case DYPLO_IOC_RESET_FIFO_WRITE:
			iowrite32_quick(arg, (cfg_dev->control_base + (DYPLO_REG_FIFO_RESET_WRITE/4)));
			status = 0;
			break;
		case DYPLO_IOC_RESET_FIFO_READ:
			iowrite32_quick(arg, (cfg_dev->control_base + (DYPLO_REG_FIFO_RESET_READ/4)));
			status = 0;
			break;
		default:
			printk(KERN_WARNING "DYPLO ioctl unknown command: %d (arg=0x%lx).\n", _IOC_NR(cmd), arg);
			status = -ENOTTY;
	}

	return status;
}

static struct file_operations dyplo_cfg_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_cfg_read,
	.write = dyplo_cfg_write,
	.llseek = dyplo_cfg_llseek,
	.mmap = dyplo_cfg_mmap,
	.unlocked_ioctl = dyplo_cfg_ioctl,
	.open = dyplo_cfg_open,
	.release = dyplo_cfg_release,
};

/* Utilities for fifo functions */
static int __iomem * dyplo_fifo_memory_location(struct dyplo_fifo_dev *fifo_dev)
{
	struct dyplo_config_dev *cfg_dev = fifo_dev->config_parent;
	return
		cfg_dev->base + (fifo_dev->index * (DYPLO_FIFO_MEMORY_SIZE>>2));
}

static int dyplo_fifo_read_level(struct dyplo_fifo_dev *fifo_dev)
{
	int index = fifo_dev->index;
	int __iomem *control_base =
		fifo_dev->config_parent->control_base;
	int result = ioread32_quick(control_base + (DYPLO_REG_FIFO_READ_LEVEL_BASE>>2) + index);
	pr_debug("%s index=%d result=%d @ %p\n", __func__, index, result,
		(control_base + (DYPLO_REG_FIFO_READ_LEVEL_BASE>>2) + index));
	return result;
}

static void dyplo_fifo_read_enable_interrupt(struct dyplo_fifo_dev *fifo_dev, int thd)
{
	int index = fifo_dev->index;
	int __iomem *control_base =
		fifo_dev->config_parent->control_base;
	if (thd > (DYPLO_FIFO_READ_SIZE*2)/4)
		thd = (DYPLO_FIFO_READ_SIZE*2)/4;
	else if (thd)
		--thd; /* Treshold of "15" will alert when 16 words are present in the FIFO */
	pr_debug("%s index=%d thd=%d mask=%x\n", __func__, index, thd,
		*(control_base + (DYPLO_REG_FIFO_READ_IRQ_MASK>>2)));
	iowrite32(thd, control_base + (DYPLO_REG_FIFO_READ_THD_BASE>>2) + index);
	iowrite32(BIT(index), control_base + (DYPLO_REG_FIFO_READ_IRQ_SET>>2));
}

static int dyplo_fifo_read_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct dyplo_dev *dev = container_of(inode->i_cdev, struct dyplo_dev, cdev_fifo_read);
	int index = iminor(inode) - dev->number_of_config_devices - 1;
	/* Using "write" devices looks strange here, but the index starts
	 * there and continues into the fifo_read_devices */
	struct dyplo_fifo_dev *fifo_dev = &dev->fifo_write_devices[index];

	if (filp->f_mode & FMODE_WRITE) /* read-only device */
		return -EINVAL;
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	if (fifo_dev->is_open) {
		result = -EBUSY;
		goto error;
	}
	fifo_dev->is_open = true;
	fifo_dev->poll_treshold = 1;
	filp->private_data = fifo_dev;
error:
	up(&dev->fop_sem);
	return result;
}

static int dyplo_fifo_read_release(struct inode *inode, struct file *filp)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	fifo_dev->is_open = false;
	return 0;
}

static ssize_t dyplo_fifo_read_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	int __iomem *mapped_memory = dyplo_fifo_memory_location(fifo_dev);
	int status = 0;
	size_t len = 0;
#ifndef ALLOW_DIRECT_USER_IOMEM_TRANSFERS
	int kernel_buffer[DYPLO_FIFO_READ_MAX_BURST_SIZE/sizeof(int)];
#endif
	pr_debug("%s(%d)\n", __func__, count);

	if (count < 4) /* Do not allow read or write below word size */
		return -EINVAL;

	count &= ~0x03; /* Align to words */

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	while (count)
	{
		int words_available;
		size_t bytes;
		if (filp->f_flags & O_NONBLOCK) {
			words_available = dyplo_fifo_read_level(fifo_dev);
			if (!words_available) {
				/* Non-blocking IO, return what we have */
				if (len)
					break;
				/* nothing copied yet, notify caller */
				status = -EAGAIN;
				goto error;
			}
		}
		else {
			DEFINE_WAIT(wait);
			for (;;) {
				prepare_to_wait(&fifo_dev->fifo_wait_queue, &wait, TASK_INTERRUPTIBLE);
				words_available = dyplo_fifo_read_level(fifo_dev);
				if (words_available)
					break; /* Done waiting */
				if (!signal_pending(current)) {
					dyplo_fifo_read_enable_interrupt(fifo_dev, count >> 2);
					schedule();
					continue;
				}
				status = -ERESTARTSYS;
				break;
			}
			finish_wait(&fifo_dev->fifo_wait_queue, &wait);
			if (status)
				goto error;
		}
		do {
			unsigned int words;
			bytes = words_available << 2;
			if (bytes > DYPLO_FIFO_READ_MAX_BURST_SIZE)
				bytes = DYPLO_FIFO_READ_MAX_BURST_SIZE;
			if (count < bytes)
				bytes = count;
			words = bytes >> 2;
			pr_debug("%s copy_to_user %p (%d)\n", __func__, mapped_memory, bytes);
#ifdef ALLOW_DIRECT_USER_IOMEM_TRANSFERS
			if (unlikely(__copy_to_user(buf, mapped_memory, bytes))) {
				status = -EFAULT;
				goto error;
			}
#else
			ioread32_rep(mapped_memory, kernel_buffer, words);
			if (unlikely(__copy_to_user(buf, kernel_buffer, bytes))) {
				status = -EFAULT;
				goto error;
			}
#endif
			fifo_dev->words_transfered += words;
			len += bytes;
			buf += bytes;
			count -= bytes;
			if (!count)
				break;
			words_available -= words;
		}
		while (words_available);
	}

	status = len;
	*f_pos += len;
error:
	pr_debug("%s -> %d pos=%u\n", __func__, status, (unsigned int)*f_pos);
	return status;
}

static unsigned int dyplo_fifo_read_poll(struct file *filp, poll_table *wait)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	unsigned int mask;

	poll_wait(filp, &fifo_dev->fifo_wait_queue, wait);
	if (dyplo_fifo_read_level(fifo_dev))
		mask = (POLLIN | POLLRDNORM); /* Data available */
	else {
		/* Set IRQ to occur on user-defined treshold (default=1) */
		dyplo_fifo_read_enable_interrupt(fifo_dev, fifo_dev->poll_treshold);
		mask = 0;
	}

	pr_debug("%s -> %#x\n", __func__, mask);

	return mask;
}

static long dyplo_fifo_rw_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	if (unlikely(fifo_dev == NULL))
		return -ENODEV;

	/* pr_debug("%s(%x, %lx)\n", __func__, cmd, arg); */
	if (_IOC_TYPE(cmd) != DYPLO_IOC_MAGIC)
		return -ENOTTY;

	switch (_IOC_NR(cmd))
	{
		case DYPLO_IOC_TRESHOLD_QUERY:
			return fifo_dev->poll_treshold;
		case DYPLO_IOC_TRESHOLD_TELL:
			if (arg < 1)
				arg = 1;
			else if (arg > 192)
				arg = 192;
			fifo_dev->poll_treshold = arg;
			return 0;
		/* ioctl value or type does not matter, this always resets the
		 * associated fifo in the hardware. */
		case DYPLO_IOC_RESET_FIFO_WRITE:
		case DYPLO_IOC_RESET_FIFO_READ:
			if ((filp->f_mode & FMODE_WRITE) != 0)
				iowrite32_quick(1 << fifo_dev->index, (fifo_dev->config_parent->control_base + (DYPLO_REG_FIFO_RESET_WRITE/4)));
			else
				iowrite32_quick(1 << fifo_dev->index, (fifo_dev->config_parent->control_base + (DYPLO_REG_FIFO_RESET_READ/4)));
			return 0;
		default:
			return -ENOTTY;
	}
}


static struct file_operations dyplo_fifo_read_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_fifo_read_read,
	.llseek = no_llseek,
	.poll = dyplo_fifo_read_poll,
	.unlocked_ioctl = dyplo_fifo_rw_ioctl,
	.open = dyplo_fifo_read_open,
	.release = dyplo_fifo_read_release,
};

static int dyplo_fifo_write_level(struct dyplo_fifo_dev *fifo_dev)
{
	int index = fifo_dev->index;
	__iomem int *control_base =
		fifo_dev->config_parent->control_base;
	u32 result = ioread32_quick(control_base + (DYPLO_REG_FIFO_WRITE_LEVEL_BASE>>2) + index);
	pr_debug("%s index=%d value=0x%x (%d free)\n", __func__, index, result, result);
	return result;
}

static void dyplo_fifo_write_enable_interrupt(struct dyplo_fifo_dev *fifo_dev, int thd)
{
	int index = fifo_dev->index;
	__iomem int *control_base =
		fifo_dev->config_parent->control_base;
	if (thd > (DYPLO_FIFO_WRITE_SIZE*2)/3)
		thd = (DYPLO_FIFO_WRITE_SIZE*2)/3;
	else if (thd)
		--thd; /* IRQ will trigger when level is above thd */
	pr_debug("%s index=%d thd=%d mask=%x\n", __func__,
		index, thd, *(control_base + (DYPLO_REG_FIFO_WRITE_IRQ_MASK>>2)));
	iowrite32(thd, control_base + (DYPLO_REG_FIFO_WRITE_THD_BASE>>2) + index);
	iowrite32(BIT(index), control_base + (DYPLO_REG_FIFO_WRITE_IRQ_SET>>2));
}

static int dyplo_fifo_write_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct dyplo_dev *dev = container_of(inode->i_cdev, struct dyplo_dev, cdev_fifo_write);
	int index = iminor(inode) - dev->number_of_config_devices - 1;
	struct dyplo_fifo_dev *fifo_dev = &dev->fifo_write_devices[index];

	if (filp->f_mode & FMODE_READ) /* write-only device */
		return -EINVAL;

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	if (fifo_dev->is_open) {
		result = -EBUSY;
		goto error;
	}
	fifo_dev->is_open = true;
	fifo_dev->poll_treshold = DYPLO_FIFO_WRITE_SIZE / 2;
	filp->private_data = fifo_dev;
error:
	up(&dev->fop_sem);
	return result;
}

static int dyplo_fifo_write_release(struct inode *inode, struct file *filp)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	fifo_dev->is_open = false;
	return 0;
}

static ssize_t dyplo_fifo_write_write (struct file *filp, const char __user *buf, size_t count,
	loff_t *f_pos)
{
	int status = 0;
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	int __iomem *mapped_memory = dyplo_fifo_memory_location(fifo_dev);
	size_t len = 0;
#ifndef ALLOW_DIRECT_USER_IOMEM_TRANSFERS
	int kernel_buffer[DYPLO_FIFO_WRITE_MAX_BURST_SIZE/sizeof(int)];
#endif

	pr_debug("%s(%d)\n", __func__, count);

	if (count < 4) /* Do not allow read or write below word size */
		return -EINVAL;

	count &= ~0x03; /* Align to words */
	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	while (count)
	{
		int words_available;
		size_t bytes;
		if (filp->f_flags & O_NONBLOCK) {
			words_available = dyplo_fifo_write_level(fifo_dev);
			if (!words_available) {
				/* Non-blocking IO, return what we have */
				if (len)
					break;
				/* nothing copied yet, notify caller */
				status = -EAGAIN;
				goto error;
			}
		}
		else {
			DEFINE_WAIT(wait);
			for (;;) {
				prepare_to_wait(&fifo_dev->fifo_wait_queue, &wait, TASK_INTERRUPTIBLE);
				words_available = dyplo_fifo_write_level(fifo_dev);
				if (words_available)
					break; /* Done waiting */
				if (!signal_pending(current)) {
					dyplo_fifo_write_enable_interrupt(fifo_dev, count >> 2);
					schedule();
					continue;
				}
				status = -ERESTARTSYS;
				break;
			}
			finish_wait(&fifo_dev->fifo_wait_queue, &wait);
			if (status)
				goto error;
		}
		do {
			unsigned int words;
			bytes = words_available << 2;
			if (bytes > DYPLO_FIFO_WRITE_MAX_BURST_SIZE)
				bytes = DYPLO_FIFO_WRITE_MAX_BURST_SIZE;
			if (count < bytes)
				bytes = count;
			words = bytes >> 2;
			pr_debug("%s copy_from_user %p (%d)\n", __func__, mapped_memory, bytes);
#ifdef ALLOW_DIRECT_USER_IOMEM_TRANSFERS
			if (unlikely(__copy_from_user(mapped_memory, buf, bytes))) {
				status = -EFAULT;
				goto error;
			}
#else
			if (unlikely(__copy_from_user(kernel_buffer, buf, bytes))) {
				status = -EFAULT;
				goto error;
			}
			iowrite32_rep(mapped_memory, kernel_buffer, words);
#endif
			fifo_dev->words_transfered += words;
			len += bytes;
			buf += bytes;
			count -= bytes;
			if (!count)
				break;
			words_available -= words;
		}
		while (words_available);
	}

	status = len;
	*f_pos += len;
error:
	pr_debug("%s -> %d pos=%u\n", __func__, status, (unsigned int)*f_pos);
	return status;
}

static unsigned int dyplo_fifo_write_poll(struct file *filp, poll_table *wait)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	unsigned int mask;

	poll_wait(filp, &fifo_dev->fifo_wait_queue, wait);
	if (dyplo_fifo_write_level(fifo_dev))
		mask = (POLLOUT | POLLWRNORM);
	else {
		/* Wait for buffer crossing user-defined treshold */
		dyplo_fifo_write_enable_interrupt(fifo_dev, fifo_dev->poll_treshold);
		mask = 0;
	}

	pr_debug("%s -> %#x\n", __func__, mask);

	return mask;
}

static struct file_operations dyplo_fifo_write_fops =
{
	.write = dyplo_fifo_write_write,
	.poll = dyplo_fifo_write_poll,
	.llseek = no_llseek,
	.unlocked_ioctl = dyplo_fifo_rw_ioctl,
	.open = dyplo_fifo_write_open,
	.release = dyplo_fifo_write_release,
};


/* Interrupt service routine */
static irqreturn_t dyplo_isr(int irq, void *dev_id)
{
	struct dyplo_config_dev *cfg_dev = dev_id;
	struct dyplo_dev *dev = cfg_dev->parent;
	int index;
	u32 mask;

	u32 write_status_reg = ioread32_quick(
		cfg_dev->control_base + (DYPLO_REG_FIFO_WRITE_IRQ_STATUS>>2));
	u32 read_status_reg = ioread32_quick(
		cfg_dev->control_base + (DYPLO_REG_FIFO_READ_IRQ_STATUS>>2));
	/* Allow IRQ sharing and tell kernel when no action taken */
	if (!write_status_reg && !read_status_reg)
		return IRQ_NONE;
	/* Acknowledge the interrupt by clearing all flags that we've seen */
	if (write_status_reg)
		iowrite32_quick(write_status_reg,
			cfg_dev->control_base + (DYPLO_REG_FIFO_WRITE_IRQ_CLR>>2));
	if (read_status_reg)
		iowrite32_quick(read_status_reg,
			cfg_dev->control_base + (DYPLO_REG_FIFO_READ_IRQ_CLR>>2));
	pr_debug("%s(status=0x%x 0x%x)\n", __func__,
			write_status_reg, read_status_reg);
	/* Trigger the associated wait queues, "read" queues first */
	for (mask=1, index=0; index < dev->number_of_fifo_read_devices; ++index, mask <<= 1)
	{
		if (read_status_reg & mask)
			wake_up_interruptible(&dev->fifo_read_devices[index].fifo_wait_queue);
	}
	for (mask=1, index=0; index < dev->number_of_fifo_write_devices; ++index, mask <<= 1)
	{
		if (write_status_reg & mask)
			wake_up_interruptible(&dev->fifo_write_devices[index].fifo_wait_queue);
	}
	return IRQ_HANDLED;
}

static int create_sub_devices(struct platform_device *pdev, struct dyplo_config_dev *cfg_dev)
{
	int retval;
	struct dyplo_dev *dev = cfg_dev->parent;
	dev_t first_fifo_devt;
	int fifo_index = 0;
	int number_of_write_fifos;
	int number_of_read_fifos;
	int i;
	struct device *device;

	if (! dyplo_is_cpu_node(cfg_dev))
		return 0;

	if (dev->number_of_fifo_read_devices || dev->number_of_fifo_write_devices) {
		dev_err(&pdev->dev, "Fifo's already registered\n");
		return -EBUSY;
	}

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "IRQ resource missing\n");
		return -ENOENT;
	}

	number_of_write_fifos = ioread32_quick(cfg_dev->control_base + (DYPLO_REG_CPU_FIFO_WRITE_COUNT>>2));
	number_of_read_fifos = ioread32_quick(cfg_dev->control_base + (DYPLO_REG_CPU_FIFO_READ_COUNT>>2));
	dev->number_of_fifo_write_devices = number_of_write_fifos;
	dev->number_of_fifo_read_devices = number_of_read_fifos;
	dev->fifo_write_devices = devm_kzalloc(&pdev->dev,
		(number_of_write_fifos + number_of_read_fifos) * sizeof(struct dyplo_fifo_dev),
		GFP_KERNEL);
	if (!dev->fifo_write_devices) {
		dev_err(&pdev->dev, "No memory for %d fifo devices\n",
			number_of_write_fifos + number_of_read_fifos);
		dev->number_of_fifo_write_devices = 0;
		dev->number_of_fifo_read_devices = 0;
		return -ENOMEM;
	}
	dev->fifo_read_devices =
		dev->fifo_write_devices + number_of_write_fifos;

	first_fifo_devt = dev->devt + dev->number_of_config_devices + 1;
	retval = register_chrdev_region(first_fifo_devt,
				(number_of_write_fifos + number_of_read_fifos), DRIVER_FIFO_CLASS_NAME);
	if (retval) {
		goto error_register_chrdev_region;
	}

	cdev_init(&dev->cdev_fifo_write, &dyplo_fifo_write_fops);
	dev->cdev_fifo_write.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev_fifo_write,
		first_fifo_devt, number_of_write_fifos);
	if (retval) {
		dev_err(&pdev->dev, "cdev_add(cdev_fifo_write) failed\n");
		goto error_cdev_w;
	}
	cdev_init(&dev->cdev_fifo_read, &dyplo_fifo_read_fops);
	dev->cdev_fifo_read.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev_fifo_read,
		first_fifo_devt+number_of_write_fifos, number_of_read_fifos);
	if (retval) {
		dev_err(&pdev->dev, "cdev_add(cdev_fifo_read) failed\n");
		goto error_cdev_r;
	}

	for (i = 0; i < number_of_write_fifos; ++i)
	{
		struct dyplo_fifo_dev *fifo_dev = &dev->fifo_write_devices[i];
		fifo_dev->config_parent = cfg_dev;
		fifo_dev->index = i;
		init_waitqueue_head(&fifo_dev->fifo_wait_queue);
		device = device_create(dev->class, &pdev->dev,
			first_fifo_devt + fifo_index,
			fifo_dev, DRIVER_FIFO_WRITE_NAME, i);
		if (IS_ERR(device)) {
			dev_err(&pdev->dev, "unable to create fifo write device %d\n",
				i);
			retval = PTR_ERR(device);
			goto failed_device_create;
		}
		++fifo_index;
	}
	for (i = 0; i < number_of_read_fifos; ++i)
	{
		struct dyplo_fifo_dev *fifo_dev = &dev->fifo_read_devices[i];
		fifo_dev->config_parent = cfg_dev;
		fifo_dev->index = i;
		init_waitqueue_head(&fifo_dev->fifo_wait_queue);
		device = device_create(dev->class, &pdev->dev,
			first_fifo_devt + fifo_index,
			fifo_dev, DRIVER_FIFO_READ_NAME, i);
		if (IS_ERR(device)) {
			dev_err(&pdev->dev, "unable to create fifo read device %d\n",
				i);
			retval = PTR_ERR(device);
			goto failed_device_create;
		}
		++fifo_index;
	}

	/* Connect IRQ with this cfg_dev */
	retval = request_irq(dev->irq, dyplo_isr, 0, pdev->name, cfg_dev);
	if (retval) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto failed_request_irq;
	}
	dev->fifo_config_device = cfg_dev;

	return 0;

failed_request_irq:
failed_device_create:
	while (fifo_index) {
		device_destroy(dev->class, first_fifo_devt + fifo_index);
		--fifo_index;
	}
error_cdev_r:
error_cdev_w:
	unregister_chrdev_region(first_fifo_devt, dev->number_of_fifo_write_devices + dev->number_of_fifo_read_devices);
error_register_chrdev_region:
	dev->number_of_fifo_write_devices = 0;
	dev->number_of_fifo_read_devices = 0;
	return retval;
}

static int dyplo_proc_show(struct seq_file *m, void *offset)
{
	struct dyplo_dev *dev = m->private;
	unsigned int i;
	int ctl_index;
	unsigned int irq_r_mask;
	unsigned int irq_r_status;
	unsigned int irq_w_mask;
	unsigned int irq_w_status;
	unsigned int number_of_fifo_devices;
	__iomem int *control_base;
	if (dev == NULL) {
		seq_printf(m, "No dyplo device instance!\n");
		return 0;
	}
	seq_printf(m, "ncfg=%d nfifo w=%d r=%d\nFIFO states:\n",
		dev->number_of_config_devices, dev->number_of_fifo_write_devices, dev->number_of_fifo_read_devices);
	control_base = dev->fifo_write_devices[0].config_parent->control_base;
	irq_r_mask = *(control_base + (DYPLO_REG_FIFO_READ_IRQ_MASK>>2));
	irq_r_status = *(control_base + (DYPLO_REG_FIFO_READ_IRQ_STATUS>>2));
	irq_w_mask = *(control_base + (DYPLO_REG_FIFO_WRITE_IRQ_MASK>>2));
	irq_w_status = *(control_base + (DYPLO_REG_FIFO_WRITE_IRQ_STATUS>>2));
	number_of_fifo_devices = dev->number_of_fifo_write_devices;
	if (dev->number_of_fifo_read_devices > number_of_fifo_devices)
		number_of_fifo_devices = dev->number_of_fifo_read_devices;
	for (i = 0; i < number_of_fifo_devices; ++i)
	{
		unsigned int mask = BIT(i);
		unsigned int tr_w;
		unsigned int tr_r;
		seq_printf(m, "fifo=%2d ", i);
		if (i < dev->number_of_fifo_write_devices) {
			int lw = dyplo_fifo_write_level(&dev->fifo_write_devices[i]);
			int tw = *(control_base + (DYPLO_REG_FIFO_WRITE_THD_BASE>>2) + i);
			seq_printf(m, "w=%3d (%3d%c%c) ",
				lw, tw, irq_w_mask & mask ? 'w' : '.', irq_w_status & mask ? 'i' : '.');
			tr_w = dev->fifo_write_devices[i].words_transfered;
		}
		else {
			seq_printf(m, "             ");
			tr_w = 0;
		}
		if (i < dev->number_of_fifo_read_devices) {
			int lr = dyplo_fifo_read_level(&dev->fifo_read_devices[i]);
			int tr = *(control_base + (DYPLO_REG_FIFO_READ_THD_BASE>>2) + i);
			seq_printf(m, "r=%3d (%3d%c%c) ",
				lr, tr, irq_r_mask & mask ? 'w' : '.', irq_r_status & mask ? 'i' : '.');
			tr_r = dev->fifo_read_devices[i].words_transfered;
		}
		else {
			seq_printf(m, "             ");
			tr_r = 0;
		}
		seq_printf(m, "total w=%d r=%d\n", tr_w, tr_r);
	}
	seq_printf(m, "Route table:\n");
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		int queue_index;
		int __iomem *ctl_base = dev->config_devices[ctl_index].control_base;
		int __iomem *ctl_route_base = ctl_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		const int number_of_fifos_out =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		const int number_of_fifos_in =
			dyplo_number_of_input_queues(&dev->config_devices[ctl_index]);
		seq_printf(m, "ctl_index=%d id=%#x fifos in=%d out=%d\n", ctl_index,
				ioread32_quick(ctl_base + (DYPLO_REG_ID>>2)),
				number_of_fifos_in, number_of_fifos_out);
		for (queue_index = 0; queue_index < number_of_fifos_out; ++queue_index)
		{
			unsigned int route = ctl_route_base[queue_index];
			if (route)
			{
				int src_ctl_index = route >> 5;
				if (src_ctl_index > 0)
				{
					int src_index = route & 0x1F;
					seq_printf(m, "route %d,%d -> %d,%d\n",
						ctl_index, queue_index, src_ctl_index-1, src_index);
				}
			}
		}
	}
	seq_printf(m, "Backplane counters:");
	for (i = 0; i < dev->number_of_config_devices; ++i)
		seq_printf(m, " %d", dev->base[(DYPLO_REG_BACKPLANE_COUNTER_BASE/4) + i]);
	seq_printf(m, "\nAXI overhead: %d, Stream in: %d, Stream out: %d\n",
		dev->base[DYPLO_REG_AXI_COUNTER_BASE/4],
		dev->base[(DYPLO_REG_CPU_COUNTER_BASE/4)+0],
		dev->base[(DYPLO_REG_CPU_COUNTER_BASE/4)+1]);
	return 0;
}

 static int dyplo_proc_open(struct inode *inode, struct file *file)
 {
     return single_open(file, dyplo_proc_show, PDE_DATA(inode));
 }

static const struct file_operations dyplo_proc_fops = {
	.owner	= THIS_MODULE,
	.open	= dyplo_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release = single_release,
};

static int dyplo_probe(struct platform_device *pdev)
{
	struct dyplo_dev *dev;
	struct device *device;
	dev_t devt;
	int retval;
	int device_index;
	u32 control_id;
	struct proc_dir_entry *proc_file_entry;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, dev);
	sema_init(&dev->fop_sem, 1);

	dev->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base = devm_request_and_ioremap(&pdev->dev, dev->mem);
	if (!dev->base) {
		dev_err(&pdev->dev, "Failed to map device memory\n");
		return -ENXIO;
	}

	control_id = *(dev->base + (DYPLO_REG_ID>>2));
	if ((control_id & DYPLO_REG_ID_MASK_VENDOR_PRODUCT) !=
			DYPLO_REG_ID_PRODUCT_TOPIC_CONTROL)
	{
		dev_err(&pdev->dev, "Bad device ID: 0x%x\n", control_id);
		return -EINVAL;
	}

	dev->number_of_config_devices =
		ioread32_quick(dev->base + (DYPLO_REG_CONTROL_CPU_NODES_COUNT>>2)) +
		ioread32_quick(dev->base + (DYPLO_REG_CONTROL_IO_NODES_COUNT>>2)) +
		ioread32_quick(dev->base + (DYPLO_REG_CONTROL_PR_NODES_COUNT>>2)) +
		ioread32_quick(dev->base + (DYPLO_REG_CONTROL_FIXED_NODES_COUNT>>2));
	dev->config_devices = devm_kzalloc(&pdev->dev,
		dev->number_of_config_devices * sizeof(struct dyplo_config_dev),
		GFP_KERNEL);
	if (!dev->config_devices) {
		dev_err(&pdev->dev, "No memory for %d cfg devices\n", dev->number_of_config_devices);
		return -ENOMEM;
	}

	/* Create /dev/dyplo.. devices */
	retval = alloc_chrdev_region(&devt, 0, dev->number_of_config_devices + 1, DRIVER_CLASS_NAME);
	if (retval < 0)
		return retval;
	dev->devt = devt;

	cdev_init(&dev->cdev_control, &dyplo_ctl_fops);
	dev->cdev_control.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev_control, devt, 1);
	if (retval) {
		dev_err(&pdev->dev, "cdev_add(ctl) failed\n");
		goto failed_cdev;
	}

	cdev_init(&dev->cdev_config, &dyplo_cfg_fops);
	dev->cdev_config.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev_config, devt + 1, dev->number_of_config_devices);
	if (retval) {
		dev_err(&pdev->dev, "cdev_add(cfg) failed\n");
		goto failed_cdev;
	}

	dev->class = class_create(THIS_MODULE, DRIVER_CLASS_NAME);
	if (IS_ERR(dev->class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		retval = PTR_ERR(dev->class);
		goto failed_class;
	}

	device_index = 0;

	device = device_create(dev->class, &pdev->dev, devt, dev,
				DRIVER_CONTROL_NAME);
	if (IS_ERR(device)) {
		dev_err(&pdev->dev, "unable to create device\n");
		retval = PTR_ERR(device);
		goto failed_device_create;
	}

	while (device_index < dev->number_of_config_devices)
	{
		struct dyplo_config_dev* cfg_dev =
				&dev->config_devices[device_index];
		cfg_dev->parent = dev;
		cfg_dev->base =
			(dev->base + ((DYPLO_CONFIG_SIZE>>2) * (device_index + 1)));
		cfg_dev->control_base =
			(dev->base + ((DYPLO_NODE_REG_SIZE>>2) * (device_index + 1)));

		device = device_create(dev->class, &pdev->dev,
			devt + 1 + device_index,
			cfg_dev, DRIVER_CONFIG_NAME, device_index);
		if (IS_ERR(device)) {
			dev_err(&pdev->dev, "unable to create config device %d\n",
				device_index);
			retval = PTR_ERR(device);
			goto failed_device_create_cfg;
		}
		retval = create_sub_devices(pdev, cfg_dev);
		if (retval) {
			dev_err(&pdev->dev, "unable to create sub-device %d: %d\n",
				device_index, retval);
			/* Should we abort? */
		}
		++device_index;
	}

	proc_file_entry = proc_create_data(DRIVER_CLASS_NAME, 0444, NULL, &dyplo_proc_fops, dev);
	if (proc_file_entry == NULL)
		dev_err(&pdev->dev, "unable to create proc entry\n");

	/* And finally, enable the backplane */
	*(dev->base + (DYPLO_REG_BACKPLANE_ENABLE_SET>>2)) =
		(2 << dev->number_of_config_devices) - 1;

	return 0;

failed_device_create_cfg:
	while (device_index) {
		device_destroy(dev->class, dev->devt + 1 + device_index);
		--device_index;
	}
	if (dev->fifo_config_device)
		free_irq(dev->irq, dev->fifo_config_device);
failed_device_create:
	class_destroy(dev->class);
failed_class:
failed_cdev:
	unregister_chrdev_region(devt, dev->number_of_config_devices + 1);
	return retval;
}

static int dyplo_remove(struct platform_device *pdev)
{
	struct dyplo_dev *dev;
	int i;

	dev = dev_get_drvdata(&pdev->dev);
	if (!dev)
		return -ENODEV;

	remove_proc_entry(DRIVER_CLASS_NAME, NULL);

	for (i = dev->number_of_config_devices +
		dev->number_of_fifo_write_devices + dev->number_of_fifo_read_devices;
			i >= 0; --i)
		device_destroy(dev->class, dev->devt + i);
	class_destroy(dev->class);
	unregister_chrdev_region(dev->devt, 1 + dev->number_of_config_devices +
		dev->number_of_fifo_write_devices + dev->number_of_fifo_read_devices);
	if (dev->fifo_config_device)
		free_irq(dev->irq, dev->fifo_config_device);
	return 0;
}


static const struct of_device_id dyplo_ids[] = {
	{ .compatible = "topic,dyplo-1.00.a" },
	{ },
};
MODULE_DEVICE_TABLE(of, dyplo_ids);

static struct platform_driver dyplo_driver = {
	.driver = {
		.name = "dyplo",
		.owner = THIS_MODULE,
		.of_match_table = dyplo_ids,
	},
	.probe = dyplo_probe,
	.remove = dyplo_remove,
};
module_platform_driver(dyplo_driver);
