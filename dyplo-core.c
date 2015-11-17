/*
 * dyplo-core.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/dma-mapping.h>
#include <linux/kfifo.h>
#include "dyplo-core.h"
#include "dyplo.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Topic Embedded Products <www.topic.nl>");

/* When defined, transfers end-of-file markers through Dyplo as
 * user signals. This allows you to use something like
 * cat inputfile > /dev/dyplow0 & ; cat /dev/dyplor0 > resultfile
 * The extra transfers may confuse existing code, so don't define it
 * yet. When defined, opening with O_APPEND disables it at runtime. */
/* #define DYPLO_TRANSFER_EOF_MARKERS_AS_USER_SIGNALS */

static const char DRIVER_CLASS_NAME[] = "dyplo";
static const char DRIVER_CONTROL_NAME[] = "dyploctl";
static const char DRIVER_CONFIG_NAME[] = "dyplocfg%d";
static const char DRIVER_FIFO_CLASS_NAME[] = "dyplo-fifo";
static const char DRIVER_FIFO_WRITE_NAME[] = "dyplow%d";
static const char DRIVER_FIFO_READ_NAME[] = "dyplor%d";
static const char DRIVER_DMA_CLASS_NAME[] = "dyplo-dma";
static const char DRIVER_DMA_DEVICE_NAME[] = "dyplod%d";

/* Maximum number of commands, i.e. the size of the command queue in
 * logic. This is mostly dynamically used, but in some places, it's
 * good to know how far we can go. */
#define DMA_MAX_NUMBER_OF_COMMANDS	8

static const unsigned int dyplo_dma_default_block_size = 64 * 1024;
static const size_t dyplo_dma_memory_size = 256 * 1024;

/* How to do IO. We rarely need any memory barriers, so add a "quick"
 * version that skips the memory barriers. */
#define ioread32_quick	__raw_readl
#define iowrite32_quick	__raw_writel

static inline void dyplo_reg_write_quick(u32 __iomem *base, u32 reg, u32 value)
{
	iowrite32_quick(value, base + (reg >> 2));
}

static inline u32 dyplo_reg_read(u32 __iomem *base, u32 reg)
{
	return ioread32(base + (reg >> 2));
}

static inline u32 dyplo_reg_read_quick(u32 __iomem *base, u32 reg)
{
	return ioread32_quick(base + (reg >> 2));
}

static inline u32 dyplo_reg_read_quick_index(u32 __iomem *base, u32 reg, u32 index)
{
	return ioread32_quick(base + (reg >> 2) + index);
}

struct dyplo_fifo_dev
{
	struct dyplo_config_dev *config_parent;
	wait_queue_head_t fifo_wait_queue; /* So the IRQ handler can notify waiting threads */
	int index;
	unsigned int words_transfered;
	unsigned int poll_treshold;
	void* transfer_buffer;
	u16 user_signal;
	bool eof;
	bool is_open;
};

struct dyplo_fifo_control_dev
{
	struct dyplo_config_dev *config_parent;
	struct dyplo_fifo_dev *fifo_devices;
	struct cdev cdev_fifo_write;
	struct cdev cdev_fifo_read;
	dev_t devt_first_fifo_device;
	u8 number_of_fifo_write_devices;
	u8 number_of_fifo_read_devices;
};

struct dyplo_dma_to_logic_operation {
	dma_addr_t addr;
	unsigned int size;
};

struct dyplo_dma_from_logic_operation {
	char* addr;
	unsigned int size;
	unsigned int next_tail;
	u16 user_signal;
	u16 short_transfer; /* Non-zero if size < blocksize */
};

struct dyplo_dma_dev;

struct dyplo_dma_block {
	/* Kernel part */
	struct dyplo_dma_dev* parent;
	dma_addr_t phys_addr;
	void* mem_addr;
	/* User part */
	struct dyplo_buffer_block data;
};

struct dyplo_dma_block_set {
	struct dyplo_dma_block *blocks;
	u32 size;
	u32 count;
	u32 flags;
};

/* Use DMA coherent memory. Depending on hardware HP/ACP, this may yield
 * non-cachable memory which is particularly noticeable on logic-to-cpu
 * unless you have hardware coherency (dma-coherent in DT). */
#define DYPLO_DMA_BLOCK_FLAG_COHERENT	1
/* Use streaming instead of coherent memory. This requires cacheline
 * maintenance which may cost more than actually copying the data. */
#define DYPLO_DMA_BLOCK_FLAG_STREAMING	2
/* Indicates that the memory pointers point to a shared block and should
 * not be freed. */
#define DYPLO_DMA_BLOCK_FLAG_SHAREDMEM	4

struct dyplo_dma_dev
{
	struct dyplo_config_dev* config_parent;
	struct cdev cdev_dma;
	mode_t open_mode; /* FMODE_READ FMODE_WRITE FMODE_EXCL */

	struct dyplo_dma_block_set dma_to_logic_blocks;
	struct dyplo_dma_block_set dma_from_logic_blocks;

	/* big blocks of memory for read/write transfers */
	dma_addr_t dma_to_logic_handle;
	void* dma_to_logic_memory;
	unsigned int dma_to_logic_memory_size;
	unsigned int dma_to_logic_head;
	unsigned int dma_to_logic_tail;
	unsigned int dma_to_logic_block_size;
	DECLARE_KFIFO(dma_to_logic_wip, struct dyplo_dma_to_logic_operation, 16);
	wait_queue_head_t wait_queue_to_logic;

	dma_addr_t dma_from_logic_handle;
	void* dma_from_logic_memory;
	unsigned int dma_from_logic_memory_size;
	unsigned int dma_from_logic_head;
	unsigned int dma_from_logic_tail;
	unsigned int dma_from_logic_block_size;
	wait_queue_head_t wait_queue_from_logic;
	struct dyplo_dma_from_logic_operation dma_from_logic_current_op;
	bool dma_from_logic_full;
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

static u32 dyplo_cfg_get_version_id(const struct dyplo_config_dev *cfg_dev)
{
	return dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_REG_VERSION_ID);
}
static u8 dyplo_cfg_get_node_type(const struct dyplo_config_dev *cfg_dev)
{
	return (dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_REG_TYPE_ID) >> 8) & 0xFF;
}
static u8 dyplo_number_of_input_queues(const struct dyplo_config_dev *cfg_dev)
{
	return dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_REG_NODE_INFO) & 0x0F;
}
static u8 dyplo_number_of_output_queues(const struct dyplo_config_dev *cfg_dev)
{
	return (dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_REG_NODE_INFO) >> 4) & 0x0F;
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

static loff_t dyplo_ctl_llseek(struct file *filp, loff_t off, int whence)
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

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (dev->mem)
		return vm_iomap_memory(vma, dev->mem->start, DYPLO_CONFIG_SIZE);
	else
		return vm_iomap_memory(vma, virt_to_phys(dev->base), DYPLO_CONFIG_SIZE);
}

static void dyplo_ctl_route_remove_dst(struct dyplo_dev *dev, u32 route)
{
	int ctl_index;
	int queue_index;
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		int __iomem *ctl_route_base = dev->base +
			(DYPLO_REG_CONTROL_ROUTE_TABLE>>2) +
			(ctl_index << DYPLO_STREAM_ID_WIDTH);

		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		{
			if (ioread32_quick(ctl_route_base + queue_index) == route) {
				pr_debug("removed route %d,%d->%d,%d\n",
					ctl_index, queue_index,
					(route >> DYPLO_STREAM_ID_WIDTH) - 1,
					route & ((0x1 << DYPLO_STREAM_ID_WIDTH) - 1));
				iowrite32_quick(0, ctl_route_base + queue_index);
			}
		}
	}
}

static int dyplo_ctl_route_add(struct dyplo_dev *dev, struct dyplo_route_item_t route)
{
	u32 __iomem* dst_control_addr;
	u32 dst_route;

	pr_debug("%s %d,%d->%d,%d\n", __func__,
		route.srcNode, route.srcFifo, route.dstNode, route.dstFifo);
	if ((route.srcNode >= dev->number_of_config_devices) ||
	    (route.dstNode >= dev->number_of_config_devices))
	{
		pr_debug("%s: Invalid source or destination\n", __func__);
	    return -EINVAL;
	}
	dst_route = ((route.dstNode + 1) << DYPLO_STREAM_ID_WIDTH) | route.dstFifo;
	dyplo_ctl_route_remove_dst(dev, dst_route);
	/* Setup route. The PL assumes that "0" is the control node, hence
	 * the "+1" in config node indices */
	dst_control_addr = dev->base +
		(DYPLO_REG_CONTROL_ROUTE_TABLE>>2) +
		(route.srcNode << DYPLO_STREAM_ID_WIDTH) +
		route.srcFifo;
	pr_debug("%s (%d) @ %p: %x\n", __func__, route.srcNode,
		dst_control_addr, dst_route);
	iowrite32_quick(dst_route, dst_control_addr);
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
		u32 __iomem *ctl_route_base = dev->base +
			(DYPLO_REG_CONTROL_ROUTE_TABLE>>2) +
			(ctl_index << DYPLO_STREAM_ID_WIDTH);
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		{
			u32 route = ioread32_quick(ctl_route_base + queue_index);
			if (route)
			{
				int src_ctl_index = route >> DYPLO_STREAM_ID_WIDTH;
				if (src_ctl_index > 0)
				{
					int src_index = route & ( (0x1 << DYPLO_STREAM_ID_WIDTH) - 1);
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
	const int match = (ctl_index_to_delete + 1) << DYPLO_STREAM_ID_WIDTH;
	const int number_of_fifos =
		dyplo_number_of_output_queues(&dev->config_devices[ctl_index_to_delete]);
	u32 __iomem *ctl_route_base = dev->base +
		(DYPLO_REG_CONTROL_ROUTE_TABLE>>2) +
		(ctl_index_to_delete << DYPLO_STREAM_ID_WIDTH);

	/* Erase outgoing routes */
	for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
		iowrite32_quick(0, ctl_route_base + queue_index);

	/* Erase incoming routes */
	for (ctl_index = 0; ctl_index < ctl_index_to_delete; ++ctl_index) {
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		ctl_route_base = dev->base +
			(DYPLO_REG_CONTROL_ROUTE_TABLE>>2) +
			(ctl_index << DYPLO_STREAM_ID_WIDTH);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index) {
			if ((ioread32_quick(ctl_route_base + queue_index) & (0xFFFF << DYPLO_STREAM_ID_WIDTH) ) == match)
				iowrite32_quick(0, ctl_route_base + queue_index);
		}
	}
	for (ctl_index = ctl_index_to_delete+1; ctl_index < dev->number_of_config_devices; ++ctl_index) {
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		ctl_route_base = dev->base +
			(DYPLO_REG_CONTROL_ROUTE_TABLE>>2) +
			(ctl_index << DYPLO_STREAM_ID_WIDTH);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index) {
			if ((ioread32_quick(ctl_route_base + queue_index) & (0xFFFF << DYPLO_STREAM_ID_WIDTH) ) == match)
				iowrite32_quick(0, ctl_route_base + queue_index);
		}
	}
	return 0;
}

static int dyplo_ctl_route_clear(struct dyplo_dev *dev)
{
	int ctl_index;
	int queue_index;

	u32 __iomem *ctl_route_base = dev->base + (DYPLO_REG_CONTROL_ROUTE_TABLE>>2);
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)	{
		/* Remove outgoing routes */
		const int number_of_fifos =
			dyplo_number_of_output_queues(&dev->config_devices[ctl_index]);
		for (queue_index = 0; queue_index < number_of_fifos; ++queue_index)
			iowrite32_quick(0, ctl_route_base + queue_index);
		ctl_route_base += 4;
	}
	return 0;
}

static long dyplo_ctl_license_key(struct dyplo_dev *dev, unsigned int cmd, void __user *user_key)
{
	int status;
	u32 key[2];

	if (_IOC_SIZE(cmd) != sizeof(key))
		return -EINVAL;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		/* Already checked memory with access_ok */
		status = __copy_from_user(key, user_key, sizeof(key));
		if (status)
			return status;
		dyplo_reg_write_quick(dev->base, DYPLO_REG_CONTROL_LICENSE_KEY0, key[0]);
		dyplo_reg_write_quick(dev->base, DYPLO_REG_CONTROL_LICENSE_KEY1, key[1]);
	}
	if (_IOC_DIR(cmd) & _IOC_READ) {
		key[0] = dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_LICENSE_KEY0);
		key[1] = dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_LICENSE_KEY1);
		status = __copy_to_user(user_key, key, sizeof(key));
		if (status)
			return status;
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
			status = dyplo_reg_read_quick(dev->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS) >> 1;
			break;
		case DYPLO_IOC_BACKPLANE_ENABLE:
			dyplo_reg_write_quick(dev->base, DYPLO_REG_BACKPLANE_ENABLE_SET, arg << 1);
			status = dyplo_reg_read_quick(dev->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS) >> 1;
			break;
		case DYPLO_IOC_BACKPLANE_DISABLE:
			dyplo_reg_write_quick(dev->base, DYPLO_REG_BACKPLANE_ENABLE_CLR, arg << 1);
			status = dyplo_reg_read_quick(dev->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS) >> 1;
			break;
		case DYPLO_IOC_LICENSE_KEY:
			status = dyplo_ctl_license_key(dev, cmd, (void __user *)arg);
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
	pr_debug("%s cmd=%#x (%d) arg=%#lx\n", __func__, cmd, _IOC_NR(cmd), arg);
	return dyplo_ctl_ioctl_impl(dev, cmd, arg);
}

static const struct file_operations dyplo_ctl_fops =
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

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (cfg_dev->parent->mem)
		return vm_iomap_memory(vma,
			cfg_dev->parent->mem->start + dyplo_get_config_mem_offset(cfg_dev),
			DYPLO_CONFIG_SIZE);
	else
		return vm_iomap_memory(vma,
			virt_to_phys(cfg_dev->base),
			DYPLO_CONFIG_SIZE);
}

static long dyplo_cfg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_config_dev *cfg_dev = filp->private_data;
	int status;

	pr_debug("%s cmd=%#x (%d) arg=%#lx\n", __func__, cmd, _IOC_NR(cmd), arg);

	if (unlikely(cfg_dev == NULL))
		return -ENODEV;
	if (_IOC_TYPE(cmd) != DYPLO_IOC_MAGIC)
		return -ENOTTY;

	switch (_IOC_NR(cmd))
	{
		case DYPLO_IOC_ROUTE_CLEAR:
		case DYPLO_IOC_ROUTE_DELETE: /* Remove routes to this node */
			status = dyplo_ctl_route_delete(cfg_dev->parent, dyplo_get_config_index(cfg_dev));
			break;
		case DYPLO_IOC_ROUTE_QUERY_ID:
			status = dyplo_get_config_index(cfg_dev);
			break;
		case DYPLO_IOC_BACKPLANE_STATUS:
			{
				int index = dyplo_get_config_index(cfg_dev);
				status = dyplo_reg_read_quick(cfg_dev->parent->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS) >> 1;
				status &= (1 << index);
			}
			break;
		case DYPLO_IOC_BACKPLANE_ENABLE:
			{
				int index = dyplo_get_config_index(cfg_dev);
				dyplo_reg_write_quick(cfg_dev->parent->base, DYPLO_REG_BACKPLANE_ENABLE_SET, 1 << (index+1));
				status = dyplo_reg_read_quick(cfg_dev->parent->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS) >> 1;
			}
			break;
		case DYPLO_IOC_BACKPLANE_DISABLE:
			{
				int index = dyplo_get_config_index(cfg_dev);
				dyplo_reg_write_quick(cfg_dev->parent->base, DYPLO_REG_BACKPLANE_ENABLE_CLR, 1 << (index+1));
				status = dyplo_reg_read_quick(cfg_dev->parent->base, DYPLO_REG_BACKPLANE_ENABLE_STATUS) >> 1;
			}
			break;
		case DYPLO_IOC_RESET_FIFO_WRITE:
			dyplo_reg_write_quick(cfg_dev->control_base, DYPLO_REG_NODE_RESET_FIFOS, arg);
			status = 0;
			break;
		case DYPLO_IOC_RESET_FIFO_READ:
			dyplo_reg_write_quick(cfg_dev->control_base, DYPLO_REG_FIFO_RESET_READ, arg);
			status = 0;
			break;
		default:
			printk(KERN_WARNING "DYPLO ioctl unknown command: %d (arg=0x%lx).\n", _IOC_NR(cmd), arg);
			status = -ENOTTY;
	}

	pr_debug("%s cmd=%#x (%d) arg=%#lx result=%#x\n", __func__, cmd, _IOC_NR(cmd), arg, status);
	return status;
}

static const struct file_operations dyplo_cfg_fops =
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

static bool dyplo_fifo_write_usersignal(struct dyplo_fifo_dev *fifo_dev, u16 user_signal)
{
	__iomem u32 *control_base_us =
		fifo_dev->config_parent->control_base +
		(DYPLO_REG_FIFO_WRITE_USERSIGNAL_BASE>>2) +
		fifo_dev->index;
	iowrite32((u32)user_signal, control_base_us);
	/* Test if user signals are supported by reading back the value */
	return (u16)ioread32_quick(control_base_us) == user_signal;
}

static u32 dyplo_fifo_read_level(struct dyplo_fifo_dev *fifo_dev)
{
	return dyplo_reg_read_quick_index(
		fifo_dev->config_parent->control_base,
		DYPLO_REG_FIFO_READ_LEVEL_BASE,
		fifo_dev->index);
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
	iowrite32(thd, control_base + (DYPLO_REG_FIFO_READ_THD_BASE>>2) + index);
	/* v2 uses upper 16 bits of shared IRQ registers */
	pr_debug("%s index=%d thd=%d v2\n", __func__, index, thd);
	iowrite32(BIT(index + 16), control_base + (DYPLO_REG_FIFO_IRQ_SET>>2));
}

static int dyplo_fifo_read_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct dyplo_fifo_control_dev *fifo_ctl_dev =
		container_of(inode->i_cdev, struct dyplo_fifo_control_dev, cdev_fifo_read);
	int index = inode->i_rdev - fifo_ctl_dev->devt_first_fifo_device;
	struct dyplo_fifo_dev *fifo_dev = &fifo_ctl_dev->fifo_devices[index];
	struct dyplo_dev* dev = fifo_ctl_dev->config_parent->parent;

	pr_debug("%s index=%d mode=%#x flags=%#x i-devt=%u d=%u f=%u\n", __func__,
		index, filp->f_mode, filp->f_flags,
		inode->i_rdev, inode->i_cdev->dev, fifo_ctl_dev->devt_first_fifo_device);

	if (filp->f_mode & FMODE_WRITE) /* read-only device */
		return -EINVAL;
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	if (fifo_dev->is_open) {
		result = -EBUSY;
		goto error;
	}
	fifo_dev->transfer_buffer = kmalloc(DYPLO_FIFO_READ_MAX_BURST_SIZE, GFP_KERNEL);
	if (unlikely(fifo_dev->transfer_buffer == NULL)) {
		result = -ENOMEM;
		goto error;
	}
	fifo_dev->user_signal = 0;
	fifo_dev->eof = false;
	fifo_dev->is_open = true;
	fifo_dev->poll_treshold = 1;
	filp->private_data = fifo_dev;
	nonseekable_open(inode, filp);
error:
	up(&dev->fop_sem);
	return result;
}

static int dyplo_fifo_read_release(struct inode *inode, struct file *filp)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	struct dyplo_dev* dev = fifo_dev->config_parent->parent;

	pr_debug("%s index=%d\n", __func__, fifo_dev->index);
#ifdef DYPLO_TRANSFER_EOF_MARKERS_AS_USER_SIGNALS
	if (!fifo_dev->eof) {
		/* We haven't seen any EOF sentinel yet, see if it's there and
		 * consume it when present */
		u32 words_available = dyplo_fifo_read_level(fifo_dev);
		u16 user_signal = words_available >> 16;
		words_available &= 0xFFFF;
		if (words_available && (user_signal == DYPLO_USERSIGNAL_EOF)) {
			ioread32_quick(dyplo_fifo_memory_location(fifo_dev));
			pr_debug("%s consume extra EOF sentinel\n", __func__);
		}
		fifo_dev->eof = true;
	}
#endif
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	kfree(fifo_dev->transfer_buffer);
	fifo_dev->transfer_buffer = NULL;
	fifo_dev->is_open = false;
	up(&dev->fop_sem);
	return 0;
}

static ssize_t dyplo_fifo_read_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	int __iomem *mapped_memory = dyplo_fifo_memory_location(fifo_dev);
	int status = 0;
	size_t len = 0;
	pr_debug("%s(%u)\n", __func__, (unsigned int)count);

#ifdef DYPLO_TRANSFER_EOF_MARKERS_AS_USER_SIGNALS
	if (fifo_dev->eof)
		return 0; /* Indicate end of file once sentinel received */
#endif

	if (count < 4) /* Do not allow read or write below word size */
		return -EINVAL;

	count &= ~0x03; /* Align to words */

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	while (count)
	{
		u32 words_available;
		u16 user_signal;
		size_t bytes;
		if (filp->f_flags & O_NONBLOCK) {
			words_available = dyplo_fifo_read_level(fifo_dev);
			user_signal = words_available >> 16;
			words_available &= 0xFFFF; /* Lower 16-bits only */
			if (!words_available) {
				/* Non-blocking IO, return what we have */
				if (len)
					break;
				/* nothing copied yet, notify caller */
				status = -EAGAIN;
				goto error;
			}
			/* user_signal is valid because words_available is non-nul */
			if (user_signal != fifo_dev->user_signal) {
				fifo_dev->user_signal = user_signal;
#ifdef DYPLO_TRANSFER_EOF_MARKERS_AS_USER_SIGNALS
				if (user_signal == DYPLO_USERSIGNAL_EOF) {
					pr_debug("%s: Got EOF\n", __func__);
					fifo_dev->eof = true;
					ioread32_quick(mapped_memory); /* Fetch sentinel */
					break;
				}
#endif
				goto exit_ok;
			}
		}
		else {
			DEFINE_WAIT(wait);
			for (;;) {
				prepare_to_wait(&fifo_dev->fifo_wait_queue, &wait, TASK_INTERRUPTIBLE);
				words_available = dyplo_fifo_read_level(fifo_dev);
				user_signal = words_available >> 16;
				words_available &= 0xFFFF;
				if (words_available) {
					/* usersignal is only valid when there is data */
					if (user_signal != fifo_dev->user_signal) {
						fifo_dev->user_signal = user_signal;
#ifdef DYPLO_TRANSFER_EOF_MARKERS_AS_USER_SIGNALS
						if (user_signal == DYPLO_USERSIGNAL_EOF) {
							pr_debug("%s: Got EOF\n", __func__);
							fifo_dev->eof = true;
							ioread32_quick(mapped_memory); /* Fetch sentinel */
						}
#endif
						finish_wait(&fifo_dev->fifo_wait_queue, &wait);
						goto exit_ok;
					}
					break; /* Done waiting */
				}
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
			pr_debug("%s copy_to_user %p (%u)\n", __func__, mapped_memory, (unsigned int)bytes);
			ioread32_rep(mapped_memory, fifo_dev->transfer_buffer, words);
			if (unlikely(__copy_to_user(buf, fifo_dev->transfer_buffer, bytes))) {
				status = -EFAULT;
				goto error;
			}
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
exit_ok:
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
	if (fifo_dev->eof || (dyplo_fifo_read_level(fifo_dev) & 0xFFFF))
		mask = (POLLIN | POLLRDNORM); /* Data available */
	else {
		/* Set IRQ to occur on user-defined treshold (default=1) */
		dyplo_fifo_read_enable_interrupt(fifo_dev, fifo_dev->poll_treshold);
		mask = 0;
	}

	pr_debug("%s -> %#x\n", __func__, mask);

	return mask;
}

static int dyplo_fifo_rw_get_route_id(struct dyplo_fifo_dev *fifo_dev)
{
	return dyplo_get_config_index(fifo_dev->config_parent) |
		(fifo_dev->index << 8);
}

static int dyplo_fifo_rw_add_route(struct dyplo_fifo_dev *fifo_dev, int source, int dest)
{
	struct dyplo_route_item_t route;
	route.srcFifo = (source >> 8) & 0xFF;
	route.srcNode = source & 0xFF;
	route.dstFifo = (dest >> 8) & 0xFF;
	route.dstNode = dest & 0xFF;
	dyplo_ctl_route_add(fifo_dev->config_parent->parent, route);
	return 0;
}

static long dyplo_fifo_rw_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	if (unlikely(fifo_dev == NULL))
		return -ENODEV;

	pr_debug("%s cmd=%#x (%d) arg=%#lx\n", __func__, cmd, _IOC_NR(cmd), arg);
	if (_IOC_TYPE(cmd) != DYPLO_IOC_MAGIC)
		return -ENOTTY;

	switch (_IOC_NR(cmd))
	{
		case DYPLO_IOC_ROUTE_QUERY_ID:
			return dyplo_fifo_rw_get_route_id(fifo_dev);
		case DYPLO_IOC_ROUTE_TELL_TO_LOGIC:
			if ((filp->f_mode & FMODE_WRITE) == 0)
				return -ENOTTY; /* Cannot route from this node */
			return dyplo_fifo_rw_add_route(fifo_dev, dyplo_fifo_rw_get_route_id(fifo_dev), arg);
		case DYPLO_IOC_ROUTE_TELL_FROM_LOGIC:
			if ((filp->f_mode & FMODE_READ) == 0)
				return -ENOTTY; /* Cannot route to this node */
			return dyplo_fifo_rw_add_route(fifo_dev, arg, dyplo_fifo_rw_get_route_id(fifo_dev));
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
				dyplo_reg_write_quick(fifo_dev->config_parent->control_base,
					DYPLO_REG_FIFO_RESET_WRITE, 1 << fifo_dev->index);
			else
				dyplo_reg_write_quick(fifo_dev->config_parent->control_base,
					DYPLO_REG_FIFO_RESET_READ, 1 << fifo_dev->index);
			return 0;
		case DYPLO_IOC_USERSIGNAL_QUERY:
			/* TODO: Return LAST usersignal, not next */
			return fifo_dev->user_signal;
		case DYPLO_IOC_USERSIGNAL_TELL:
			if (!(filp->f_mode & FMODE_WRITE))
				return -EINVAL;
			arg &= 0xFFFF; /* Only lower bits */
			if (!dyplo_fifo_write_usersignal(fifo_dev, arg)) {
				printk(KERN_ERR "%s: Failed to set usersignal\n", __func__);
				return -EIO;
			}
			fifo_dev->user_signal = arg;
			return 0;
		default:
			return -ENOTTY;
	}
}


static const struct file_operations dyplo_fifo_read_fops =
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
	return dyplo_reg_read_quick_index(
		fifo_dev->config_parent->control_base,
		DYPLO_REG_FIFO_WRITE_LEVEL_BASE,
		fifo_dev->index);
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
	pr_debug("%s index=%d thd=%d\n", __func__, index, thd);
	iowrite32(thd, control_base + (DYPLO_REG_FIFO_WRITE_THD_BASE>>2) + index);
	iowrite32(BIT(index), control_base + (DYPLO_REG_FIFO_IRQ_SET>>2));
}

static int dyplo_fifo_write_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct dyplo_fifo_control_dev *fifo_ctl_dev =
		container_of(inode->i_cdev, struct dyplo_fifo_control_dev, cdev_fifo_write);
	int index = inode->i_rdev - fifo_ctl_dev->devt_first_fifo_device;
	struct dyplo_fifo_dev *fifo_dev = &fifo_ctl_dev->fifo_devices[index];
	struct dyplo_dev* dev = fifo_ctl_dev->config_parent->parent;

	pr_debug("%s index=%d mode=%#x flags=%#x i-devt=%u d=%u f=%u\n", __func__,
		index, filp->f_mode, filp->f_flags,
		inode->i_rdev, inode->i_cdev->dev, fifo_ctl_dev->devt_first_fifo_device);

	if (filp->f_mode & FMODE_READ) /* write-only device */
		return -EINVAL;

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	if (fifo_dev->is_open) {
		result = -EBUSY;
		goto error;
	}
	fifo_dev->poll_treshold = DYPLO_FIFO_WRITE_SIZE / 2;
	filp->private_data = fifo_dev;
	fifo_dev->user_signal = DYPLO_USERSIGNAL_ZERO;
	fifo_dev->eof = false;
	fifo_dev->transfer_buffer = kmalloc(DYPLO_FIFO_WRITE_MAX_BURST_SIZE, GFP_KERNEL);
	if (unlikely(fifo_dev->transfer_buffer == NULL)) {
		result = -ENOMEM;
		goto error;
	}
	/* Set user signal register */
	if (!dyplo_fifo_write_usersignal(fifo_dev, DYPLO_USERSIGNAL_ZERO)) {
		printk(KERN_ERR "%s: Failed to reset usersignals on w%d\n",
			__func__, index);
		result = -EIO;
		goto error;
	}
	fifo_dev->is_open = true;
	nonseekable_open(inode, filp);
error:
	up(&dev->fop_sem);
	return result;
}

static int dyplo_fifo_write_release(struct inode *inode, struct file *filp)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	struct dyplo_dev* dev = fifo_dev->config_parent->parent;
	int status = 0;

	pr_debug("%s index=%d\n", __func__, fifo_dev->index);
#ifdef DYPLO_TRANSFER_EOF_MARKERS_AS_USER_SIGNALS
	/* Setting O_APPEND mode prevents sending a EOF on close, which
	 * is not quite intuitive but convenient. */
	if (!fifo_dev->eof &&
		((filp->f_flags & O_APPEND) == 0) &&
		dyplo_fifo_write_usersignal(fifo_dev, DYPLO_USERSIGNAL_EOF)) {
		/* Transfer the EOF sentinel, we may need to wait for available
		 * space in the outgoing fifo. */
		u32 words_available;
		DEFINE_WAIT(wait);
		for (;;) {
			prepare_to_wait(&fifo_dev->fifo_wait_queue, &wait, TASK_INTERRUPTIBLE);
			words_available = dyplo_fifo_write_level(fifo_dev);
			if (words_available)
				break; /* Done waiting */
			pr_debug("%s: Waiting to send EOF sentinel\n", __func__);
			if (!signal_pending(current)) {
				dyplo_fifo_write_enable_interrupt(fifo_dev, 1);
				schedule();
				continue;
			}
			status = -ERESTARTSYS;
			break;
		}
		finish_wait(&fifo_dev->fifo_wait_queue, &wait);
		if (status)
			goto error;
		pr_debug("%s: Send EOF sentinel\n", __func__);
		iowrite32_quick(0, dyplo_fifo_memory_location(fifo_dev));
		fifo_dev->eof = true;
	} else
		pr_debug("%s: EOF sentinel not sent\n", __func__);
error:
#endif
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	kfree(fifo_dev->transfer_buffer);
	fifo_dev->transfer_buffer = NULL;
	fifo_dev->is_open = false;
	up(&dev->fop_sem);
	return status;
}

static ssize_t dyplo_fifo_write_write (struct file *filp, const char __user *buf, size_t count,
	loff_t *f_pos)
{
	int status = 0;
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	int __iomem *mapped_memory = dyplo_fifo_memory_location(fifo_dev);
	size_t len = 0;

	pr_debug("%s(%u)\n", __func__, (unsigned int)count);

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
			pr_debug("%s copy_from_user %p (%u)\n", __func__, mapped_memory, (unsigned int)bytes);
			if (unlikely(__copy_from_user(fifo_dev->transfer_buffer, buf, bytes))) {
				status = -EFAULT;
				goto error;
			}
			iowrite32_rep(mapped_memory, fifo_dev->transfer_buffer, words);
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

static const struct file_operations dyplo_fifo_write_fops =
{
	.write = dyplo_fifo_write_write,
	.poll = dyplo_fifo_write_poll,
	.llseek = no_llseek,
	.unlocked_ioctl = dyplo_fifo_rw_ioctl,
	.open = dyplo_fifo_write_open,
	.release = dyplo_fifo_write_release,
};


/* Interrupt service routine for CPU fifo node, version 2 */
static irqreturn_t dyplo_fifo_isr(struct dyplo_dev *dev, struct dyplo_config_dev *cfg_dev)
{
	struct dyplo_fifo_control_dev *fifo_ctl_dev = cfg_dev->private_data;
	u32 status_reg = dyplo_reg_read_quick(
		cfg_dev->control_base, DYPLO_REG_FIFO_IRQ_STATUS);
	u16 read_status_reg;
	u16 write_status_reg;
	u8 index;

	/* Allow IRQ sharing */
	if (!status_reg)
		return IRQ_NONE;

	/* Acknowledge interrupt to hardware */
	iowrite32_quick(status_reg,
			cfg_dev->control_base + (DYPLO_REG_FIFO_IRQ_CLR>>2));
	pr_debug("%s(status=0x%x)\n", __func__, status_reg);
	/* Trigger the associated wait queues, "read" queues first. These
	 * are in the upper 16 bits of the interrupt status word */
	read_status_reg = status_reg >> 16;
	for (index = 0; (read_status_reg != 0) && (index < fifo_ctl_dev->number_of_fifo_read_devices); ++index)
	{
		if (read_status_reg & 1)
			wake_up_interruptible(&fifo_ctl_dev->fifo_devices[fifo_ctl_dev->number_of_fifo_write_devices + index].fifo_wait_queue);
		read_status_reg >>= 1;
	}
	write_status_reg = status_reg & 0xFFFF;
	for (index = 0; (write_status_reg != 0) && (index < fifo_ctl_dev->number_of_fifo_write_devices); ++index)
	{
		if (write_status_reg & 1)
			wake_up_interruptible(&fifo_ctl_dev->fifo_devices[index].fifo_wait_queue);
		write_status_reg >>= 1;
	}
	return IRQ_HANDLED;
}


static unsigned int dyplo_dma_get_index(const struct dyplo_dma_dev *dma_dev)
{
	return dyplo_get_config_index(dma_dev->config_parent);
}

static void dyplo_dma_to_logic_irq_enable(u32 __iomem *control_base)
{
	pr_debug("%s\n", __func__);
	iowrite32_quick(BIT(0), control_base + (DYPLO_REG_FIFO_IRQ_SET>>2));
}

static void dyplo_dma_from_logic_irq_enable(u32 __iomem *control_base)
{
	pr_debug("%s\n", __func__);
	iowrite32_quick(BIT(16), control_base + (DYPLO_REG_FIFO_IRQ_SET>>2));
}

static void dyplo_set_bits(u32 __iomem *reg_ptr, u32 mask, bool enable)
{
	u32 value = ioread32_quick(reg_ptr);
	u32 next = enable ? (value | mask) : (value & ~mask);
	if (next != value)
		iowrite32(next, reg_ptr);
}

static void dyplo_dma_from_logic_enable(u32 __iomem *control_base, bool value)
{
	dyplo_set_bits(control_base + (DYPLO_DMA_FROMLOGIC_CONTROL>>2), BIT(0), value);
}

static void dyplo_dma_to_logic_enable(u32 __iomem *control_base, bool value)
{
	dyplo_set_bits(control_base + (DYPLO_DMA_TOLOGIC_CONTROL>>2), BIT(0), value);
}

static bool dyplo_dma_common_is_standalone_mode(struct dyplo_dma_dev *dma_dev)
{
	return !!(dyplo_reg_read_quick(dma_dev->config_parent->control_base,
			DYPLO_DMA_STANDALONE_CONTROL) && BIT(0));
}

static int dyplo_dma_from_logic_reset(struct dyplo_dma_dev *dma_dev);
static int dyplo_dma_to_logic_reset(struct dyplo_dma_dev *dma_dev);

static void dyplo_dma_common_set_standalone_mode(struct dyplo_dma_dev *dma_dev, bool standalone)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;

	if (dyplo_dma_common_is_standalone_mode(dma_dev) == standalone)
		return; /* Already in that mode */
	pr_debug("%s: Switching to %s mode\n", __func__,
		standalone ? "standalone" : "normal");
	dyplo_dma_to_logic_enable(control_base, false);
	dyplo_dma_from_logic_enable(control_base, false);
	iowrite32(standalone ? BIT(0) : 0, control_base + (DYPLO_DMA_STANDALONE_CONTROL>>2));
	if (!standalone) {
		dyplo_dma_to_logic_enable(control_base, true);
		dyplo_dma_to_logic_reset(dma_dev);
		dyplo_dma_from_logic_enable(control_base, true);
		dyplo_dma_from_logic_reset(dma_dev);
	}
}

/* Kills ongoing DMA transactions and resets everything. */
static int dyplo_dma_to_logic_reset(struct dyplo_dma_dev *dma_dev)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	u32 reg;
	int result;
	DEFINE_WAIT(wait);

	reg = dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_CONTROL);
	pr_debug("%s ctl=%#x\n", __func__, reg);
	if (reg & BIT(1)) {
		pr_err("%s: Reset already in progress\n", __func__);
		return -EBUSY;
	}
	if (!(reg & BIT(0))) {
		pr_debug("%s: DMA hardware not running\n", __func__);
		return -EINVAL;
	}
	reg |= BIT(1);
	prepare_to_wait(&dma_dev->wait_queue_to_logic, &wait, TASK_INTERRUPTIBLE);
	/* Enable reset-ready-interrupt */
	iowrite32(BIT(15), control_base + (DYPLO_REG_FIFO_IRQ_SET>>2));
	/* Send reset command */
	iowrite32_quick(reg, control_base + (DYPLO_DMA_TOLOGIC_CONTROL>>2));
	for(;;) {
		if ((dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_CONTROL) & BIT(1)) == 0) {
			result = 0;
			break;
		}
		if (signal_pending(current)) {
			result = -ERESTARTSYS;
			break;
		}
		if (schedule_timeout(HZ/4) == 0) {
			pr_err("%s: TIMEOUT waiting for reset complete IRQ.\n", __func__);
			result = -ETIMEDOUT;
			break;
		}
		prepare_to_wait(&dma_dev->wait_queue_to_logic, &wait, TASK_INTERRUPTIBLE);
	}
	finish_wait(&dma_dev->wait_queue_from_logic, &wait);

	/* Re-enable the node */
	iowrite32_quick(BIT(0), control_base + (DYPLO_DMA_TOLOGIC_CONTROL>>2));
	dma_dev->dma_to_logic_head = 0;
	dma_dev->dma_to_logic_tail = 0;
	kfifo_reset(&dma_dev->dma_to_logic_wip);
	return 0;
}

static int dyplo_dma_from_logic_reset(struct dyplo_dma_dev *dma_dev)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	u32 reg;
	int result;
	DEFINE_WAIT(wait);


	reg = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_CONTROL);
	pr_debug("%s ctl=%#x\n", __func__, reg);
	if (reg & BIT(1)) {
		pr_err("%s: Reset already in progress\n", __func__);
		return -EBUSY;
	}
	if (!(reg & BIT(0))) {
		pr_debug("%s: DMA hardware not running\n", __func__);
		return -EINVAL;
	}
	reg |= BIT(1);
	prepare_to_wait(&dma_dev->wait_queue_from_logic, &wait, TASK_INTERRUPTIBLE);
	/* Enable reset-ready-interrupt */
	iowrite32(BIT(31), control_base + (DYPLO_REG_FIFO_IRQ_SET>>2));
	/* Send reset command */
	iowrite32_quick(BIT(1)|BIT(0), control_base + (DYPLO_DMA_FROMLOGIC_CONTROL>>2));
	for(;;) {
		if ((dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_CONTROL) & BIT(1)) == 0) {
			result = 0;
			break;
		}
		if (signal_pending(current)) {
			result = -ERESTARTSYS;
			break;
		}
		if (schedule_timeout(HZ) == 0) {
			pr_err("%s: TIMEOUT waiting for reset complete IRQ ctrl=%#x ists=%#x\n",
				__func__,
				dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_CONTROL),
				dyplo_reg_read_quick(control_base, DYPLO_REG_FIFO_IRQ_STATUS));
			result = -ETIMEDOUT;
			break;
		}
		prepare_to_wait(&dma_dev->wait_queue_from_logic, &wait, TASK_INTERRUPTIBLE);
	}
	finish_wait(&dma_dev->wait_queue_from_logic, &wait);

	/* Re-enable the node */
	iowrite32_quick(BIT(0), control_base + (DYPLO_DMA_FROMLOGIC_CONTROL>>2));

	dma_dev->dma_from_logic_head = 0;
	dma_dev->dma_from_logic_tail = 0;
	dma_dev->dma_from_logic_current_op.size = 0;
	dma_dev->dma_from_logic_full = false;
	return result;
}

/* Forward declaration */
static const struct file_operations dyplo_dma_to_logic_fops;
static const struct file_operations dyplo_dma_from_logic_fops;

static int dyplo_dma_open(struct inode *inode, struct file *filp)
{
	struct dyplo_dma_dev *dma_dev = container_of(
		inode->i_cdev, struct dyplo_dma_dev, cdev_dma);
	struct dyplo_config_dev *cfg_dev = dma_dev->config_parent;
	struct dyplo_dev *dev = cfg_dev->parent;
	int status = 0;

	pr_debug("%s(mode=%#x flags=%#x)\n", __func__,
		filp->f_mode, filp->f_flags);

	/* Must specify either read or write mode */
	if ((filp->f_mode & (FMODE_READ | FMODE_WRITE)) == 0)
		return -EINVAL;

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	filp->private_data = dma_dev; /* for other methods */
	nonseekable_open(inode, filp);

	if (filp->f_mode & FMODE_WRITE) {
		/* For mmap to work, the device must be opened in R+W mode, so
		 * consider this the same as opening for write-only. */
		if (dma_dev->open_mode & FMODE_WRITE) {
			status = -EBUSY;
			goto exit_open;
		}
		/* Normally, RDWR means just write. If O_SYNC is specified, the
		 * intention is to use standalone mode, which needs both ends */
		if  (filp->f_flags & O_SYNC) {
			if (dma_dev->open_mode & FMODE_READ) {
				status = -EBUSY;
				goto exit_open;
			}
			dma_dev->open_mode |= (FMODE_WRITE | FMODE_READ | FMODE_EXCL);
		} else {
			dma_dev->open_mode |= FMODE_WRITE; /* Set in-use bits */
		}
		filp->f_op = &dyplo_dma_to_logic_fops;
		/* Reset usersignal */
		iowrite32_quick(DYPLO_USERSIGNAL_ZERO,
			cfg_dev->control_base + (DYPLO_DMA_TOLOGIC_USERBITS>>2));
		/* Default to generic size */
		dma_dev->dma_to_logic_block_size = dyplo_dma_default_block_size;
	} else {
		if (dma_dev->open_mode & FMODE_READ) {
			status = -EBUSY;
			goto exit_open;
		}
		dma_dev->open_mode |= FMODE_READ; /* Set in-use bits */
		filp->f_op = &dyplo_dma_from_logic_fops;
	}
exit_open:
	up(&dev->fop_sem);
	pr_debug("%s(mode=%#x flags=%#x) -> %d\n", __func__,
		filp->f_mode, filp->f_flags, status);
	return status;
}

static int dyplo_dma_common_release(struct file *filp, mode_t flag_to_clear)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;
	struct dyplo_dev *dev = dma_dev->config_parent->parent;

	pr_debug("%s(mode=%#x %#x)\n", __func__, filp->f_mode, dma_dev->open_mode);
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	if (dma_dev->open_mode & FMODE_EXCL) {
		dyplo_dma_common_set_standalone_mode(dma_dev, false);
		dma_dev->open_mode = 0; /* Closes both ends */
	} else
		dma_dev->open_mode &= ~flag_to_clear; /* Clear in use bit */
	up(&dev->fop_sem);
	return 0;
}

static int dyplo_dma_to_logic_release(struct inode *inode, struct file *filp)
{
	return dyplo_dma_common_release(filp, FMODE_WRITE);
}

static int dyplo_dma_from_logic_release(struct inode *inode, struct file *filp)
{
	return dyplo_dma_common_release(filp, FMODE_READ);
}

static unsigned int dyplo_dma_to_logic_avail(struct dyplo_dma_dev *dma_dev)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	u32 status = dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_STATUS);
	/* Status: bits 24..31: #results; 16..23: available to execute */
	u8 num_results;

	pr_debug("%s status=%#x\n", __func__, status);
	for (num_results = (status >> 24); num_results != 0; --num_results) {
		/* Fetch result from queue */
		struct dyplo_dma_to_logic_operation op;
		u32 addr = dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_RESULT_ADDR);
		if (unlikely(!kfifo_get(&dma_dev->dma_to_logic_wip, &op))) {
			pr_err("Nothing in fifo of DMA node %u but still %u results\n",
				dyplo_dma_get_index(dma_dev), num_results);
			BUG();
		}
		pr_debug("%s addr=%#x wip=%#x,%u\n", __func__, addr, (u32)op.addr, op.size);
		if (unlikely(op.addr != addr)) {
			pr_err("Mismatch in result of DMA node %u: phys=%pa expected %#x (size %d) actual %#x\n",
				dyplo_dma_get_index(dma_dev),
				&dma_dev->dma_to_logic_handle,
				(u32)op.addr, op.size, addr);
			pr_err("head=%#x (%d) tail=%#x (%d)\n",
				dma_dev->dma_to_logic_head,
				dma_dev->dma_to_logic_head,
				dma_dev->dma_to_logic_tail,
				dma_dev->dma_to_logic_tail);
			for (;;) {
				if (!kfifo_get(&dma_dev->dma_to_logic_wip, &op))
					break;
				pr_err("Internal entry: %#x (size %d)\n", (u32)op.addr, op.size);
			}
			while (num_results) {
				addr = dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_RESULT_ADDR);
				pr_err("Logic result: %#x\n", addr);
				--num_results;
			}
			BUG();
		}
		dma_dev->dma_to_logic_tail += op.size;
		if (dma_dev->dma_to_logic_tail == dma_dev->dma_to_logic_memory_size)
			dma_dev->dma_to_logic_tail = 0;
		pr_debug("%s tail=%u\n", __func__, dma_dev->dma_to_logic_tail);
		if (unlikely(dma_dev->dma_to_logic_tail > dma_dev->dma_to_logic_memory_size)) {
			pr_err("Overflow in DMA node %u: tail %u size %u\n",
				dyplo_dma_get_index(dma_dev),
				dma_dev->dma_to_logic_tail, dma_dev->dma_to_logic_memory_size);
			BUG();
		}
	}
	/* Calculate available space */
	if (dma_dev->dma_to_logic_tail > dma_dev->dma_to_logic_head)
		return dma_dev->dma_to_logic_tail - dma_dev->dma_to_logic_head;
	else if (dma_dev->dma_to_logic_tail == dma_dev->dma_to_logic_head) {
		/* Can mean "full" or "empty" */
		if (!kfifo_is_empty(&dma_dev->dma_to_logic_wip))
			return 0; /* head==tail and there is work in progress */
	}
	/* Return available bytes until end of buffer */
	return dma_dev->dma_to_logic_memory_size - dma_dev->dma_to_logic_head;
}


/* Two things may block: There's no room in the ring, or there's no room
 * in the command buffer. */
static ssize_t dyplo_dma_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	int status = 0;
	struct dyplo_dma_dev *dma_dev = filp->private_data;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	unsigned int bytes_to_copy;
	unsigned int bytes_copied = 0;
	unsigned int bytes_avail;
	struct dyplo_dma_to_logic_operation dma_op;
	DEFINE_WAIT(wait);
	const bool is_blocking = (filp->f_flags & O_NONBLOCK) == 0;

	pr_debug("%s(%u)\n", __func__, (unsigned int)count);

	if (count < 4) /* Do not allow read or write below word size */
		return -EINVAL;
	count &= ~0x03;

	if (dma_dev->dma_to_logic_blocks.blocks)
		return -EBUSY;

	while (count) {
		bytes_to_copy = min((unsigned int)count, dma_dev->dma_to_logic_block_size);
		for(;;) {
			if (is_blocking)
				prepare_to_wait(&dma_dev->wait_queue_to_logic, &wait, TASK_INTERRUPTIBLE);
			bytes_avail = dyplo_dma_to_logic_avail(dma_dev);
			pr_debug("%s bytes_avail=%u head=%u tail=%u\n", __func__,
				bytes_avail, dma_dev->dma_to_logic_head, dma_dev->dma_to_logic_tail);
			if (bytes_avail != 0)
				break;
			if (signal_pending(current))
				goto error_interrupted;
			/* Enable interrupt */
			dyplo_dma_to_logic_irq_enable(control_base);
			if (is_blocking)
				schedule();
			else {
				if (bytes_copied)
					goto exit_ok; /* Some data transferred */
				else {
					/* No data available, tell user */
					status = -EAGAIN;
					goto error_exit;
				}
			}
		}
		if (is_blocking)
			finish_wait(&dma_dev->wait_queue_to_logic, &wait);
		if (bytes_avail < bytes_to_copy)
			bytes_to_copy = bytes_avail;

		/* Copy data into DMA buffer */
		if (unlikely(copy_from_user(
				(char __iomem *)dma_dev->dma_to_logic_memory + dma_dev->dma_to_logic_head,
				buf, bytes_to_copy))) {
			status = -EFAULT;
			goto error_exit;
		}

		/* Submit command to engine, wait for availability first */
		dma_op.addr = dma_dev->dma_to_logic_handle + dma_dev->dma_to_logic_head;
		dma_op.size = bytes_to_copy;
		for(;;) {
			if (is_blocking)
				prepare_to_wait(&dma_dev->wait_queue_to_logic, &wait, TASK_INTERRUPTIBLE);
			if (dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_STATUS) & 0xFF0000)
				break; /* There is room in the command buffer */
			if (signal_pending(current))
				goto error_interrupted;
			/* Enable interrupt */
			dyplo_dma_to_logic_irq_enable(control_base);
			if (is_blocking)
				schedule();
			else {
				if (bytes_copied)
					goto exit_ok; /* Some data transferred */
				else {
					/* No data available, tell user */
					status = -EAGAIN;
					goto error_exit;
				}
			}
		}
		if (is_blocking)
			finish_wait(&dma_dev->wait_queue_to_logic, &wait);
		pr_debug("%s sending addr=%#x size=%u\n", __func__,
			(unsigned int)dma_op.addr, dma_op.size);
		iowrite32_quick(dma_op.addr, control_base + (DYPLO_DMA_TOLOGIC_STARTADDR>>2));
		iowrite32(dma_op.size, control_base + (DYPLO_DMA_TOLOGIC_BYTESIZE>>2));
		if (unlikely(kfifo_put(&dma_dev->dma_to_logic_wip, dma_op) == 0)) {
			pr_err("dma_to_logic_wip kfifo was full, cannot put %#x %u\n",
				(u32)dma_op.addr, dma_op.size);
			BUG();
		}

		/* Update pointers for next chunk, if any */
		dma_dev->dma_to_logic_head += bytes_to_copy;
		if (dma_dev->dma_to_logic_head == dma_dev->dma_to_logic_memory_size)
			dma_dev->dma_to_logic_head = 0;
		pr_debug("%s head=%u\n", __func__, dma_dev->dma_to_logic_head);
		BUG_ON(dma_dev->dma_to_logic_head > dma_dev->dma_to_logic_memory_size);
		buf += bytes_to_copy;
		bytes_copied += bytes_to_copy;
		count -= bytes_to_copy;
	}
exit_ok:
	status = bytes_copied;
	*f_pos += bytes_copied;
error_exit:
	pr_debug("%s -> %d\n", __func__, status);
	return status;

error_interrupted:
	if (is_blocking)
		finish_wait(&dma_dev->wait_queue_to_logic, &wait);
	pr_debug("%s -> ERESTARTSYS\n", __func__);
	return -ERESTARTSYS;
}

/* Adds new read commands to the queue and returns number of results */
static unsigned int dyplo_dma_from_logic_pump(struct dyplo_dma_dev *dma_dev)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	u32 status_reg;
	u8 num_free_entries;

	status_reg = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_STATUS);
	pr_debug("%s status=%#x\n", __func__, status_reg);
	num_free_entries = (status_reg >> 16) & 0xFF;

	while (!dma_dev->dma_from_logic_full) {
		if (!num_free_entries)
			break; /* No more room for commands */
		pr_debug("%s sending addr=%#x size=%u\n", __func__,
			(unsigned int)dma_dev->dma_from_logic_handle + dma_dev->dma_from_logic_head, dma_dev->dma_from_logic_block_size);
		iowrite32(dma_dev->dma_from_logic_handle + dma_dev->dma_from_logic_head, control_base + (DYPLO_DMA_FROMLOGIC_STARTADDR>>2));
		iowrite32(dma_dev->dma_from_logic_block_size, control_base + (DYPLO_DMA_FROMLOGIC_BYTESIZE>>2));
		dma_dev->dma_from_logic_head += dma_dev->dma_from_logic_block_size;
		if (dma_dev->dma_from_logic_head == dma_dev->dma_from_logic_memory_size)
			dma_dev->dma_from_logic_head = 0;
		if (dma_dev->dma_from_logic_head == dma_dev->dma_from_logic_tail)
			dma_dev->dma_from_logic_full = true;
		--num_free_entries;
	}

	return status_reg >> 24;
}

static ssize_t dyplo_dma_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	int status = 0;
	unsigned int bytes_to_copy;
	unsigned int bytes_copied = 0;
	unsigned int results_avail = 0;
	struct dyplo_dma_from_logic_operation *current_op =
		&dma_dev->dma_from_logic_current_op;
	DEFINE_WAIT(wait);
	const bool is_blocking = (filp->f_flags & O_NONBLOCK) == 0;

	pr_debug("%s(%u)\n", __func__, (unsigned int)count);

	if (count < 4) /* Do not allow read or write below word size */
		return -EINVAL;
	count &= ~0x03;

	if (dma_dev->dma_from_logic_blocks.blocks)
		return -EBUSY;

	while (count) {
		while (current_op->size == 0) {
			/* Fetch a new operation from logic */
			if (results_avail) {
				dma_addr_t start_addr = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_RESULT_ADDR);
				unsigned int tail = start_addr - dma_dev->dma_from_logic_handle;
				current_op->addr = ((char*)dma_dev->dma_from_logic_memory) + tail;
				current_op->user_signal = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_RESULT_USERBITS);
				current_op->size = dyplo_reg_read(control_base, DYPLO_DMA_FROMLOGIC_RESULT_BYTESIZE);
				current_op->short_transfer = (current_op->size != dma_dev->dma_from_logic_block_size);
				tail += dma_dev->dma_from_logic_block_size;
				if (tail == dma_dev->dma_from_logic_memory_size)
					tail = 0;
				current_op->next_tail = tail;
				--results_avail;
				pr_debug("%s: nexttail=%u size=%u addr=%p\n", __func__,
					tail, current_op->size, current_op->addr);
			} else {
				for(;;) {
					if (is_blocking)
						prepare_to_wait(&dma_dev->wait_queue_from_logic, &wait, TASK_INTERRUPTIBLE);
					results_avail = dyplo_dma_from_logic_pump(dma_dev);
					pr_debug("%s results_avail=%u head=%u tail=%u\n", __func__,
						results_avail, dma_dev->dma_from_logic_head, dma_dev->dma_from_logic_tail);
					if (results_avail != 0)
						break;
					if (signal_pending(current))
						goto error_interrupted;
					/* Enable interrupt */
					dyplo_dma_from_logic_irq_enable(control_base);
					if (is_blocking)
						schedule();
					else {
						if (bytes_copied)
							goto exit_ok; /* Some data transferred */
						else {
							/* No data available, tell user */
							status = -EAGAIN;
							goto error_exit;
						}
					}
				}
				if (is_blocking)
					finish_wait(&dma_dev->wait_queue_from_logic, &wait);
			}
		}
		/* Copy any remaining data into the user's buffer */
		if (current_op->size) {
			bytes_to_copy = current_op->size;
			if (bytes_to_copy > count)
				bytes_to_copy = count;
			/* pr_debug("%s: copy_to_user %p (%u)\n", __func__, current_op->addr, bytes_to_copy); */
			if (unlikely(__copy_to_user(buf, current_op->addr, bytes_to_copy))) {
				status = -EFAULT;
				goto error_exit;
			}
			bytes_copied += bytes_to_copy;
			count -= bytes_to_copy;
			buf += bytes_to_copy;
			current_op->size -= bytes_to_copy;
			if (current_op->size != 0) {
				/* No more room in user buffer */
				current_op->addr += bytes_to_copy;
				break;
			} else {
				dma_dev->dma_from_logic_tail = current_op->next_tail;
				dma_dev->dma_from_logic_full = false;
				pr_debug("%s: move tail %u\n", __func__,
					dma_dev->dma_from_logic_tail);
				/* We moved the tail up, so submit more work to logic */
				results_avail = dyplo_dma_from_logic_pump(dma_dev);
				if (current_op->short_transfer)
					break; /* Usersignal change, return immediately */
			}
		}
	}
exit_ok:
	status = bytes_copied;
	*f_pos += bytes_copied;
error_exit:
	return status;
error_interrupted:
	if (is_blocking)
		finish_wait(&dma_dev->wait_queue_from_logic, &wait);
	return -ERESTARTSYS;
}

static unsigned int dyplo_dma_to_logic_poll(struct file *filp, poll_table *wait)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	unsigned int mask = 0;
	unsigned int avail;

	poll_wait(filp, &dma_dev->wait_queue_to_logic, wait);

	if (dma_dev->dma_to_logic_blocks.blocks) {
		/* Writable when not all blocks have been submitted, or when
		 * results are available and can be dequeued */
		avail = dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_STATUS);
		if ((avail & 0xFF000000) == 0) {
			/* No results yet, see if there are blocks available */
			avail = ((avail >> 16) & 0xFF) + dma_dev->dma_to_logic_blocks.count - DMA_MAX_NUMBER_OF_COMMANDS;
		}
	} else
		avail = dyplo_dma_to_logic_avail(dma_dev);
	if (avail)
		mask |= (POLLOUT | POLLWRNORM);
	else
		dyplo_dma_to_logic_irq_enable(control_base);

	pr_debug("%s(%#x) -> %#x\n", __func__, avail, mask);
	return mask;
}

static unsigned int dyplo_dma_from_logic_poll(struct file *filp, poll_table *wait)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	unsigned int mask = 0;
	unsigned int avail;

	poll_wait(filp, &dma_dev->wait_queue_from_logic, wait);

	if (dma_dev->dma_from_logic_blocks.blocks) {
		avail = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_STATUS);
		pr_debug("%s(status=%#x)\n", __func__, avail);
		avail &= 0xFF000000;
	} else {
		if (dma_dev->dma_from_logic_current_op.size)
			avail = 1;
		else
			avail = dyplo_dma_from_logic_pump(dma_dev);
	}
	if (avail)
		mask |= (POLLIN | POLLRDNORM);
	else
		dyplo_dma_from_logic_irq_enable(control_base);

	pr_debug("%s(%x) -> %#x\n", __func__, avail, mask);
	return mask;
}

static int dyplo_dma_add_route(struct dyplo_dma_dev *dma_dev, int source, int dest)
{
	struct dyplo_route_item_t route;
	route.srcFifo = (source >> 8) & 0xFF;
	route.srcNode = source & 0xFF;
	route.dstFifo = (dest >> 8) & 0xFF;
	route.dstNode = dest & 0xFF;
	dyplo_ctl_route_add(dma_dev->config_parent->parent, route);
	return 0;
}

static int dyplo_dma_get_route_id(struct dyplo_dma_dev *dma_dev)
{
	/* Only one fifo, so upper 8 bits are always 0 */
	return dyplo_dma_get_index(dma_dev);
}

static void dyplo_dma_common_block_free_streaming(struct dyplo_dev *dev,
	struct dyplo_dma_block_set* dma_block_set,
	enum dma_data_direction direction)
{
	u32 i;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &attrs);
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
	dma_set_attr(DMA_ATTR_NON_CONSISTENT, &attrs);

	for (i = 0; i < dma_block_set->count; ++i) {
		struct dyplo_dma_block *block = &dma_block_set->blocks[i];
		if (block->mem_addr) {
			dma_unmap_single_attrs(dev->device, block->phys_addr,
					block->data.size, direction, &attrs);
			kfree(block->mem_addr);
		}
	}
}

static void dyplo_dma_common_block_free_coherent(struct dyplo_dev *dev,
	struct dyplo_dma_block_set* dma_block_set,
	enum dma_data_direction direction)
{
	u32 i;

	for (i = 0; i < dma_block_set->count; ++i) {
		struct dyplo_dma_block *block = &dma_block_set->blocks[i];
		if (block->mem_addr)
			dma_free_coherent(dev->device, block->data.size,
				block->mem_addr, block->phys_addr);
	}
}

static int dyplo_dma_common_block_free(struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_block_set* dma_block_set,
	enum dma_data_direction direction)
{
	if (!(dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_SHAREDMEM)) {
		if (dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_STREAMING)
			dyplo_dma_common_block_free_streaming(dma_dev->config_parent->parent, dma_block_set, direction);
		else
			dyplo_dma_common_block_free_coherent(dma_dev->config_parent->parent, dma_block_set, direction);
	}
	kfree(dma_block_set->blocks);
	dma_block_set->blocks = NULL;
	dma_block_set->count = 0;
	dma_block_set->size = 0;
	dma_block_set->flags = 0;
	return 0;
}

static int dyplo_dma_to_logic_block_free(struct dyplo_dma_dev *dma_dev)
{
	/* Reset the device to release all resources */
	if (!dyplo_dma_common_is_standalone_mode(dma_dev))
		dyplo_dma_to_logic_reset(dma_dev);
	return dyplo_dma_common_block_free(dma_dev, &dma_dev->dma_to_logic_blocks, DMA_TO_DEVICE);
}

static int dyplo_dma_common_block_alloc_one_coherent(
	struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_block *block,
	enum dma_data_direction direction)
{
	struct dyplo_dev *dev = dma_dev->config_parent->parent;

	block->mem_addr = dma_alloc_coherent(dev->device,
		block->data.size, &block->phys_addr, GFP_KERNEL);
	if (!block->mem_addr)
		return -ENOMEM;

	return 0;
}

static int dyplo_dma_common_block_alloc_one_streaming(
	struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_block *block,
	enum dma_data_direction direction)
{
	struct dyplo_dev *dev = dma_dev->config_parent->parent;
	int ret;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &attrs);
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
	dma_set_attr(DMA_ATTR_NON_CONSISTENT, &attrs);

	block->mem_addr = kmalloc(block->data.size, GFP_KERNEL | GFP_DMA);
	if (!block->mem_addr)
		return -ENOMEM;
	block->phys_addr = dma_map_single_attrs(dev->device, block->mem_addr,
		block->data.size, direction, &attrs);
	ret = dma_mapping_error(dev->device, block->phys_addr);
	if (unlikely(ret)) {
		kfree(block->mem_addr);
		block->mem_addr = NULL;
		return ret;
	}

	return 0;
}

static int dyplo_dma_common_block_alloc(struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_configuration_req *request,
	struct dyplo_dma_block_set* dma_block_set,
	enum dma_data_direction direction)
{
	struct dyplo_dma_block *block;
	u32 i;
	int ret;

	pr_debug("%s mode=%u count=%u size=%u\n", __func__,
		request->mode, request->count, request->size);

	if (!request->size || !request->count)
		return -EINVAL;
	request->size = PAGE_ALIGN(request->size);
	/* Pointless to use more */
	if (request->count > DMA_MAX_NUMBER_OF_COMMANDS)
		request->count = DMA_MAX_NUMBER_OF_COMMANDS;
	block = kcalloc(request->count, sizeof(*block), GFP_KERNEL);
	if (!block)
		return -ENOMEM;
	dma_block_set->blocks = block;
	dma_block_set->size = request->size;
	dma_block_set->count = request->count;
	dma_block_set->flags = (request->mode == DYPLO_DMA_MODE_BLOCK_STREAMING) ?
		DYPLO_DMA_BLOCK_FLAG_STREAMING : DYPLO_DMA_BLOCK_FLAG_COHERENT;
	if (request->mode != DYPLO_DMA_MODE_BLOCK_STREAMING) {
		/* The pre-allocated buffers are coherent, so if the blocks fit
		 * in there, we can just re-use the already allocated one */
		if (direction == DMA_FROM_DEVICE) {
			if (request->count * request->size <= dma_dev->dma_from_logic_memory_size) {
				dma_block_set->flags |= DYPLO_DMA_BLOCK_FLAG_SHAREDMEM;
				for (i = 0; i < request->count; ++i, ++block) {
					block->data.id = i;
					block->data.size = request->size;
					block->data.offset = i * request->size;
					block->mem_addr = ((char*)dma_dev->dma_from_logic_memory) + block->data.offset;
					block->phys_addr = dma_dev->dma_from_logic_handle + block->data.offset;
				}
				return 0;
			}
		} else {
			if (request->count * request->size <= dma_dev->dma_to_logic_memory_size) {
				dma_block_set->flags |= DYPLO_DMA_BLOCK_FLAG_SHAREDMEM;
				for (i = 0; i < request->count; ++i, ++block) {
					block->data.id = i;
					block->data.size = request->size;
					block->data.offset = i * request->size;
					block->mem_addr = ((char*)dma_dev->dma_to_logic_memory) + block->data.offset;
					block->phys_addr = dma_dev->dma_to_logic_handle + block->data.offset;
				}
				return 0;
			}
		}
	}
	for (i = 0; i < request->count; ++i, ++block) {
		block->data.id = i;
		block->data.size = request->size;
		block->data.offset = i * request->size;
		if (dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_STREAMING)
			ret = dyplo_dma_common_block_alloc_one_streaming(dma_dev, block, direction);
		else
			ret = dyplo_dma_common_block_alloc_one_coherent(dma_dev, block, direction);
		if (unlikely(ret)) {
			dyplo_dma_common_block_free(dma_dev, dma_block_set, direction);
			return ret;
		}
	}
	return 0;
}

static int dyplo_dma_standalone_block_alloc(struct dyplo_dma_dev *dma_dev,
	u32 size, u32 count)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	struct dyplo_dma_configuration_req request;
	int ret;

	if (count > 255)
		return -EINVAL;
	/* Needs a contiguous single block. */
	request.mode = DYPLO_DMA_MODE_STANDALONE;
	request.size = size * count;
	request.count = 1;
	ret = dyplo_dma_common_block_alloc(dma_dev, &request, &dma_dev->dma_to_logic_blocks, DMA_BIDIRECTIONAL);
	if (ret)
		return ret;
	/* Configure the DMA node */
	iowrite32(dma_dev->dma_to_logic_blocks.blocks[0].phys_addr, control_base + (DYPLO_DMA_STANDALONE_STARTADDR>>2));
	iowrite32(size, control_base + (DYPLO_DMA_STANDALONE_BLOCKSIZE>>2));
	iowrite32(BIT(0) | (count << 8), control_base + (DYPLO_DMA_STANDALONE_CONTROL>>2));

	return 0;
}

/* For backward compatibility */
static int dyplo_dma_to_logic_block_alloc(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block_alloc_req __user *arg)
{
	struct dyplo_buffer_block_alloc_req request;
	struct dyplo_dma_configuration_req r;
	int ret;

	if (copy_from_user(&request, arg, sizeof(request)))
		return -EFAULT;
	r.mode = DYPLO_DMA_MODE_BLOCK_COHERENT;
	r.size = request.size;
	r.count = request.count;

	dyplo_dma_to_logic_block_free(dma_dev);
	ret = dyplo_dma_common_block_alloc(dma_dev, &r, &dma_dev->dma_to_logic_blocks, DMA_TO_DEVICE);
	if (ret)
		return ret;

	request.size = r.size;
	request.count = r.count;
	if (copy_to_user(arg, &request, sizeof(request)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_to_logic_block_query(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block __user *arg)
{
	__u32 request_id;

	if (get_user(request_id, &arg->id))
		return -EFAULT;

	if (request_id >= dma_dev->dma_to_logic_blocks.count)
		return -EINVAL;

	if (copy_to_user(arg, &dma_dev->dma_to_logic_blocks.blocks[request_id].data, sizeof(struct dyplo_buffer_block)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_to_logic_block_enqueue(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block __user *arg)
{
	struct dyplo_buffer_block request;
	struct dyplo_dma_block *block;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;

	if (copy_from_user(&request, arg, sizeof(request)))
		return -EFAULT;

	if (request.id >= dma_dev->dma_to_logic_blocks.count)
		return -EINVAL;

	block = &dma_dev->dma_to_logic_blocks.blocks[request.id];
	if (block->data.state)
		return -EBUSY;

	block->data.bytes_used = request.bytes_used;
	block->data.user_signal = request.user_signal;

	if (dma_dev->dma_to_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING)
		dma_sync_single_for_device(dma_dev->config_parent->parent->device,
			block->phys_addr, block->data.bytes_used, DMA_TO_DEVICE);

	/* This operation never blocks, unless something is wrong in HW */
	if (!(dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_STATUS) & 0xFF0000))
		return -EWOULDBLOCK;
	pr_debug("%s sending addr=%#x size=%u\n", __func__,
			(unsigned int)block->phys_addr, block->data.bytes_used);
	iowrite32_quick(block->phys_addr, control_base + (DYPLO_DMA_TOLOGIC_STARTADDR>>2));
	iowrite32_quick(block->data.user_signal, control_base + (DYPLO_DMA_TOLOGIC_USERBITS>>2));
	iowrite32(block->data.bytes_used, control_base + (DYPLO_DMA_TOLOGIC_BYTESIZE>>2));
	block->data.state = 1;

	if (copy_to_user(arg, &block->data, sizeof(struct dyplo_buffer_block)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_to_logic_block_dequeue(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block __user *arg, bool is_blocking)
{
	struct dyplo_buffer_block request;
	struct dyplo_dma_block *block;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	u32 status;
	dma_addr_t start_addr;

	if (copy_from_user(&request, arg, sizeof(request)))
		return -EFAULT;

	if (request.id >= dma_dev->dma_to_logic_blocks.count)
		return -EINVAL;

	block = &dma_dev->dma_to_logic_blocks.blocks[request.id];
	if (!block->data.state)
		return -EINVAL;

	if (is_blocking) {
		DEFINE_WAIT(wait);
		for (;;) {
			prepare_to_wait(&dma_dev->wait_queue_to_logic, &wait, TASK_INTERRUPTIBLE);
			status = dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_STATUS);
			if (status & 0xFF000000)
				break; /* Results available, done waiting */
			if (signal_pending(current)) {
				finish_wait(&dma_dev->wait_queue_from_logic, &wait);
				return -ERESTARTSYS;
			}
			/* Enable interrupt */
			dyplo_dma_to_logic_irq_enable(control_base);
			schedule();
		}
		finish_wait(&dma_dev->wait_queue_from_logic, &wait);
	} else {
		status = dyplo_reg_read_quick(control_base, DYPLO_DMA_TOLOGIC_STATUS);
		if ((status & 0xFF000000) == 0)
			return -EAGAIN;
	}
	start_addr = dyplo_reg_read(control_base, DYPLO_DMA_TOLOGIC_RESULT_ADDR);
	if (start_addr != block->phys_addr) {
		pr_err("%s Expected addr %#x result %#x\n", __func__, (u32)block->phys_addr, (u32)start_addr);
		return -EIO;
	}

	if (dma_dev->dma_to_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING)
		dma_sync_single_for_cpu(dma_dev->config_parent->parent->device,
			block->phys_addr, block->data.bytes_used, DMA_TO_DEVICE);

	block->data.state = 0;

	if (copy_to_user(arg, &block->data, sizeof(struct dyplo_buffer_block)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_common_mmap(struct dyplo_dma_dev *dma_dev,
	struct vm_area_struct *vma,
	struct dyplo_dma_block_set* dma_block_set)
{
	const unsigned int count = dma_block_set->count;
	struct dyplo_dma_block *block = NULL;
	unsigned int vm_offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned int i;

	pr_debug("%s offset=%u\n", __func__, vm_offset);

	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;

	if (dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_SHAREDMEM) {
		block = &dma_block_set->blocks[0];
		return dma_mmap_coherent(dma_dev->config_parent->parent->device,
			vma, block->mem_addr, block->phys_addr,
			dma_block_set->size * dma_block_set->count);
	}

	for (i = 0; i < count; ++i) {
		if (dma_block_set->blocks[i].data.offset == vm_offset) {
			block = &dma_block_set->blocks[i];
			break;
		}
	}

	if (block == NULL) {
		pr_err("%s offset %u not found\n", __func__, vm_offset);
		return -EINVAL;
	}

	if (PAGE_ALIGN(block->data.size) < vma->vm_end - vma->vm_start) {
		pr_err("%s size mismatch\n", __func__);
		return -EINVAL;
	}

	vma->vm_pgoff = 0;

	if (dma_block_set->flags & DYPLO_DMA_BLOCK_FLAG_STREAMING)
		return remap_pfn_range(vma, vma->vm_start,
			page_to_pfn(virt_to_page(block->mem_addr)), /* Or just block->phys_addr? */
			block->data.size, vma->vm_page_prot);
	else
		return dma_mmap_coherent(dma_dev->config_parent->parent->device,
			vma, block->mem_addr, block->phys_addr, block->data.size);
}

static int dyplo_dma_to_logic_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;

	return dyplo_dma_common_mmap(dma_dev, vma,
		&dma_dev->dma_to_logic_blocks);
}

/* forward */
static int dyplo_dma_from_logic_block_free(struct dyplo_dma_dev *dma_dev);

static int dyplo_dma_to_logic_reconfigure(struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_configuration_req __user *arg)
{
	struct dyplo_dma_configuration_req request;
	int ret;

	if (copy_from_user(&request, arg, sizeof(request)))
		return -EFAULT;

	pr_debug("%s mode=%d count=%u size=%u\n", __func__,
		request.mode, request.count, request.size);

	dyplo_dma_to_logic_block_free(dma_dev);

	switch (request.mode) {
		case DYPLO_DMA_MODE_STANDALONE:
			/* Device must be open in special mode */
			if (!(dma_dev->open_mode & FMODE_EXCL))
				return -EACCES;
			/* Clean up the "from" side as well */
			dyplo_dma_from_logic_block_free(dma_dev);
			dyplo_dma_common_set_standalone_mode(dma_dev, true);
			ret = dyplo_dma_standalone_block_alloc(dma_dev,
				request.size, request.count);
			break;
		case DYPLO_DMA_MODE_RINGBUFFER_BOUNCE:
			dyplo_dma_common_set_standalone_mode(dma_dev, false);
			request.size = dma_dev->dma_to_logic_block_size;
			request.count = dma_dev->dma_to_logic_memory_size / dma_dev->dma_to_logic_block_size;
			ret = 0;
			break;
		case DYPLO_DMA_MODE_BLOCK_COHERENT:
		case DYPLO_DMA_MODE_BLOCK_STREAMING:
			dyplo_dma_common_set_standalone_mode(dma_dev, false);
			ret = dyplo_dma_common_block_alloc(dma_dev,
				&request, &dma_dev->dma_to_logic_blocks, DMA_TO_DEVICE);
			break;
		default:
			ret = -EINVAL;
	}
	if (ret)
		return ret;

	if (copy_to_user(arg, &request, sizeof(request)))
		return -EFAULT;

	return 0;
}

static long dyplo_dma_standalone_enable(
	struct dyplo_dma_dev *dma_dev,
	enum dma_data_direction direction,
	bool enable)
{
	u32 __iomem *ctrl_reg = dma_dev->config_parent->control_base;
	ctrl_reg += (direction == DMA_TO_DEVICE) ? DYPLO_DMA_TOLOGIC_CONTROL>>2 : DYPLO_DMA_FROMLOGIC_CONTROL>>2;

	dyplo_set_bits(ctrl_reg, BIT(0), enable);

	return 0;
}

static long dyplo_dma_standalone_setconfig(
	struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_standalone_config __user *arg,
	enum dma_data_direction direction)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	struct dyplo_dma_standalone_config cfg;

	if (copy_from_user(&cfg, arg, sizeof(cfg)))
		return -EFAULT;

	switch (direction) {
		case DMA_TO_DEVICE:
			memcpy_toio(control_base + (DYPLO_DMA_STANDALONE_TOLOGIC_BASE>>2),
				&cfg, sizeof(cfg));
			break;
		case DMA_FROM_DEVICE:
			memcpy_toio(control_base + (DYPLO_DMA_STANDALONE_FROMLOGIC_BASE>>2),
				&cfg, sizeof(cfg));
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static long dyplo_dma_standalone_getconfig(
	struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_standalone_config __user *arg,
	enum dma_data_direction direction)
{
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	struct dyplo_dma_standalone_config cfg;

	switch (direction) {
		case DMA_TO_DEVICE:
			memcpy_fromio(&cfg,
				control_base + (DYPLO_DMA_STANDALONE_TOLOGIC_BASE>>2),
				sizeof(cfg));
			break;
		case DMA_FROM_DEVICE:
			memcpy_fromio(&cfg,
				control_base + (DYPLO_DMA_STANDALONE_FROMLOGIC_BASE>>2),
				sizeof(cfg));
			break;
		default:
			return -EINVAL;
	}

	if (copy_to_user(arg, &cfg, sizeof(cfg)))
		return -EFAULT;

	return 0;
}

static long dyplo_dma_to_logic_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;
	if (unlikely(dma_dev == NULL))
		return -ENODEV;

	pr_debug("%s cmd=%#x (%d) arg=%#lx\n", __func__, cmd, _IOC_NR(cmd), arg);

	if (_IOC_TYPE(cmd) != DYPLO_IOC_MAGIC)
		return -ENOTTY;

	switch (_IOC_NR(cmd))
	{
		case DYPLO_IOC_ROUTE_QUERY_ID:
			return dyplo_dma_get_route_id(dma_dev);
		case DYPLO_IOC_ROUTE_TELL_TO_LOGIC:
			return dyplo_dma_add_route(dma_dev, dyplo_dma_get_route_id(dma_dev), arg);
		case DYPLO_IOC_ROUTE_TELL_FROM_LOGIC:
			return -ENOTTY; /* Cannot route to this node */
		case DYPLO_IOC_TRESHOLD_QUERY:
			return dma_dev->dma_to_logic_block_size;
		case DYPLO_IOC_TRESHOLD_TELL:
			if (dma_dev->dma_to_logic_block_size == arg)
				return 0;
			if ((dma_dev->dma_to_logic_head != dma_dev->dma_to_logic_tail) ||
					!kfifo_is_empty(&dma_dev->dma_to_logic_wip))
				return -EBUSY;
			if (dma_dev->dma_to_logic_memory_size % arg)
				return -EINVAL; /* Must be divisable */
			dma_dev->dma_to_logic_block_size = arg;
			dma_dev->dma_to_logic_head = 0;
			dma_dev->dma_to_logic_tail = 0;
			return 0;
		case DYPLO_IOC_RESET_FIFO_WRITE:
		case DYPLO_IOC_RESET_FIFO_READ:
			return dyplo_dma_to_logic_reset(dma_dev);
		case DYPLO_IOC_USERSIGNAL_QUERY:
			return dyplo_reg_read_quick(dma_dev->config_parent->control_base, DYPLO_DMA_TOLOGIC_USERBITS);
		case DYPLO_IOC_USERSIGNAL_TELL:
			iowrite32_quick(arg, dma_dev->config_parent->control_base + (DYPLO_DMA_TOLOGIC_USERBITS>>2));
			return 0;
		case DYPLO_IOC_DMA_RECONFIGURE:
			return dyplo_dma_to_logic_reconfigure(dma_dev,
				(struct dyplo_dma_configuration_req __user *)arg);
		case DYPLO_IOC_DMABLOCK_ALLOC:
			return dyplo_dma_to_logic_block_alloc(dma_dev,
				(struct dyplo_buffer_block_alloc_req __user *)arg);
		case DYPLO_IOC_DMABLOCK_FREE:
			return dyplo_dma_to_logic_block_free(dma_dev);
		case DYPLO_IOC_DMABLOCK_QUERY:
			return dyplo_dma_to_logic_block_query(dma_dev,
				(struct dyplo_buffer_block __user *)arg);
		case DYPLO_IOC_DMABLOCK_ENQUEUE:
			return dyplo_dma_to_logic_block_enqueue(dma_dev,
				(struct dyplo_buffer_block __user *)arg);
		case DYPLO_IOC_DMABLOCK_DEQUEUE:
			return dyplo_dma_to_logic_block_dequeue(dma_dev,
				(struct dyplo_buffer_block __user *)arg,
				(filp->f_flags & O_NONBLOCK) == 0);
		case DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC:
			if (_IOC_DIR(cmd) & _IOC_WRITE)
				return dyplo_dma_standalone_setconfig(dma_dev,
					(struct dyplo_dma_standalone_config __user *)arg,
					DMA_TO_DEVICE);
			if (_IOC_DIR(cmd) & _IOC_READ)
				return dyplo_dma_standalone_getconfig(dma_dev,
					(struct dyplo_dma_standalone_config __user *)arg,
					DMA_TO_DEVICE);
		case DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC:
			if (_IOC_DIR(cmd) & _IOC_WRITE)
				return dyplo_dma_standalone_setconfig(dma_dev,
					(struct dyplo_dma_standalone_config __user *)arg,
					DMA_FROM_DEVICE);
			if (_IOC_DIR(cmd) & _IOC_READ)
				return dyplo_dma_standalone_getconfig(dma_dev,
					(struct dyplo_dma_standalone_config __user *)arg,
					DMA_FROM_DEVICE);
		case DYPLO_IOC_DMASTANDALONE_START_TO_LOGIC:
			return dyplo_dma_standalone_enable(dma_dev, DMA_TO_DEVICE, true);
		case DYPLO_IOC_DMASTANDALONE_START_FROM_LOGIC:
			return dyplo_dma_standalone_enable(dma_dev, DMA_FROM_DEVICE, true);
		case DYPLO_IOC_DMASTANDALONE_STOP_TO_LOGIC:
			return dyplo_dma_standalone_enable(dma_dev, DMA_TO_DEVICE, false);
		case DYPLO_IOC_DMASTANDALONE_STOP_FROM_LOGIC:
			return dyplo_dma_standalone_enable(dma_dev, DMA_FROM_DEVICE, false);
		default:
			return -ENOTTY;
	}
}

static int dyplo_dma_from_logic_block_free(struct dyplo_dma_dev *dma_dev)
{
	/* Reset the device to release all resources */
	dyplo_dma_from_logic_reset(dma_dev);
	return dyplo_dma_common_block_free(dma_dev, &dma_dev->dma_from_logic_blocks, DMA_FROM_DEVICE);
}

static int dyplo_dma_from_logic_block_alloc(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block_alloc_req __user *arg)
{
	struct dyplo_buffer_block_alloc_req request;
	struct dyplo_dma_configuration_req r;
	int ret;

	if (copy_from_user(&request, arg, sizeof(request)))
		return -EFAULT;
	r.mode = DYPLO_DMA_MODE_BLOCK_STREAMING;
	r.size = request.size;
	r.count = request.count;

	dyplo_dma_from_logic_block_free(dma_dev);
	ret = dyplo_dma_common_block_alloc(dma_dev, &r,
		&dma_dev->dma_from_logic_blocks, DMA_FROM_DEVICE);
	if (ret)
		return ret;

	request.size = r.size;
	request.count = r.count;
	if (copy_to_user(arg, &request, sizeof(request)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_from_logic_block_query(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block __user *arg)
{
	__u32 request_id;

	if (get_user(request_id, &arg->id))
		return -EFAULT;

	if (request_id >= dma_dev->dma_from_logic_blocks.count)
		return -EINVAL;

	if (copy_to_user(arg, &dma_dev->dma_from_logic_blocks.blocks[request_id].data, sizeof(struct dyplo_buffer_block)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_from_logic_block_enqueue(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block __user *arg)
{
	__u32 request_id;
	__u32 request_bytes_used;
	struct dyplo_dma_block *block;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	u32 status_reg;

	if (get_user(request_id, &arg->id))
		return -EFAULT;

	if (request_id >= dma_dev->dma_from_logic_blocks.count)
		return -EINVAL;

	block = &dma_dev->dma_from_logic_blocks.blocks[request_id];
	if (block->data.state)
		return -EBUSY;

	if (get_user(request_bytes_used, &arg->bytes_used))
		return -EFAULT;
	if ((request_bytes_used > block->data.size) || (request_bytes_used == 0))
		return -EINVAL;

	if (dma_dev->dma_from_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING)
		dma_sync_single_for_device(dma_dev->config_parent->parent->device,
			block->phys_addr, block->data.bytes_used, DMA_FROM_DEVICE);

	/* Should not block here because we never allocate more blocks than
	 * what fits in the hardware queue. */
	status_reg = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_STATUS);
	pr_debug("%s status=%#x\n", __func__, status_reg);
	if (!(status_reg & 0xFF0000))
		return -EWOULDBLOCK;

	/* Send to logic */
	pr_debug("%s sending addr=%#x size=%u\n", __func__,
			(u32)block->phys_addr, block->data.size);
	iowrite32(block->phys_addr, control_base + (DYPLO_DMA_FROMLOGIC_STARTADDR>>2));
	iowrite32(request_bytes_used, control_base + (DYPLO_DMA_FROMLOGIC_BYTESIZE>>2));
	block->data.bytes_used = 0;
	block->data.state = 1;

	if (copy_to_user(arg, &block->data, sizeof(struct dyplo_buffer_block)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_from_logic_block_dequeue(struct dyplo_dma_dev *dma_dev,
	struct dyplo_buffer_block __user *arg, bool is_blocking)
{
	struct dyplo_buffer_block request;
	struct dyplo_dma_block *block;
	u32 __iomem *control_base = dma_dev->config_parent->control_base;
	u32 status_reg;
	dma_addr_t start_addr;
	DEFINE_WAIT(wait);

	if (copy_from_user(&request, arg, sizeof(request)))
		return -EFAULT;

	if (request.id >= dma_dev->dma_from_logic_blocks.count)
		return -EINVAL;

	block = &dma_dev->dma_from_logic_blocks.blocks[request.id];
	if (!block->data.state)
		return -EINVAL;

	for(;;) {
		if (is_blocking)
			prepare_to_wait(&dma_dev->wait_queue_from_logic, &wait, TASK_INTERRUPTIBLE);
		status_reg = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_STATUS);
		pr_debug("%s status=%#x\n", __func__, status_reg);
		/* TODO: Blocking */
		if (status_reg & 0xFF000000)
			break; /* Result(s) available, we're done */
		if (signal_pending(current)) {
			if (is_blocking)
				finish_wait(&dma_dev->wait_queue_from_logic, &wait);
			return -ERESTARTSYS;
		}
		/* Enable interrupt */
		dyplo_dma_from_logic_irq_enable(control_base);
		if (!is_blocking)
			return -EAGAIN;
		schedule();
	}
	if (is_blocking)
		finish_wait(&dma_dev->wait_queue_from_logic, &wait);

	start_addr = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_RESULT_ADDR);
	if (start_addr != block->phys_addr) {
		pr_err("%s Expected addr %#x result %#x\n", __func__, (u32)block->phys_addr, (u32)start_addr);
		return -EIO;
	}
	block->data.user_signal = dyplo_reg_read_quick(control_base, DYPLO_DMA_FROMLOGIC_RESULT_USERBITS);
	block->data.bytes_used = dyplo_reg_read(control_base, DYPLO_DMA_FROMLOGIC_RESULT_BYTESIZE);
	block->data.state = 0;

	if (dma_dev->dma_from_logic_blocks.flags & DYPLO_DMA_BLOCK_FLAG_STREAMING)
		dma_sync_single_for_cpu(dma_dev->config_parent->parent->device,
			block->phys_addr, block->data.bytes_used, DMA_FROM_DEVICE);

	if (copy_to_user(arg, &block->data, sizeof(struct dyplo_buffer_block)))
		return -EFAULT;

	return 0;
}

static int dyplo_dma_from_logic_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;

	return dyplo_dma_common_mmap(dma_dev, vma,
		&dma_dev->dma_from_logic_blocks);
}

static int dyplo_dma_from_logic_reconfigure(struct dyplo_dma_dev *dma_dev,
	struct dyplo_dma_configuration_req __user *arg)
{
	struct dyplo_dma_configuration_req request;
	int ret;

	if (copy_from_user(&request, arg, sizeof(request)))
		return -EFAULT;

	pr_debug("%s mode=%d count=%u size=%u\n", __func__,
		request.mode, request.count, request.size);

	dyplo_dma_from_logic_block_free(dma_dev);

	switch (request.mode) {
		case DYPLO_DMA_MODE_STANDALONE:
			ret = -EACCES; /* Must open in read+write mode for this */
			break;
		case DYPLO_DMA_MODE_RINGBUFFER_BOUNCE:
			request.size = dma_dev->dma_from_logic_block_size;
			request.count = dma_dev->dma_from_logic_memory_size / dma_dev->dma_from_logic_block_size;
			ret = 0;
			break;
		case DYPLO_DMA_MODE_BLOCK_COHERENT:
		case DYPLO_DMA_MODE_BLOCK_STREAMING:
			ret = dyplo_dma_common_block_alloc(dma_dev,
				&request, &dma_dev->dma_from_logic_blocks, DMA_FROM_DEVICE);
			break;
		default:
			ret = -EINVAL;
	}
	if (ret)
		return ret;

	if (copy_to_user(arg, &request, sizeof(request)))
		return -EFAULT;

	return 0;
}

static long dyplo_dma_from_logic_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_dma_dev *dma_dev = filp->private_data;
	if (unlikely(dma_dev == NULL))
		return -ENODEV;

	pr_debug("%s cmd=%#x (%d) arg=%#lx\n", __func__, cmd, _IOC_NR(cmd), arg);

	if (_IOC_TYPE(cmd) != DYPLO_IOC_MAGIC)
		return -ENOTTY;

	switch (_IOC_NR(cmd))
	{
		case DYPLO_IOC_ROUTE_QUERY_ID:
			return dyplo_dma_get_route_id(dma_dev);
		case DYPLO_IOC_ROUTE_TELL_TO_LOGIC:
			return -ENOTTY; /* Cannot route from this node */
		case DYPLO_IOC_ROUTE_TELL_FROM_LOGIC:
			return dyplo_dma_add_route(dma_dev, arg, dyplo_dma_get_route_id(dma_dev));
		case DYPLO_IOC_TRESHOLD_QUERY:
			return dma_dev->dma_from_logic_block_size;
		case DYPLO_IOC_TRESHOLD_TELL:
			if (dma_dev->dma_from_logic_block_size == arg)
				return 0;
			if ((dma_dev->dma_from_logic_head != dma_dev->dma_from_logic_tail) ||
					dma_dev->dma_from_logic_full)
				return -EBUSY; /* Cannot change value */
			if (dma_dev->dma_from_logic_memory_size % arg)
				return -EINVAL; /* Must be divisable */
			dma_dev->dma_from_logic_block_size = arg;
			dma_dev->dma_from_logic_head = 0;
			dma_dev->dma_from_logic_tail = 0;
			return 0;
		case DYPLO_IOC_RESET_FIFO_WRITE:
		case DYPLO_IOC_RESET_FIFO_READ:
			return dyplo_dma_from_logic_reset(dma_dev);
		case DYPLO_IOC_USERSIGNAL_QUERY:
			return dma_dev->dma_from_logic_current_op.user_signal;
		case DYPLO_IOC_USERSIGNAL_TELL:
			return -EACCES;
		case DYPLO_IOC_DMA_RECONFIGURE:
			return dyplo_dma_from_logic_reconfigure(dma_dev,
				(struct dyplo_dma_configuration_req __user *)arg);
		case DYPLO_IOC_DMABLOCK_ALLOC:
			return dyplo_dma_from_logic_block_alloc(dma_dev,
				(struct dyplo_buffer_block_alloc_req __user *)arg);
		case DYPLO_IOC_DMABLOCK_FREE:
			return dyplo_dma_from_logic_block_free(dma_dev);
		case DYPLO_IOC_DMABLOCK_QUERY:
			return dyplo_dma_from_logic_block_query(dma_dev,
				(struct dyplo_buffer_block __user *)arg);
		case DYPLO_IOC_DMABLOCK_ENQUEUE:
			return dyplo_dma_from_logic_block_enqueue(dma_dev,
				(struct dyplo_buffer_block __user *)arg);
		case DYPLO_IOC_DMABLOCK_DEQUEUE:
			return dyplo_dma_from_logic_block_dequeue(dma_dev,
				(struct dyplo_buffer_block __user *)arg,
				(filp->f_flags & O_NONBLOCK) == 0);
		default:
			return -ENOTTY;
	}
}

static const struct file_operations dyplo_dma_to_logic_fops =
{
	.owner = THIS_MODULE,
	.write = dyplo_dma_write,
	.llseek = no_llseek,
	.poll = dyplo_dma_to_logic_poll,
	.mmap = dyplo_dma_to_logic_mmap,
	.unlocked_ioctl = dyplo_dma_to_logic_ioctl,
	.open = dyplo_dma_open,
	.release = dyplo_dma_to_logic_release,
};

static const struct file_operations dyplo_dma_from_logic_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_dma_read,
	.llseek = no_llseek,
	.poll = dyplo_dma_from_logic_poll,
	.mmap = dyplo_dma_from_logic_mmap,
	.unlocked_ioctl = dyplo_dma_from_logic_ioctl,
	.open = dyplo_dma_open,
	.release = dyplo_dma_from_logic_release,
};

/* Common file operations struct. "open" will set one of the above into
 * the inode. */
static const struct file_operations dyplo_dma_fops =
{
	.owner = THIS_MODULE,
	.open = dyplo_dma_open,
};


/* Interrupt service routine for DMA node */
static irqreturn_t dyplo_dma_isr(struct dyplo_dev *dev, struct dyplo_config_dev *cfg_dev)
{
	struct dyplo_dma_dev *dma_dev = cfg_dev->private_data;
	u32 status = dyplo_reg_read_quick(
		cfg_dev->control_base, DYPLO_REG_FIFO_IRQ_STATUS);
	pr_debug("%s(status=%#x)\n", __func__, status);
	if (!status)
		return IRQ_NONE;
	/* Acknowledge IRQ */
	iowrite32_quick(status,
			cfg_dev->control_base + (DYPLO_REG_FIFO_IRQ_CLR>>2));
	/* Clear the reset command when done */
	if (status & BIT(15))
		iowrite32(
			dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_DMA_TOLOGIC_CONTROL) & ~BIT(1),
			cfg_dev->control_base + (DYPLO_DMA_TOLOGIC_CONTROL>>2));
	if (status & BIT(31))
		iowrite32(
			dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_DMA_FROMLOGIC_CONTROL) & ~BIT(1),
			cfg_dev->control_base + (DYPLO_DMA_FROMLOGIC_CONTROL>>2));
	/* Wake up the proper queues */
	if (status & (BIT(0) | BIT(15)))
		wake_up_interruptible(&dma_dev->wait_queue_to_logic);
	if (status & (BIT(16) | BIT(31)))
		wake_up_interruptible(&dma_dev->wait_queue_from_logic);
	return IRQ_HANDLED;
}

/* Interrupt service routine for generic nodes (clear RESET command) */
static irqreturn_t dyplo_generic_isr(struct dyplo_dev *dev, struct dyplo_config_dev *cfg_dev)
{
	u32 status = dyplo_reg_read_quick(
		cfg_dev->control_base, DYPLO_REG_FIFO_IRQ_STATUS);
	pr_debug("%s(status=%#x)\n", __func__, status);
	if (!status)
		return IRQ_NONE;
	/* Acknowledge IRQ */
	dyplo_reg_write_quick(cfg_dev->control_base,
		DYPLO_REG_FIFO_IRQ_CLR, status);
	/* Clear the reset command when done */
	if (status & BIT(0))
		dyplo_reg_write_quick(cfg_dev->control_base,
			DYPLO_REG_NODE_RESET_FIFOS, 0);
	/* TODO: Wake up whomever triggered the reset */
	return IRQ_HANDLED;
}

static irqreturn_t dyplo_isr(int irq, void *dev_id)
{
	struct dyplo_dev *dev = (struct dyplo_dev*)dev_id;
	u32 mask;
	int index = 0;
	irqreturn_t result = IRQ_NONE;

	mask = dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_IRQ_MASK);
	pr_debug("%s(mask=0x%x)\n", __func__, mask);
	while (mask) {
		mask >>= 1; /* CPU node is '0', ctl doesn't need interrupt */
		if (mask & 1) {
			struct dyplo_config_dev *cfg_dev = &dev->config_devices[index];
			if (cfg_dev->isr && (cfg_dev->isr(dev, cfg_dev) != IRQ_NONE))
				result = IRQ_HANDLED;
		}
		++index;
	}
	return result;
}

static int create_sub_devices_cpu_fifo(struct dyplo_config_dev *cfg_dev)
{
	int retval;
	struct dyplo_fifo_control_dev *fifo_ctl_dev;
	struct dyplo_dev *dev = cfg_dev->parent;
	struct device *device = dev->device;
	dev_t first_fifo_devt;
	int fifo_index = 0;
	struct device *char_device;
	u32 version_id = dyplo_cfg_get_version_id(cfg_dev);
	u8 i;
	u8 number_of_write_fifos;
	u8 number_of_read_fifos;

	if ((version_id & DYPLO_VERSION_ID_MASK_REVISION) != 0x0100) {
		dev_err(device, "Unsupported CPU FIFO node version: %#x\n",
			version_id);
		return -EINVAL;
	}

	fifo_ctl_dev = devm_kzalloc(device,
		sizeof(struct dyplo_fifo_control_dev), GFP_KERNEL);
	if (!fifo_ctl_dev)
		return -ENOMEM;
	fifo_ctl_dev->config_parent = cfg_dev;
	cfg_dev->private_data = fifo_ctl_dev;

	number_of_write_fifos = dyplo_number_of_output_queues(cfg_dev);
	number_of_read_fifos = dyplo_number_of_input_queues(cfg_dev);
	fifo_ctl_dev->fifo_devices = devm_kcalloc(device,
		number_of_write_fifos + number_of_read_fifos, sizeof(struct dyplo_fifo_dev),
		GFP_KERNEL);
	if (!fifo_ctl_dev->fifo_devices) {
		dev_err(device, "No memory for %d fifo devices\n",
			number_of_write_fifos + number_of_read_fifos);
		return -ENOMEM;
	}
	fifo_ctl_dev->number_of_fifo_write_devices = number_of_write_fifos;
	fifo_ctl_dev->number_of_fifo_read_devices = number_of_read_fifos;

	first_fifo_devt = dev->devt_last;
	retval = register_chrdev_region(first_fifo_devt,
				(number_of_write_fifos + number_of_read_fifos), DRIVER_FIFO_CLASS_NAME);
	if (retval)
		goto error_register_chrdev_region;
	dev->devt_last = first_fifo_devt +
		number_of_write_fifos + number_of_read_fifos;
	fifo_ctl_dev->devt_first_fifo_device = first_fifo_devt;

	cdev_init(&fifo_ctl_dev->cdev_fifo_write, &dyplo_fifo_write_fops);
	fifo_ctl_dev->cdev_fifo_write.owner = THIS_MODULE;
	retval = cdev_add(&fifo_ctl_dev->cdev_fifo_write,
		first_fifo_devt, number_of_write_fifos);
	if (retval) {
		dev_err(device, "cdev_add(cdev_fifo_write) failed\n");
		goto error_cdev_w;
	}
	cdev_init(&fifo_ctl_dev->cdev_fifo_read, &dyplo_fifo_read_fops);
	fifo_ctl_dev->cdev_fifo_read.owner = THIS_MODULE;
	retval = cdev_add(&fifo_ctl_dev->cdev_fifo_read,
		first_fifo_devt+number_of_write_fifos, number_of_read_fifos);
	if (retval) {
		dev_err(device, "cdev_add(cdev_fifo_read) failed\n");
		goto error_cdev_r;
	}

	for (i = 0; i < number_of_write_fifos; ++i) {
		struct dyplo_fifo_dev *fifo_dev = &fifo_ctl_dev->fifo_devices[fifo_index];
		fifo_dev->config_parent = cfg_dev;
		fifo_dev->index = i;
		init_waitqueue_head(&fifo_dev->fifo_wait_queue);
		char_device = device_create(dev->class, device,
			first_fifo_devt + fifo_index,
			fifo_dev, DRIVER_FIFO_WRITE_NAME, dev->count_fifo_write_devices + i);
		if (IS_ERR(char_device)) {
			dev_err(device, "unable to create fifo write device %d\n",
				i);
			retval = PTR_ERR(char_device);
			goto failed_device_create;
		}
		++fifo_index;
	}
	for (i = 0; i < number_of_read_fifos; ++i) {
		struct dyplo_fifo_dev *fifo_dev = &fifo_ctl_dev->fifo_devices[fifo_index];
		fifo_dev->config_parent = cfg_dev;
		fifo_dev->index = i;
		init_waitqueue_head(&fifo_dev->fifo_wait_queue);
		char_device = device_create(dev->class, device,
			first_fifo_devt + fifo_index,
			fifo_dev, DRIVER_FIFO_READ_NAME, dev->count_fifo_read_devices + i);
		if (IS_ERR(char_device)) {
			dev_err(char_device, "unable to create fifo read device %d\n",
				i);
			retval = PTR_ERR(char_device);
			goto failed_device_create;
		}
		++fifo_index;
	}

	cfg_dev->isr = dyplo_fifo_isr;

	dev->count_fifo_write_devices += number_of_write_fifos;
	dev->count_fifo_read_devices += number_of_read_fifos;

	return 0;

failed_device_create:
	while (fifo_index) {
		device_destroy(dev->class, first_fifo_devt + fifo_index);
		--fifo_index;
	}
error_cdev_r:
error_cdev_w:
	unregister_chrdev_region(first_fifo_devt, dev->devt_last);
	dev->devt_last = first_fifo_devt;
error_register_chrdev_region:
	return retval;
}

static int create_sub_devices_dma_fifo(
	struct dyplo_config_dev *cfg_dev)
{
	struct dyplo_dev *dev = cfg_dev->parent;
	struct device *device = dev->device;
	int retval;
	dev_t first_fifo_devt;
	struct dyplo_dma_dev* dma_dev;
	struct device *char_device;
	u32 version_id = dyplo_cfg_get_version_id(cfg_dev);

	if ((version_id & DYPLO_VERSION_ID_MASK_REVISION) != 0x0100) {
		dev_err(device, "Unsupported DMA FIFO node revision: %#x\n",
			version_id);
		return -EINVAL;
	}

	dma_dev = devm_kzalloc(device, sizeof(struct dyplo_dma_dev), GFP_KERNEL);
	if (!dma_dev) {
		dev_err(device, "No memory for DMA device\n");
		return -ENOMEM;
	}
	cfg_dev->private_data = dma_dev;
	dma_dev->config_parent = cfg_dev;
	init_waitqueue_head(&dma_dev->wait_queue_to_logic);
	init_waitqueue_head(&dma_dev->wait_queue_from_logic);
	INIT_KFIFO(dma_dev->dma_to_logic_wip);

	first_fifo_devt = dev->devt_last;
	retval = register_chrdev_region(first_fifo_devt, 1, DRIVER_DMA_CLASS_NAME);
	if (retval)
		goto error_register_chrdev_region;
	dev->devt_last += 1;

	dma_dev->dma_to_logic_memory = dma_alloc_coherent(device,
		dyplo_dma_memory_size, &dma_dev->dma_to_logic_handle, GFP_DMA | GFP_KERNEL);
	if (!dma_dev->dma_to_logic_memory) {
		dev_err(device, "Failed dma_alloc_coherent for DMA device\n");
		retval = -ENOMEM;
		goto error_dma_to_logic_alloc;
	}
	dma_dev->dma_to_logic_memory_size = dyplo_dma_memory_size;
	dma_dev->dma_to_logic_block_size = dyplo_dma_default_block_size;

	dma_dev->dma_from_logic_memory = dma_alloc_coherent(device,
		dyplo_dma_memory_size, &dma_dev->dma_from_logic_handle, GFP_DMA | GFP_KERNEL);
	if (!dma_dev->dma_from_logic_memory) {
		dev_err(device, "Failed dma_alloc_coherent for DMA device\n");
		retval = -ENOMEM;
		goto error_dma_from_logic_alloc;
	}
	dma_dev->dma_from_logic_memory_size = dyplo_dma_memory_size;
	dma_dev->dma_from_logic_block_size = dyplo_dma_default_block_size;

	cdev_init(&dma_dev->cdev_dma, &dyplo_dma_fops);
	dma_dev->cdev_dma.owner = THIS_MODULE;
	retval = cdev_add(&dma_dev->cdev_dma, first_fifo_devt, 1);
	if (retval) {
		dev_err(device, "cdev_add(dma_dev) failed\n");
		goto error_cdev_add;
	}
	char_device = device_create(dev->class, device,
		first_fifo_devt, dma_dev, DRIVER_DMA_DEVICE_NAME, dev->number_of_dma_devices);
	if (IS_ERR(char_device)) {
		dev_err(device, "unable to create DMA device %d\n", dev->number_of_dma_devices);
		retval = PTR_ERR(char_device);
		goto failed_device_create;
	}

	++dev->number_of_dma_devices;
	/* Enable the DMA controller */
	iowrite32_quick(BIT(0), cfg_dev->control_base + (DYPLO_DMA_TOLOGIC_CONTROL>>2));
	iowrite32_quick(BIT(0), cfg_dev->control_base + (DYPLO_DMA_FROMLOGIC_CONTROL>>2));
	cfg_dev->isr = dyplo_dma_isr;

	return 0;

failed_device_create:
error_cdev_add:
	dma_free_coherent(device, dma_dev->dma_from_logic_memory_size,
		dma_dev->dma_from_logic_memory, dma_dev->dma_from_logic_handle);
error_dma_from_logic_alloc:
	dma_free_coherent(device, dma_dev->dma_to_logic_memory_size,
		dma_dev->dma_to_logic_memory, dma_dev->dma_to_logic_handle);
error_dma_to_logic_alloc:
	unregister_chrdev_region(first_fifo_devt, dev->devt_last);
	dev->devt_last = first_fifo_devt;
error_register_chrdev_region:
	devm_kfree(device, dma_dev);
	return retval;
}

static void destroy_sub_devices_dma_fifo(
	struct dyplo_config_dev *cfg_dev)
{
	struct dyplo_dma_dev* dma_dev = cfg_dev->private_data;
	struct device *device = cfg_dev->parent->device;
	/* Free any transfers */
	dyplo_dma_to_logic_block_free(dma_dev);
	dyplo_dma_from_logic_block_free(dma_dev);
	/* Stop the DMA cores */
	iowrite32_quick(0, cfg_dev->control_base + (DYPLO_DMA_FROMLOGIC_CONTROL>>2));
	iowrite32_quick(0, cfg_dev->control_base + (DYPLO_DMA_TOLOGIC_CONTROL>>2));
	/* Release internal buffers */
	dma_free_coherent(device, dma_dev->dma_from_logic_memory_size,
		dma_dev->dma_from_logic_memory, dma_dev->dma_from_logic_handle);
	dma_free_coherent(device, dma_dev->dma_to_logic_memory_size,
		dma_dev->dma_to_logic_memory, dma_dev->dma_to_logic_handle);
	device_destroy(cfg_dev->parent->class, dma_dev->cdev_dma.dev);
}

static int create_sub_devices(struct dyplo_config_dev *cfg_dev)
{
	switch (dyplo_cfg_get_node_type(cfg_dev)) {
		case DYPLO_TYPE_ID_TOPIC_CPU:
			return create_sub_devices_cpu_fifo(cfg_dev);
		case DYPLO_TYPE_ID_TOPIC_DMA:
			return create_sub_devices_dma_fifo(cfg_dev);
		default:
			cfg_dev->isr = dyplo_generic_isr;
			return 0; /* No subdevice needed */
	}
}

static void destroy_sub_devices(struct dyplo_config_dev *cfg_dev)
{
	switch (dyplo_cfg_get_node_type(cfg_dev)) {
		case DYPLO_TYPE_ID_TOPIC_CPU:
			break; /* No particular destroy yet */
		case DYPLO_TYPE_ID_TOPIC_DMA:
			return destroy_sub_devices_dma_fifo(cfg_dev);
		default:
			break;
	}
}

static void dyplo_proc_show_cpu(struct seq_file *m, struct dyplo_config_dev *cfg_dev)
{
	__iomem int *control_base = cfg_dev->control_base;
	struct dyplo_fifo_control_dev *fifo_dev = cfg_dev->private_data;
	unsigned int irq_r_mask;
	unsigned int irq_r_status;
	unsigned int irq_w_mask;
	unsigned int irq_w_status;
	u8 number_of_fifo_devices;
	u8 i;

	if (!fifo_dev) {
		seq_printf(m, "  CPU node not registered\n");
		return;
	}

	irq_w_mask = dyplo_reg_read_quick(control_base, DYPLO_REG_FIFO_IRQ_MASK);
	irq_w_status = dyplo_reg_read_quick(control_base, DYPLO_REG_FIFO_IRQ_STATUS);
	irq_r_mask = irq_w_mask >> 16;
	irq_w_mask &= 0xFFFF;
	irq_r_status = irq_w_status >> 16;
	irq_w_status &= 0xFFFF;

	number_of_fifo_devices = fifo_dev->number_of_fifo_write_devices;
	if (fifo_dev->number_of_fifo_read_devices > number_of_fifo_devices)
		number_of_fifo_devices = fifo_dev->number_of_fifo_read_devices;
	for (i = 0; i < number_of_fifo_devices; ++i)
	{
		unsigned int mask = BIT(i);
		unsigned int tr_w;
		unsigned int tr_r;
		seq_printf(m, "  fifo=%2d ", i);
		if (i < fifo_dev->number_of_fifo_write_devices) {
			int lw = dyplo_fifo_write_level(&fifo_dev->fifo_devices[i]);
			int tw = dyplo_reg_read_quick_index(control_base, DYPLO_REG_FIFO_WRITE_THD_BASE, i);
			u32 us = dyplo_reg_read_quick_index(control_base, DYPLO_REG_FIFO_WRITE_USERSIGNAL_BASE, i);
			seq_printf(m, "%c=%3d %x (%3d%c%c) ",
				fifo_dev->fifo_devices[i].is_open ? 'W' : 'w',
				lw, us, tw,
				irq_w_mask & mask ? 'w' : '.',
				irq_w_status & mask ? 'i' : '.');
			tr_w = fifo_dev->fifo_devices[i].words_transfered;
		}
		else {
			seq_printf(m, "             ");
			tr_w = 0;
		}
		if (i < fifo_dev->number_of_fifo_read_devices) {
			u32 lr = dyplo_fifo_read_level(&fifo_dev->fifo_devices[fifo_dev->number_of_fifo_write_devices + i]);
			u32 tr = dyplo_reg_read_quick_index(control_base, DYPLO_REG_FIFO_READ_THD_BASE, i);
			seq_printf(m, "%c=%3d %x (%3u%c%c) ",
				fifo_dev->fifo_devices[fifo_dev->number_of_fifo_write_devices + i].is_open ? 'R' : 'r',
				lr & 0xFFFF, lr >> 16, tr,
				irq_r_mask & mask ? 'w' : '.',
				irq_r_status & mask ? 'i' : '.');
			tr_r = fifo_dev->fifo_devices[fifo_dev->number_of_fifo_write_devices + i].words_transfered;
		}
		else {
			seq_printf(m, "             ");
			tr_r = 0;
		}
		seq_printf(m, "total w=%d r=%d\n", tr_w, tr_r);
	}
	seq_printf(m, "  Counters: read=%u write=%u\n",
		dyplo_reg_read_quick(control_base, DYPLO_REG_FIFO_READ_COUNT),
		dyplo_reg_read_quick(control_base, DYPLO_REG_FIFO_WRITE_COUNT));
}

static void dyplo_proc_show_dma(struct seq_file *m, struct dyplo_config_dev *cfg_dev)
{
	/* __iomem int *control_base = cfg_dev->control_base; */
	struct dyplo_dma_dev *dma_dev = cfg_dev->private_data;
	u32 status;

	if (!dma_dev) {
		seq_printf(m, "  DMA node not registered\n");
		return;
	}

	status = dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_DMA_STANDALONE_CONTROL);
	if (status & BIT(0)) {
		seq_printf(m, "  STANDALONE: nb=%u ", (status >> 8) & 0xFF);
		status = dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_DMA_STANDALONE_BLOCKSIZE);
		seq_printf(m, "bs=%u\n", status);
	} else {
		seq_printf(m, "  CPU to PL (%c):",
			(dma_dev->open_mode & FMODE_WRITE) ? 'w' : '-');
		if (dma_dev->dma_to_logic_blocks.blocks)
			seq_printf(m, " cnt=%u sz=%u fl=%#x",
				dma_dev->dma_to_logic_blocks.count,
				dma_dev->dma_to_logic_blocks.size,
				dma_dev->dma_to_logic_blocks.flags);
		else
			seq_printf(m, " sz=%u hd=%u tl=%u",
				dma_dev->dma_to_logic_memory_size,
				dma_dev->dma_to_logic_head,
				dma_dev->dma_to_logic_tail);
		status = dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_DMA_TOLOGIC_STATUS);
		seq_printf(m, " re=%u fr=%u idle=%c\n",
			status >> 24, (status >> 16) & 0xFF, (status & 0x01) ? 'Y' : 'N');

		seq_printf(m, "  PL to CPU (%c):",
			(dma_dev->open_mode & FMODE_READ) ? 'r' : '-');
		if (dma_dev->dma_from_logic_blocks.blocks)
			seq_printf(m, " cnt=%u sz=%u fl=%#x",
				dma_dev->dma_from_logic_blocks.count,
				dma_dev->dma_from_logic_blocks.size,
				dma_dev->dma_from_logic_blocks.flags);
		else
			seq_printf(m, " sz=%u hd=%u tl=%u full=%c",
				dma_dev->dma_from_logic_memory_size,
				dma_dev->dma_from_logic_head,
				dma_dev->dma_from_logic_tail,
				dma_dev->dma_from_logic_full ? 'Y':'N');
		status = dyplo_reg_read_quick(cfg_dev->control_base, DYPLO_DMA_FROMLOGIC_STATUS);
		seq_printf(m, " re=%u fr=%u idle=%c\n",
			status >> 24, (status >> 16) & 0xFF, (status & 0x01) ? 'Y' : 'N');
	}
}


static int dyplo_proc_show(struct seq_file *m, void *offset)
{
	struct dyplo_dev *dev = m->private;
	unsigned int i;
	int ctl_index;

	if (dev == NULL) {
		seq_printf(m, "No dyplo device instance!\n");
		return 0;
	}
	seq_printf(m, "ncfg=%d, nfifo w=%u r=%u, ndma=%u\n",
		dev->number_of_config_devices,
		dev->count_fifo_write_devices, dev->count_fifo_read_devices,
		dev->number_of_dma_devices);

	seq_printf(m, "Route table:\n");
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		int queue_index;
		struct dyplo_config_dev *cfg_dev =
				&dev->config_devices[ctl_index];
		u32 __iomem *ctl_route_base = dev->base +
			(DYPLO_REG_CONTROL_ROUTE_TABLE>>2) +
			(ctl_index << DYPLO_STREAM_ID_WIDTH);
		const int number_of_fifos_out =
				dyplo_number_of_output_queues(cfg_dev);
		const int number_of_fifos_in =
				dyplo_number_of_input_queues(cfg_dev);
		u8 node_type = dyplo_cfg_get_node_type(cfg_dev);

		seq_printf(m, "ctl_index=%d (%c%c) type=%d id=%#x fifos in=%d out=%d\n",
				ctl_index,
				(cfg_dev->open_mode & FMODE_READ) ? 'r' : '-',
				(cfg_dev->open_mode & FMODE_WRITE) ? 'w' : '-',
				node_type,
				dyplo_cfg_get_version_id(cfg_dev),
				number_of_fifos_in, number_of_fifos_out);

		switch (node_type) {
			case DYPLO_TYPE_ID_TOPIC_CPU:
				seq_printf(m, " CPU FIFO node\n");
				dyplo_proc_show_cpu(m, cfg_dev);
				break;
			case DYPLO_TYPE_ID_TOPIC_DMA:
				seq_printf(m, " DMA transfer node\n");
				dyplo_proc_show_dma(m, cfg_dev);
				break;
		}

		for (queue_index = 0; queue_index < number_of_fifos_out; ++queue_index)
		{
			u32 route = ioread32_quick(ctl_route_base + queue_index);
			if (route)
			{
				int src_ctl_index = route >> DYPLO_STREAM_ID_WIDTH;
				if (src_ctl_index > 0)
				{
					int src_index = route & ( (0x1 << DYPLO_STREAM_ID_WIDTH) - 1);
					seq_printf(m, " route %d,%d -> %d,%d\n",
						ctl_index, queue_index, src_ctl_index-1, src_index);
				}
			}
		}
	}
	seq_printf(m, "Backplane counters:\n F2B:");
	for (i = 0; i < dev->number_of_config_devices; ++i)
		seq_printf(m, " %u", dyplo_reg_read_quick_index(
			dev->base, DYPLO_REG_BACKPLANE_COUNTER_F2B_BASE, i));
	seq_printf(m, "\n B2F:");
	for (i = 0; i < dev->number_of_config_devices; ++i)
		seq_printf(m, " %u", dyplo_reg_read_quick_index(
			dev->base, DYPLO_REG_BACKPLANE_COUNTER_B2F_BASE, i));
	seq_printf(m, "\n BPT:");
	for (i = 0; i < dev->number_of_config_devices; ++i)
		seq_printf(m, " %u", dyplo_reg_read_quick_index(
			dev->base, DYPLO_REG_BACKPLANE_COUNTER_BPT_BASE, i));
	seq_printf(m, "\nAXI overhead: r=%u w=%u\n",
		dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_AXI_READ),
		dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_AXI_WRITE));

	if (!((dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_LICENSE_INFO)) & 1))
		seq_printf(m, "WARNING: License expired, logic is locked.\n");

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

static int dyplo_core_check_version(struct device *device, struct dyplo_dev *dev)
{
	u32 dyplo_version =	dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_DYPLO_VERSION);

	dev_info(device, "Dyplo version %d.%d.%02x\n",
		dyplo_version >> 16, (dyplo_version >> 8) & 0xFF,
		dyplo_version & 0xFF);
	if ((dyplo_version >> 16) != 2015) {
		dev_err(device, "Unsupported version, only 2015 interface supported\n");
		return -EINVAL;
	}
	return 0;
}

static u32 dyplo_core_get_number_of_config_devices(struct dyplo_dev *dev)
{
	u32 count1 =
		dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_NODE_COUNT_1);
	u32 count2 =
		dyplo_reg_read_quick(dev->base, DYPLO_REG_CONTROL_NODE_COUNT_2);

	return
		((count1 >> 24) & 0xFF) +
		((count1 >> 16) & 0xFF) +
		((count1 >>  8) & 0xFF) +
		((count1      ) & 0xFF) +
		((count2 >> 24) & 0xFF) +
		((count2 >> 16) & 0xFF) +
		((count2 >>  8) & 0xFF) +
		((count2      ) & 0xFF);
}

int dyplo_core_probe(struct device *device, struct dyplo_dev *dev)
{
	dev_t devt;
	int retval;
	int device_index;
	struct proc_dir_entry *proc_file_entry;
	struct device *char_device;

	sema_init(&dev->fop_sem, 1);
	dev->device = device;

	retval = dyplo_core_check_version(device, dev);
	if (unlikely(retval))
		return retval;

	retval = dma_set_mask_and_coherent(device, DMA_BIT_MASK(32));
	if (unlikely(retval))
		dev_warn(device, "Failed to set DMA mask: %d", retval);

	dev->number_of_config_devices =
		dyplo_core_get_number_of_config_devices(dev);

	dev->config_devices = devm_kcalloc(device,
		dev->number_of_config_devices, sizeof(struct dyplo_config_dev),
		GFP_KERNEL);
	if (!dev->config_devices) {
		dev_err(device, "No memory for %d cfg devices\n", dev->number_of_config_devices);
		return -ENOMEM;
	}

	/* Create /dev/dyplo.. devices */
	retval = alloc_chrdev_region(&devt, 0, dev->number_of_config_devices + 1, DRIVER_CLASS_NAME);
	if (retval < 0)
		return retval;
	dev->devt = devt;
	dev->devt_last = devt + dev->number_of_config_devices + 1;

	cdev_init(&dev->cdev_control, &dyplo_ctl_fops);
	dev->cdev_control.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev_control, devt, 1);
	if (retval) {
		dev_err(device, "cdev_add(ctl) failed\n");
		goto failed_cdev;
	}

	cdev_init(&dev->cdev_config, &dyplo_cfg_fops);
	dev->cdev_config.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev_config, devt + 1, dev->number_of_config_devices);
	if (retval) {
		dev_err(device, "cdev_add(cfg) failed\n");
		goto failed_cdev;
	}

	dev->class = class_create(THIS_MODULE, DRIVER_CLASS_NAME);
	if (IS_ERR(dev->class)) {
		dev_err(device, "failed to create class\n");
		retval = PTR_ERR(dev->class);
		goto failed_class;
	}

	device_index = 0;

	char_device = device_create(dev->class, device, devt, dev,
				DRIVER_CONTROL_NAME);
	if (IS_ERR(char_device)) {
		dev_err(device, "unable to create device\n");
		retval = PTR_ERR(char_device);
		goto failed_device_create;
	}

	retval = devm_request_irq(device, dev->irq, dyplo_isr, 0,
		DRIVER_CLASS_NAME, dev);
	if (retval) {
		dev_err(device, "Cannot claim IRQ\n");
		goto failed_request_irq;
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

		char_device = device_create(dev->class, device,
			devt + 1 + device_index,
			cfg_dev, DRIVER_CONFIG_NAME, device_index);
		if (IS_ERR(char_device)) {
			dev_err(device, "unable to create config device %d\n",
				device_index);
			retval = PTR_ERR(device);
			goto failed_device_create_cfg;
		}
		retval = create_sub_devices(cfg_dev);
		if (retval) {
			dev_err(device, "unable to create sub-device %d: %d\n",
				device_index, retval);
			/* Should we abort? */
		}
		++device_index;
	}

	proc_file_entry = proc_create_data(DRIVER_CLASS_NAME, 0444, NULL, &dyplo_proc_fops, dev);
	if (proc_file_entry == NULL)
		dev_err(device, "unable to create proc entry\n");

	/* And finally, enable the backplane */
	dyplo_reg_write_quick(dev->base, DYPLO_REG_BACKPLANE_ENABLE_SET,
		(2 << dev->number_of_config_devices) - 1);

	return 0;

failed_device_create_cfg:
	while (device_index) {
		device_destroy(dev->class, dev->devt + 1 + device_index);
		--device_index;
	}
failed_request_irq:
failed_device_create:
	class_destroy(dev->class);
failed_class:
failed_cdev:
	unregister_chrdev_region(devt, dev->number_of_config_devices + 1);
	return retval;
}
EXPORT_SYMBOL(dyplo_core_probe);

int dyplo_core_remove(struct device *device, struct dyplo_dev *dev)
{
	int i;

	remove_proc_entry(DRIVER_CLASS_NAME, NULL);

	for (i = 0; i < dev->number_of_config_devices; ++i)
		destroy_sub_devices(&dev->config_devices[i]);

	for (i = dev->number_of_config_devices +
		dev->count_fifo_write_devices + dev->count_fifo_read_devices;
			i >= 0; --i)
		device_destroy(dev->class, dev->devt + i);
	class_destroy(dev->class);
	unregister_chrdev_region(dev->devt, dev->devt_last);

	return 0;
}
EXPORT_SYMBOL(dyplo_core_remove);
