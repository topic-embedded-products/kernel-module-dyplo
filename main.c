// #define DEBUG
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
#include "dyplo.h"
/* Try to keep the following statement on line 21 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Topic Embedded Systems");

static const char DRIVER_CLASS_NAME[] = "dyplo";
static const char DRIVER_CONTROL_NAME[] = "dyploctl";
static const char DRIVER_CONFIG_NAME[] = "dyplocfg%d";
static const char DRIVER_FIFO_CLASS_NAME[] = "dyplo-fifo";
static const char DRIVER_FIFO_WRITE_NAME[] = "dyplow%d";
static const char DRIVER_FIFO_READ_NAME[] = "dyplor%d";

struct dyplo_dev; /* forward */

struct dyplo_config_dev
{
	struct dyplo_dev* parent;
	u32 __iomem *base;
	u32 __iomem *control_base;
	mode_t open_mode; /* Only FMODE_READ and FMODE_WRITE */
	/* more to come... */
};

struct dyplo_fifo_dev
{
	struct dyplo_config_dev* config_parent;
	wait_queue_head_t fifo_wait_queue; /* So the IRQ handler can notify waiting threads */
	int index;
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
	int number_of_fifo_devices;
	struct dyplo_fifo_dev *fifo_devices;
	/* Need to know wich device is the CPU-PL interface */
	struct dyplo_config_dev* fifo_config_device;
};

static unsigned int dyplo_get_config_mem_offset(struct dyplo_config_dev *cfg_dev)
{
	return ((char*)cfg_dev->base - (char*)cfg_dev->parent->base);
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
	loff_t *pos)
{
	return count;
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

	if (vma->vm_pgoff != 0)
	{
		 printk(KERN_WARNING "dyplo_ctl_mmap with non-zero offset\n");
		 return -EFAULT;
	}

	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, dev->mem->start >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static void dyplo_ctl_route_remove_src(struct dyplo_config_dev *cfg_dev, int queue)
{
	u32 route = *(cfg_dev->control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2) + queue);
	const int index_cfg_dst = ((route >> 5) & 0x1F) - 1;
	if (index_cfg_dst >= 0)
	{
		const int index_queue_dst = (route) & 0x1F;
		pr_debug("%s(%d) route=%x remove %d[%d]\n", __func__, queue, route, index_cfg_dst, index_queue_dst);
		*(cfg_dev->parent->config_devices[index_cfg_dst].control_base +
			(DYPLO_REG_FIFO_READ_SOURCE_BASE>>2) + index_queue_dst) = 0;
	}
}

static void dyplo_ctl_route_remove_dst(struct dyplo_config_dev *cfg_dev, int queue)
{
	u32 route = *(cfg_dev->control_base + (DYPLO_REG_FIFO_READ_SOURCE_BASE>>2) + queue);
	const int index_cfg_src = ((route >> 5) & 0x1F) - 1;
	if (index_cfg_src >= 0)
	{
		const int index_queue_src = (route) & 0x1F;
		pr_debug("%s(%d) remove %d[%d]\n", __func__, queue, index_cfg_src, index_queue_src);
		*(cfg_dev->parent->config_devices[index_cfg_src].control_base +
			(DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2) + index_queue_src) = 0;
	}
}

static int dyplo_ctl_route_add(struct dyplo_dev *dev, u32 route)
{
	const int index_cfg_src = (route >> 5) & 0x1F;
	const int index_queue_src = route & 0x1F;
	const int index_cfg_dst = (route >> 21) & 0x1F;
	const int index_queue_dst = (route >> 16) & 0x1F;
	pr_debug("%s(0x%x) %d,%d->%d,%d\n", __func__, route,
		index_cfg_src, index_queue_src, index_cfg_dst, index_queue_dst);
	if ((index_cfg_src >= dev->number_of_config_devices) ||
	    (index_cfg_dst >= dev->number_of_config_devices))
	    return -EINVAL;
	/* Remove existing routes using the same endpoints */
	dyplo_ctl_route_remove_src(&dev->config_devices[index_cfg_src], index_queue_src);
	dyplo_ctl_route_remove_dst(&dev->config_devices[index_cfg_dst], index_queue_dst);
	/* Setup route. The PL assumes that "0" is the control node, hence
	 * the "+1" in config node indices */
	int __iomem* dst_control_addr =
		dev->config_devices[index_cfg_src].control_base +
		(DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2) +
		index_queue_src;
	int __iomem* src_control_addr =
		dev->config_devices[index_cfg_dst].control_base +
		(DYPLO_REG_FIFO_READ_SOURCE_BASE>>2) +
		index_queue_dst;
	*dst_control_addr = ((index_cfg_dst+1) << 5) | index_queue_dst;
	pr_debug("%s (%d) @ %p: %x\n", __func__, index_cfg_src,
		dst_control_addr, ((index_cfg_dst+1) << 5) | index_queue_dst);
	*src_control_addr = ((index_cfg_src+1) << 5) | index_queue_src;
	pr_debug("%s (%d) @ %p: %x\n", __func__, index_cfg_dst,
		src_control_addr, ((index_cfg_src+1) << 5) | index_queue_src);
		
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
		unsigned int route;
		status = get_user(route, routes.proutes);
		if (status)
			break;
		status = dyplo_ctl_route_add(dev, route);
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
		int __iomem *ctl_route_base = dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		for (queue_index = 0; queue_index < 32; ++queue_index)
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
					route = ((src_ctl_index-1) << 21) | (src_index << 16) | (ctl_index << 5) | queue_index;
					pr_debug("%s: cfg=%d 0x%x @ %p\n", __func__, ctl_index, route, ctl_route_base + queue_index);
					status = put_user(route, routes.proutes + nr);
					if (status)
						return status;
					++nr;
				}
			}
		}
	}
	return status ? status : nr; /* Return number of items found */
}

static int dyplo_ctl_route_clear(struct dyplo_dev *dev)
{
	int ctl_index;
	int queue_index;
	for (ctl_index = 0; ctl_index < dev->number_of_config_devices; ++ctl_index)
	{
		int __iomem *ctl_route_base;
		/* Remove outgoing routes */
		ctl_route_base = dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_WRITE_SOURCE_BASE>>2);
		for (queue_index = 0; queue_index < 32; ++queue_index)
		{
			ctl_route_base[queue_index] = 0;
		}
		/* Remove incoming routes */
		ctl_route_base = dev->config_devices[ctl_index].control_base + (DYPLO_REG_FIFO_READ_SOURCE_BASE>>2);
		for (queue_index = 0; queue_index < 32; ++queue_index)
		{
			ctl_route_base[queue_index] = 0;
		}
	}
	return 0;
}

static long dyplo_ctl_ioctl_impl(struct dyplo_dev *dev, unsigned int cmd, unsigned long arg)
{
	int status;

	/* printk(KERN_DEBUG "%s(%x, %lx)\n", __func__, cmd, arg); */
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
			status = dyplo_ctl_route_add_from_user(dev, arg);
			break;
		case DYPLO_IOC_ROUTE_GET: /* Get routes. */
			status = dyplo_ctl_route_get_from_user(dev, arg);
			break;
		case DYPLO_IOC_ROUTE_TELL: /* Tell route: Adds a single route entry */
			status = dyplo_ctl_route_add(dev, arg);
			break;
		default:
			printk(KERN_WARNING "DYPLO ioctl unknown command: %d (arg=0x%x).\n", _IOC_NR(cmd), arg);
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

	if (remap_pfn_range(vma, vma->vm_start, physical, vsize, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static struct file_operations dyplo_cfg_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_cfg_read,
	.write = dyplo_cfg_write,
	.llseek = dyplo_cfg_llseek,
	.mmap = dyplo_cfg_mmap,
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
	int index = fifo_dev->index - 32; /* Read devices start at 32 */
	int __iomem *control_base =
		fifo_dev->config_parent->control_base;
	int result = *(control_base + (DYPLO_REG_FIFO_READ_LEVEL_BASE>>2) + index);
	pr_debug("%s index=%d result=%d @ %p\n", __func__, index, result,
		(control_base + (DYPLO_REG_FIFO_READ_LEVEL_BASE>>2) + index));
	return result;
}

static void dyplo_fifo_read_enable_interrupt(struct dyplo_fifo_dev *fifo_dev, int thd)
{
	int index = fifo_dev->index - 32; /* Read devices start at 32 */
	int __iomem *control_base =
		fifo_dev->config_parent->control_base;
	if (thd > DYPLO_FIFO_READ_SIZE/2)
		thd = DYPLO_FIFO_READ_SIZE/2;
	else if (thd)
		--thd; /* Treshold of "15" will alert when 16 words are present in the FIFO */
	*(control_base + (DYPLO_REG_FIFO_READ_THD_BASE>>2) + index) = thd;
	*(control_base + (DYPLO_REG_FIFO_READ_IRQ_SET>>2)) = BIT(index);
	pr_debug("%s index=%d thd=%d mask=%x\n", __func__, index, thd, 
		*(control_base + (DYPLO_REG_FIFO_READ_IRQ_MASK>>2)));
}

static int dyplo_fifo_read_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct dyplo_dev *dev = container_of(inode->i_cdev, struct dyplo_dev, cdev_fifo_read);
	int index = iminor(inode) - dev->number_of_config_devices - 1;
	struct dyplo_fifo_dev *fifo_dev = &dev->fifo_devices[index];

	if (filp->f_mode & FMODE_WRITE) /* read-only device */
		return -EINVAL;
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	if (fifo_dev->is_open) {
		result = -EBUSY;
		goto error;
	}
	fifo_dev->is_open = true;
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
			bytes = words_available << 2;
			if (bytes > DYPLO_FIFO_READ_MAX_BURST_SIZE)
				bytes = DYPLO_FIFO_READ_MAX_BURST_SIZE;
			if (count < bytes)
				bytes = count;
			pr_debug("%s copy_to_user %p (%d)\n", __func__, mapped_memory, bytes);
			if (unlikely(__copy_to_user(buf, mapped_memory, bytes))) {
				status = -EFAULT;
				goto error;
			}
			len += bytes;
			buf += bytes;
			count -= bytes;
			if (!count)
				break;
			words_available -= bytes >> 2;
		}
		while (words_available);
	}

	status = len;
	*f_pos += len;
error:
	return status;
}

static unsigned int dyplo_fifo_read_poll(struct file *filp, poll_table *wait)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	unsigned int mask = 0;

	//down(&dev->pps.fop_sem); /* Guard to ensure consistency with read() */
	if (dyplo_fifo_read_level(fifo_dev))
			mask |= (POLLIN | POLLRDNORM); /* Data available */
	else
			/* Set IRQ to occur ASAP. May be good to have an ioctl
			 * to increase this limit */
			dyplo_fifo_read_enable_interrupt(fifo_dev, 1);
	poll_wait(filp, &fifo_dev->fifo_wait_queue,  wait);
	//up(&dev->pps.fop_sem);

	return mask;
}

static struct file_operations dyplo_fifo_read_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_fifo_read_read,
	.llseek = no_llseek,
	.poll = dyplo_fifo_read_poll,
	.open = dyplo_fifo_read_open,
	.release = dyplo_fifo_read_release,
};

static int dyplo_fifo_write_level(struct dyplo_fifo_dev *fifo_dev)
{
	int index = fifo_dev->index;
	__iomem int *control_base =
		fifo_dev->config_parent->control_base;
	u32 result = *(control_base + (DYPLO_REG_FIFO_WRITE_LEVEL_BASE>>2) + index);
	pr_debug("%s index=%d value=0x%x (%d free)\n", __func__, index, result, result);
	return result;
}

static void dyplo_fifo_write_enable_interrupt(struct dyplo_fifo_dev *fifo_dev, int thd)
{
	int index = fifo_dev->index;
	__iomem int *control_base =
		fifo_dev->config_parent->control_base;
	if (thd > DYPLO_FIFO_WRITE_SIZE/2)
		thd = DYPLO_FIFO_WRITE_SIZE/2;
	else if (thd)
		--thd; /* IRQ will trigger when level is above thd */
	*(control_base + (DYPLO_REG_FIFO_WRITE_THD_BASE>>2) + index) = thd;
	*(control_base + (DYPLO_REG_FIFO_WRITE_IRQ_SET>>2)) = BIT(index);
	pr_debug("%s index=%d thd=%d mask=%x\n", __func__, index, thd, *(control_base + (DYPLO_REG_FIFO_WRITE_IRQ_MASK>>2)));
}

static int dyplo_fifo_write_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct dyplo_dev *dev = container_of(inode->i_cdev, struct dyplo_dev, cdev_fifo_write);
	int index = iminor(inode) - dev->number_of_config_devices - 1;
	struct dyplo_fifo_dev *fifo_dev = &dev->fifo_devices[index];
	
	if (filp->f_mode & FMODE_READ) /* write-only device */
		return -EINVAL;

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	if (fifo_dev->is_open) {
		result = -EBUSY;
		goto error;
	}
	fifo_dev->is_open = true;
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
		do
		{
			bytes = words_available << 2;
			if (bytes > DYPLO_FIFO_WRITE_MAX_BURST_SIZE)
				bytes = DYPLO_FIFO_WRITE_MAX_BURST_SIZE;
			if (count < bytes)
				bytes = count;
			pr_debug("%s copy_from_user %p (%d)\n", __func__, mapped_memory, bytes);
			if (unlikely(__copy_from_user(mapped_memory, buf, bytes))) {
				status = -EFAULT;
				goto error;
			}
			len += bytes;
			buf += bytes;
			count -= bytes;
			if (!count)
				break;
			words_available -= bytes >> 2;
		}
		while (words_available);
	}

	status = len;
	*f_pos += len;
error:
	return status;
}

static unsigned int dyplo_fifo_write_poll(struct file *filp, poll_table *wait)
{
	struct dyplo_fifo_dev *fifo_dev = filp->private_data;
	unsigned int mask = 0;

	//down(&dev->pps.fop_sem); /* Guard to ensure consistency with write() */
	if (dyplo_fifo_write_level(fifo_dev))
			mask |= (POLLOUT | POLLWRNORM);
	else
			/* Wait for buffer half-empty */
			dyplo_fifo_write_enable_interrupt(fifo_dev, DYPLO_FIFO_WRITE_SIZE / 2);
	poll_wait(filp, &fifo_dev->fifo_wait_queue,  wait);
	//up(&dev->pps.fop_sem);

	return mask;
}

static struct file_operations dyplo_fifo_write_fops =
{
	.write = dyplo_fifo_write_write,
	.poll = dyplo_fifo_write_poll,
	.llseek = no_llseek,
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

	u32 write_status_reg = *(cfg_dev->control_base + (DYPLO_REG_FIFO_WRITE_IRQ_STATUS>>2));
	u32 read_status_reg = *(cfg_dev->control_base + (DYPLO_REG_FIFO_READ_IRQ_STATUS>>2));
	/* Acknowledge the interrupt by clearing all flags that we've seen */
	if (write_status_reg)
		*(cfg_dev->control_base + (DYPLO_REG_FIFO_WRITE_IRQ_CLR>>2)) = write_status_reg;
	if (read_status_reg)
		*(cfg_dev->control_base + (DYPLO_REG_FIFO_READ_IRQ_CLR>>2)) = read_status_reg;
	pr_debug("%s(status=0x%x 0x%x)\n", __func__,
			write_status_reg, read_status_reg);
	/* Trigger the associated wait queues */
	for (mask=1, index=0; index < 32; ++index, mask <<= 1)
	{
		if (write_status_reg & mask)
			wake_up_interruptible(&dev->fifo_devices[index].fifo_wait_queue);
	}
	for (mask=1, index=32; index < dev->number_of_fifo_devices; ++index, mask <<= 1)
	{
		if (read_status_reg & mask)
			wake_up_interruptible(&dev->fifo_devices[index].fifo_wait_queue);
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
	u32 config_id;

	config_id = *(cfg_dev->control_base + (DYPLO_REG_ID>>2));
	if ((config_id & DYPLO_REG_ID_MASK_VENDOR_PRODUCT) != DYPLO_REG_ID_PRODUCT_TOPIC_CPU)
		return 0;

	if (dev->number_of_fifo_devices) {
		dev_err(&pdev->dev, "Fifo's already registered\n");
		return -EBUSY;
	}

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "IRQ resource missing\n");
		return -ENOENT;
	}
	
	dev->number_of_fifo_devices = DYPLO_CONFIG_SIZE / DYPLO_FIFO_MEMORY_SIZE;
	dev->fifo_devices = devm_kzalloc(&pdev->dev,
		dev->number_of_fifo_devices * sizeof(struct dyplo_fifo_dev),
		GFP_KERNEL);
	if (!dev->fifo_devices) {
		dev_err(&pdev->dev, "No memory for %d fifo devices\n",
			dev->number_of_fifo_devices);
		dev->number_of_fifo_devices = 0;
		return -ENOMEM;
	}
	number_of_write_fifos = dev->number_of_fifo_devices / 2;
	number_of_read_fifos = dev->number_of_fifo_devices - number_of_write_fifos;
	
	first_fifo_devt = dev->devt + dev->number_of_config_devices + 1;
	retval = register_chrdev_region(first_fifo_devt,
				dev->number_of_fifo_devices, DRIVER_FIFO_CLASS_NAME);
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
		struct dyplo_fifo_dev *fifo_dev = 
				&dev->fifo_devices[fifo_index];
		fifo_dev->config_parent = cfg_dev;
		fifo_dev->index = fifo_index;
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
		struct dyplo_fifo_dev *fifo_dev = 
				&dev->fifo_devices[fifo_index];
		fifo_dev->config_parent = cfg_dev;
		fifo_dev->index = fifo_index;
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
	unregister_chrdev_region(first_fifo_devt, dev->number_of_fifo_devices);
error_register_chrdev_region:
	dev->number_of_fifo_devices = 0;
	return retval;
}

static int dyplo_probe(struct platform_device *pdev)
{
	struct dyplo_dev *dev;
	struct device *device;
	dev_t devt;
	int retval;
	int device_index;
	u32 control_id;

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

	/* Each logic block takes 64k address space, so the number of
	 * devices is the address space divided by 64, minus one for the
	 * generic control (which is accomplished by "end" being inclusive).
	 * */
	dev->number_of_config_devices = 3; /* TODO */
		//(dev->mem->end - dev->mem->start) / DYPLO_CONFIG_SIZE;
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
	
	for (i = dev->number_of_config_devices + dev->number_of_fifo_devices;
			i >= 0; --i)
		device_destroy(dev->class, dev->devt + i);
	class_destroy(dev->class);
	unregister_chrdev_region(dev->devt, 1 + dev->number_of_config_devices + dev->number_of_fifo_devices);
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

/* modprobe -r dyplo ; opkg remove --force-depends kernel-module-dyplo && opkg update && opkg install kernel-module-dyplo && cat /usr/share/fpga.bin > /dev/xdevcfg && modprobe -v dyplo */
