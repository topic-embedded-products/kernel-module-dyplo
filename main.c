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
#include <asm/uaccess.h>
#include <asm/io.h>



/* Try to keep the following statement on line 21 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Topic Embedded Systems");

static const char DRIVER_CLASS_NAME[] = "dyplo";
static const char DRIVER_CONTROL_NAME[] = "dyploctl";
static const char DRIVER_CONFIG_NAME[] = "dyplocfg%d";
static const char DRIVER_FIFO_CLASS_NAME[] = "dyplo-fifo";
static const char DRIVER_FIFO_WRITE_NAME[] = "dyplow%d";
static const char DRIVER_FIFO_READ_NAME[] = "dyplor%d";

/* Memory range for a processing block is 64k */
#define CONFIG_SIZE	(64*1024)
/* Each FIFO occupies 256 words address range */
#define FIFO_MEMORY_SIZE (4*256)

struct dyplo_dev; /* forward */

struct dyplo_config_dev
{
	struct dyplo_dev* parent;
	void __iomem *base;
	mode_t open_mode; /* Only FMODE_READ and FMODE_WRITE */
	/* more to come... */
};

struct dyplo_fifo_dev
{
	struct dyplo_config_dev* config_parent;
	int index;
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
	void __iomem *base;
	int irq;
	int number_of_config_devices;
	struct dyplo_config_dev *config_devices;
	int number_of_fifo_devices;
	struct dyplo_fifo_dev *fifo_devices;
};

static unsigned int dyplo_get_config_mem_offset(struct dyplo_config_dev *cfg_dev)
{
	return ((char*)cfg_dev->base - (char*)cfg_dev->parent->base);
}

static int dyplo_ctl_open(struct inode *inode, struct file *filp)
{
	int status = 0;
	struct dyplo_dev *dev; /* device information */

	printk(KERN_DEBUG "%s i_rdev=%x i_cdev=%p\n",
		__func__, inode->i_rdev, inode->i_cdev);

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
	int __iomem *mapped_memory = dev->base;
	int bytes_to_copy;

	// Make "cat /dev/dyplo" return quickly
	if (*f_pos)
		return 0;

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;

	if (count > 0x40) {
		bytes_to_copy = 0x40;
		count = 0x41;
	}
	else {
		bytes_to_copy = count & ~3; /* Must be 32-bit aligned */
	}
	if (copy_to_user(buf, mapped_memory, bytes_to_copy))
	{
		status = -EFAULT;
	}
	else
	{
		if (count > bytes_to_copy) {
			put_user((char)'\n', buf + bytes_to_copy);
		}
		status = count;
		*f_pos += status;
	}

	up(&dev->fop_sem);
	return status;
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

static long dyplo_ctl_ioctl_impl(struct dyplo_dev *dev, unsigned int cmd, unsigned long arg)
{
	printk(KERN_WARNING "%s(%x, %lx)\n", __func__, cmd, arg);
	return -ENOTTY;
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
	.mmap = dyplo_ctl_mmap,
	.unlocked_ioctl = dyplo_ctl_ioctl,
	.open = dyplo_ctl_open,
	.release = dyplo_ctl_release,
};

static struct file_operations dyplo_fifo_read_fops =
{
	.owner = THIS_MODULE,
};

static struct file_operations dyplo_fifo_write_fops =
{
	.owner = THIS_MODULE,
};

static int dyplo_cfg_open(struct inode *inode, struct file *filp)
{
	struct dyplo_dev *dev =  container_of(inode->i_cdev, struct dyplo_dev, cdev_config);
	int index = MINOR(inode->i_rdev);
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
	struct dyplo_dev *dev = cfg_dev->parent;
	int __iomem *mapped_memory = dev->base;
	size_t offset;

	/* EOF when past our area */
	if (*f_pos >= CONFIG_SIZE)
		return 0;
	
	offset = ((size_t)*f_pos) & ~0x03; /* Align to word size */
	count &= ~0x03;
	if ((offset + count) > CONFIG_SIZE)
		count = CONFIG_SIZE - offset;

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
	struct dyplo_dev *dev = cfg_dev->parent;
	int __iomem *mapped_memory = dev->base;
	size_t offset;

	/* EOF when past our area */
	if (*f_pos >= CONFIG_SIZE)
		return 0;
	
	offset = ((size_t)*f_pos) & ~0x03; /* Align to word size */
	count &= ~0x03;
	if ((offset + count) > CONFIG_SIZE)
		count = CONFIG_SIZE - offset;

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
        newpos = CONFIG_SIZE + off;
        break;

      default: /* can't happen */
        return -EINVAL;
    }
    if (newpos < 0) return -EINVAL;
    if (newpos > CONFIG_SIZE) return -EINVAL;
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
	if (vsize > (CONFIG_SIZE - off))
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


/* Interrupt service routine */
static irqreturn_t dyplo_isr(int irq, void *dev_id)
{
	printk(KERN_DEBUG "%s()\n", __func__);
	return IRQ_HANDLED;
}

static int create_sub_devices(struct platform_device *pdev, struct dyplo_config_dev *cfg_dev)
{
	int retval;
	struct dyplo_dev *dev = cfg_dev->parent;
	int index = dyplo_get_config_mem_offset(cfg_dev) / CONFIG_SIZE;
	dev_t first_fifo_devt;
	int fifo_index = 0;
	int number_of_write_fifos;
	int number_of_read_fifos;
	int i;
	struct device *device;

	if (index != 1) /* For now, assume device 1 is the fifo interface */
		return 0;

	if (dev->number_of_fifo_devices) {
		dev_err(&pdev->dev, "Fifo's already registered\n");
		return -EBUSY;
	}
	dev->number_of_fifo_devices = CONFIG_SIZE / FIFO_MEMORY_SIZE;
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

	return 0;

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

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, dev);
	sema_init(&dev->fop_sem, 1);

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "IRQ resource missing\n");
		return -ENOENT;
	}

	dev->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base = devm_request_and_ioremap(&pdev->dev, dev->mem);
	if (!dev->base) {
		dev_err(&pdev->dev, "Failed to map device memory\n");
		return -ENXIO;
	}

	/* Each logic block takes 64k address space, so the number of
	 * devices is the address space divided by 64, minus one for the
	 * generic control (which is accomplished by "end" being inclusive).
	 * */
	dev->number_of_config_devices =
		(dev->mem->end - dev->mem->start) / CONFIG_SIZE;
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

	retval = request_irq(dev->irq, dyplo_isr, 0, pdev->name, dev);
	if (retval) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto failed_request_irq;
	}

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
			((char*)dev->base + (CONFIG_SIZE * device_index));
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
failed_device_create:
	class_destroy(dev->class);
failed_class:
failed_cdev:
	free_irq(dev->irq, dev);
failed_request_irq:
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
	free_irq(dev->irq, dev);
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

/* modprobe -r dyplo ; opkg remove --force-depends kernel-module-dyplo && opkg update && opkg install kernel-module-dyplo && modprobe -v dyplo */
