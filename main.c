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
static const char DRIVER_DATA_NAME[] = "dyplo%d";

/* Expected to become something that is read from the logic */
#define NUMBER_OF_DATA_DEVICES	8

struct dyplo_dev; /* forward */

struct dyplo_data_dev
{
	struct dyplo_dev* parent;
	/* more to come... */
};

struct dyplo_dev
{
	struct cdev cdev_control;
	struct cdev cdev_data;
	dev_t devt;
	struct class *class;
	struct semaphore fop_sem;
	struct resource *mem;
	void __iomem *base;
	int irq;
	int number_of_data_devices;
	struct dyplo_data_dev *data_devices;
};

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

static int dyplo_data_open(struct inode *inode, struct file *filp)
{
	int status = 0;
	struct dyplo_dev *dev; /* device information */

	printk(KERN_DEBUG "%s index=%d i_cdev=%p\n",
		__func__, MINOR(inode->i_rdev), inode->i_cdev);

	dev = container_of(inode->i_cdev, struct dyplo_dev, cdev_data);
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	filp->private_data = dev; /* for other methods */
	up(&dev->fop_sem);
	return status;
}

static struct file_operations dyplo_data_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_ctl_read,
	.write = dyplo_ctl_write,
	.mmap = dyplo_ctl_mmap,
	.unlocked_ioctl = dyplo_ctl_ioctl,
	.open = dyplo_data_open,
	.release = dyplo_ctl_release,
};


/* Interrupt service routine */
static irqreturn_t dyplo_isr(int irq, void *dev_id)
{
	printk(KERN_DEBUG "%s()\n", __func__);
	return IRQ_HANDLED;
}


static int dyplo_probe(struct platform_device *pdev)
{
	struct dyplo_dev *dev;
	dev_t devt;
	struct device *device;
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
		
	dev->number_of_data_devices = NUMBER_OF_DATA_DEVICES;
	dev->data_devices = devm_kzalloc(&pdev->dev,
		dev->number_of_data_devices * sizeof(struct dyplo_data_dev),
		GFP_KERNEL);
	if (!dev->data_devices) {
		dev_err(&pdev->dev, "No memory for %d data devices\n", dev->number_of_data_devices);
		return -ENOMEM;
	}
	

	/* Create /dev/dyplo.. devices */
	retval = alloc_chrdev_region(&devt, 0, dev->number_of_data_devices + 1, DRIVER_CLASS_NAME);
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
		dev_err(&pdev->dev, "cdev_add(control) failed\n");
		goto failed_cdev;
	}

	cdev_init(&dev->cdev_data, &dyplo_data_fops);
	dev->cdev_data.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev_data, devt + 1, dev->number_of_data_devices);
	if (retval) {
		dev_err(&pdev->dev, "cdev_add(data) failed\n");
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

	while (device_index < dev->number_of_data_devices)
	{
		dev->data_devices[device_index].parent = dev;
		device = device_create(dev->class, &pdev->dev, devt + 1 + device_index, 
			&dev->data_devices[device_index], DRIVER_DATA_NAME, device_index);
		if (IS_ERR(device)) {
			dev_err(&pdev->dev, "unable to create data device %d\n", device_index);
			retval = PTR_ERR(device);
			goto failed_device_create_data;
		}
		++device_index;
	}

	printk(KERN_NOTICE "dyplo OK start=%x end=%x name=%s dev=%p data=%p\n",
		dev->mem->start, dev->mem->end, dev->mem->name,
		dev, dev->data_devices);

	return 0;
	
		
failed_device_create_data:
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
	unregister_chrdev_region(devt, dev->number_of_data_devices + 1);
	return retval;
}

static int dyplo_remove(struct platform_device *pdev)
{
	struct dyplo_dev *dev;
	int i;
	
	dev = dev_get_drvdata(&pdev->dev);
	if (!dev)
		return -ENODEV;
	
	for (i = dev->number_of_data_devices; i >= 0; --i)
		device_destroy(dev->class, dev->devt + i);
	class_destroy(dev->class);
	unregister_chrdev_region(dev->devt, dev->number_of_data_devices + 1);
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
