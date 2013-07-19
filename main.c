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

static const char DRIVER_NAME[] = "dyplo";

struct dyplo_dev
{
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct semaphore fop_sem;
	struct resource *mem;
	void __iomem *base;
};

static int dyplo_open(struct inode *inode, struct file *filp)
{
	int status = 0;
	struct dyplo_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct dyplo_dev, cdev);
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	filp->private_data = dev; /* for other methods */
	up(&dev->fop_sem);
	return status;
}

static int dyplo_release(struct inode *inode, struct file *filp)
{
	//struct dyplo_dev *dev = filp->private_data;
	return 0;
}

static ssize_t dyplo_write (struct file *filp, const char __user *buf, size_t count,
	loff_t *pos)
{
	return count;
}

static ssize_t dyplo_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	int status;
	struct dyplo_dev *dev = filp->private_data;
	int __iomem *mapped_memory = dev->base;
	int i;
	int buffer[0x11];

	// Make "cat /dev/dyplo" return quickly
	if (*f_pos)
		return 0;

	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;

	if (count > 0x40)
	{
		count = 0x41;
		((char*)buffer)[0x40] = '\n';
	}
	for (i = 0; i < 0x10; ++i)
		buffer[i] = mapped_memory[i];
	if (copy_to_user(buf, buffer, count))
	{
		status = -EFAULT;
	}
	else
	{
		status = count;
		*f_pos += status;
	}

	up(&dev->fop_sem);
	return status;
}

static int dyplo_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dyplo_dev *dev = filp->private_data;

	if (vma->vm_pgoff != 0)
	{
		 printk(KERN_WARNING "dyplo_mmap with non-zero offset\n");
		 return -EFAULT;
	}

	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, dev->mem->start >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static long dyplo_ioctl_impl(struct dyplo_dev *dev, unsigned int cmd, unsigned long arg)
{
	printk(KERN_WARNING "%s(%x, %lx)\n", __func__, cmd, arg);
	return -ENOTTY;
}

static long dyplo_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dyplo_dev *dev = filp->private_data;
	if (unlikely(dev == NULL))
		return -ENODEV;
	return dyplo_ioctl_impl(dev, cmd, arg);
}

static struct file_operations dyplo_fops =
{
	.owner = THIS_MODULE,
	.read = dyplo_read,
	.write = dyplo_write,
	.mmap = dyplo_mmap,
	.unlocked_ioctl = dyplo_ioctl,
	.open = dyplo_open,
	.release = dyplo_release,
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
	int irq;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, dev);
	sema_init(&dev->fop_sem, 1);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "IRQ resource missing\n");
		return -ENOENT;
	}

	dev->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base = devm_request_and_ioremap(&pdev->dev, dev->mem);
	if (!dev->base)
		return -ENXIO;

	/* Create /dev/dyplo device */
	retval = alloc_chrdev_region(&devt, 0, 1, DRIVER_NAME);
	if (retval < 0)
		return retval;
	dev->devt = devt;

	retval = request_irq(irq, dyplo_isr, 0, pdev->name, dev);
	if (retval) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto failed_request_irq;
	}

	cdev_init(&dev->cdev, &dyplo_fops);
	dev->cdev.owner = THIS_MODULE;
	retval = cdev_add(&dev->cdev, devt, 1);
	if (retval) {
		dev_err(&pdev->dev, "cdev_add() failed\n");
		goto failed_cdev;
	}

	dev->class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(dev->class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		retval = PTR_ERR(dev->class);
		goto failed_class;
	}

	device = device_create(dev->class, &pdev->dev, devt, dev,
				DRIVER_NAME);
	if (IS_ERR(device)) {
			dev_err(&pdev->dev, "unable to create device\n");
			retval = PTR_ERR(device);
			goto failed_device_create;
	}

	printk(KERN_NOTICE "dyplo OK start=%x end=%x name=%s\n",
		dev->mem->start, dev->mem->end, dev->mem->name);

	return 0;

failed_not_used_yet:
	device_destroy(dev->class, dev->devt);
failed_device_create:
	class_destroy(dev->class);
failed_class:
failed_cdev:
failed_request_irq:
	unregister_chrdev_region(devt, 1);
	return retval;
}

static int dyplo_remove(struct platform_device *pdev)
{
	struct dyplo_dev *dev;
	printk(KERN_WARNING "%s\n", __func__);
	
	dev = dev_get_drvdata(&pdev->dev);
	if (!dev)
		return -ENODEV;
	
	unregister_chrdev_region(dev->devt, 1);
	device_destroy(dev->class, dev->devt);
	class_destroy(dev->class);
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
