#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/wait.h>

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
};

int dyplo_core_remove(struct device *device, struct dyplo_dev *dev);

int dyplo_core_probe(struct device *device, struct dyplo_dev *dev);
