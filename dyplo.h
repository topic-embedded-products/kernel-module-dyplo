/* Memory range for a processing block is 64k */
#define CONFIG_SIZE	(64*1024)
/* Each FIFO occupies 256 words address range */
#define FIFO_MEMORY_SIZE (4*256)

/* memory map offsets */
#define DYPLO_REG_ID	0x00

#define DYPLO_REG_ID_MASK_VENDOR	0xFF000000
#define DYPLO_REG_ID_MASK_PRODUCT	0x00FF0000
#define DYPLO_REG_ID_MASK_VENDOR_PRODUCT	(DYPLO_REG_ID_MASK_VENDOR|DYPLO_REG_ID_MASK_PRODUCT)

#define DYPLO_REG_ID_VENDOR_TOPIC	0x01000000
#define DYPLO_REG_ID_PRODUCT_TOPIC_CONTROL	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00010000)
#define DYPLO_REG_ID_PRODUCT_TOPIC_CPU	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00020000)

/* Size of the global configuration map for each node */
#define DYPLO_NODE_REG_SIZE	0x800
#define DYPLO_REG_NODE_ID	0x00

/* Specific layout of the CPU/PL communication node */
#define DYPLO_REG_FIFO_IRQ_MASK_WRITE	0x08
#define DYPLO_REG_FIFO_IRQ_MASK_READ	0x0C
#define DYPLO_REG_FIFO_READ_EMTPY	0x10
#define DYPLO_REG_FIFO_READ_FULL	0x14
#define DYPLO_REG_FIFO_READ_THD_REACHED	0x18
#define DYPLO_REG_FIFO_READ_ERROR	0x1C
#define DYPLO_REG_FIFO_WRITE_EMPTY	0x20
#define DYPLO_REG_FIFO_WRITE_FULL	0x24
#define DYPLO_REG_FIFO_WRITE_THD_REACHED	0x28
#define DYPLO_REG_FIFO_WRITE_ERROR	0x2C

#define DYPLO_REG_FIFO_WRITE_IRQ_MASK	0x40
#define DYPLO_REG_FIFO_WRITE_IRQ_STATUS	0x44
#define DYPLO_REG_FIFO_WRITE_IRQ_SET	0x48
#define DYPLO_REG_FIFO_WRITE_IRQ_CLR	0x4C
#define DYPLO_REG_FIFO_READ_IRQ_MASK	0x50
#define DYPLO_REG_FIFO_READ_IRQ_STATUS	0x54
#define DYPLO_REG_FIFO_READ_IRQ_SET	0x58
#define DYPLO_REG_FIFO_READ_IRQ_CLR	0x5C

/* Base address of the source registers */
#define DYPLO_REG_FIFO_READ_SOURCE_BASE	0x080
/* Read level threshold */
#define DYPLO_REG_FIFO_READ_THD_BASE	0x100
/* Actual fill level */
#define DYPLO_REG_FIFO_READ_LEVEL_BASE	0x180
/* Base address of the source registers */
#define DYPLO_REG_FIFO_WRITE_SOURCE_BASE	0x200
/* Write level threshold */
#define DYPLO_REG_FIFO_WRITE_THD_BASE	0x280
/* Actual fill level */
#define DYPLO_REG_FIFO_WRITE_LEVEL_BASE	0x300

/* Queue sizes in words */
#define DYPLO_FIFO_WRITE_SIZE	255
#define DYPLO_FIFO_READ_SIZE	1024

/* Hack: Write with burst doesn't work, limit to <32 bytes per call */
#define DYPLO_FIFO_WRITE_MAX_BURST_SIZE	28
/* Reading does not suffer from this problem it appears */
#define DYPLO_FIFO_READ_MAX_BURST_SIZE FIFO_MEMORY_SIZE

/* ioctl values for dyploctl device, set and get routing tables */
struct dyplo_route_t  {
	unsigned int n_routes;
	unsigned int* proutes;
};

#define DYPLO_IOC_MAGIC	'd'
#define DYPLO_IOC_ROUTE_SET	0x01
#define DYPLO_IOC_ROUTE_GET	0x02
#define DYPLO_IOC_ROUTE_TELL	0x03
/* S means "Set" through a ptr,
 * T means "Tell", sets directly
 * G means "Get" through a ptr
 * Q means "Query", return value */
#define DYPLO_IOCSROUTE   _IOW(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_SET, struct dyplo_route_t)
#define DYPLO_IOCGROUTE   _IOR(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_GET, struct dyplo_route_t)
#define DYPLO_IOCTROUTE   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_TELL)
