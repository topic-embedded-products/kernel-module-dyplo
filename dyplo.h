/* Memory range for a processing block is 64k */
#define CONFIG_SIZE	(64*1024)
/* Each FIFO occupies 256 words address range */
#define FIFO_MEMORY_SIZE (4*256)

/* memory map offsets */
#define DYPLO_REG_ID	0x00
#define DYPLO_REG_IRQ_STATUS	0x0004
#define DYPLO_REG_IRQ_MASK	0x0008
#define DYPLO_REG_IRQ_ENABLE	0x000C
#define DYPLO_REG_IRQ_STATUS	0x0004

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

/* Base address of the source registers */
#define DYPLO_REG_FIFO_READ_SOURCE_BASE	0x080
/* Read level threshold */
#define DYPLO_REG_FIFO_READ_THD_BASE	0x100
/* Actual fill level */
#define DYPLO_REG_FIFO_READ_LEVEL_BASE	0x180
/* Base address of the source registers */
#define DYPLO_REG_FIFO_WRITE_SOURCE_BASE	0x200
/* Read level threshold */
#define DYPLO_REG_FIFO_WRITE_THD_BASE	0x280
/* Actual fill level */
#define DYPLO_REG_FIFO_WRITE_LEVEL_BASE	0x300