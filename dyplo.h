/*
 * dyplo.h
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
 * paper mail at the following address:
 * Postbus 440, 5680 AK Best, The Netherlands.
 */

/* Memory range for a processing block is 64k */
#define DYPLO_CONFIG_SIZE	(64*1024)
/* Each FIFO occupies 256 words address range */
#define DYPLO_FIFO_MEMORY_SIZE (4*256)

/* memory map offsets */
#define DYPLO_REG_ID	0x00

#define DYPLO_REG_ID_MASK_VENDOR	0xFF000000
#define DYPLO_REG_ID_MASK_PRODUCT	0x00FF0000
#define DYPLO_REG_ID_MASK_REVISION	0x0000FF00
#define DYPLO_REG_ID_MASK_VERSION	0x000000FF
#define DYPLO_REG_ID_MASK_VENDOR_PRODUCT	(DYPLO_REG_ID_MASK_VENDOR|DYPLO_REG_ID_MASK_PRODUCT)

#define DYPLO_REG_ID_VENDOR_TOPIC	0x01000000
#define DYPLO_REG_ID_PRODUCT_TOPIC_CONTROL	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00010000)
#define DYPLO_REG_ID_PRODUCT_TOPIC_CPU	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00020000)
#define DYPLO_REG_ID_PRODUCT_TOPIC_DMA	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00060000)
/* #define DYPLO_REG_ID_PRODUCT_TOPIC_DMA	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00030000) * Testing */

#define DYPLO_REG_BACKPLANE_ENABLE_STATUS	0x10
#define DYPLO_REG_BACKPLANE_ENABLE_SET	0x14
#define DYPLO_REG_BACKPLANE_ENABLE_CLR	0x18

#define DYPLO_REG_CONTROL_IRQ_MASK	0x04
#define DYPLO_REG_CONTROL_DYPLO_VERSION	0x30

#define DYPLO_REG_NODE_INDEX	0x40
#define DYPLO_REG_CONTROL_CPU_NODES_COUNT	0x44
#define DYPLO_REG_CONTROL_IO_NODES_COUNT	0x48
#define DYPLO_REG_CONTROL_PR_NODES_COUNT	0x4C
#define DYPLO_REG_CONTROL_FIXED_NODES_COUNT	0x50
#define DYPLO_REG_CONTROL_CONNECT_LANES_COUNT	0x54
#define DYPLO_REG_CONTROL_LICENSE_VALID	0x68


/* Size of the global configuration map for each node */
#define DYPLO_NODE_REG_SIZE	0x800
#define DYPLO_REG_NODE_ID	0x00

/* Counters for performance measurements */
#define DYPLO_REG_BACKPLANE_COUNTER_BASE	0x488
#define DYPLO_REG_AXI_COUNTER_BASE	0x1C
#define DYPLO_REG_CPU_COUNTER_BASE	(0x60+DYPLO_NODE_REG_SIZE)

/* Layout common to all nodes */

/* Queue information */
#define DYPLO_REG_FIFO_FROM_BACKPLANE_COUNT	0x44
#define DYPLO_REG_FIFO_TO_BACKPLANE_COUNT	0x48
#define DYPLO_REG_FIFO_FROM_BACKPLANE_DEPTH	0x4C
#define DYPLO_REG_FIFO_TO_BACKPLANE_DEPTH	0x50

/* Specific layout of the CPU/PL communication node */

/* Layout of V2 interrupt status registers */
#define DYPLO_REG_FIFO_IRQ_MASK	0x20
#define DYPLO_REG_FIFO_IRQ_STATUS	0x24
#define DYPLO_REG_FIFO_IRQ_SET	0x28
#define DYPLO_REG_FIFO_IRQ_CLR	0x2C
/* Layout of V1 interrupt status registers  */
#define DYPLO_REG_FIFO_WRITE_IRQ_MASK	0x20
#define DYPLO_REG_FIFO_WRITE_IRQ_STATUS	0x24
#define DYPLO_REG_FIFO_WRITE_IRQ_SET	0x28
#define DYPLO_REG_FIFO_WRITE_IRQ_CLR	0x2C
#define DYPLO_REG_FIFO_READ_IRQ_MASK	0x30
#define DYPLO_REG_FIFO_READ_IRQ_STATUS	0x34
#define DYPLO_REG_FIFO_READ_IRQ_SET	0x38
#define DYPLO_REG_FIFO_READ_IRQ_CLR	0x3C

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
/* Extra user signal bits */
#define DYPLO_REG_FIFO_WRITE_USERSIGNAL_BASE	0x400

/* user signal values used by driver */
#define DYPLO_USERSIGNAL_ZERO	0
#define DYPLO_USERSIGNAL_BYTES1	1
#define DYPLO_USERSIGNAL_BYTES2	2
#define DYPLO_USERSIGNAL_BYTES3	3
#define DYPLO_USERSIGNAL_EOF	4

#define DYPLO_REG_FIFO_RESET_WRITE	0x54
#define DYPLO_REG_FIFO_RESET_READ	0x58

/* Queue sizes in words */
#define DYPLO_FIFO_WRITE_SIZE	255
#define DYPLO_FIFO_READ_SIZE	255

#define DYPLO_FIFO_WRITE_MAX_BURST_SIZE	DYPLO_FIFO_MEMORY_SIZE
#define DYPLO_FIFO_READ_MAX_BURST_SIZE DYPLO_FIFO_MEMORY_SIZE

/* DMA controller address space */
#define DYPLO_DMA_STANDALONE_CONTROL	0x30
#define DYPLO_DMA_STANDALONE_STARTADDR	0x34
#define DYPLO_DMA_STANDALONE_BLOCKSIZE	0x38
#define DYPLO_DMA_STANDALONE_TOLOGIC_BASE	0x70
#define DYPLO_DMA_STANDALONE_FROMLOGIC_BASE	0xA0

#define DYPLO_DMA_TOLOGIC_CONTROL	0x60
#define DYPLO_DMA_TOLOGIC_STATUS	0x64
#define DYPLO_DMA_TOLOGIC_STARTADDR	0x70
#define DYPLO_DMA_TOLOGIC_USERBITS	0x74
/* Writing BYTESIZE starts the transfer */
#define DYPLO_DMA_TOLOGIC_BYTESIZE	0x78
/* Reading RESULT_ADDR removes the result from the queue */
#define DYPLO_DMA_TOLOGIC_RESULT_ADDR	0x80

#define DYPLO_DMA_FROMLOGIC_CONTROL	0x90
#define DYPLO_DMA_FROMLOGIC_STATUS	0x94
#define DYPLO_DMA_FROMLOGIC_STARTADDR	0xA0
/* Writing BYTESIZE starts the transfer */
#define DYPLO_DMA_FROMLOGIC_BYTESIZE	0xA8
#define DYPLO_DMA_FROMLOGIC_RESULT_ADDR	0xB0
#define DYPLO_DMA_FROMLOGIC_RESULT_USERBITS	0xB4
/* Reading RESULT_BYTESIZE removes the result from the queue */
#define DYPLO_DMA_FROMLOGIC_RESULT_BYTESIZE	0xB8

/* ioctl values for dyploctl device, set and get routing tables */
struct dyplo_route_item_t {
	unsigned char dstFifo; /* LSB */
	unsigned char dstNode;
	unsigned char srcFifo;
	unsigned char srcNode; /* MSB */
};

struct dyplo_route_t  {
	unsigned int n_routes;
	struct dyplo_route_item_t* proutes;
};

struct dyplo_buffer_block_alloc_req {
	__u32 size;	/* Size of each buffer (will be page aligned) */
	__u32 count;	/* Number of buffers */
};

struct dyplo_buffer_block {
	__u32 id;	/* 0-based index of the buffer */
	__u32 offset;	/* Location of data in memory map */
	__u32 size;	/* Size of buffer */
	__u32 bytes_used; /* How much actually is in use */
	__u16 user_signal; /* User signals (framing) either way */
	__u16 state; /* Who's owner of the buffer */
};

struct dyplo_dma_standalone_config {
	__u32 offset;
	__u32 burst_size;
	__u32 incr_a;
	__u32 iterations_a;
	__u32 incr_b;
	__u32 iterations_b;
	__u32 incr_c;
	__u32 iterations_c;
};

/* DMA not used for CPU-logic transfers at all, only for logic
 * storage. Buffer can be mmap'ed for inspection. */
#define DYPLO_DMA_MODE_STANDALONE	0
/* (default) Copies data from userspace into a kernel buffer and
 * vice versa. */
#define DYPLO_DMA_MODE_RINGBUFFER_BOUNCE	1
/* Blockwise data transfers, using coherent memory. This will result in
 * slow non-cached memory being used when hardware coherency is not
 * available, but it is the fastest mode. */
#define DYPLO_DMA_MODE_BLOCK_COHERENT	2
/* Blockwise data transfers, using  streaming DMA into cachable memory.
 * Managing the cache may cost more than actually copying the data. */
#define DYPLO_DMA_MODE_BLOCK_STREAMING	3

struct dyplo_dma_configuration_req {
	__u32 mode;	/* One of DYPLO_DMA_MODE.. */
	__u32 size;	/* Size of each buffer (will be page aligned) */
	__u32 count;	/* Number of buffers */
};

#define DYPLO_IOC_MAGIC	'd'
#define DYPLO_IOC_ROUTE_CLEAR	0x00
#define DYPLO_IOC_ROUTE_SET	0x01
#define DYPLO_IOC_ROUTE_GET	0x02
#define DYPLO_IOC_ROUTE_TELL	0x03
#define DYPLO_IOC_ROUTE_DELETE	0x04
#define DYPLO_IOC_ROUTE_TELL_TO_LOGIC	0x05
#define DYPLO_IOC_ROUTE_TELL_FROM_LOGIC	0x06
#define DYPLO_IOC_ROUTE_QUERY_ID	0x07

#define DYPLO_IOC_BACKPLANE_STATUS	0x08
#define DYPLO_IOC_BACKPLANE_DISABLE	0x09
#define DYPLO_IOC_BACKPLANE_ENABLE	0x0A

#define DYPLO_IOC_RESET_FIFO_WRITE	0x0C
#define DYPLO_IOC_RESET_FIFO_READ	0x0D

#define DYPLO_IOC_TRESHOLD_QUERY	0x10
#define DYPLO_IOC_TRESHOLD_TELL	0x11

#define DYPLO_IOC_USERSIGNAL_QUERY	0x12
#define DYPLO_IOC_USERSIGNAL_TELL	0x13

#define DYPLO_IOC_DMA_RECONFIGURE	0x1F
#define DYPLO_IOC_DMABLOCK_ALLOC	0x20
#define DYPLO_IOC_DMABLOCK_FREE 	0x21
#define DYPLO_IOC_DMABLOCK_QUERY	0x22
#define DYPLO_IOC_DMABLOCK_ENQUEUE	0x23
#define DYPLO_IOC_DMABLOCK_DEQUEUE	0x24
#define DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC	0x28
#define DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC	0x29
#define DYPLO_IOC_DMASTANDALONE_START_TO_LOGIC	0x2A
#define DYPLO_IOC_DMASTANDALONE_START_FROM_LOGIC	0x2B
#define DYPLO_IOC_DMASTANDALONE_STOP_TO_LOGIC	0x2C
#define DYPLO_IOC_DMASTANDALONE_STOP_FROM_LOGIC	0x2D

/* S means "Set" through a ptr,
 * T means "Tell", sets directly
 * G means "Get" through a ptr
 * Q means "Query", return value */

/* Delete all existing routes */
#define DYPLO_IOCROUTE_CLEAR	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_CLEAR)
/* Define a set of routes, to be added to the currently active set */
#define DYPLO_IOCSROUTE   _IOW(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_SET, struct dyplo_route_t)
/* Get the currently active routes. Returns number of entries. */
#define DYPLO_IOCGROUTE   _IOR(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_GET, struct dyplo_route_t)
/* Add a single route. Argument is a dyplo_route_item_t cast to integer */
#define DYPLO_IOCTROUTE   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_TELL)
/* Remove routes to a node. Argument is a integer node number. */
#define DYPLO_IOCTROUTE_DELETE   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_DELETE)

/* Add a route from "this" dma or cpu node to another node. The argument
 * is an integer of destination node | fifo << 8 */
#define DYPLO_IOCTROUTE_TELL_TO_LOGIC	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_TELL_TO_LOGIC)
/* Add a route from another node into "this" dma or cpu node. Argument
 * is an integer of source node | fifo << 8 */
#define DYPLO_IOCTROUTE_TELL_FROM_LOGIC	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_TELL_FROM_LOGIC)
/* Get the node number and fifo (if applicable) for this cpu or dma
 * node. Returns an integer of node | fifo << 8 */
#define DYPLO_IOCQROUTE_QUERY_ID	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_ROUTE_QUERY_ID)

/* Get backplane status. When called on control node, returns a bit mask where 0=CPU and
 * 1=first HDL node and so on. When called on config node, returns the status for only
 * that node, 0=disabled, non-zero is enabled */
#define DYPLO_IOCQBACKPLANE_STATUS   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_BACKPLANE_STATUS)
/* Enable or disable backplane status. Disable is required when the logic is active and
 * you want to replace a node using partial configuration. Operations are atomic. */
#define DYPLO_IOCTBACKPLANE_ENABLE   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_BACKPLANE_ENABLE)
#define DYPLO_IOCTBACKPLANE_DISABLE  _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_BACKPLANE_DISABLE)
/* Set the thresholds for "writeable" or "readable" on a CPU node fifo. Allows
 * tuning for low latency or reduced interrupt rate. */
#define DYPLO_IOCQTRESHOLD   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_TRESHOLD_QUERY)
#define DYPLO_IOCTTRESHOLD   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_TRESHOLD_TELL)
/* Reset FIFO data (i.e. throw it away). Can be applied to config
 * nodes to reset its incoming fifos (argument is bitmask for queues to
 * reset), or to a CPU read/write fifo (argument ignored). */
#define DYPLO_IOCRESET_FIFO_WRITE	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_RESET_FIFO_WRITE)
#define DYPLO_IOCRESET_FIFO_READ	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_RESET_FIFO_READ)
/* Set or get user signal bits. These are the upper 4 bits of Dyplo data
 * that aren't part of the actual data, but control the flow. */
#define DYPLO_IOCQUSERSIGNAL   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_USERSIGNAL_QUERY)
#define DYPLO_IOCTUSERSIGNAL   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_USERSIGNAL_TELL)

/* DMA configuration */
#define DYPLO_IOCDMA_RECONFIGURE _IOWR(DYPLO_IOC_MAGIC, DYPLO_IOC_DMA_RECONFIGURE, struct dyplo_dma_configuration_req)

/* Dyplo's IIO-alike DMA block interface */
#define DYPLO_IOCDMABLOCK_ALLOC	_IOWR(DYPLO_IOC_MAGIC, DYPLO_IOC_DMABLOCK_ALLOC, struct dyplo_buffer_block_alloc_req)
#define DYPLO_IOCDMABLOCK_FREE 	_IO(DYPLO_IOC_MAGIC, DYPLO_IOC_DMABLOCK_FREE)
#define DYPLO_IOCDMABLOCK_QUERY	_IOWR(DYPLO_IOC_MAGIC, DYPLO_IOC_DMABLOCK_QUERY, struct dyplo_buffer_block)
#define DYPLO_IOCDMABLOCK_ENQUEUE	_IOWR(DYPLO_IOC_MAGIC, DYPLO_IOC_DMABLOCK_ENQUEUE, struct dyplo_buffer_block)
#define DYPLO_IOCDMABLOCK_DEQUEUE	_IOWR(DYPLO_IOC_MAGIC, DYPLO_IOC_DMABLOCK_DEQUEUE, struct dyplo_buffer_block)

/* Standalone DMA configuration and control */
#define DYPLO_IOCSDMASTANDALONE_CONFIGURE_TO_LOGIC	_IOW(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC, struct dyplo_dma_standalone_config)
#define DYPLO_IOCGDMASTANDALONE_CONFIGURE_TO_LOGIC	_IOR(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_CONFIGURE_TO_LOGIC, struct dyplo_dma_standalone_config)
#define DYPLO_IOCSDMASTANDALONE_CONFIGURE_FROM_LOGIC	_IOW(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC, struct dyplo_dma_standalone_config)
#define DYPLO_IOCGDMASTANDALONE_CONFIGURE_FROM_LOGIC	_IOR(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_CONFIGURE_FROM_LOGIC, struct dyplo_dma_standalone_config)
#define DYPLO_IOCDMASTANDALONE_START_TO_LOGIC   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_START_TO_LOGIC)
#define DYPLO_IOCDMASTANDALONE_START_FROM_LOGIC   _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_START_FROM_LOGIC)
#define DYPLO_IOCDMASTANDALONE_STOP_TO_LOGIC  _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_STOP_TO_LOGIC)
#define DYPLO_IOCDMASTANDALONE_STOP_FROM_LOGIC  _IO(DYPLO_IOC_MAGIC, DYPLO_IOC_DMASTANDALONE_STOP_FROM_LOGIC)
