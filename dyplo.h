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
 * paper mail at the following address: Postbus 440, 5680 AK Best, The Netherlands.
 */
/* Memory range for a processing block is 64k */
#define DYPLO_CONFIG_SIZE	(64*1024)
/* Each FIFO occupies 256 words address range */
#define DYPLO_FIFO_MEMORY_SIZE (4*256)

/* memory map offsets */
#define DYPLO_REG_ID	0x00

#define DYPLO_REG_ID_MASK_VENDOR	0xFF000000
#define DYPLO_REG_ID_MASK_PRODUCT	0x00FF0000
#define DYPLO_REG_ID_MASK_VENDOR_PRODUCT	(DYPLO_REG_ID_MASK_VENDOR|DYPLO_REG_ID_MASK_PRODUCT)

#define DYPLO_REG_ID_VENDOR_TOPIC	0x01000000
#define DYPLO_REG_ID_PRODUCT_TOPIC_CONTROL	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00010000)
#define DYPLO_REG_ID_PRODUCT_TOPIC_CPU	(DYPLO_REG_ID_VENDOR_TOPIC | 0x00020000)

#define DYPLO_REG_BACKPLANE_ENABLE_STATUS	0x10
#define DYPLO_REG_BACKPLANE_ENABLE_SET	0x14
#define DYPLO_REG_BACKPLANE_ENABLE_CLR	0x18

#define DYPLO_REG_NODE_INDEX	0x40
#define DYPLO_REG_CONTROL_CPU_NODES_COUNT	0x44
#define DYPLO_REG_CONTROL_IO_NODES_COUNT	0x48
#define DYPLO_REG_CONTROL_PR_NODES_COUNT	0x4C
#define DYPLO_REG_CONTROL_FIXED_NODES_COUNT	0x50
#define DYPLO_REG_CONTROL_CONNECT_LANES_COUNT	0x54

/* Size of the global configuration map for each node */
#define DYPLO_NODE_REG_SIZE	0x800
#define DYPLO_REG_NODE_ID	0x00

/* Counters for performance measurements */
#define DYPLO_REG_BACKPLANE_COUNTER_BASE	0x488
#define DYPLO_REG_AXI_COUNTER_BASE	0x1C
#define DYPLO_REG_CPU_COUNTER_BASE	(0x60+DYPLO_NODE_REG_SIZE)

/* Specific layout of the CPU/PL communication node */
#define DYPLO_REG_FIFO_WRITE_IRQ_MASK	0x20
#define DYPLO_REG_FIFO_WRITE_IRQ_STATUS	0x24
#define DYPLO_REG_FIFO_WRITE_IRQ_SET	0x28
#define DYPLO_REG_FIFO_WRITE_IRQ_CLR	0x2C
#define DYPLO_REG_FIFO_READ_IRQ_MASK	0x30
#define DYPLO_REG_FIFO_READ_IRQ_STATUS	0x34
#define DYPLO_REG_FIFO_READ_IRQ_SET	0x38
#define DYPLO_REG_FIFO_READ_IRQ_CLR	0x3C

#define DYPLO_REG_CPU_FIFO_WRITE_COUNT	0x44
#define DYPLO_REG_CPU_FIFO_READ_COUNT	0x48
#define DYPLO_REG_CPU_FIFO_WRITE_DEPTH	0x4C
#define DYPLO_REG_CPU_FIFO_READ_DEPTH	0x50

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

#define DYPLO_REG_FIFO_RESET_WRITE	0x54
#define DYPLO_REG_FIFO_RESET_READ	0x58

/* Queue sizes in words */
#define DYPLO_FIFO_WRITE_SIZE	255
#define DYPLO_FIFO_READ_SIZE	255

/* Hack: Write with burst doesn't work, limit to <32 bytes per call */
#define DYPLO_FIFO_WRITE_MAX_BURST_SIZE	DYPLO_FIFO_MEMORY_SIZE
/* Reading does not suffer from this problem it appears */
#define DYPLO_FIFO_READ_MAX_BURST_SIZE DYPLO_FIFO_MEMORY_SIZE

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


#define DYPLO_IOC_MAGIC	'd'
#define DYPLO_IOC_ROUTE_CLEAR	0x00
#define DYPLO_IOC_ROUTE_SET	0x01
#define DYPLO_IOC_ROUTE_GET	0x02
#define DYPLO_IOC_ROUTE_TELL	0x03
#define DYPLO_IOC_ROUTE_DELETE	0x04

#define DYPLO_IOC_BACKPLANE_STATUS	0x08
#define DYPLO_IOC_BACKPLANE_DISABLE	0x09
#define DYPLO_IOC_BACKPLANE_ENABLE	0x0A

#define DYPLO_IOC_RESET_FIFO_WRITE	0x0C
#define DYPLO_IOC_RESET_FIFO_READ	0x0D

#define DYPLO_IOC_TRESHOLD_QUERY	0x10
#define DYPLO_IOC_TRESHOLD_TELL	0x11


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
