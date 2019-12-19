/*
 * dyplo.h
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
 * paper mail at the following address:
 * Postbus 440, 5680 AK Best, The Netherlands.
 */

/* Size of the global configuration map for each node */
#define DYPLO_NODE_REG_SIZE	0x800
/* Memory range for a processing block is 64k */
#define DYPLO_CONFIG_SIZE	(64*1024)
/* Each FIFO occupies 256 words address range */
#define DYPLO_FIFO_MEMORY_SIZE (4*256)
/* Number of bits in a stream ID address */
#define DYPLO_STREAM_ID_WIDTH 2

/* memory map offsets */

#define DYPLO_REG_TYPE_ID	0x00

/* Type ID, bits 8..15 of DYPLO_REG_TYPE_ID */
#define DYPLO_TYPE_ID_TOPIC_CPU		2
#define DYPLO_TYPE_ID_TOPIC_IO		3
#define DYPLO_TYPE_ID_TOPIC_FIXED	4
#define DYPLO_TYPE_ID_TOPIC_PR		5
#define DYPLO_TYPE_ID_TOPIC_DMA		6
#define DYPLO_TYPE_ID_TOPIC_ICAP	7

#define DYPLO_REG_VERSION_ID	0x04

#define DYPLO_VERSION_ID_MASK_VENDOR	0xFF000000
#define DYPLO_VERSION_ID_MASK_PRODUCT	0x00FF0000
#define DYPLO_VERSION_ID_MASK_REVISION	0x0000FF00
#define DYPLO_VERSION_ID_MASK_VERSION	0x000000FF
#define DYPLO_VERSION_ID_MASK_VENDOR_PRODUCT \
	(DYPLO_VERSION_ID_MASK_VENDOR|DYPLO_VERSION_ID_MASK_PRODUCT)

#define DYPLO_REG_CONTROL_STATIC_ID	0x0C
#define DYPLO_REG_CONTROL_NODE_COUNT_1	0x14
#define DYPLO_REG_CONTROL_NODE_COUNT_2	0x18
#define DYPLO_REG_CONTROL_DMA_ADDR_WIDTH	0x1C
#define DYPLO_REG_CONTROL_DYPLO_VERSION	0x30
#define DYPLO_REG_CONTROL_LICENSE_INFO	0x34
#define DYPLO_REG_CONTROL_LICENSE_KEY0	0x38
#define DYPLO_REG_CONTROL_LICENSE_KEY1	0x3C
#define DYPLO_REG_CONTROL_DEVICE_ID0	0x40
#define DYPLO_REG_CONTROL_DEVICE_ID1	0x44
#define DYPLO_REG_BACKPLANE_ENABLE_STATUS	0x50
#define DYPLO_REG_BACKPLANE_ENABLE_SET	0x54
#define DYPLO_REG_BACKPLANE_ENABLE_CLR	0x58
#define DYPLO_REG_CONTROL_IRQ_MASK	0x60
#define DYPLO_REG_CONTROL_IRQ_REARM	0x64

#define DYPLO_REG_CONTROL_AXI_READ	0x70
#define DYPLO_REG_CONTROL_AXI_WRITE	0x74

/* Counters for performance measurements */
#define DYPLO_REG_BACKPLANE_COUNTER_B2F_BASE	0x404
#define DYPLO_REG_BACKPLANE_COUNTER_F2B_BASE	0x484
#define DYPLO_REG_BACKPLANE_COUNTER_BPT_BASE	0x504

/* Base address of the routing table. Starts at 0x600, but node '0'
 * does not take part in routing, hence the extra offset */
#define DYPLO_REG_CONTROL_ROUTE_TABLE	(0x600 + (4 << DYPLO_STREAM_ID_WIDTH))

/* Layout common to all nodes */

/* Currently only contains queue count information */
#define DYPLO_REG_NODE_INFO	0x14

#define DYPLO_REG_NODE_RESET_FIFOS	0x40

/* Specific layout of the CPU/PL communication node */

/* Interrupt handling */
#define DYPLO_REG_FIFO_IRQ_MASK	0x20
#define DYPLO_REG_FIFO_IRQ_STATUS	0x24
#define DYPLO_REG_FIFO_IRQ_SET	0x28
#define DYPLO_REG_FIFO_IRQ_CLR	0x2C
/* Extra user signal bits */
#define DYPLO_REG_FIFO_WRITE_USERSIGNAL_BASE	0x30
/* Blockram reset flags */
#define DYPLO_REG_FIFO_RESET_WRITE	0x40
#define DYPLO_REG_FIFO_RESET_READ	0x44
/* Traffic counters */
#define DYPLO_REG_FIFO_READ_COUNT	0x0050
#define DYPLO_REG_FIFO_WRITE_COUNT	0x0054
/* Actual fill level */
#define DYPLO_REG_FIFO_READ_LEVEL_BASE	0x400
/* Read level threshold */
#define DYPLO_REG_FIFO_READ_THD_BASE	0x420
/* Actual fill level */
#define DYPLO_REG_FIFO_WRITE_LEVEL_BASE	0x500
/* Write level threshold */
#define DYPLO_REG_FIFO_WRITE_THD_BASE	0x520

/* user signal values used by driver */
#define DYPLO_USERSIGNAL_ZERO	0
#define DYPLO_USERSIGNAL_BYTES1	1
#define DYPLO_USERSIGNAL_BYTES2	2
#define DYPLO_USERSIGNAL_BYTES3	3
#define DYPLO_USERSIGNAL_EOF	4

/* Queue sizes in words */
#define DYPLO_FIFO_WRITE_SIZE	255
#define DYPLO_FIFO_READ_SIZE	255

#define DYPLO_FIFO_WRITE_MAX_BURST_SIZE	DYPLO_FIFO_MEMORY_SIZE
#define DYPLO_FIFO_READ_MAX_BURST_SIZE DYPLO_FIFO_MEMORY_SIZE

/* DMA controller address space */
#define DYPLO_DMA_STANDALONE_CONTROL	0x30
#define DYPLO_DMA_STANDALONE_STARTADDR	0x34
#define DYPLO_DMA_STANDALONE_BLOCKSIZE	0x38

#define DYPLO_DMA_TOLOGIC_CONTROL	0x60
#define DYPLO_DMA_TOLOGIC_STATUS	0x64

#define DYPLO_DMA_TOLOGIC_STARTADDR_LOW	0x70
#define DYPLO_DMA_TOLOGIC_USERBITS	0x74
/* Writing BYTESIZE starts the transfer */
#define DYPLO_DMA_TOLOGIC_BYTESIZE	0x78
#define DYPLO_DMA_TOLOGIC_STARTADDR_HIGH	0x7C

/* Reading RESULT_ADDR removes the result from the queue */
#define DYPLO_DMA_TOLOGIC_RESULT_ADDR_LOW	0x80
#define DYPLO_DMA_TOLOGIC_RESULT_ADDR_HIGH	0x8C

#define DYPLO_DMA_STANDALONE_TOLOGIC_BASE	0x90

#define DYPLO_DMA_FROMLOGIC_CONTROL	0xB0
#define DYPLO_DMA_FROMLOGIC_STATUS	0xB4
#define DYPLO_DMA_FROMLOGIC_STARTADDR_LOW	0xC0
/* Writing BYTESIZE starts the transfer */
#define DYPLO_DMA_FROMLOGIC_BYTESIZE	0xC8
#define DYPLO_DMA_FROMLOGIC_STARTADDR_HIGH	0xCC

#define DYPLO_DMA_FROMLOGIC_RESULT_ADDR_LOW	0xD0
#define DYPLO_DMA_FROMLOGIC_RESULT_USERBITS	0xD4
/* Reading RESULT_BYTESIZE removes the result from the queue */
#define DYPLO_DMA_FROMLOGIC_RESULT_BYTESIZE	0xD8
#define DYPLO_DMA_FROMLOGIC_RESULT_ADDR_HIGH	0xDC

#define DYPLO_DMA_STANDALONE_FROMLOGIC_BASE	0xE0
