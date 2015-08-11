Dyplo Questions:
 What choices are there for data transfers?
 Which option is best for what purpose?
 How best to connect the logic to the CPU?


Dyplo's CPU to Logic interface supports many options, most of which can
be set at runtime, a few must be implemented in hardware.


Dyplo offers two node types for CPU-Logic data transfers: DMA nodes and
CPU nodes. Data can be transferred from Logic to CPU and from CPU to
Logic.

CPU nodes:
 Low latency (IRQ driven)
 Low bandwidth (max 50MB/s total)
CPU nodes transfer data to logic by directly writing to the AXI bus.
FIFO buffers in hardware offer limited buffer space (about 1kB). They
read data from logic by directly accessing the AXI bus as well.
Transferring a single word only takes about 2 AXI transactions (one
to check buffer availability, one to actually transfer the data),
multiple words reduce the average overhead even further. Multiple words
will be transfered as AXI bursts. Logic can signal buffer states through
the interrupt handler, avoiding waiting or timing loops. Userspace can
set the buffer levels at which these interrupts occur, which is
especially useful when combined with non-blocking IO.


DMA nodes
 Higher latency (Must wait for buffer full or usersignal change)
 High bandwidth (Limited only by hardware, 600MB/s)
DMA nodes let the logic transfer data to and from CPU memory directly.
The driver sets up the memory mappings and coordinates the transfers. To
transfer a single frame, about 10 AXI transactions are required to set
up the DMA and start the transfer. This overhead is constant, regardless
of the size of the frame, so when transferring large blocks of data, the
average overhead is neglegible.
DMA nodes can run in one of four modes:
"Standalone": In this mode, no data is transferred to the CPU, the DMA
  node uses CPU memory for data storage. This allows nodes to implement
  for example an image rotation algorithm using the DMA node as
  intermediate data storage for a whole frame.
"Ringbuffer": Copies data from userspace into a ring buffer, and
  initiates DMA transfers to logic. Data from logic is placed into
  another ring buffer where the CPU can copy it to userspace. This
  allows the node to be used as a simple file or fifo.
"Block coherent": Allocates multiple buffers, and makes sure data in
  these buffers remains coherent. Userspace maps these buffers directly,
  and uses ioctl calls to allocate, enqueue, dequeue and free these
  buffers. This allows zero-copy data transfers between CPU and logic.
  On the Zynq, coherent memory will be uncacheable when the DMA node
  is connected to a HP port. This may result in very slow access
  depending on the access pattern of the application.
"Block streaming": Like coherent mode, but instead of using cache
  coherent memory, always allocates cachable memory. This allows fast
  CPU access to the buffers. On the Zynq, the CPU will have to flush
  or invalidate caches before and after logic needs to access it when
  the DMA node is connected to the HP port. This reduces the effective
  transfer speed to almost the same speed as the ringbuffer mode.


CPU or DMA?

At runtime, one can choose between CPU or DMA node for data transfers.

The CPU node is more efficient at handling small transfers, and will do
a better job handling low frequency sensor data, like 16 channels of
24-bit data at 1kHz. In particular, the CPU node allows to read in a
"as much data as is available now" mode, offering low latency display
of live data, automatically compensating in case of high system load by
transferring larger chunks. It is still fast enough to handle multi
channel audio data for example. The CPU node is also very well suited
for handling "events" from logic, like "liquid level exceeds safety
treshold" because of its low latency and simple interface.

The DMA node handles high data rates of (mostly) constant block sizes
most efficiently. It is more than capable of transferring HD video data
to and from logic, even in its simplest ringbuffer configuration which
can be used as a drop-in replacement for the CPU node. At design time,
one has to chose an appropriate port (ACP or HP for the Zynq) and at
runtime, the application can pick the transfer modes best suited for its
purposes.


ACP or HP?

This is one of the few choices that are built into the design and can
only be altered by creating a new bitstream.

The DMA nodes can be connected to the HP or ACP port of the Zynq. The
only difference for the driver is that you have to specify
"dma-coherent;" in the devicetree configuration for Dyplo, to inform
the system that the ACP port is being used. Which port is best, depends
on the application domain, so we'll try to explain how to make
this decision.

When using the HP port, all data is transferred directly to and from
DDR RAM. This efficiently uses the DDR controller and allows transfers
of 600MB/s simultaneously. The drawback of this method is that the CPU
also has to access this data in DDR RAM, and access to DDR memory is
relatively slow for the CPU. If the CPU is allowed to cache this shared
memory region, it will have to flush or invalidate cache lines

When using the ACP port, data is transferred through the "snoop" unit of
the L2 cache controller. If data destined for logic is already present
in the L2 or L1 cache, the data is transfered directly from the cache
and no access to DDR RAM is needed. If a memory address range for data
from logic is already present in the L2 cache, data is only written to
that location and no access to DDR RAM is needed. Transfers to and from
cache can run at 1200MB/s according to the Zynq documentation. The
drawback of the ACP port is that the snoop unit isn't very fast, and
transfers that do need to be handled by DDR run much slower (by a factor
2) than when performed directly. Transfers will not "allocate" cache
space, so the CPU is responsible for making sure that relevant data is
present in the L2 or L1 cache by first 'touching' it.


Ring buffer or block transfer?

The ring buffer mode is the easiest to use, because it offers simple
file-like read() or write() interfaces. Because the buffers for these
calls reside in user space, the driver has to copy the data to a DMA
capable buffer first, before it can transfer it. This is the case for
many devices. When combined with the ACP port, this memory copy will
actually take place in L2 cache. When using the HP port, the write to
DDR RAM was bound to happen anyway, and the performance impact is hardly
more than flushing cached data.

Currently the ringbuffer cannot be memory-mapped (this may change in
future though) and the only way to avoid the memory copy is to use one
of the block transfer modes instead.

In block transfer mode, the application requests the driver to allocate
a set of buffers. Currently the maximum number of buffers allowed is 8.
The size of each buffer is limited by available memory only. Once
allocated, the driver still "owns" the buffers. The application can
get ownership by dequeueing a buffer, and read or write its data. When
done, it moves the buffer back to the driver by enqueueing it, so that
logic can now access the data. This skips the memory copy step, hence
it is often referred to as "zero copy".


Coherent or streaming block transfer?

The choice for either streaming or coherent mode is determined by
application behavior. Which delivers best performance depends on what
the application intends to do with the data once it arrived from
logic.

"Coherent" means that the queue/dequeue operations are "cheap", no
cache flushing or invalidation required. Once the logic signals that a
block is ready for CPU use, the CPU can immediately access it. And vice
versa, transferring a block from CPU to logic is also immediate.
The drawback is that in case of a HP port design, the memory range
occupied by the buffer will be uncachable. Access to uncached memory is
slow, as every read involves a call to the DDR controller. Running
C code like "char* buffer; for(i=0;i<size;++i) sum+=buffer[i];" will
run 20x faster when caching is enabled.
However, if the next step the system wants to take is to send the data
through network, or to process it in larger chunks (e.g. using NEON
instructions to implement the above loop) the impact on performance is
much less. Sequential reads of DDR memory are quite fast. And if the
data was to large to fit in the L2 cache, the coherent method is likely
to be the more efficient one, since there will be few cache hits while
processing.

"Streaming" transfers (terminology used in the Linux kernel, not related
to AXI streaming) use cachable memory for the buffers. On ACP designs,
buffers are always cachable and coherent. On HP designs however, this
means that the system has to walk the cache tables and flush or invalidate
all cache lines related to the buffer that is being transferred. This
is a costly operation, and it will reduce the transfer rate to about 1/3rd
of the transfer speed of the coherent mode on the Zynq. However, after
this, the CPU has fast access to the memory again. If the next step is to
process the data in small words with random access patterns, this will more
than make up for the cache maintenance overhead. And when using the ACP
port, there wasn't any maintenance to begin with.



Benchmarks

The test allocates 4 blocks of the given size, 2 outgoing and 2
incoming. It touches all data using a memset. It then continuously
transfers these buffers, so there's always a block receiving data and
one sending. After transferring 100MB of data this way, it measures the
time elapsed and calculates and reports the average speed in MB/s.

HP, coherent
    4k:487 16k:597 64k:599 256k:599 1024k:599
HP, streaming
    4k:142 16k:169 64k:180 256k:183 1024k:184
ACP, coherent
    4k:399 16k:546 32k:582 64k:562 128k:458 256k:420 1024k:290
ACP, streaming
    4k:478 16k:436 32k:598 64k:579 128k:532 256k:397 1024k:288

As expected, the best ACP performance is with buffers of 32k..64k, as
these will fit completely in the L2 cache, while still being large
enough to have little CPU overhead in the transfer. At 128k buffer size,
it would use the 512k cache completely, so that the throughput is
slightly less. As buffer sizes increase, the throughput drops because
there will be more cache misses.

< TODO further explain how to correctly interpret results >


