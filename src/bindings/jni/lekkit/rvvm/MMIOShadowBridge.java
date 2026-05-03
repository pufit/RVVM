/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * A generic shadow MMIO region the guest can read and write at a
 * known physical address. Java owns the canonical state via a C-side
 * shadow buffer it can peek/poke at any time; guest reads return
 * whatever the shadow holds at the moment of the read with no JNI
 * upcall. Guest writes mutate the shadow AND emit a 16-byte event
 * into a ring the JVM drains on tick.
 *
 * <p><b>Write event wire format</b> — 16 bytes per event, fixed stride
 * for batched poll:
 *
 * <pre>
 *   off 0..3    uint32 LE   region offset of the access
 *   off 4..7    uint32 LE   access size in bytes (1, 2, 4, or 8)
 *   off 8..15   uint8[8]    written value, LE-padded to 8 bytes
 * </pre>
 *
 * <p>The 8-byte value field covers all natural guest write sizes
 * (rv64 STD = 8 bytes); for smaller writes the upper bytes are zero.
 *
 * <p><b>Address management.</b> The constructor takes a <i>requested</i>
 * base address; if 0 or busy, the region API picks a free address. The
 * actual assigned address is returned via {@link #getBaseAddr()}, which
 * the caller can inject into a kernel cmdline / userspace mmap call site.
 * No FDT node is generated — the guest must know the address out-of-band
 * to access the region.
 *
 * <p><b>Use cases</b> — anything that doesn't fit a standard bus shape:
 * <ul>
 *   <li>Custom mod-bus controller exposing per-device-slot registers</li>
 *   <li>Instrumentation MMIO for in-world sensors that aren't I2C/SPI/CAN</li>
 *   <li>Mailbox between mod-side block entities and guest userspace</li>
 *   <li>Doorbell / IRQ-trigger registers (if you only need write events,
 *       not the shadow contents)</li>
 * </ul>
 *
 * <p>The bridge is freed automatically when its owning machine is freed.
 */
public class MMIOShadowBridge {
    /** Fixed wire stride per event in {@link #pollWrites(byte[])} buffers. */
    public static final int EVENT_BYTES = RVVMNative.MMIO_EVENT_BYTES;

    private final RVVMMachine machine;
    private final long handle;
    private final long baseAddr;
    private final long size;

    /**
     * Attach a shadow MMIO region of {@code size} bytes. Pass 0 (or any
     * busy address) for {@code requestedAddr} to let the region API
     * pick a free address.
     */
    public MMIOShadowBridge(RVVMMachine machine, long requestedAddr, long size) {
        this.machine = machine;
        this.size    = size;

        long[] addrOut = new long[1];
        this.handle = machine.isValid()
                ? RVVMNative.mmio_shadow_bridge_init(machine.getPtr(), requestedAddr, size, addrOut)
                : 0;
        this.baseAddr = (handle != 0) ? addrOut[0] : 0;
    }

    public boolean isValid() {
        return machine.isValid() && handle != 0;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /** Assigned base physical address. Hand this to the guest however it expects. */
    public long getBaseAddr() {
        return baseAddr;
    }

    /** Region size in bytes (as requested). */
    public long getSize() {
        return size;
    }

    /** Read one byte from the shadow. Out-of-range offsets return 0. */
    public int getByte(long off) {
        if (!isValid()) return 0;
        return RVVMNative.mmio_shadow_bridge_get_byte(handle, off);
    }

    /** Write one byte to the shadow. Out-of-range offsets are no-ops. */
    public void setByte(long off, int value) {
        if (!isValid()) return;
        RVVMNative.mmio_shadow_bridge_set_byte(handle, off, value);
    }

    /**
     * Bulk shadow update. Copies {@code data} into the shadow starting
     * at {@code off}. Truncated at the shadow boundary if the write
     * would extend past the region's end. Returns the number of bytes
     * actually written.
     */
    public int setBulk(long off, byte[] data) {
        if (!isValid() || data == null) return 0;
        return RVVMNative.mmio_shadow_bridge_set_bulk(handle, off, data);
    }

    /**
     * Drain queued write events into the supplied buffer. Buffer
     * length must be a multiple of {@link #EVENT_BYTES}; extra
     * trailing bytes are ignored. Returns the number of events
     * actually drained.
     */
    public int pollWrites(byte[] out) {
        if (!isValid() || out == null) return 0;
        return RVVMNative.mmio_shadow_bridge_poll_writes(handle, out);
    }

    /**
     * Counters for instrumentation: {totalReads, totalWrites,
     * eventsDropped, ringOccupancyBytes, shadowSize}. Fills the
     * provided {@code long[5]}.
     */
    public void stats(long[] out) {
        if (!isValid() || out == null || out.length < 5) return;
        RVVMNative.mmio_shadow_bridge_stats(handle, out);
    }

    /**
     * Convenience: decode an event from a 16-byte slice of a poll
     * buffer into a {@code (offset, size, valueLE)} tuple. Allocates;
     * use the byte-array directly on hot paths if the alloc matters.
     */
    public static Event decodeEvent(byte[] buf, int slot) {
        int base = slot * EVENT_BYTES;
        long off = (((long)(buf[base    ] & 0xFF))      )
                 | (((long)(buf[base + 1] & 0xFF)) <<  8)
                 | (((long)(buf[base + 2] & 0xFF)) << 16)
                 | (((long)(buf[base + 3] & 0xFF)) << 24);
        int size = (buf[base + 4] & 0xFF)
                 | ((buf[base + 5] & 0xFF) << 8)
                 | ((buf[base + 6] & 0xFF) << 16)
                 | ((buf[base + 7] & 0xFF) << 24);
        long value = 0;
        for (int i = 0; i < 8; ++i) {
            value |= ((long)(buf[base + 8 + i] & 0xFF)) << (i * 8);
        }
        return new Event(off, size, value);
    }

    /** Decoded write event from {@link #pollWrites(byte[])}. */
    public static final class Event {
        public final long offset;
        public final int  size;
        public final long valueLE;

        public Event(long offset, int size, long valueLE) {
            this.offset  = offset;
            this.size    = size;
            this.valueLE = valueLE;
        }
    }
}
