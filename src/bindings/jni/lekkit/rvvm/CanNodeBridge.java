/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * A virtual CAN node wired to a software CAN segment that the guest
 * sees through a Microchip MCP2515 SPI controller.
 *
 * <p>Constructing a {@code CanNodeBridge} attaches a fresh SiFive SPI
 * controller, an MCP2515 on that bus, and a private CAN segment to the
 * given machine. Linux's {@code mcp251x} driver binds via the SPI
 * compatible string and exposes the segment as {@code can0} (or the
 * next free index). Frames the guest broadcasts on {@code can0} land
 * in a 256-frame ring the JVM drains via {@link #poll(byte[])}; frames
 * the JVM injects via {@link #feed(byte[], int)} are delivered to the
 * guest through the MCP2515's RX path.
 *
 * <p><b>Frame wire format</b> — fixed 16 bytes per frame for batched
 * poll/feed:
 *
 * <pre>
 *   off 0..3   uint32 LE   can_id (with CAN_EFF/RTR/ERR flag bits)
 *   off 4      uint8       dlc (0..8)
 *   off 5..7   reserved (zero on poll, ignored on feed)
 *   off 8..15  data[0..7]
 * </pre>
 *
 * <p>The flag bits in {@code can_id} mirror Linux's {@code <linux/can.h>}:
 * <ul>
 *   <li>{@code 0x80000000} — extended (29-bit) frame</li>
 *   <li>{@code 0x40000000} — remote-transmission request</li>
 *   <li>{@code 0x20000000} — error frame</li>
 *   <li>{@code 0x1FFFFFFF} — extended ID mask (or {@code 0x7FF} for SFF)</li>
 * </ul>
 *
 * <p><b>Topology note.</b> Each {@code CanNodeBridge} wires its own
 * SPI controller + MCP2515 + CAN segment. Two bridges on the same
 * machine therefore appear to the guest as two independent CAN
 * interfaces. Multi-node-on-one-segment is not yet supported.
 *
 * <p>The bridge is freed automatically when its owning machine is
 * freed — do not hold a handle past the machine's lifetime.
 */
public class CanNodeBridge {
    /** Fixed wire stride per frame in poll/feed buffers. */
    public static final int FRAME_BYTES = 16;

    public static final int CAN_EFF_FLAG = 0x80000000;
    public static final int CAN_RTR_FLAG = 0x40000000;
    public static final int CAN_ERR_FLAG = 0x20000000;
    public static final int CAN_EFF_MASK = 0x1FFFFFFF;
    public static final int CAN_SFF_MASK = 0x000007FF;

    private final RVVMMachine machine;
    private final long handle;

    public CanNodeBridge(RVVMMachine machine) {
        this.machine = machine;
        this.handle  = machine.isValid() ? RVVMNative.can_node_bridge_init(machine.getPtr()) : 0;
    }

    public boolean isValid() {
        return machine.isValid() && handle != 0;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /**
     * Drain queued guest frames into the supplied buffer. Buffer length
     * must be a multiple of {@link #FRAME_BYTES} (extra trailing bytes
     * are ignored). Returns the number of frames actually drained.
     */
    public int poll(byte[] out) {
        if (!isValid() || out == null) return 0;
        return RVVMNative.can_node_bridge_poll(handle, out);
    }

    /**
     * Broadcast {@code nFrames} packed in {@code in} onto the bus.
     * Each frame occupies {@link #FRAME_BYTES} contiguous bytes. The
     * count is capped to whatever fits in {@code in.length}; the return
     * value tells you how many actually went out.
     */
    public int feed(byte[] in, int nFrames) {
        if (!isValid() || in == null || nFrames <= 0) return 0;
        return RVVMNative.can_node_bridge_feed(handle, in, nFrames);
    }

    /** Convenience: pack a single frame into a 16-byte buffer in the wire format. */
    public static byte[] packFrame(int canId, byte[] data, int dlc) {
        if (dlc < 0) dlc = 0;
        if (dlc > 8) dlc = 8;
        byte[] out = new byte[FRAME_BYTES];
        out[0] = (byte)(canId);
        out[1] = (byte)(canId >>> 8);
        out[2] = (byte)(canId >>> 16);
        out[3] = (byte)(canId >>> 24);
        out[4] = (byte)dlc;
        if (data != null) {
            int copy = Math.min(dlc, data.length);
            System.arraycopy(data, 0, out, 8, copy);
        }
        return out;
    }

    /**
     * Counters for instrumentation: {pushed, popped, fed, rxDropped,
     * ringOccupancy} — all in bytes; divide by {@link #FRAME_BYTES} for
     * frame counts. Fills the provided {@code long[5]} to avoid
     * allocating on hot paths.
     */
    public void stats(long[] out) {
        if (!isValid() || out == null || out.length < 5) return;
        RVVMNative.can_node_bridge_stats(handle, out);
    }
}
