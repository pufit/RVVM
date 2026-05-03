/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * 256-byte shadow-register I2C device. Replaces the previous C-side
 * sensor bridge: same wire-level behaviour (cursor model, write event
 * ring, optional bulk shadow updates), implemented in Java on top of
 * the {@link I2CDevice} callback contract.
 *
 * <p><b>Transaction model.</b> Matches every common SMBus-shaped I2C
 * peripheral:
 *
 * <pre>
 *   start(isWrite=true)
 *   write(reg)                  // first byte sets the cursor
 *   write(byte)                 // shadow[cursor] = byte; emit (cursor, byte); cursor++
 *   ...
 *   stop()
 *
 *   start(isWrite=false)
 *   read() -> shadow[cursor++]  // cursor was set by a previous write transaction
 *   ...
 *   stop()
 * </pre>
 *
 * The cursor persists across transactions; userspace drivers that care
 * set it explicitly before every read.
 *
 * <p><b>What this fits.</b> Anything whose state can be summarised as
 * up to 256 bytes of registers the guest reads on demand: temperature
 * / humidity / pressure sensors, RTCs, simple IO expanders, EEPROMs
 * with an 8-bit address space, status readouts. Update the shadow on
 * server tick from in-world state ({@link #set(int, int)} /
 * {@link #setBulk(int, byte[])}) and drain
 * {@link #pollWrites(byte[])} for guest-issued register pokes.
 *
 * <p><b>What this doesn't fit.</b> Transactions whose response content
 * depends on the request *within* the same transaction (e.g. an ADC
 * where the guest writes "start conversion" and reads the result in the
 * same I2C session) — subclass {@link I2CDevice} directly and implement
 * the callbacks with whatever per-byte state machine you need. Or
 * subclass this class and override {@code start} / {@code write} /
 * {@code read} to layer custom behaviour on top of the shadow.
 *
 * <p><b>Threading.</b> Callbacks fire on the CPU/hart thread under the
 * I2C bus lock. {@link #set}, {@link #setBulk}, {@link #pollWrites},
 * {@link #stats} are JVM-thread-safe (synchronized) — call them from
 * the server tick. Subclasses overriding callbacks should keep the
 * synchronisation pattern.
 */
public class ShadowRegisterI2CDevice implements I2CDevice {

    /** Address space of the shadow register file. */
    public static final int SHADOW_BYTES = 256;

    /** Default ring capacity in (addr,value) pairs (== 64 events). */
    private static final int DEFAULT_RING_PAIRS = 64;

    private final byte[] regs = new byte[SHADOW_BYTES];
    private int cursor;
    private boolean awaitingAddr;

    // (addr, value) write events. Stored as a flat byte ring of pairs;
    // capacity is fixed at construction time. Drop-oldest on overflow.
    private final byte[] ring;
    private final int    ringCapBytes;
    private int          ringHead;     // next write index
    private int          ringTail;     // next read index
    private int          ringFill;     // bytes currently in the ring

    // Stats counters (read by stats(), incremented on the CPU thread).
    private long totalReads;
    private long totalWrites;
    private long writesDropped;

    public ShadowRegisterI2CDevice() {
        this(DEFAULT_RING_PAIRS);
    }

    /**
     * @param ringPairs maximum pending write events before drop-oldest
     *                  kicks in. Each event costs 2 bytes; pick a size
     *                  that comfortably covers one tick of guest write
     *                  traffic.
     */
    public ShadowRegisterI2CDevice(int ringPairs) {
        if (ringPairs < 1) ringPairs = 1;
        this.ringCapBytes = ringPairs * 2;
        this.ring         = new byte[ringCapBytes];
    }

    /* ------------------------------------------------------------------ */
    /* I2CDevice callbacks (CPU thread, under bus lock)                   */
    /* ------------------------------------------------------------------ */

    @Override
    public synchronized boolean start(boolean isWrite) {
        // Only writes need the address-byte gate; reads continue from
        // the cursor a previous write transaction left behind.
        awaitingAddr = isWrite;
        return true;
    }

    @Override
    public synchronized boolean write(byte b) {
        if (awaitingAddr) {
            cursor       = b & 0xFF;
            awaitingAddr = false;
            return true;
        }
        regs[cursor] = b;
        // Emit (addr, byte) write event. Drop oldest pair on overflow.
        if (ringFill + 2 > ringCapBytes) {
            ringTail        = (ringTail + 2) % ringCapBytes;
            ringFill       -= 2;
            writesDropped  += 2;
        }
        ring[ringHead]                       = (byte) cursor;
        ring[(ringHead + 1) % ringCapBytes]  = b;
        ringHead   = (ringHead + 2) % ringCapBytes;
        ringFill  += 2;
        totalWrites += 2;
        cursor = (cursor + 1) & 0xFF; // wraps, matches typical 8-bit-cursor devices
        return true;
    }

    @Override
    public synchronized int read() {
        int v = regs[cursor] & 0xFF;
        cursor = (cursor + 1) & 0xFF;
        totalReads += 1;
        return v;
    }

    /* ------------------------------------------------------------------ */
    /* JVM-thread-side accessors (server tick / sensor update)            */
    /* ------------------------------------------------------------------ */

    /**
     * Set one shadow register. Visible to the next guest read at this
     * register address.
     */
    public synchronized void set(int reg, int value) {
        regs[reg & 0xFF] = (byte) (value & 0xFF);
    }

    /**
     * Bulk update: copy {@code data} into shadow starting at register
     * {@code off}, wrapping at 256 if needed. Returns the number of
     * bytes written (== {@code data.length}).
     */
    public synchronized int setBulk(int off, byte[] data) {
        if (data == null) return 0;
        for (int i = 0; i < data.length; i++) {
            regs[(off + i) & 0xFF] = data[i];
        }
        return data.length;
    }

    /**
     * Read-only snapshot of one shadow register. Useful for unit tests
     * and for subclasses building atop the cursor model.
     */
    public synchronized int peek(int reg) {
        return regs[reg & 0xFF] & 0xFF;
    }

    /**
     * Drain queued guest write events. Each event is two bytes
     * {@code (register, value)}; {@code out} length must be even. The
     * return value is the number of <i>events</i> drained, not bytes.
     */
    public synchronized int pollWrites(byte[] out) {
        if (out == null) return 0;
        int pairs = out.length / 2;
        if (pairs <= 0) return 0;
        int wantBytes = Math.min(pairs * 2, ringFill);
        // ringFill is always even by construction (writes go in as pairs);
        // wantBytes inherits that.
        int written = 0;
        while (written < wantBytes) {
            // Copy one byte at a time around the wrap-around. Simple and
            // typically <= 32 bytes per drain — not a hot path.
            out[written++] = ring[ringTail];
            ringTail       = (ringTail + 1) % ringCapBytes;
            ringFill--;
        }
        return wantBytes / 2;
    }

    /**
     * Counters for instrumentation: {@code {totalReads, totalWrites,
     * writesDropped, ringOccupancyBytes}}. Last two are byte counts —
     * each event is two bytes.
     */
    public synchronized void stats(long[] out) {
        if (out == null || out.length < 4) return;
        out[0] = totalReads;
        out[1] = totalWrites;
        out[2] = writesDropped;
        out[3] = ringFill;
    }
}
