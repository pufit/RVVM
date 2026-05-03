/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * A memory-buffer SPI slave the guest sees behind a SiFive SPI
 * controller. Java provides a read buffer that's served as MISO data
 * on transfers; bytes the guest shifts in on MOSI are captured into a
 * write ring the JVM drains on tick.
 *
 * <p><b>Cursor model.</b> Each chip-select assertion resets the cursor
 * to 0; subsequent transfers shift bytes out at {@code cursor++},
 * wrapping at the buffer's end. This matches the typical "data product"
 * slave shape — a sensor exposing its latest sample, an EEPROM that
 * streams from offset 0 on each transaction, a status register the
 * guest polls repeatedly. Java updates the buffer at any time
 * ({@link #setBuffer(byte[])} for wholesale replacement,
 * {@link #setByte(int, int)} for single-byte writes); the guest sees
 * the new contents on its next CS assertion.
 *
 * <p><b>Write-sink slaves.</b> If the guest is blasting bytes at the
 * SPI bus with no expected response (display controllers,
 * write-through DACs), construct with a 1-byte buffer of {@code 0xFF}
 * and ignore the cursor entirely; transfers return {@code 0xFF} on
 * every byte and the MOSI ring captures the full output stream.
 *
 * <p>Each {@code SPIBufferBridge} wires its own SiFive SPI controller
 * and attaches the slave at the lowest free CS line (== 0 on a fresh
 * controller). Multi-CS-on-one-controller is not yet supported.
 *
 * <p>The bridge is freed automatically when its owning machine is
 * freed — do not hold a handle past the machine's lifetime.
 */
public class SPIBufferBridge implements IRemovableDevice {
    private final RVVMMachine machine;
    private long handle;

    /**
     * Attach a SPI slave with an initial read buffer of {@code bufBytes}
     * bytes (clamped to ≥ 1). The buffer is filled with {@code 0xFF}
     * until {@link #setBuffer(byte[])} or {@link #setByte(int, int)}
     * populates it.
     */
    public SPIBufferBridge(RVVMMachine machine, int bufBytes) {
        this.machine = machine;
        this.handle  = machine.isValid()
                ? RVVMNative.spi_buffer_bridge_init(machine.getPtr(), bufBytes)
                : 0;
    }

    @Override
    public boolean isValid() {
        return machine.isValid() && handle != 0;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /**
     * Replace the read buffer wholesale. The new contents are copied
     * into a C-side allocation; the old buffer is freed. Cursor is
     * reset to 0. Pass a zero-length array to leave the buffer empty
     * (transfers will return {@code 0xFF} unconditionally).
     */
    public void setBuffer(byte[] data) {
        if (!isValid() || data == null) return;
        RVVMNative.spi_buffer_bridge_set_buffer(handle, data);
    }

    /**
     * Update one byte of the read buffer. Out-of-range offsets are
     * silent no-ops. The cursor is not touched.
     */
    public void setByte(int off, int value) {
        if (!isValid()) return;
        RVVMNative.spi_buffer_bridge_set_byte(handle, off, value);
    }

    /**
     * Drain queued MOSI bytes the guest shifted into the slave.
     * Returns the count actually drained.
     */
    public int poll(byte[] out) {
        if (!isValid() || out == null) return 0;
        return RVVMNative.spi_buffer_bridge_poll(handle, out);
    }

    /**
     * Counters for instrumentation: {totalTransfers, writesDropped,
     * ringOccupancy, cursor}. Fills the provided {@code long[4]} to
     * avoid allocating on hot paths.
     */
    public void stats(long[] out) {
        if (!isValid() || out == null || out.length < 4) return;
        RVVMNative.spi_buffer_bridge_stats(handle, out);
    }

    /**
     * Hot-detach the slave from its CS line on the SiFive controller.
     * Native side soft-flips the slave to "absent" (in-flight transfers
     * return {@code 0xFF} and stop capturing MOSI), then clears the bus
     * slot — which fires the slave's remove hook and frees the native
     * memory backing this handle. The SiFive controller stays attached;
     * the freed CS line behaves like an unpopulated slot.
     *
     * <p>After this call, {@link #isValid()} returns {@code false} and
     * subsequent setBuffer / setByte / poll / stats calls become silent
     * no-ops. Calling {@code remove()} twice is safe.
     */
    @Override
    public void remove() {
        long h = handle;
        if (h == 0) return;
        handle = 0;
        RVVMNative.spi_buffer_bridge_detach(h);
    }
}
