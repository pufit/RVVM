/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * A 256-byte shadow-register I2C slave the guest sees as any sensor /
 * EEPROM / IO expander. Java owns the canonical state and updates the
 * shadow on its own schedule (e.g. server tick); guest reads return
 * whatever the shadow holds at the moment of the read with no JNI
 * upcall. Guest writes are captured into a {@code (register, value)}
 * event ring the JVM drains via {@link #pollWrites(byte[])}.
 *
 * <p><b>Transaction model</b> matches the standard SMBus "set cursor,
 * then read/write" pattern — the same shape every common I2C sensor
 * uses:
 *
 * <pre>
 *   master: START + addr+W
 *   master: byte0       (cursor = byte0; awaiting subsequent writes)
 *   master: byte1       (shadow[cursor] = byte1; emit (cursor, byte1); cursor++)
 *   master: STOP
 *
 *   master: START + addr+R
 *   master: read        (return shadow[cursor]; cursor++)
 *   master: read        (return shadow[cursor]; cursor++)
 *   master: STOP
 * </pre>
 *
 * <p>The cursor persists across transactions, matching most real
 * peripherals — userspace drivers that care set the cursor explicitly
 * before every read anyway.
 *
 * <p><b>What this is good for.</b> Any in-game device whose state can
 * be summarised as ≤ 256 bytes of registers the guest reads on demand:
 * temperature / humidity / pressure sensors, accelerometers, IO
 * expanders, RTCs, simple display controllers. The shadow approach
 * loses devices whose responses depend on the guest's request *within*
 * a single transaction (e.g. an ADC where the guest writes a "start
 * conversion" bit and reads a "ready" bit) — but you can usually fake
 * those with a tick-driven state machine in Java that just keeps the
 * shadow correct.
 *
 * <p><b>Requires</b> the machine to have an I2C bus already attached
 * (typically via {@link RVVMNative#i2c_bus_init_auto(long)}). Bridge
 * is freed automatically when its owning machine is freed.
 */
public class I2CSensorBridge implements IRemovableDevice {
    /** Address of an 8-bit shadow register file. */
    public static final int SHADOW_BYTES = 256;

    private final RVVMMachine machine;
    private long handle;
    private final int  requestedAddr;

    /**
     * Attach a sensor at I2C address {@code addr}. Pass 0 to auto-pick
     * the next free address starting at 0x08.
     */
    public I2CSensorBridge(RVVMMachine machine, int addr) {
        this.machine       = machine;
        this.handle        = machine.isValid() ? RVVMNative.i2c_sensor_bridge_init(machine.getPtr(), addr) : 0;
        this.requestedAddr = addr;
    }

    @Override
    public boolean isValid() {
        return machine.isValid() && handle != 0;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /**
     * Address originally requested at construction time (0 if the caller
     * asked for auto-pick). For the bus address actually assigned by the
     * bus controller, see {@link #getAssignedAddr()}.
     */
    public int getAddr() {
        return requestedAddr;
    }

    /**
     * The I2C address the bus actually assigned to this slave. Resolves
     * a {@code new I2CSensorBridge(m, 0)} auto-pick — useful when the
     * JVM needs to tell guest userspace where to find the slave (e.g.
     * via a sysfs {@code new_device} write to the i2c-dev binding).
     * Returns 0 if the bridge failed to attach or has already been
     * detached.
     */
    public int getAssignedAddr() {
        if (handle == 0) return 0;
        return RVVMNative.i2c_sensor_bridge_addr(handle);
    }

    /**
     * Set one shadow register. Visible to the next guest read at this
     * register address.
     */
    public void set(int reg, int value) {
        if (!isValid()) return;
        RVVMNative.i2c_sensor_bridge_set(handle, reg, value);
    }

    /**
     * Bulk update: copy {@code data} into shadow starting at register
     * {@code off}, wrapping at 256 if needed. Returns the number of
     * bytes written.
     */
    public int setBulk(int off, byte[] data) {
        if (!isValid() || data == null) return 0;
        return RVVMNative.i2c_sensor_bridge_set_bulk(handle, off, data);
    }

    /**
     * Drain queued guest write events. Each event is two bytes
     * {@code (register, value)}; {@code out} length must be even. The
     * return value is the number of <i>events</i> drained, not bytes.
     */
    public int pollWrites(byte[] out) {
        if (!isValid() || out == null) return 0;
        return RVVMNative.i2c_sensor_bridge_poll_writes(handle, out);
    }

    /**
     * Counters for instrumentation: {totalReads, totalWrites,
     * writesDropped, ringOccupancy} (last three in bytes — each event
     * is 2 bytes; divide by 2 for event counts). Fills the provided
     * {@code long[4]} to avoid allocating on hot paths.
     */
    public void stats(long[] out) {
        if (!isValid() || out == null || out.length < 4) return;
        RVVMNative.i2c_sensor_bridge_stats(handle, out);
    }

    /**
     * Hot-detach the slave from the I2C bus. The native side first
     * soft-NACKs the slave (any in-flight transaction sees start /
     * read / write return false), then asks the bus to vector-erase the
     * slot — which fires the slave's remove hook and frees the native
     * memory backing this handle.
     *
     * <p>After this call, {@link #isValid()} returns {@code false} and
     * subsequent calls to set / setBulk / pollWrites / stats become
     * silent no-ops. Calling {@code remove()} twice is safe (the second
     * call is a no-op).
     */
    @Override
    public void remove() {
        long h = handle;
        if (h == 0) return;
        handle = 0;
        RVVMNative.i2c_sensor_bridge_detach(h);
    }
}
