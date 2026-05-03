/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * A 256-byte shadow-register I2C slave the guest sees as any sensor /
 * EEPROM / IO expander. Java owns the canonical state and updates the
 * shadow on its own schedule (server tick); guest reads return whatever
 * the shadow holds at the moment of the read. Guest writes are captured
 * into a {@code (register, value)} event ring the JVM drains via
 * {@link #pollWrites(byte[])}.
 *
 * <p>Thin facade: the device logic lives in {@link ShadowRegisterI2CDevice}
 * and the bus attachment in {@link I2CDeviceBridge}. This class exists
 * so callers who want "the standard sensor shape with one constructor"
 * don't have to wire two objects together.
 *
 * <p><b>Transaction model</b> matches the standard SMBus "set cursor,
 * then read/write" pattern — see {@link ShadowRegisterI2CDevice} for
 * details. The cursor persists across transactions; userspace drivers
 * that care set it explicitly before every read.
 *
 * <p><b>What this fits.</b> Any in-game device whose state can be
 * summarised as ≤ 256 bytes of registers the guest reads on demand:
 * temperature / humidity / pressure sensors, accelerometers, IO
 * expanders, RTCs, simple display controllers. For per-byte state
 * machines (ADC start-conversion-then-read, multi-step command/response),
 * implement {@link I2CDevice} directly and pass it to
 * {@link I2CDeviceBridge}.
 *
 * <p><b>Requires</b> the machine to have an I2C bus already attached
 * (typically via {@link RVVMNative#i2c_bus_init_auto(long)}). Calling
 * {@link #remove()} hot-detaches the slave; freeing the machine
 * implicitly does the same.
 */
public class I2CSensorBridge extends ShadowRegisterI2CDevice implements IRemovableDevice {
    /** Address space of the shadow register file. */
    public static final int SHADOW_BYTES = ShadowRegisterI2CDevice.SHADOW_BYTES;

    private final I2CDeviceBridge bridge;

    /**
     * Attach a sensor at I2C address {@code addr}. Pass 0 to auto-pick
     * the next free address starting at 0x08.
     */
    public I2CSensorBridge(RVVMMachine machine, int addr) {
        super();
        // Passing `this` to a constructor mid-build is normally a smell;
        // here the only thing the bridge does with it synchronously is
        // a NewGlobalRef on the C side, and the device's callbacks
        // can't fire until the bus actually dispatches a guest
        // transaction (many ticks in the future). super() has run, so
        // the shadow is in a coherent state.
        this.bridge = new I2CDeviceBridge(machine, addr, this);
    }

    @Override
    public boolean isValid() {
        return bridge.isValid();
    }

    public RVVMMachine getMachine() {
        return bridge.getMachine();
    }

    /**
     * Address originally requested at construction time (0 if the
     * caller asked for auto-pick). For the bus address actually
     * assigned, see {@link #getAssignedAddr()}.
     */
    public int getAddr() {
        return bridge.getRequestedAddr();
    }

    /**
     * The I2C address the bus actually assigned to this slave. Resolves
     * a {@code new I2CSensorBridge(m, 0)} auto-pick. Returns 0 if the
     * bridge failed to attach or has already been detached.
     */
    public int getAssignedAddr() {
        return bridge.getAssignedAddr();
    }

    /**
     * Hot-detach the slave from the I2C bus. After this call,
     * {@link #isValid()} returns {@code false} and subsequent calls to
     * inherited mutators ({@link #set}, {@link #setBulk}, ...) keep
     * working against the in-memory shadow but are unobservable to the
     * guest. Calling {@code remove()} twice is safe.
     */
    @Override
    public void remove() {
        bridge.remove();
    }
}
