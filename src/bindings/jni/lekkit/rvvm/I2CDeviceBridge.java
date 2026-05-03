/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * Attaches an arbitrary {@link I2CDevice} implementation to a machine's
 * I2C bus. The native side dispatches the controller's transaction
 * callbacks ({@code start} / {@code write} / {@code read} / {@code stop})
 * synchronously into the supplied handler on each guest transfer.
 *
 * <p>For sensor-shaped devices (state updates at game-tick rate, guest
 * reads see whatever the JVM last wrote), use
 * {@link ShadowRegisterI2CDevice} as the handler — or subclass it for
 * register-driven semantics. For custom behaviour (per-byte state
 * machines, ADC-style command/response, write-only command channels),
 * implement {@link I2CDevice} directly with whatever logic the device
 * needs.
 *
 * <p>Requires the machine to have an I2C bus already attached
 * (typically via {@link RVVMNative#i2c_bus_init_auto(long)}). Calling
 * {@link #remove()} hot-detaches the slave from the bus and frees the
 * native bridge memory; freeing the machine implicitly does the same
 * via the bus's slave-vector teardown.
 *
 * <p>Holds a reference to the handler so the JVM doesn't collect it
 * while the C side has a global ref pinning it; once {@link #remove()}
 * runs and the global ref is released, the handler becomes eligible
 * for GC if no other root retains it.
 */
public class I2CDeviceBridge implements IRemovableDevice {

    private final RVVMMachine machine;
    private final I2CDevice   handler;
    private final int         requestedAddr;
    private long              handle;

    /**
     * Attach a slave at I2C address {@code addr} (pass 0 to auto-pick
     * the next free address from 0x08 upward). The {@code handler}
     * receives every transaction step on the CPU thread; see
     * {@link I2CDevice} for the contract.
     */
    public I2CDeviceBridge(RVVMMachine machine, int addr, I2CDevice handler) {
        this.machine       = machine;
        this.handler       = handler;
        this.requestedAddr = addr;
        this.handle        = (machine.isValid() && handler != null)
                ? RVVMNative.i2c_dev_bridge_init(machine.getPtr(), addr, handler)
                : 0;
    }

    @Override
    public boolean isValid() {
        return machine.isValid() && handle != 0;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /** The handler this bridge dispatches to. Non-null for the bridge's lifetime. */
    public I2CDevice getHandler() {
        return handler;
    }

    /**
     * Address originally requested at construction (0 if the caller
     * asked for auto-pick). For the bus address actually assigned, see
     * {@link #getAssignedAddr()}.
     */
    public int getRequestedAddr() {
        return requestedAddr;
    }

    /**
     * The I2C address the bus actually assigned to this slave. Resolves
     * a {@code new I2CDeviceBridge(m, 0, ...)} auto-pick to its concrete
     * 7-bit address. Returns 0 if the bridge failed to attach or has
     * already been detached.
     */
    public int getAssignedAddr() {
        if (handle == 0) return 0;
        return RVVMNative.i2c_dev_bridge_addr(handle);
    }

    /**
     * Hot-detach the slave from the bus. The native side asks the
     * controller to vector-erase the slot under the bus lock — fires the
     * slave's remove hook (drops the global ref to {@link #handler} and
     * frees the bridge struct). After this, {@link #isValid()} returns
     * false and {@code remove()} is a no-op (idempotent).
     */
    @Override
    public void remove() {
        long h = handle;
        if (h == 0) return;
        handle = 0;
        RVVMNative.i2c_dev_bridge_detach(h);
    }
}
