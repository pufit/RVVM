/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * I2C slave device callbacks, dispatched from the i2c-oc controller's
 * MMIO write thread (the CPU/hart thread that ran the guest store).
 * One instance per attached slave; methods are called serially under
 * the bus lock for that slave, so implementations don't need their own
 * synchronisation against re-entry from the bus.
 *
 * <p><b>Transaction model.</b> A guest I2C transaction looks like:
 *
 * <pre>
 *   start(isWrite=true)   // master sent addr+W, slave returns ACK
 *   write(b0)             // first data byte after addr+W
 *   write(b1)
 *   ...
 *   stop()                // master released the bus
 *
 *   start(isWrite=false)  // addr+R; slave returns ACK
 *   read() -> b0          // slave drives MISO
 *   read() -> b1
 *   ...
 *   stop()
 * </pre>
 *
 * <p><b>NACK semantics.</b> Returning {@code false} from {@code start} or
 * {@code write}, or {@code -1} from {@code read}, signals NACK to the
 * master — the controller aborts the transaction and the guest sees the
 * appropriate error condition. Use this for register-out-of-range or
 * "I'm busy" responses.
 *
 * <p><b>What you must not do.</b> Do not issue I2C transactions back
 * into the same machine from inside a callback — the bus lock is held
 * around the entire callback chain and re-entering would deadlock.
 * Other RVVM operations (memory reads, IRQ raises) are fine.
 *
 * <p><b>Performance.</b> Each method call is one JNI upcall on the CPU
 * thread, blocking the guest until the call returns. Keep them short
 * (no blocking I/O, no taking of slow locks). For sensor-shaped devices
 * whose register state updates at game-tick rate, subclass
 * {@link ShadowRegisterI2CDevice} and update the shadow on tick — it
 * implements all four callbacks against an in-memory register file with
 * no further coordination required.
 */
public interface I2CDevice {

    /**
     * Begin a transaction at the master's choice of direction.
     *
     * @param isWrite true if the master sent {@code addr+W} and is about
     *                to push data bytes; false if {@code addr+R} and the
     *                master is about to clock data out.
     * @return true to ACK and continue; false to NACK (controller will
     *         abort the transaction).
     */
    boolean start(boolean isWrite);

    /**
     * Receive one data byte from the master.
     *
     * @param b the data byte the master shifted out on SDA.
     * @return true to ACK; false to NACK (master aborts the rest of the
     *         transaction).
     */
    boolean write(byte b);

    /**
     * Produce one data byte for the master to clock in.
     *
     * @return the data byte to drive on SDA in the range
     *         {@code 0..0xFF}; or a negative value to NACK (the master
     *         sees the read fail).
     */
    int read();

    /**
     * Finalise the transaction. Default does nothing — most devices
     * don't need a stop hook because state updates happen on each
     * read/write.
     */
    default void stop() {}
}
