/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * A NetMos 9900 PCI parallel port wired to JVM-managed ring buffers.
 *
 * <p>Once attached to a machine, the guest sees a standard SPP/IEEE 1284
 * parallel port (Linux's {@code parport_pc} binds via PCI ID match,
 * exposing {@code /dev/parport0} + {@code /dev/lp0}). Data flows in two
 * directions:
 *
 * <ul>
 *   <li><b>Forward (guest → JVM)</b>: each byte the guest pulses out via
 *       Centronics strobe lands in a 64 KiB native ring. {@link #poll(byte[])}
 *       drains it without blocking.</li>
 *   <li><b>Reverse (JVM → guest)</b>: {@link #feed(byte[])} pushes bytes
 *       into the device's 256-byte input ring. The guest receives them
 *       after issuing {@code PPNEGOT IEEE1284_MODE_NIBBLE} on
 *       {@code /dev/parport0} and reading from it (e.g. via {@code ppdev},
 *       {@code libieee1284}, or PLIP).</li>
 * </ul>
 *
 * <p>The bridge is freed automatically when its owning machine is freed —
 * do not hold a handle past the machine's lifetime.
 *
 * @see <a href="https://wiki.osdev.org/Parallel_port">Parallel port (OSDev)</a>
 */
public class ParportBridge {
    private final RVVMMachine machine;
    private long handle;

    public ParportBridge(RVVMMachine machine) {
        this.machine = machine;
        this.handle = RVVMNative.parport_bridge_init(machine.getPtr());
    }

    public boolean isValid() {
        return machine.isValid() && handle != 0;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /**
     * Drain up to {@code out.length} bytes of forward data (guest writes
     * to {@code /dev/lp0} or raw {@code /dev/parport0} writes via ppdev)
     * into {@code out}. Returns the number of bytes actually written.
     * Non-blocking; returns 0 if the guest hasn't strobed anything new.
     */
    public int poll(byte[] out) {
        if (!isValid() || out == null) return 0;
        return RVVMNative.parport_bridge_poll(handle, out);
    }

    /**
     * Push bytes into the reverse channel. Returns the count actually
     * accepted into the device's 256-byte input ring; if the guest is
     * slow to drain, this may be less than {@code in.length} and the
     * caller should retry the unsent tail later.
     *
     * <p>The guest must have negotiated to nibble mode via {@code PPNEGOT}
     * for these bytes to be readable; bytes fed before negotiation queue
     * up and surface to the first reader.
     */
    public int feed(byte[] in) {
        if (!isValid() || in == null) return 0;
        return RVVMNative.parport_bridge_feed(handle, in);
    }

    /**
     * Counters for instrumentation: {pushed, popped, fed, accepted,
     * tx_dropped}. Fills the provided {@code long[5]} to avoid allocating
     * on hot paths. {@code pushed} = guest forward writes; {@code popped}
     * = bytes the JVM has drained; {@code fed} = bytes passed to
     * {@link #feed(byte[])}; {@code accepted} = bytes the device's input
     * ring took; {@code tx_dropped} = forward bytes lost to TX overflow.
     */
    public void stats(long[] out) {
        if (!isValid() || out == null || out.length < 5) return;
        RVVMNative.parport_bridge_stats(handle, out);
    }
}
