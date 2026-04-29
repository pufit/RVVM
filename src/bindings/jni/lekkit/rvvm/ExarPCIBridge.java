/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * An Exar XR17V35x-family PCIe combo serial card wired to JVM-managed
 * ring buffers — one bridge per port.
 *
 * <p>Linux's {@code 8250_exar} driver binds via vendor 0x13a8 + Exar device
 * IDs and exposes each port as {@code /dev/ttyS<n>} (numbering starts at
 * whatever line the 8250 layer hands out, often ttyS0..ttyS<count-1> on
 * stock kernels where PCI probes before of_serial). All ports share one
 * INTx line (auto-fan-in inside the emulation).
 *
 * <p><b>Models</b> — pass one of these to the constructor's {@code numPorts}:
 * <ul>
 *   <li>{@link #PORTS_V352} — XR17V352 (2 ports)</li>
 *   <li>{@link #PORTS_V354} — XR17V354 (4 ports)</li>
 *   <li>{@link #PORTS_V358} — XR17V358 (8 ports)</li>
 *   <li>{@link #PORTS_V4358} — XR17V4358 (12 ports — V354 main + V358 expansion)</li>
 *   <li>{@link #PORTS_V8358} — XR17V8358 (16 ports — V358 main + V358 expansion)</li>
 * </ul>
 *
 * <p><b>PCI bus is mandatory.</b> The machine must have a PCI root complex
 * attached before constructing this bridge (typically via
 * {@code RVVMNative.pci_bus_init_auto(machine.getPtr())}). If no PCI bus
 * is present {@link #isValid()} returns false and all per-port operations
 * become no-ops; the native side logs an error to stderr explaining what's
 * missing. The bridge is freed automatically when its owning machine is
 * freed — do not hold a handle past the machine's lifetime.
 *
 * <p><b>NR_UARTS caveat.</b> Stock Alpine and many distro kernels build
 * with {@code CONFIG_SERIAL_8250_NR_UARTS=4}. For Exar configs with more
 * than 3 ports, the existing on-board ns16550a's {@code of_serial}
 * registration silently fails with {@code -ENOSPC}. Bump
 * {@code 8250.nr_uarts=<n>} on the kernel cmdline if the on-board UART
 * matters; the Exar card itself works regardless.
 *
 * <p>Per-port operations route through {@link RVVMNative#ns16550a_bridge_poll
 * ns16550a_bridge_poll}/{@code feed}/{@code stats} since the underlying
 * native bridge is the same primitive. The {@code ns16550a_} prefix on
 * those methods is historical — they take any bridge handle.
 */
public class ExarPCIBridge {
    public static final int PORTS_V352  = 2;
    public static final int PORTS_V354  = 4;
    public static final int PORTS_V358  = 8;
    public static final int PORTS_V4358 = 12;
    public static final int PORTS_V8358 = 16;

    private final RVVMMachine machine;
    private final long[]      handles; // null if init failed
    private final int         numPorts;

    /**
     * Attach an Exar combo card with {@code numPorts} ports to {@code machine}.
     * The machine must already have a PCI bus; if not, {@link #isValid()}
     * returns false and the native side logs the cause.
     *
     * @param numPorts one of {@link #PORTS_V352}, {@link #PORTS_V354},
     *                 {@link #PORTS_V358}, {@link #PORTS_V4358}, {@link #PORTS_V8358}
     */
    public ExarPCIBridge(RVVMMachine machine, int numPorts) {
        this.machine  = machine;
        this.handles  = RVVMNative.exar_pci_bridge_init(machine.getPtr(), numPorts);
        this.numPorts = (handles != null) ? handles.length : 0;
    }

    public boolean isValid() {
        return machine.isValid() && handles != null;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /** Number of ports this card exposes (0 if init failed). */
    public int numPorts() {
        return numPorts;
    }

    private boolean validPort(int port) {
        return isValid() && port >= 0 && port < numPorts;
    }

    /**
     * Drain up to {@code out.length} bytes of guest TX from port
     * {@code port} into {@code out}. Returns the number of bytes written.
     * Non-blocking; returns 0 if the port hasn't produced anything or the
     * port index is out of range.
     */
    public int poll(int port, byte[] out) {
        if (!validPort(port) || out == null) return 0;
        return RVVMNative.ns16550a_bridge_poll(handles[port], out);
    }

    /**
     * Push bytes into guest RX on port {@code port}. Returns the count
     * accepted; may be less than {@code in.length} if the port's RX ring
     * is near-full.
     */
    public int feed(int port, byte[] in) {
        if (!validPort(port) || in == null) return 0;
        return RVVMNative.ns16550a_bridge_feed(handles[port], in);
    }

    /**
     * Counters for instrumentation on port {@code port}: {pushed, popped,
     * fed, consumed, dropped}. Fills the provided {@code long[5]} to avoid
     * allocating on hot paths. No-op if port index is out of range.
     */
    public void stats(int port, long[] out) {
        if (!validPort(port) || out == null || out.length < 5) return;
        RVVMNative.ns16550a_bridge_stats(handles[port], out);
    }
}
