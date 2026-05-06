/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

public class RTL8169 extends PCIDevice {
    private long tap;

    public RTL8169(RVVMMachine machine) {
        super(machine);
        if (machine.isValid()) {
            long pci_bus = RVVMNative.get_pci_bus(machine.getPtr());
            if (pci_bus != 0) {
                this.tap = RVVMNative.tap_user_open();
                if (this.tap != 0) {
                    setPCIHandle(RVVMNative.rtl8169_init(pci_bus, this.tap));
                }
            }
        }
    }

    /**
     * Raw TAP handle. Stable for the NIC's lifetime; zero if the TAP
     * failed to open (e.g. machine not yet valid at construction).
     */
    public long getTapHandle() {
        return tap;
    }

    /**
     * Forward a host port into the guest network. Format matches
     * QEMU-style user networking:
     * <pre>
     *   "tcp/2022=22"                 host 0.0.0.0:2022 -&gt; guest DHCP:22
     *   "udp/5353=5353"               UDP variant
     *   "[::1]:2022=22"               IPv6 host bind
     *   "127.0.0.1:2022=10.0.2.15:22" explicit guest IP
     * </pre>
     * Returns true on success, false if parsing failed or the host
     * bind was rejected (port already in use, &lt;1024 without privilege).
     * Forwards survive until machine teardown.
     */
    public boolean portfwd(String fwd) {
        if (tap == 0 || fwd == null) return false;
        return RVVMNative.tap_portfwd(tap, fwd);
    }

    /**
     * Override the host interface address that gets bound for
     * forwarded ports. Default is all interfaces. Pass e.g.
     * {@code "127.0.0.1"} to keep the forwarded ports loopback-only.
     */
    public boolean ifaddr(String addr) {
        if (tap == 0 || addr == null) return false;
        return RVVMNative.tap_ifaddr(tap, addr);
    }
}
