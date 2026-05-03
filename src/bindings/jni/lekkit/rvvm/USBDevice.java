/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * A USB 2.0 High-Speed device exposed to the guest via the machine's
 * XHCI controller.
 *
 * <p>Usage pattern: subclass, declare endpoints and optional class
 * descriptors in your constructor, then implement your protocol on top
 * of {@link #feed(byte, byte[], int, int)} (push bytes toward the
 * guest on IN endpoints) and {@link #poll(byte, byte[])} (drain bytes
 * the guest sent on OUT endpoints).
 *
 * <p>Standard Chapter 9 enumeration (device/config/string descriptors,
 * SET_ADDRESS, SET_CONFIGURATION, etc.) is handled entirely in C from
 * the values supplied to the constructor — Java never sees those
 * requests. Class-specific requests (GET_REPORT, SET_IDLE for HID,
 * vendor requests, etc.) currently stall at the C layer; a future
 * extension will mailbox them to Java.
 *
 * <p>Endpoint addresses follow USB convention: bit 7 set = IN
 * direction, low nibble = endpoint number 1..15. EP0 is reserved for
 * control and not addressable via feed/poll.
 *
 * <p>Lifetime: the device lives until {@link #remove()} is called or
 * the owning machine is freed. It does NOT finalize automatically on
 * GC — keep a reference, call remove explicitly when done.
 */
public class USBDevice implements IRemovableDevice {
    /** Control transfer endpoint — not addressable via feed/poll. */
    public static final byte EP_TYPE_CONTROL     = 0x0;
    /** Isochronous transfer — not implemented end-to-end yet, do not use. */
    public static final byte EP_TYPE_ISOCH       = 0x1;
    /** Bulk transfer — best for large chunks with no timing requirement. */
    public static final byte EP_TYPE_BULK        = 0x2;
    /** Interrupt transfer — best for HID and low-latency small packets. */
    public static final byte EP_TYPE_INTERRUPT   = 0x3;

    /** USB class codes for the most common cases. */
    public static final byte CLASS_PER_INTERFACE = 0x00;
    public static final byte CLASS_AUDIO         = 0x01;
    public static final byte CLASS_CDC           = 0x02;
    public static final byte CLASS_HID           = 0x03;
    public static final byte CLASS_MSC           = 0x08;
    public static final byte CLASS_VENDOR        = (byte) 0xFF;

    /** Standard USB descriptor types of interest to subclasses. */
    public static final byte DESC_HID            = 0x21;
    public static final byte DESC_HID_REPORT     = 0x22;

    private final RVVMMachine machine;
    private long handle;
    private final byte[] epAddresses;

    /**
     * Attach a generic USB device. For an HID device pass
     * classDescType = {@link #DESC_HID}, a 9-byte HID class descriptor
     * as classDesc, and the raw HID report descriptor as reportDesc.
     * For non-HID devices classDescType=0 and classDesc/reportDesc
     * null works.
     */
    public USBDevice(
            RVVMMachine machine,
            short vid, short pid,
            byte classCode, byte subclass, byte protocol,
            String manufacturer, String product, String serial,
            byte[] epAddresses, byte[] epTypes, short[] epSizes,
            byte classDescType, byte[] classDesc, byte[] reportDesc) {
        this.machine = machine;
        this.epAddresses = epAddresses == null ? new byte[0] : epAddresses.clone();
        if (machine.isValid() && RVVMNative.isLoaded()) {
            this.handle = RVVMNative.usb_dev_attach(
                    machine.getPtr(),
                    vid, pid,
                    classCode, subclass, protocol,
                    manufacturer, product, serial,
                    epAddresses, epTypes, epSizes,
                    classDescType, classDesc, reportDesc);
        } else {
            this.handle = 0;
        }
    }

    /** Convenience constructor for devices without class descriptors. */
    public USBDevice(
            RVVMMachine machine,
            short vid, short pid,
            byte classCode, byte subclass, byte protocol,
            String manufacturer, String product, String serial,
            byte[] epAddresses, byte[] epTypes, short[] epSizes) {
        this(machine, vid, pid, classCode, subclass, protocol,
             manufacturer, product, serial,
             epAddresses, epTypes, epSizes,
             (byte) 0, null, null);
    }

    @Override
    public boolean isValid() {
        return machine != null && machine.isValid() && handle != 0;
    }

    public RVVMMachine getMachine() {
        return machine;
    }

    /** Endpoint addresses the device was attached with (defensive copy). */
    public byte[] getEndpointAddresses() {
        return epAddresses.clone();
    }

    /**
     * Push a packet toward the guest on an IN endpoint. Returns bytes
     * accepted. If the guest has a pending read on this endpoint it
     * completes synchronously; otherwise the packet queues for the
     * guest's next transfer.
     */
    public int feed(byte endpoint, byte[] data, int off, int len) {
        if (!isValid() || data == null) return 0;
        return RVVMNative.usb_ep_feed(handle, endpoint, data, off, len);
    }

    public int feed(byte endpoint, byte[] data) {
        return feed(endpoint, data, 0, data == null ? 0 : data.length);
    }

    /**
     * Drain one packet the guest wrote to an OUT endpoint into {@code out}.
     * Returns bytes written, or 0 if no packet was queued.
     */
    public int poll(byte endpoint, byte[] out) {
        if (!isValid() || out == null) return 0;
        return RVVMNative.usb_ep_poll(handle, endpoint, out);
    }

    /**
     * Fill {@code out[0..6]} with monotonic counters:
     * {in_fed, in_consumed, out_pushed, out_popped, in_dropped, out_dropped}.
     */
    public void stats(long[] out) {
        if (!isValid() || out == null || out.length < 6) return;
        RVVMNative.usb_dev_stats(handle, out);
    }

    @Override
    public synchronized void remove() {
        if (handle != 0) {
            RVVMNative.usb_dev_detach(handle);
            handle = 0;
        }
    }
}
