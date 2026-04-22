/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

/**
 * Convenience wrapper for HID-class USB devices (sensors, custom input
 * controllers, keyboards, mice, etc.).
 *
 * <p>You supply a raw HID report descriptor and one or more endpoints;
 * the 9-byte HID class descriptor is built automatically and embedded
 * in the configuration descriptor at attach time. The Linux
 * <code>usbhid</code> driver will bind, parse the report descriptor,
 * and create <code>/dev/input/event*</code> and/or <code>/dev/hidraw*</code>
 * nodes accordingly.
 *
 * <p>Most HID devices use a single interrupt IN endpoint for reports;
 * for those the {@link #USBHIDDevice(RVVMMachine, short, short, byte, String,
 * String, String, byte, short, byte[])} three-arg constructor with
 * <code>inEndpoint = 0x81</code>, <code>pollBytes = 8</code> (for
 * 8-byte reports) and the report descriptor is all you need.
 */
public class USBHIDDevice extends USBDevice {

    /** HID subclass: no subclass (most devices). */
    public static final byte SUBCLASS_NONE     = 0x00;
    /** HID subclass: boot interface (keyboards, mice that need BIOS support). */
    public static final byte SUBCLASS_BOOT     = 0x01;
    /** HID boot protocol: none (vendor defined). */
    public static final byte PROTOCOL_NONE     = 0x00;
    /** HID boot protocol: keyboard. */
    public static final byte PROTOCOL_KEYBOARD = 0x01;
    /** HID boot protocol: mouse. */
    public static final byte PROTOCOL_MOUSE    = 0x02;

    /**
     * Attach a single-endpoint HID-IN device.
     *
     * @param machine       target VM
     * @param vid/pid       USB vendor/product
     * @param subclass      HID subclass (usually {@link #SUBCLASS_NONE})
     * @param inEndpoint    IN endpoint address (bit 7 set; e.g. 0x81 = EP1 IN)
     * @param maxPacketSize report wMaxPacketSize in bytes (1..1024 for HS interrupt)
     * @param reportDesc    raw HID report descriptor bytes
     */
    public USBHIDDevice(RVVMMachine machine,
                        short vid, short pid,
                        byte subclass, byte protocol,
                        String manufacturer, String product, String serial,
                        byte inEndpoint, short maxPacketSize,
                        byte[] reportDesc) {
        super(machine,
              vid, pid,
              CLASS_HID, subclass, protocol,
              manufacturer, product, serial,
              new byte[]  { inEndpoint },
              new byte[]  { EP_TYPE_INTERRUPT },
              new short[] { maxPacketSize },
              DESC_HID,
              buildHidClassDescriptor(reportDesc == null ? 0 : reportDesc.length),
              reportDesc);
    }

    /**
     * Full constructor for multi-endpoint HID devices (e.g. IN + OUT
     * for keyboards with LED control).
     */
    public USBHIDDevice(RVVMMachine machine,
                        short vid, short pid,
                        byte subclass, byte protocol,
                        String manufacturer, String product, String serial,
                        byte[] epAddresses, byte[] epTypes, short[] epSizes,
                        byte[] reportDesc) {
        super(machine,
              vid, pid,
              CLASS_HID, subclass, protocol,
              manufacturer, product, serial,
              epAddresses, epTypes, epSizes,
              DESC_HID,
              buildHidClassDescriptor(reportDesc == null ? 0 : reportDesc.length),
              reportDesc);
    }

    /**
     * Build the 9-byte HID class descriptor for a given report descriptor
     * length. Uses bcdHID 1.11, no country code, one embedded descriptor
     * (the REPORT descriptor).
     */
    public static byte[] buildHidClassDescriptor(int reportDescLen) {
        byte[] b = new byte[9];
        b[0] = 9;                               /* bLength */
        b[1] = DESC_HID;                        /* bDescriptorType = 0x21 */
        b[2] = 0x11;                            /* bcdHID lo */
        b[3] = 0x01;                            /* bcdHID hi = 1.11 */
        b[4] = 0x00;                            /* bCountryCode */
        b[5] = 1;                               /* bNumDescriptors */
        b[6] = DESC_HID_REPORT;                 /* bDescriptorType2 = 0x22 */
        b[7] = (byte) (reportDescLen & 0xFF);
        b[8] = (byte) ((reportDescLen >> 8) & 0xFF);
        return b;
    }
}
