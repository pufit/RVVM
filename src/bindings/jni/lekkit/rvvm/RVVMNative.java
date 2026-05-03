/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

package lekkit.rvvm;

import java.nio.ByteBuffer;

// Regenerate C prototypes: $ javac -h . RVVMNative.java

public class RVVMNative {
    // Do not crash the JVM if we failed to load native lib
    public static boolean loaded = false;

    private static void checkABI() {
        if (check_abi(9)) {
            loaded = true;
        } else {
            System.out.println("ERROR: Invalid librvvm ABI version! Please update your JNI bindings!");
        }
    }

    // Manually load librvvm
    public static boolean loadLib(String path) {
        if (loaded) return true;
        try {
            System.load(path);
            checkABI();
        } catch (Throwable e) {
            System.out.println("ERROR: Failed to load librvvm: " + e.toString());
        }
        return loaded;
    }

    static {
        try {
            System.loadLibrary("rvvm");
            checkABI();
        } catch (Throwable e) {
            System.out.println("INFO: Failed to load system-wide librvvm: " + e.toString());
        }
    }

    public static boolean isLoaded() {
        return loaded;
    }

    //
    // Common RVVM API functions
    //

    public static native boolean check_abi(int abi);

    ///
    // RVVM Machine Management API
    //

    public static native long    create_machine(long mem_size, int smp, String isa);
    public static native void    set_cmdline(long machine, String cmdline);
    public static native void    append_cmdline(long machine, String cmdline);
    public static native boolean load_bootrom(long machine, String path);
    public static native boolean load_kernel(long machine, String path);
    public static native boolean load_dtb(long machine, String path);
    public static native boolean dump_dtb(long machine, String path);
    public static native long    get_opt(long machine, int opt);
    public static native void    set_opt(long machine, int opt, long val);
    public static native boolean start_machine(long machine);
    public static native boolean pause_machine(long machine);
    public static native boolean reset_machine(long machine, boolean reset);
    public static native boolean machine_running(long machine);
    public static native boolean machine_powered(long machine);
    public static native void    free_machine(long machine);
    public static native void    run_eventloop();

    //
    // RVVM Device API
    //

    public static native ByteBuffer get_dma_buf(long machine, long addr, long size);
    public static native long       mmio_zone_auto(long machine, long addr, long size);

    public static native void remove_mmio(long mmio_dev);

    // Get wired interrupt controller
    public static native long get_intc(long machine);
    public static native void set_intc(long machine, long intc);

    // Get PCI bus (root complex)
    public static native long get_pci_bus(long machine);
    public static native void set_pci_bus(long machine, long pci_bus);

    // Get I2C bus
    public static native long get_i2c_bus(long machine);
    public static native void set_i2c_bus(long machine, long i2c_bus);

    //
    // RVVM Devices
    //

    public static native void riscv_clint_init_auto(long machine);
    public static native void riscv_imsic_init_auto(long machine);

    // Returns wired IRQ controller handle
    public static native long riscv_plic_init_auto(long machine);
    public static native long riscv_aplic_init_auto(long machine);

    // Returns PCI bus handle
    public static native long pci_bus_init_auto(long machine);

    // Returns I2C bus handle
    public static native long i2c_bus_init_auto(long machine);

    // Returns TAP device handle
    public static native long tap_user_open();

    // Returns MMIO handle
    public static native long syscon_init_auto(long machine);
    public static native long rtc_goldfish_init_auto(long machine);
    public static native long rtc_ds1742_init_auto(long machine);
    public static native long ns16550a_init_auto(long machine);
    public static native long gpio_sifive_init_auto(long machine, long gpio);
    public static native long mtd_physmap_init_auto(long machine, String image_path, boolean rw);
    public static native long framebuffer_init_auto(long machine, ByteBuffer[] fb, int x, int y, int bpp);

    // Returns PCI device handle
    public static native long rtl8169_init(long pci_bus, long tap);
    public static native long nvme_init(long pci_bus, String image_path, boolean rw);
    public static native long sound_hda_init_auto(long machine);

    /**
     * Attach an HDA PCI device that stages PCM into a native ring buffer.
     * Java polls the buffer on its own schedule via {@link #sound_hda_poll}.
     *
     * <p>Design note: we'd prefer a direct callback into Java, but on
     * JDK 21 / macOS arm64 {@code AttachCurrentThread(AsDaemon)} fails
     * with {@code JNI_ERR} when called from RVVM's pthread-created stream
     * worker. Staging through a native ring and polling from a
     * JVM-native thread sidesteps the attach problem entirely.
     *
     * <p>Ring capacity: 1 MiB. Cushion in seconds depends on the rate the
     * guest configures (the codec advertises 44.1 / 48 / 88.2 / 96 kHz at
     * 16-bit mono): ~10.9 s at the common-case 48 kHz, ~5.5 s at 96 kHz.
     * Overflow drops oldest bytes (latency over completeness).
     *
     * @param machine       Pointer to the RVVM machine.
     * @param pciDevOut     One-element long[] receiving the
     *                      {@code pci_dev_t*} for {@link PCIDevice#setPCIHandle}.
     * @return Opaque handle to the native ring (pass to {@link #sound_hda_poll}
     *         and {@link #sound_hda_stats}), or 0 on failure.
     */
    public static native long sound_hda_init_with_ring(long machine, long[] pciDevOut);

    /**
     * Drain up to {@code out.length} PCM bytes from the ring into the
     * supplied Java array. Returns the number of bytes actually read.
     * Safe to call from any JVM-native thread — takes a mutex internally.
     *
     * @param sinkHandle Ring handle from {@link #sound_hda_init_with_ring}.
     * @param out        Destination byte buffer.
     * @return Number of bytes written to {@code out}.
     */
    public static native int sound_hda_poll(long sinkHandle, byte[] out);

    /**
     * Query a monotonic counter from the native ring.
     *
     * @param sinkHandle Ring handle.
     * @param which      0=total pushed bytes, 1=total popped, 2=dropped, 3=current occupancy.
     */
    public static native long sound_hda_stats(long sinkHandle, int which);

    // Returns HID mouse handle
    public static native long hid_mouse_init_auto(long machine);

    // Returns HID keyboard handle
    public static native long hid_keyboard_init_auto(long machine);

    // Returns GPIO handle
    public static native long gpio_dev_create();

    // Takes PCI device hande
    public static native void pci_remove_device(long dev);

    // Takes GPIO handle
    public static native void    gpio_dev_free(long gpio);
    public static native int     gpio_read_pins(long gpio, int off);
    public static native boolean gpio_write_pins(long gpio, int off, int pins);

    // Takes HID mouse handle
    public static native void hid_mouse_resolution(long mouse, int x, int y);
    public static native void hid_mouse_place(long mouse, int x, int y);
    public static native void hid_mouse_move(long mouse, int x, int y);
    public static native void hid_mouse_press(long mouse, byte btns);
    public static native void hid_mouse_release(long mouse, byte btns);
    public static native void hid_mouse_scroll(long mouse, int offset);

    // Takes HID keyboard handle
    public static native void hid_keyboard_press(long kb, byte key);
    public static native void hid_keyboard_release(long kb, byte key);

    //
    // NS16550A JNI bridge
    //
    // Attaches a second UART backed by two 64 KiB ring buffers instead of
    // stdio. Java drains guest TX with poll() and injects guest RX with
    // feed(). The bridge handle is freed automatically when the owning
    // machine is freed — no explicit remove call needed.
    //

    // Returns a bridge handle (opaque), or 0 on failure.
    public static native long ns16550a_bridge_init(long machine);

    // Drains up to out.length bytes of guest TX into `out`. Returns count written.
    public static native int  ns16550a_bridge_poll(long handle, byte[] out);

    // Feeds up to in.length bytes into guest RX. Returns count accepted.
    public static native int  ns16550a_bridge_feed(long handle, byte[] in);

    // Fills a long[5] with {pushed, popped, fed, consumed, dropped}.
    public static native void ns16550a_bridge_stats(long handle, long[] out);

    //
    // Parport JNI bridge
    //
    // Attaches a NetMos 9900 PCI parallel port whose forward writes
    // (Centronics strobes from the guest) land in a 64 KiB ring the JVM
    // drains via poll(). Reverse data goes back to the guest via feed(),
    // which delivers bytes through the IEEE 1284 nibble-mode handshake
    // when the guest issues PPNEGOT + read on /dev/parport0.
    //

    // Returns a bridge handle (opaque), or 0 on failure.
    public static native long parport_bridge_init(long machine);

    // Drains up to out.length bytes of forward data into `out`. Returns count.
    public static native int  parport_bridge_poll(long handle, byte[] out);

    // Feeds up to in.length bytes into the reverse channel. Returns
    // count accepted; remainder should be retried after the guest has
    // had a chance to read.
    public static native int  parport_bridge_feed(long handle, byte[] in);

    // Fills a long[5] with {pushed, popped, fed, accepted, tx_dropped}.
    public static native void parport_bridge_stats(long handle, long[] out);

    //
    // Exar XR17V35x PCIe combo serial JNI bridge
    //
    // Attaches an Exar PCIe combo serial card with 2/4/8/12/16 ports
    // (XR17V352/V354/V358/V4358/V8358). Returns a long[] of per-port
    // bridge handles, each compatible with ns16550a_bridge_poll/feed/stats.
    //
    // REQUIREMENT: the machine must have a PCI bus attached
    // (RVVMNative.pci_bus_init_auto, or whatever sets one). Returns null
    // if the machine has no PCI bus, the port count is invalid, or PCI
    // device attach fails.
    //

    public static native long[] exar_pci_bridge_init(long machine, int n_ports);

    //
    // CAN node JNI bridge
    //
    // Attaches a virtual CAN node that the guest sees through an MCP2515
    // SPI controller (Linux's mcp251x driver binds; can0 appears). Frames
    // the guest broadcasts on can0 land in a 256-frame ring the JVM drains
    // via poll(); frames the JVM pushes via feed() are broadcast on the
    // bus and surface to the guest via the MCP2515's RX path.
    //
    // Frame wire format on disk (16 bytes, fixed stride):
    //   off 0..3   uint32 LE   can_id (with CAN_EFF/RTR/ERR flag bits)
    //   off 4      uint8       dlc (0..8)
    //   off 5..7   reserved (zero)
    //   off 8..15  data[0..7]
    //
    // The bridge handle is freed automatically when its owning machine is
    // freed.
    //

    // Returns a bridge handle (opaque), or 0 on failure.
    public static native long can_node_bridge_init(long machine);

    // Drains queued guest frames into out (length must be multiple of 16).
    // Returns the number of frames actually drained.
    public static native int  can_node_bridge_poll(long handle, byte[] out);

    // Broadcasts n_frames packed in `in` onto the bus. Returns the count
    // actually broadcast (== n_frames unless `in` is too short).
    public static native int  can_node_bridge_feed(long handle, byte[] in, int n_frames);

    // Fills a long[5] with {pushed, popped, fed, rx_dropped, ring_occupancy}
    // (all in bytes; divide by 16 for frame counts).
    public static native void can_node_bridge_stats(long handle, long[] out);

    //
    // I2C sensor JNI bridge — read-shadow register file
    //
    // Attaches a 256-byte shadow-register I2C slave the guest can read like
    // any sensor / EEPROM / IO expander. Java updates the shadow at any time
    // via set/setBulk; the next guest read at that register address sees
    // the new value. Guest writes are captured into a (register, value)
    // event ring the JVM drains via pollWrites on tick.
    //
    // No C→Java upcalls — read latency is one memcpy from the shadow.
    // Requires the machine to have an I2C bus already attached
    // (i2c_bus_init_auto).
    //

    // addr=0 picks an automatic address. Returns bridge handle, or 0 on failure.
    public static native long i2c_sensor_bridge_init(long machine, int addr);

    // Update one shadow register; reg masked to 0..255.
    public static native void i2c_sensor_bridge_set(long handle, int reg, int value);

    // Bulk shadow update: copy data into shadow[off..off+data.length), wrapping at 256.
    public static native int  i2c_sensor_bridge_set_bulk(long handle, int off, byte[] data);

    // Drain queued (addr, value) write events; out length must be even.
    // Returns number of (addr, value) PAIRS drained (not bytes).
    public static native int  i2c_sensor_bridge_poll_writes(long handle, byte[] out);

    // Fills long[4] with {total_reads, total_writes, writes_dropped, ring_occupancy}.
    public static native void i2c_sensor_bridge_stats(long handle, long[] out);

    //
    // USB device bridge
    //
    // Attach a USB 2.0 High-Speed device whose control/xfer callbacks
    // are serviced by C (descriptor synthesis + per-endpoint packet
    // queues). Java code drives endpoints with the same polled-mailbox
    // pattern as the NS16550A and HDA bridges: feed() pushes packets
    // toward the guest on IN endpoints; poll() drains packets the
    // guest wrote on OUT endpoints.
    //
    // Endpoint address encoding follows USB convention:
    //   bit 7   = direction (1 = IN / device-to-host)
    //   bits 3..0 = endpoint number (1..15; 0 is control EP0, handled
    //               internally and not exposed via feed/poll).
    //
    // For HID devices, pass class_desc_type=0x21 (USB HID descriptor
    // type), class_desc = 9-byte HID class descriptor to embed in the
    // configuration descriptor, and report_desc = the raw HID report
    // descriptor to serve on GET_DESCRIPTOR(REPORT). For non-HID
    // classes either pass class_desc null or provide whatever the
    // class spec mandates; report_desc can be null.
    //

    // Returns a usb device handle (opaque), or 0 on failure.
    public static native long usb_dev_attach(
            long machine,
            short vid,
            short pid,
            byte  classCode,
            byte  subclass,
            byte  protocol,
            String manufacturer,
            String product,
            String serial,
            byte[]  epAddresses,
            byte[]  epTypes,   /* 0=control 1=iso 2=bulk 3=interrupt */
            short[] epSizes,
            byte    classDescType,  /* 0x21 for HID, 0 to suppress */
            byte[]  classDesc,      /* embedded in config descriptor; null for none */
            byte[]  reportDesc      /* served on GET_DESCRIPTOR(type=0x22); null for none */
    );

    public static native void usb_dev_detach(long handle);

    // Push one packet toward the guest on an IN endpoint. Returns
    // bytes accepted (may drop oldest if the queue is full).
    public static native int usb_ep_feed(long handle, byte ep, byte[] data, int off, int len);

    // Drain one packet the guest wrote to an OUT endpoint. Returns
    // bytes written into out, 0 if empty.
    public static native int usb_ep_poll(long handle, byte ep, byte[] out);

    // Fill a long[6] with {in_fed, in_consumed, out_pushed, out_popped,
    // in_dropped, out_dropped}.
    public static native void usb_dev_stats(long handle, long[] out);

    //
    // SPI buffer JNI bridge — memory-buffer slave with cursor reset on CS
    //
    // Attaches a SPI slave behind a fresh SiFive SPI controller. Java
    // provides a read buffer (variable size, set at attach time and
    // replaceable via setBuffer); guest CS assertions reset a cursor to 0
    // and subsequent transfers shift bytes out of the buffer at cursor++,
    // wrapping at buffer end. Bytes the guest shifts in on MOSI are
    // captured into a 4 KiB write ring the JVM drains via poll on tick.
    //
    // For pure write-sink slaves (display blasting, no MISO response
    // expected), set the buffer to a single 0xFF byte and ignore poll's
    // MISO data — transfers will return 0xFF on every byte and the slave
    // looks like an open MISO line.
    //

    // bufBytes sets initial read-buffer size in bytes (clamped to ≥ 1).
    // Returns bridge handle, or 0 on failure.
    public static native long spi_buffer_bridge_init(long machine, int bufBytes);

    // Replace the read buffer wholesale; cursor reset to 0.
    public static native void spi_buffer_bridge_set_buffer(long handle, byte[] data);

    // Update one byte in-place; out-of-range offsets are no-ops; cursor untouched.
    public static native void spi_buffer_bridge_set_byte(long handle, int off, int value);

    // Drain queued MOSI bytes the guest shifted into the slave.
    public static native int  spi_buffer_bridge_poll(long handle, byte[] out);

    // Fills long[4] with {total_transfers, writes_dropped, ring_occupancy, cursor}.
    public static native void spi_buffer_bridge_stats(long handle, long[] out);
}
