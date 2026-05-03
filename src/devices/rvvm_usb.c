/*
rvvm_usb.c - RVVM USB Core
Copyright (C) 2026  Sol Astrius <me@danielsol.dev>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * Backing for include/rvvm/rvvm_usb.h: USB bus + device lifecycle,
 * transfer forwarding, async completion plumbing, and a standard
 * Chapter 9 request synthesiser (rvvm_usb_std_control) that serves
 * descriptors and the common device/interface/endpoint requests
 * from rvvm_usb_dev_prod_t.
 *
 * Scope: USB 2.0 High Speed. The core is speed-agnostic internally
 * — wMaxPacketSize comes from the device's endpoint list — but
 * synthesised device descriptors report bcdUSB = 0x0200 and
 * bMaxPacketSize0 = 64 (HS control packet).
 *
 * Devices that need class-specific control behaviour (HID, MSC, CDC)
 * provide their own type->control; for standard requests they can
 * delegate to rvvm_usb_std_control and only intercept class/vendor
 * requests themselves. If type->control is NULL, the core installs
 * rvvm_usb_std_control implicitly.
 *
 * Threading: the bus lock is short-held; it guards port table mutation
 * only. Transfer callbacks (xfer / control / cancel / reset / suspend)
 * run outside the lock — devices provide their own synchronisation.
 */

#include <rvvm/rvvm_usb.h>

#include "utils.h"
#include "mem_ops.h"
#include "spinlock.h"
#include "atomics.h"

#include <string.h>

/*
 * Root hub owns port 0; devices occupy ports 1..RVVM_USB_MAX_PORTS.
 * Eight is plenty — XHCI root hubs usually expose 4-8 per speed.
 */
#define RVVM_USB_MAX_PORTS 8

/*
 * USB standard request codes (bRequest)
 */
#define USB_REQ_GET_STATUS        0x00
#define USB_REQ_CLEAR_FEATURE     0x01
#define USB_REQ_SET_FEATURE       0x03
#define USB_REQ_SET_ADDRESS       0x05
#define USB_REQ_GET_DESCRIPTOR    0x06
#define USB_REQ_SET_DESCRIPTOR    0x07
#define USB_REQ_GET_CONFIGURATION 0x08
#define USB_REQ_SET_CONFIGURATION 0x09
#define USB_REQ_GET_INTERFACE     0x0A
#define USB_REQ_SET_INTERFACE     0x0B
#define USB_REQ_SYNCH_FRAME       0x0C

/*
 * Descriptor type (high byte of wValue for GET_DESCRIPTOR)
 */
#define USB_DT_DEVICE                     0x01
#define USB_DT_CONFIGURATION              0x02
#define USB_DT_STRING                     0x03
#define USB_DT_INTERFACE                  0x04
#define USB_DT_ENDPOINT                   0x05
#define USB_DT_DEVICE_QUALIFIER           0x06
#define USB_DT_OTHER_SPEED_CONFIGURATION  0x07

/*
 * bmRequestType decode
 */
#define USB_DIR_MASK           0x80u
#define USB_DIR_IN             0x80u
#define USB_TYPE_MASK          0x60u
#define USB_TYPE_STANDARD      0x00u
#define USB_RECIP_MASK         0x1Fu
#define USB_RECIP_DEVICE       0x00u
#define USB_RECIP_INTERFACE    0x01u
#define USB_RECIP_ENDPOINT     0x02u

/*
 * Synthesized device descriptor constants
 */
#define USB_BCD_USB_20   0x0200u
#define USB_EP0_HS_MPS   64u
#define USB_LANG_ID_ENUS 0x0409u

struct rvvm_usb_bus {
    const rvvm_usb_bus_cb_t* cb;
    void*                    data;
    spinlock_t               lock;
    rvvm_usb_dev_t*          ports[RVVM_USB_MAX_PORTS + 1]; /* 1-indexed */
};

struct rvvm_usb_dev {
    rvvm_usb_bus_t*            bus;
    const rvvm_usb_dev_type_t* type;
    void*                      data;
    uint32_t                   port;

    /*
     * After rvvm_usb_dev_free sets `freeing`, the core drops any late
     * rvvm_usb_dev_xfer_done calls on the floor. Devices are supposed
     * to honour cancel() + stop reporting — this guard is defence in
     * depth for races between controller cancel and device completion.
     */
    uint32_t                   freeing;

    /* SET_ADDRESS is a no-op for XHCI-driven buses but we honour GET/SET_CONFIGURATION. */
    uint8_t                    config_value;
    uint8_t                    _pad[3];
};

/*
 * ---------------------------------------------------------------------
 * Bus / device lifecycle
 * ---------------------------------------------------------------------
 */

RVVM_PUBLIC rvvm_usb_bus_t* rvvm_usb_bus_init(const rvvm_usb_bus_cb_t* cb, void* data)
{
    rvvm_usb_bus_t* bus = safe_new_obj(rvvm_usb_bus_t);
    bus->cb   = cb;
    bus->data = data;
    spin_init(&bus->lock);
    return bus;
}

RVVM_PUBLIC void rvvm_usb_bus_free(rvvm_usb_bus_t* bus)
{
    if (bus == NULL) return;

    /*
     * Snap the port table under the lock, then tear each device down
     * with the lock released so type->free can take its own locks
     * (e.g. JNI device ring locks) without ordering hazards.
     */
    rvvm_usb_dev_t* victims[RVVM_USB_MAX_PORTS + 1];
    scoped_spin_lock (&bus->lock) {
        memcpy(victims, bus->ports, sizeof(victims));
        memset(bus->ports, 0, sizeof(bus->ports));
    }
    for (size_t i = 1; i <= RVVM_USB_MAX_PORTS; ++i) {
        if (victims[i]) {
            rvvm_usb_dev_free(victims[i]);
        }
    }
    free(bus);
}

RVVM_PUBLIC void* rvvm_usb_bus_get_data(rvvm_usb_bus_t* bus)
{
    return bus ? bus->data : NULL;
}

RVVM_PUBLIC rvvm_usb_dev_t* rvvm_usb_bus_port_dev(rvvm_usb_bus_t* bus, uint32_t port)
{
    if (bus == NULL || port == 0 || port > RVVM_USB_MAX_PORTS) return NULL;
    rvvm_usb_dev_t* ret;
    scoped_spin_read_lock (&bus->lock) {
        ret = bus->ports[port];
    }
    return ret;
}

RVVM_PUBLIC rvvm_usb_dev_t* rvvm_usb_dev_init_at(rvvm_usb_bus_t*            bus,
                                                 const rvvm_usb_dev_type_t* type,
                                                 void*                      data,
                                                 uint32_t                   port)
{
    if (bus == NULL || type == NULL) return NULL;

    rvvm_usb_dev_t* dev = safe_new_obj(rvvm_usb_dev_t);
    dev->bus  = bus;
    dev->type = type;
    dev->data = data;

    uint32_t claimed = 0;
    scoped_spin_lock (&bus->lock) {
        if (port == RVVM_USB_PORT_ANY) {
            for (uint32_t i = 1; i <= RVVM_USB_MAX_PORTS; ++i) {
                if (bus->ports[i] == NULL) {
                    claimed = i;
                    break;
                }
            }
        } else if (port >= 1 && port <= RVVM_USB_MAX_PORTS && bus->ports[port] == NULL) {
            claimed = port;
        }
        if (claimed) {
            dev->port = claimed;
            bus->ports[claimed] = dev;
        }
    }

    if (claimed == 0) {
        /*
         * Header contract: on failure invoke the device's cleanup
         * callback, so the caller's private data is released.
         */
        if (type->free) type->free(dev);
        free(dev);
        return NULL;
    }

    /*
     * Tell the controller a device just appeared. port_update is
     * expected to post a Port Status Change Event on XHCI-like
     * controllers; the guest then drives address assignment and
     * enumeration through EP0.
     */
    if (bus->cb && bus->cb->port_update) {
        bus->cb->port_update(bus, claimed, true);
    }
    return dev;
}

RVVM_PUBLIC void rvvm_usb_dev_free(rvvm_usb_dev_t* dev)
{
    if (dev == NULL) return;

    /*
     * Latch `freeing` first so any in-flight completion racing with
     * us gets dropped instead of calling back into a half-freed device.
     */
    atomic_store_uint32(&dev->freeing, 1);

    rvvm_usb_bus_t* bus  = dev->bus;
    uint32_t        port = dev->port;

    if (bus) {
        scoped_spin_lock (&bus->lock) {
            if (port >= 1 && port <= RVVM_USB_MAX_PORTS && bus->ports[port] == dev) {
                bus->ports[port] = NULL;
            }
        }
        /*
         * Controller aborts pending transfers on disconnect. The
         * contract is: by the time port_update(false) returns, no more
         * xfer_done callbacks are outstanding for this device — the
         * controller has walked its transfer rings and cancelled them
         * via rvvm_usb_dev_cancel.
         */
        if (bus->cb && bus->cb->port_update) {
            bus->cb->port_update(bus, port, false);
        }
    }

    if (dev->type && dev->type->free) {
        dev->type->free(dev);
    }
    free(dev);
}

RVVM_PUBLIC void* rvvm_usb_dev_get_data(rvvm_usb_dev_t* dev)
{
    return dev ? dev->data : NULL;
}

/*
 * ---------------------------------------------------------------------
 * Transfer forwarding
 * ---------------------------------------------------------------------
 *
 * These thin wrappers exist so controllers go through a single choke
 * point and don't deref device vtables directly. That makes life-cycle
 * guarantees easier to reason about (e.g. freeing() guard) and lets us
 * expand later with logging, throttling, or snapshot hooks.
 */

RVVM_PUBLIC int32_t rvvm_usb_dev_control(rvvm_usb_dev_t* dev,
                                         const void*     setup,
                                         void*           data,
                                         size_t          size)
{
    if (dev == NULL || dev->type == NULL || setup == NULL) return RVVM_USB_XFER_ERROR;
    if (atomic_load_uint32_relax(&dev->freeing)) return RVVM_USB_XFER_ERROR;

    if (dev->type->control) {
        return dev->type->control(dev, setup, data, size);
    }
    /*
     * No device-level control callback — fall back to Chapter 9
     * synthesis driven by prod. Devices with no prod and no control
     * can't answer EP0; return a stall so enumeration fails cleanly.
     */
    if (dev->type->prod == NULL) return RVVM_USB_XFER_STALL;
    return rvvm_usb_std_control(dev, setup, data, size);
}

RVVM_PUBLIC int32_t rvvm_usb_dev_xfer(rvvm_usb_dev_t* dev,
                                      uint8_t         ep,
                                      void*           data,
                                      size_t          size)
{
    if (dev == NULL || dev->type == NULL) return RVVM_USB_XFER_ERROR;
    if (atomic_load_uint32_relax(&dev->freeing)) return RVVM_USB_XFER_ERROR;
    if (dev->type->xfer == NULL) return RVVM_USB_XFER_STALL;
    return dev->type->xfer(dev, ep, data, size);
}

RVVM_PUBLIC void rvvm_usb_dev_cancel(rvvm_usb_dev_t* dev, uint8_t ep)
{
    if (dev == NULL || dev->type == NULL || dev->type->cancel == NULL) return;
    dev->type->cancel(dev, ep);
}

RVVM_PUBLIC void rvvm_usb_dev_reset(rvvm_usb_dev_t* dev)
{
    if (dev == NULL || dev->type == NULL) return;
    dev->config_value = 0;
    if (dev->type->reset) dev->type->reset(dev);
}

RVVM_PUBLIC void rvvm_usb_dev_suspend(rvvm_usb_dev_t* dev, rvvm_snapshot_t* snap, bool resume)
{
    if (dev == NULL || dev->type == NULL || dev->type->suspend == NULL) return;
    dev->type->suspend(dev, snap, resume);
}

RVVM_PUBLIC void rvvm_usb_dev_xfer_done(rvvm_usb_dev_t* dev, uint8_t ep, int32_t ret)
{
    if (dev == NULL || dev->bus == NULL) return;
    /*
     * Drop completions that land after rvvm_usb_dev_free has started
     * tearing things down. Device authors are supposed to stop
     * reporting after cancel(), but the controller may fan out
     * xfer_done across threads so we can't rely on strict ordering.
     */
    if (atomic_load_uint32_relax(&dev->freeing)) return;

    const rvvm_usb_bus_cb_t* cb = dev->bus->cb;
    if (cb && cb->xfer_done) {
        cb->xfer_done(dev->bus, dev, ep, ret);
    }
}

/*
 * ---------------------------------------------------------------------
 * Chapter 9 standard request synthesis
 * ---------------------------------------------------------------------
 *
 * Public helper — exposed so custom control handlers can handle
 * class/vendor requests themselves and delegate the standard cases
 * back here instead of re-implementing descriptor serialisation.
 */

static size_t usb_copy_clipped(void* dst, size_t dst_size, const void* src, size_t src_size)
{
    size_t n = src_size < dst_size ? src_size : dst_size;
    if (n && dst && src) memcpy(dst, src, n);
    return n;
}

/*
 * Encode a UTF-8 string into a USB STRING descriptor (little-endian
 * UTF-16). Guest requests a max length via wLength — we truncate at
 * the boundary, never emit a split surrogate.
 */
static size_t usb_encode_string_desc(const char* utf8, uint8_t* dst, size_t dst_size)
{
    if (dst_size < 2) return 0;

    /*
     * Leave room for the 2-byte header and round the remaining
     * budget down to an even byte count — UTF-16 is 2 or 4 bytes
     * per codepoint.
     */
    size_t budget = (dst_size - 2) & ~(size_t)1;
    uint8_t* cursor = dst + 2;
    size_t written = 0;

    while (*utf8 && written + 2 <= budget) {
        uint32_t cp;
        uint8_t c = (uint8_t)*utf8++;

        if (c < 0x80) {
            cp = c;
        } else if ((c & 0xE0) == 0xC0 && (utf8[0] & 0xC0) == 0x80) {
            cp = ((uint32_t)(c & 0x1F) << 6) | (utf8[0] & 0x3F);
            utf8 += 1;
        } else if ((c & 0xF0) == 0xE0 && (utf8[0] & 0xC0) == 0x80 && (utf8[1] & 0xC0) == 0x80) {
            cp = ((uint32_t)(c & 0x0F) << 12) | ((uint32_t)(utf8[0] & 0x3F) << 6) | (utf8[1] & 0x3F);
            utf8 += 2;
        } else if ((c & 0xF8) == 0xF0 && (utf8[0] & 0xC0) == 0x80 && (utf8[1] & 0xC0) == 0x80 && (utf8[2] & 0xC0) == 0x80) {
            cp = ((uint32_t)(c & 0x07) << 18) | ((uint32_t)(utf8[0] & 0x3F) << 12) | ((uint32_t)(utf8[1] & 0x3F) << 6) | (utf8[2] & 0x3F);
            utf8 += 3;
        } else {
            cp = 0xFFFD;  /* replacement */
        }

        if (cp <= 0xFFFF) {
            write_uint16_le(cursor + written, (uint16_t)cp);
            written += 2;
        } else if (written + 4 <= budget) {
            uint32_t u = cp - 0x10000;
            write_uint16_le(cursor + written,     0xD800 | (u >> 10));
            write_uint16_le(cursor + written + 2, 0xDC00 | (u & 0x3FF));
            written += 4;
        } else {
            break;  /* not enough budget for a surrogate pair */
        }
    }

    size_t total = written + 2;
    dst[0] = (uint8_t)total;
    dst[1] = USB_DT_STRING;
    return total;
}

/* Emit LANGID descriptor (string index 0) — fixed to en-US. */
static size_t usb_emit_langid(uint8_t* dst, size_t dst_size)
{
    if (dst_size < 4) return 0;
    dst[0] = 4;
    dst[1] = USB_DT_STRING;
    write_uint16_le(dst + 2, USB_LANG_ID_ENUS);
    return 4;
}

/*
 * Count endpoints and total config-descriptor length for a device.
 * Layout (our synthesis): [cfg 9][intf 9][ep 7 * n]
 */
static size_t usb_config_total_len(const rvvm_usb_dev_prod_t* prod)
{
    return 9 + 9 + 7 * prod->ep_size;
}

/*
 * Serialise the device's full configuration descriptor — config header
 * + single interface + all endpoints — into `out`. Clips to out_size.
 *
 * The configuration descriptor supports multiple interfaces and
 * alt-settings; we synthesise the single-interface case because that's
 * all rvvm_usb_dev_prod_t can express. Devices needing multi-interface
 * layouts supply their own control callback and serialise themselves.
 */
static size_t usb_emit_config_desc(const rvvm_usb_dev_prod_t* prod, uint8_t* out, size_t out_size)
{
    uint8_t buf[9 + 9 + 7 * 16];  /* 16 eps max — plenty, USB allows up to 30 per interface */

    size_t  total = usb_config_total_len(prod);
    if (total > sizeof(buf)) total = sizeof(buf);

    /* Configuration descriptor (bmAttributes 0xC0 = self-powered, no remote wakeup). */
    buf[0] = 9;
    buf[1] = USB_DT_CONFIGURATION;
    write_uint16_le(buf + 2, (uint16_t)total);
    buf[4] = 1;  /* bNumInterfaces */
    buf[5] = 1;  /* bConfigurationValue */
    buf[6] = 0;  /* iConfiguration */
    buf[7] = 0xC0;
    buf[8] = 0;  /* bMaxPower (mA/2); 0 is "self-powered, no bus current needed" */

    /* Interface descriptor (single, alt 0). */
    buf[9]  = 9;
    buf[10] = USB_DT_INTERFACE;
    buf[11] = 0;  /* bInterfaceNumber */
    buf[12] = 0;  /* bAlternateSetting */
    buf[13] = (uint8_t)prod->ep_size;
    buf[14] = prod->class_code;
    buf[15] = prod->subclass;
    buf[16] = prod->protocol;
    buf[17] = 0;  /* iInterface */

    /* Endpoint descriptors. */
    size_t ep_count = prod->ep_size;
    if (ep_count > 16) ep_count = 16;
    for (size_t i = 0; i < ep_count; ++i) {
        uint8_t* ep = buf + 18 + 7 * i;
        ep[0] = 7;
        ep[1] = USB_DT_ENDPOINT;
        ep[2] = prod->ep_list[i].addr;
        ep[3] = prod->ep_list[i].type & 0x03;
        write_uint16_le(ep + 4, prod->ep_list[i].size);
        /*
         * bInterval: interrupt endpoints at HS use 2^(n-1) microframes
         * (125us units). Default to a 125us period (n=1 → 1 uframe)
         * for interrupt/isoch; bulk/control ignore it.
         */
        ep[6] = (prod->ep_list[i].type == RVVM_USB_EP_INTERRUPT || prod->ep_list[i].type == RVVM_USB_EP_ISOCH) ? 1 : 0;
    }

    return usb_copy_clipped(out, out_size, buf, total);
}

/*
 * Serialise the 18-byte USB 2.0 device descriptor.
 */
static size_t usb_emit_device_desc(const rvvm_usb_dev_prod_t* prod, uint8_t* out, size_t out_size)
{
    uint8_t desc[18];
    desc[0]  = 18;
    desc[1]  = USB_DT_DEVICE;
    write_uint16_le(desc + 2, USB_BCD_USB_20);
    desc[4]  = prod->class_code;
    desc[5]  = prod->subclass;
    desc[6]  = prod->protocol;
    desc[7]  = USB_EP0_HS_MPS;
    write_uint16_le(desc + 8,  prod->vendor_id);
    write_uint16_le(desc + 10, prod->product_id);
    write_uint16_le(desc + 12, 0x0100);  /* bcdDevice */
    /* String indices 1..3 are the manufacturer/product/serial slots. */
    desc[14] = prod->manufacturer ? 1 : 0;
    desc[15] = prod->product      ? 2 : 0;
    desc[16] = prod->serial       ? 3 : 0;
    desc[17] = 1;  /* bNumConfigurations */
    return usb_copy_clipped(out, out_size, desc, sizeof(desc));
}

RVVM_PUBLIC int32_t rvvm_usb_std_control(rvvm_usb_dev_t* dev,
                                         const void*     setup_buf,
                                         void*           data,
                                         size_t          size)
{
    if (dev == NULL || dev->type == NULL || dev->type->prod == NULL || setup_buf == NULL) {
        return RVVM_USB_XFER_STALL;
    }

    const uint8_t* setup    = (const uint8_t*)setup_buf;
    uint8_t  bmRequestType  = setup[0];
    uint8_t  bRequest       = setup[1];
    uint16_t wValue         = read_uint16_le(setup + 2);
    uint16_t wIndex         = read_uint16_le(setup + 4);
    uint16_t wLength        = read_uint16_le(setup + 6);
    bool     dir_in         = (bmRequestType & USB_DIR_MASK) == USB_DIR_IN;
    uint8_t  recip          = bmRequestType & USB_RECIP_MASK;

    /* Class/vendor requests aren't our job — the caller should handle those. */
    if ((bmRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD) {
        return RVVM_USB_XFER_STALL;
    }

    const rvvm_usb_dev_prod_t* prod = dev->type->prod;

    /*
     * Clip size to wLength. Guest may supply a larger IN buffer than
     * it wants consumed; obeying wLength keeps us from scribbling past
     * the requested length even if the controller hands us extra.
     */
    if (size > wLength) size = wLength;

    switch (bRequest) {
        case USB_REQ_GET_STATUS:
            if (!dir_in || size < 2) return RVVM_USB_XFER_STALL;
            /* bit 0: self-powered (1) for device, 0 for intf/ep. Bit 1 remote wakeup: 0. */
            write_uint16_le(data, recip == USB_RECIP_DEVICE ? 1 : 0);
            return 2;

        case USB_REQ_CLEAR_FEATURE:
        case USB_REQ_SET_FEATURE:
            /* Ignore DEVICE_REMOTE_WAKEUP / ENDPOINT_HALT at this layer. */
            return 0;

        case USB_REQ_SET_ADDRESS:
            /*
             * XHCI owns device addressing; by the time this SETUP
             * would reach us the controller has already stuffed the
             * slot context. Accept to keep a hypothetical non-XHCI
             * caller happy.
             */
            return 0;

        case USB_REQ_GET_DESCRIPTOR: {
            uint8_t dtype = (uint8_t)(wValue >> 8);
            uint8_t dindex = (uint8_t)(wValue & 0xFF);
            if (!dir_in) return RVVM_USB_XFER_STALL;

            switch (dtype) {
                case USB_DT_DEVICE:
                    return (int32_t)usb_emit_device_desc(prod, data, size);

                case USB_DT_CONFIGURATION:
                    if (dindex != 0) return RVVM_USB_XFER_STALL;
                    return (int32_t)usb_emit_config_desc(prod, data, size);

                case USB_DT_STRING: {
                    /* wIndex is the LANGID for non-zero string indices; we serve en-US only. */
                    uint8_t* out = data;
                    if (dindex == 0) return (int32_t)usb_emit_langid(out, size);
                    const char* s = NULL;
                    if (dindex == 1) s = prod->manufacturer;
                    else if (dindex == 2) s = prod->product;
                    else if (dindex == 3) s = prod->serial;
                    if (s == NULL) return RVVM_USB_XFER_STALL;
                    UNUSED(wIndex);  /* LANGID ignored — en-US only */
                    return (int32_t)usb_encode_string_desc(s, out, size);
                }

                case USB_DT_DEVICE_QUALIFIER:
                case USB_DT_OTHER_SPEED_CONFIGURATION:
                    /*
                     * Stalling these tells the host "I'm a single-speed
                     * device" — HS devices that can't fall back to FS
                     * are allowed and common. Linux tolerates the stall.
                     */
                    return RVVM_USB_XFER_STALL;

                default:
                    return RVVM_USB_XFER_STALL;
            }
        }

        case USB_REQ_GET_CONFIGURATION:
            if (!dir_in || size < 1) return RVVM_USB_XFER_STALL;
            ((uint8_t*)data)[0] = dev->config_value;
            return 1;

        case USB_REQ_SET_CONFIGURATION:
            /*
             * The only valid values are 0 (unconfigured) and 1 — we
             * only advertise bNumConfigurations=1. Reject anything
             * else with a stall; Linux treats that as "device refused".
             */
            if (wValue > 1) return RVVM_USB_XFER_STALL;
            dev->config_value = (uint8_t)wValue;
            return 0;

        case USB_REQ_GET_INTERFACE:
            if (!dir_in || size < 1) return RVVM_USB_XFER_STALL;
            ((uint8_t*)data)[0] = 0;  /* only alt-setting 0 */
            return 1;

        case USB_REQ_SET_INTERFACE:
            return wValue == 0 ? 0 : RVVM_USB_XFER_STALL;

        case USB_REQ_SYNCH_FRAME:
            if (!dir_in || size < 2) return RVVM_USB_XFER_STALL;
            write_uint16_le(data, 0);
            return 2;

        default:
            return RVVM_USB_XFER_STALL;
    }
}
