/*
jni_usb.c - JNI USB device bridge
Copyright (C) 2026  Sol Astrius <me@danielsol.dev>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * Lets Java-side code attach arbitrary USB 2.0 High-Speed devices
 * onto a machine's USB bus without ever upcalling into the JVM.
 *
 * Design rationale (see src/bindings/jni/rvvm_jni.c:454-468 for the
 * house style): every C→Java path in this tree is a polled mailbox
 * because AttachCurrentThread on an RVVM-owned pthread fails on
 * JDK 21 / macOS arm64. Applying the same pattern here: guest
 * transfers land in per-endpoint packet queues that Java drains on
 * its own schedule; Java-produced packets are enqueued for the
 * guest to pull on its next ring walk.
 *
 * Layer:
 *   XHCI worker   ---xfer()---> jni_usb_dev_t -> per-EP queues
 *                                                     ^
 *              JVM tick thread ---feed/poll----------/
 *
 * Control-endpoint handling is deliberately split:
 *   Default mode: C synthesises standard Chapter 9 requests from the
 *                 descriptor blob Java supplied at attach time, so
 *                 enumeration doesn't cross JNI at all.
 *   Raw mode:     (future) Java sees every SETUP packet via a control
 *                 mailbox and replies manually.
 *
 * Packet queues are fixed-size: JNI_USB_PACKET_QUEUE slots of
 * JNI_USB_MAX_PACKET bytes each per direction per endpoint. On IN
 * overflow we drop the oldest packet (match existing house style);
 * on OUT overflow we drop the oldest as well — USB guarantees
 * reliable delivery in principle, but if Java can't keep up the
 * right answer is to drop-and-report rather than wedge the guest.
 */

#include <pthread.h>
#include <string.h>

#include "compiler.h"
#include "tiny-jni.h"
#include "utils.h"
#include "spinlock.h"
#include "atomics.h"

#include "rvvmlib.h"
#include <rvvm/rvvm_usb.h>

PUSH_OPTIMIZATION_SIZE

#define JNI_USB_MAX_ENDPOINTS 16   /* USB allows up to 16 per direction */
#define JNI_USB_PACKET_QUEUE  32   /* Slots per direction per endpoint */
#define JNI_USB_MAX_PACKET    1024 /* Big enough for HS interrupt (1024) and bulk (512) */

typedef struct {
    uint16_t len;
    uint8_t  data[JNI_USB_MAX_PACKET];
} jni_usb_packet_t;

typedef struct {
    spinlock_t       lock;
    jni_usb_packet_t packets[JNI_USB_PACKET_QUEUE];
    uint8_t          head;
    uint8_t          tail;
    uint8_t          count;

    /* Pending IN-endpoint async xfer — the guest is blocked waiting
     * for data and we'll wake it the moment Java feeds a packet. */
    bool        pending;
    uint8_t     pending_ep;
    uint8_t*    pending_buf;  /* host pointer into guest DMA, valid until xfer_done */
    uint32_t    pending_size;
} jni_usb_ep_q_t;

typedef struct {
    spinlock_t       lock;
    rvvm_usb_dev_t*  dev;

    /* Backing storage for prod + ep_list + strings. rvvm_usb_dev_prod_t's
     * pointers must outlive the device, so we own the whole blob here. */
    rvvm_usb_dev_prod_t prod;
    rvvm_usb_ep_desc_t  ep_storage[JNI_USB_MAX_ENDPOINTS];
    char*               s_manufacturer;
    char*               s_product;
    char*               s_serial;

    /*
     * Optional class-specific descriptors the JNI caller supplied at
     * attach. For HID these are:
     *   class_desc:  the 9-byte HID descriptor (type 0x21), embedded in
     *                the config descriptor after the interface block and
     *                also served on GET_DESCRIPTOR(HID).
     *   report_desc: the raw HID report descriptor, served on
     *                GET_DESCRIPTOR(REPORT type 0x22).
     *
     * Non-HID classes (MSC, CDC, audio, etc.) can leave these null and
     * provide their own config-embedded descriptors by supplying
     * class_desc with whatever bytes the class spec calls for.
     */
    uint8_t*  class_desc;
    uint16_t  class_desc_len;
    uint8_t   class_desc_type;  /* 0x21 for HID, or 0 to suppress type-matched GET_DESCRIPTOR replies */
    uint8_t*  report_desc;
    uint16_t  report_desc_len;

    /*
     * Per direction queues. IN = data the device sends to the host
     * (Java feeds, XHCI drains). OUT = data the host sends to the
     * device (XHCI writes, Java polls).
     */
    jni_usb_ep_q_t in_q[JNI_USB_MAX_ENDPOINTS];
    jni_usb_ep_q_t out_q[JNI_USB_MAX_ENDPOINTS];

    /* Monotonic counters for instrumentation. */
    uint64_t in_fed;
    uint64_t in_consumed;
    uint64_t out_pushed;
    uint64_t out_popped;
    uint64_t in_dropped;
    uint64_t out_dropped;
} jni_usb_dev_t;

/*
 * ---------------------------------------------------------------------
 * rvvm_usb_dev_type_t callbacks
 * ---------------------------------------------------------------------
 */

/*
 * Build a complete configuration descriptor with the optional
 * class_desc blob embedded between the interface descriptor and the
 * endpoint descriptors. Returns bytes written to `out`.
 */
static size_t jni_usb_emit_config(jni_usb_dev_t* u, uint8_t* out, size_t out_size)
{
    const rvvm_usb_dev_prod_t* prod = &u->prod;
    size_t ep_count = prod->ep_size > JNI_USB_MAX_ENDPOINTS ? JNI_USB_MAX_ENDPOINTS : prod->ep_size;
    size_t class_len = u->class_desc ? u->class_desc_len : 0;
    size_t total = 9 + 9 + class_len + 7 * ep_count;

    uint8_t buf[9 + 9 + 512 + 7 * JNI_USB_MAX_ENDPOINTS];
    if (total > sizeof(buf)) return 0;

    /* Config header. */
    buf[0] = 9;
    buf[1] = 0x02;  /* CONFIGURATION */
    buf[2] = (uint8_t)(total & 0xFF);
    buf[3] = (uint8_t)((total >> 8) & 0xFF);
    buf[4] = 1;     /* bNumInterfaces */
    buf[5] = 1;     /* bConfigurationValue */
    buf[6] = 0;     /* iConfiguration */
    buf[7] = 0xC0;  /* self-powered, no remote wake */
    buf[8] = 0;     /* bMaxPower */

    /* Interface. */
    buf[9]  = 9;
    buf[10] = 0x04; /* INTERFACE */
    buf[11] = 0;    /* bInterfaceNumber */
    buf[12] = 0;    /* bAlternateSetting */
    buf[13] = (uint8_t)ep_count;
    buf[14] = prod->class_code;
    buf[15] = prod->subclass;
    buf[16] = prod->protocol;
    buf[17] = 0;    /* iInterface */

    size_t pos = 18;
    if (class_len) {
        memcpy(buf + pos, u->class_desc, class_len);
        pos += class_len;
    }

    for (size_t i = 0; i < ep_count; ++i) {
        uint8_t* ep = buf + pos;
        ep[0] = 7;
        ep[1] = 0x05;  /* ENDPOINT */
        ep[2] = prod->ep_list[i].addr;
        ep[3] = prod->ep_list[i].type & 0x3;
        ep[4] = (uint8_t)(prod->ep_list[i].size & 0xFF);
        ep[5] = (uint8_t)((prod->ep_list[i].size >> 8) & 0xFF);
        /* Default poll interval: 1 uframe for interrupt/isoch, 0 otherwise. */
        ep[6] = (prod->ep_list[i].type == RVVM_USB_EP_INTERRUPT
                 || prod->ep_list[i].type == RVVM_USB_EP_ISOCH) ? 1 : 0;
        pos += 7;
    }

    size_t n = total < out_size ? total : out_size;
    if (out && n) memcpy(out, buf, n);
    return n;
}

static int32_t jni_usb_control(rvvm_usb_dev_t* dev, const void* setup_buf, void* data, size_t size)
{
    jni_usb_dev_t* u = rvvm_usb_dev_get_data(dev);
    const uint8_t* setup = setup_buf;
    uint8_t  bmRequestType = setup[0];
    uint8_t  bRequest      = setup[1];
    uint16_t wValue        = (uint16_t)setup[2] | ((uint16_t)setup[3] << 8);
    uint16_t wLength       = (uint16_t)setup[6] | ((uint16_t)setup[7] << 8);
    bool     dir_in        = (bmRequestType & 0x80) != 0;
    uint8_t  type_bits     = bmRequestType & 0x60;

    if (size > wLength) size = wLength;

    /*
     * Standard GET_DESCRIPTOR is the only request we intercept before
     * rvvm_usb_std_control — needed to embed class_desc in the
     * configuration and to serve class-specific descriptor types
     * (HID 0x21, REPORT 0x22).
     */
    if (type_bits == 0x00 && bRequest == 0x06 /*GET_DESCRIPTOR*/ && dir_in) {
        uint8_t dtype = (uint8_t)(wValue >> 8);
        if (dtype == 0x02 /*CONFIG*/ && u->class_desc) {
            size_t n = jni_usb_emit_config(u, data, size);
            return n ? (int32_t)n : RVVM_USB_XFER_STALL;
        }
        if (dtype != 0 && u->class_desc && u->class_desc_type == dtype) {
            size_t n = u->class_desc_len < size ? u->class_desc_len : size;
            if (data && n) memcpy(data, u->class_desc, n);
            return (int32_t)n;
        }
        if (dtype == 0x22 /*HID REPORT*/ && u->report_desc) {
            size_t n = u->report_desc_len < size ? u->report_desc_len : size;
            if (data && n) memcpy(data, u->report_desc, n);
            return (int32_t)n;
        }
    }

    /*
     * Class/vendor-specific requests aren't routed to Java yet. A
     * forthcoming raw-control mode will mailbox these; until then
     * stall, which is what most HID drivers tolerate (they'll skip
     * optional features like SET_IDLE / GET_REPORT over EP0 rather
     * than failing the probe).
     */
    if (type_bits != 0x00) return RVVM_USB_XFER_STALL;

    return rvvm_usb_std_control(dev, setup_buf, data, size);
}

static int32_t jni_usb_xfer(rvvm_usb_dev_t* dev, uint8_t ep, void* data, size_t size)
{
    jni_usb_dev_t* u = rvvm_usb_dev_get_data(dev);
    uint8_t ep_num = ep & 0x0F;
    bool    is_in  = (ep & 0x80) != 0;
    if (ep_num >= JNI_USB_MAX_ENDPOINTS) return RVVM_USB_XFER_STALL;

    jni_usb_ep_q_t* q = is_in ? &u->in_q[ep_num] : &u->out_q[ep_num];

    if (is_in) {
        /* Pop one packet if available; otherwise park and report ASYNC. */
        int32_t ret;
        scoped_spin_lock (&q->lock) {
            if (q->count > 0) {
                jni_usb_packet_t* p = &q->packets[q->head];
                size_t n = p->len < size ? p->len : size;
                if (data && n) memcpy(data, p->data, n);
                q->head = (q->head + 1) % JNI_USB_PACKET_QUEUE;
                q->count--;
                u->in_consumed += n;
                ret = (int32_t)n;
            } else if (q->pending) {
                /* Shouldn't normally happen — xHCI serialises one TD per EP.
                 * Return BUSY so the controller can retry after completion. */
                ret = RVVM_USB_XFER_BUSY;
            } else {
                q->pending = true;
                q->pending_ep = ep;
                q->pending_buf = (uint8_t*)data;
                q->pending_size = (uint32_t)size;
                ret = RVVM_USB_XFER_ASYNC;
            }
        }
        return ret;
    }

    /* OUT: enqueue a packet for Java to drain later. */
    if (size > JNI_USB_MAX_PACKET) size = JNI_USB_MAX_PACKET;
    scoped_spin_lock (&q->lock) {
        if (q->count == JNI_USB_PACKET_QUEUE) {
            /* Drop oldest. */
            q->head = (q->head + 1) % JNI_USB_PACKET_QUEUE;
            q->count--;
            u->out_dropped++;
        }
        jni_usb_packet_t* p = &q->packets[q->tail];
        p->len = (uint16_t)size;
        if (data && size) memcpy(p->data, data, size);
        q->tail = (q->tail + 1) % JNI_USB_PACKET_QUEUE;
        q->count++;
        u->out_pushed += size;
    }
    return (int32_t)size;
}

static void jni_usb_cancel(rvvm_usb_dev_t* dev, uint8_t ep)
{
    jni_usb_dev_t* u = rvvm_usb_dev_get_data(dev);
    uint8_t ep_num = ep & 0x0F;
    bool    is_in  = (ep & 0x80) != 0;
    if (ep_num >= JNI_USB_MAX_ENDPOINTS) return;
    jni_usb_ep_q_t* q = is_in ? &u->in_q[ep_num] : &u->out_q[ep_num];
    scoped_spin_lock (&q->lock) {
        q->pending = false;
        q->pending_buf = NULL;
        q->pending_size = 0;
    }
}

static void jni_usb_reset(rvvm_usb_dev_t* dev)
{
    jni_usb_dev_t* u = rvvm_usb_dev_get_data(dev);
    /* Wipe queues but keep descriptors intact. */
    for (size_t i = 0; i < JNI_USB_MAX_ENDPOINTS; ++i) {
        scoped_spin_lock (&u->in_q[i].lock) {
            u->in_q[i].head = u->in_q[i].tail = u->in_q[i].count = 0;
            u->in_q[i].pending = false;
        }
        scoped_spin_lock (&u->out_q[i].lock) {
            u->out_q[i].head = u->out_q[i].tail = u->out_q[i].count = 0;
            u->out_q[i].pending = false;
        }
    }
}

static void jni_usb_free(rvvm_usb_dev_t* dev)
{
    jni_usb_dev_t* u = rvvm_usb_dev_get_data(dev);
    if (u == NULL) return;
    free(u->s_manufacturer);
    free(u->s_product);
    free(u->s_serial);
    free(u->class_desc);
    free(u->report_desc);
    free(u);
}

/*
 * Static type descriptor shared across all JNI devices — the per-device
 * state lives in `data` (jni_usb_dev_t*), not in the vtable. `prod` is
 * patched per-device below; to keep this static we use a per-device
 * copy of rvvm_usb_dev_type_t instead.
 */

/*
 * ---------------------------------------------------------------------
 * JNI entry points
 * ---------------------------------------------------------------------
 */

static char* jni_strdup(JNIEnv* env, jstring js)
{
    if (js == NULL) return NULL;
    const char* u8 = (*env)->GetStringUTFChars(env, js, NULL);
    if (u8 == NULL) return NULL;
    size_t len = strlen(u8) + 1;
    char* out = safe_new_arr(char, len);
    memcpy(out, u8, len);
    (*env)->ReleaseStringUTFChars(env, js, u8);
    return out;
}

/*
 * Attach a USB device to the machine's bus. Descriptor strings may be
 * null; endpoint arrays may be null for devices with only EP0 (no
 * practical use, but permitted).
 *
 * ep_kinds[i]: 0=control 1=iso 2=bulk 3=interrupt (USB EP type field)
 * ep_addresses[i]: bit 7 direction (1 = IN), low nibble endpoint number
 *
 * Returns an opaque jni_usb_dev_t handle, 0 on failure.
 */
JNIEXPORT jlong JNICALL Java_lekkit_rvvm_RVVMNative_usb_1dev_1attach(
    JNIEnv* env, jclass cls,
    jlong   machine,
    jshort  vid,
    jshort  pid,
    jbyte   class_code,
    jbyte   subclass,
    jbyte   protocol,
    jstring manufacturer,
    jstring product,
    jstring serial,
    jbyteArray  ep_addrs,
    jbyteArray  ep_kinds,
    jshortArray ep_sizes,
    jbyte       class_desc_type,
    jbyteArray  class_desc,
    jbyteArray  report_desc)
{
    UNUSED(cls);

    rvvm_usb_bus_t* bus = rvvm_get_usb_bus((rvvm_machine_t*)(size_t)machine);
    if (bus == NULL) return 0;

    jni_usb_dev_t* u = safe_new_obj(jni_usb_dev_t);
    spin_init(&u->lock);
    for (size_t i = 0; i < JNI_USB_MAX_ENDPOINTS; ++i) {
        spin_init(&u->in_q[i].lock);
        spin_init(&u->out_q[i].lock);
    }

    u->s_manufacturer = jni_strdup(env, manufacturer);
    u->s_product      = jni_strdup(env, product);
    u->s_serial       = jni_strdup(env, serial);

    size_t ep_count = 0;
    if (ep_addrs != NULL && ep_kinds != NULL && ep_sizes != NULL) {
        jsize n_addrs = (*env)->GetArrayLength(env, ep_addrs);
        jsize n_kinds = (*env)->GetArrayLength(env, ep_kinds);
        jsize n_sizes = (*env)->GetArrayLength(env, ep_sizes);
        jsize n = n_addrs < n_kinds ? n_addrs : n_kinds;
        if (n_sizes < n) n = n_sizes;
        if (n > JNI_USB_MAX_ENDPOINTS) n = JNI_USB_MAX_ENDPOINTS;
        ep_count = (size_t)n;

        jbyte  addrs_buf[JNI_USB_MAX_ENDPOINTS];
        jbyte  kinds_buf[JNI_USB_MAX_ENDPOINTS];
        jshort sizes_buf[JNI_USB_MAX_ENDPOINTS];
        (*env)->GetByteArrayRegion(env,  ep_addrs,  0, n, addrs_buf);
        (*env)->GetByteArrayRegion(env,  ep_kinds,  0, n, kinds_buf);
        (*env)->GetShortArrayRegion(env, ep_sizes,  0, n, sizes_buf);
        for (size_t i = 0; i < ep_count; ++i) {
            u->ep_storage[i].addr = (uint8_t)addrs_buf[i];
            u->ep_storage[i].type = (uint8_t)(kinds_buf[i] & 0x3);
            u->ep_storage[i].size = (uint16_t)sizes_buf[i];
        }
    }

    u->prod.vendor_id    = (uint16_t)vid;
    u->prod.product_id   = (uint16_t)pid;
    u->prod.class_code   = (uint8_t)class_code;
    u->prod.subclass     = (uint8_t)subclass;
    u->prod.protocol     = (uint8_t)protocol;
    u->prod.manufacturer = u->s_manufacturer;
    u->prod.product      = u->s_product;
    u->prod.serial       = u->s_serial;
    u->prod.ep_list      = u->ep_storage;
    u->prod.ep_size      = ep_count;

    u->class_desc_type = (uint8_t)class_desc_type;
    if (class_desc != NULL) {
        jsize n = (*env)->GetArrayLength(env, class_desc);
        if (n > 0 && n <= 512) {
            u->class_desc = safe_new_arr(uint8_t, (size_t)n);
            u->class_desc_len = (uint16_t)n;
            (*env)->GetByteArrayRegion(env, class_desc, 0, n, (jbyte*)u->class_desc);
        }
    }
    if (report_desc != NULL) {
        jsize n = (*env)->GetArrayLength(env, report_desc);
        if (n > 0 && n <= 4096) {
            u->report_desc = safe_new_arr(uint8_t, (size_t)n);
            u->report_desc_len = (uint16_t)n;
            (*env)->GetByteArrayRegion(env, report_desc, 0, n, (jbyte*)u->report_desc);
        }
    }

    /*
     * Per-device type vtable. Allocated here because prod is per-device
     * too; a shared static table would require a separate prod lookup.
     */
    rvvm_usb_dev_type_t* type = safe_new_obj(rvvm_usb_dev_type_t);
    type->prod    = &u->prod;
    /*
     * Always route control through jni_usb_control: for non-HID devices
     * (class_desc/report_desc both null) it just delegates to
     * rvvm_usb_std_control, so the extra indirection is one branch.
     */
    type->control = jni_usb_control;
    type->xfer    = jni_usb_xfer;
    type->cancel  = jni_usb_cancel;
    type->reset   = jni_usb_reset;
    type->free    = jni_usb_free;
    type->suspend = NULL;

    rvvm_usb_dev_t* dev = rvvm_usb_dev_init(bus, type, u);
    if (dev == NULL) {
        /*
         * rvvm_usb_dev_init_at invoked type->free on failure, which
         * already freed `u`. The per-device type struct is leaked here
         * intentionally — it has no refcount, and we can't free it
         * before the core is guaranteed to stop derefing it.
         */
        free(type);
        return 0;
    }
    u->dev = dev;

    return (jlong)(size_t)u;
}

JNIEXPORT void JNICALL Java_lekkit_rvvm_RVVMNative_usb_1dev_1detach(
    JNIEnv* env, jclass cls, jlong handle)
{
    UNUSED(env); UNUSED(cls);
    if (handle == 0) return;
    jni_usb_dev_t* u = (jni_usb_dev_t*)(size_t)handle;
    /*
     * rvvm_usb_dev_free will cancel pending xfers (via controller
     * port_update false → xfer cancel), then call our type->free
     * which tears down u. The vtable allocation is intentionally
     * leaked — see attach.
     */
    rvvm_usb_dev_free(u->dev);
}

/*
 * Push a packet to an IN endpoint. If the guest has an xfer pending on
 * that endpoint, we hand the packet straight to it and complete the
 * xfer synchronously. Otherwise we enqueue.
 *
 * Returns the number of bytes actually queued/delivered, or -1 on error.
 */
JNIEXPORT jint JNICALL Java_lekkit_rvvm_RVVMNative_usb_1ep_1feed(
    JNIEnv* env, jclass cls, jlong handle, jbyte ep, jbyteArray data, jint off, jint len)
{
    UNUSED(cls);
    if (handle == 0 || data == NULL) return -1;

    jni_usb_dev_t* u = (jni_usb_dev_t*)(size_t)handle;
    uint8_t ep_num = ((uint8_t)ep) & 0x0F;
    bool    is_in  = ((uint8_t)ep & 0x80) != 0;
    if (!is_in || ep_num >= JNI_USB_MAX_ENDPOINTS) return -1;

    jsize cap = (*env)->GetArrayLength(env, data);
    if (off < 0 || len < 0 || off > cap || (cap - off) < len) return -1;
    if (len > JNI_USB_MAX_PACKET) len = JNI_USB_MAX_PACKET;
    if (len == 0) return 0;

    uint8_t tmp[JNI_USB_MAX_PACKET];
    (*env)->GetByteArrayRegion(env, data, off, len, (jbyte*)tmp);

    jni_usb_ep_q_t* q = &u->in_q[ep_num];

    /*
     * Path 1: guest has a pending xfer — complete it right here. Copy
     * to the pending DMA buffer, clear pending, and ping the bus.
     * xfer_done is called outside the spinlock since it acquires the
     * xhci lock and we don't want to invert ordering.
     */
    bool complete = false;
    int32_t complete_bytes = 0;
    uint8_t complete_ep = 0;

    scoped_spin_lock (&q->lock) {
        if (q->pending) {
            size_t n = (size_t)len;
            if (n > q->pending_size) n = q->pending_size;
            if (n && q->pending_buf) memcpy(q->pending_buf, tmp, n);
            complete_ep = q->pending_ep;
            complete_bytes = (int32_t)n;
            q->pending = false;
            q->pending_buf = NULL;
            q->pending_size = 0;
            u->in_fed += n;
            u->in_consumed += n;
            complete = true;
        } else {
            if (q->count == JNI_USB_PACKET_QUEUE) {
                q->head = (q->head + 1) % JNI_USB_PACKET_QUEUE;
                q->count--;
                u->in_dropped++;
            }
            jni_usb_packet_t* p = &q->packets[q->tail];
            p->len = (uint16_t)len;
            memcpy(p->data, tmp, (size_t)len);
            q->tail = (q->tail + 1) % JNI_USB_PACKET_QUEUE;
            q->count++;
            u->in_fed += (uint64_t)len;
        }
    }

    if (complete) {
        rvvm_usb_dev_xfer_done(u->dev, complete_ep, complete_bytes);
    }

    return len;
}

/*
 * Drain one packet from an OUT endpoint. Returns the number of bytes
 * written into `out`, or 0 if empty.
 */
JNIEXPORT jint JNICALL Java_lekkit_rvvm_RVVMNative_usb_1ep_1poll(
    JNIEnv* env, jclass cls, jlong handle, jbyte ep, jbyteArray out)
{
    UNUSED(cls);
    if (handle == 0 || out == NULL) return 0;

    jni_usb_dev_t* u = (jni_usb_dev_t*)(size_t)handle;
    uint8_t ep_num = ((uint8_t)ep) & 0x0F;
    bool    is_in  = ((uint8_t)ep & 0x80) != 0;
    /* Java reads OUT endpoints; IN endpoints are for feed. */
    if (is_in || ep_num >= JNI_USB_MAX_ENDPOINTS) return 0;

    jsize cap = (*env)->GetArrayLength(env, out);
    if (cap <= 0) return 0;

    uint8_t tmp[JNI_USB_MAX_PACKET];
    size_t n = 0;
    jni_usb_ep_q_t* q = &u->out_q[ep_num];
    scoped_spin_lock (&q->lock) {
        if (q->count > 0) {
            jni_usb_packet_t* p = &q->packets[q->head];
            n = p->len < (size_t)cap ? p->len : (size_t)cap;
            memcpy(tmp, p->data, n);
            q->head = (q->head + 1) % JNI_USB_PACKET_QUEUE;
            q->count--;
            u->out_popped += n;
        }
    }

    if (n > 0) {
        (*env)->SetByteArrayRegion(env, out, 0, (jsize)n, (const jbyte*)tmp);
    }
    return (jint)n;
}

/*
 * Fill a long[6] with {in_fed, in_consumed, out_pushed, out_popped,
 * in_dropped, out_dropped}.
 */
JNIEXPORT void JNICALL Java_lekkit_rvvm_RVVMNative_usb_1dev_1stats(
    JNIEnv* env, jclass cls, jlong handle, jlongArray out)
{
    UNUSED(cls);
    if (handle == 0 || out == NULL) return;
    if ((*env)->GetArrayLength(env, out) < 6) return;
    jni_usb_dev_t* u = (jni_usb_dev_t*)(size_t)handle;
    jlong stats[6];
    scoped_spin_lock (&u->lock) {
        stats[0] = (jlong)u->in_fed;
        stats[1] = (jlong)u->in_consumed;
        stats[2] = (jlong)u->out_pushed;
        stats[3] = (jlong)u->out_popped;
        stats[4] = (jlong)u->in_dropped;
        stats[5] = (jlong)u->out_dropped;
    }
    (*env)->SetLongArrayRegion(env, out, 0, 6, stats);
}

POP_OPTIMIZATION_SIZE
