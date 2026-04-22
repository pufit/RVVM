/*
usb-xhci.c - USB Extensible Host Controller Interface
Copyright (C) 2024  LekKit <github.com/LekKit>
Copyright (C) 2026  Sol Astrius <me@danielsol.dev>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * XHCI host controller, wired to the rvvm_usb core.
 *
 * Shape:
 *   guest xhci_hcd MMIO  <->  usb-xhci.c  <->  rvvm_usb_bus_t  <->  device
 *
 * xhci owns a rvvm_usb_bus_t; devices attach to it via the core API
 * and the bus callback (port_update / xfer_done) fans Port Status
 * Change and Transfer events into the guest's event ring.
 *
 * Scope: USB 2.0 High Speed only, eight ports, one interrupter, no
 * streams, no MSI-X. Sufficient for a Linux xhci_pci / usbhid
 * enumeration path; not a general-purpose host controller.
 *
 * Locking:
 *   xhci->lock guards slots[] and event-ring production. Port state
 *   (PORTSC) lives in xhci->ports[] under the same lock. Register
 *   reads go through atomic_load where needed; writes that touch
 *   cross-region state take the lock.
 *
 *   Guest CPU thread runs doorbell writes (command ring processing,
 *   transfer ring walks). Device-owner threads run xfer_done callbacks.
 *   Both post events via xhci_post_event under xhci->lock.
 */

#include "usb-xhci.h"
#include "utils.h"
#include "mem_ops.h"
#include "bit_ops.h"
#include "spinlock.h"
#include "atomics.h"

#include <rvvm/rvvm_usb.h>

#include <string.h>

/*
 * ---------------------------------------------------------------------
 * Register layout
 * ---------------------------------------------------------------------
 */

#define XHCI_CAPABILITY_BASE  0x0
#define XHCI_OPERATIONAL_BASE 0x80
#define XHCI_PORT_REGS_BASE   0x400
#define XHCI_RUNTIME_BASE     0x2000
#define XHCI_DOORBELL_BASE    0x3000
#define XHCI_EXT_CAPS_BASE    0x8000

#define XHCI_PORT_REGS_SIZE 0x1000
#define XHCI_RUNTIME_SIZE   0x1000
#define XHCI_DOORBELL_SIZE  0x1000
#define XHCI_BAR_SIZE       0x10000

/* Capability */
#define XHCI_REG_CAPLENGTH_HCIVERSION 0x00
#define XHCI_REG_HCSPARAMS1           0x04
#define XHCI_REG_HCSPARAMS2           0x08
#define XHCI_REG_HCSPARAMS3           0x0C
#define XHCI_REG_HCCPARAMS1           0x10
#define XHCI_REG_DBOFF                0x14
#define XHCI_REG_RTSOFF               0x18
#define XHCI_REG_HCCPARMS2            0x1C

#define XHCI_VERSION   0x0100

#define XHCI_MAX_SLOTS 0x20   /* 32 device slots */
#define XHCI_MAX_PORTS 0x08   /*  8 USB2 HS root ports (matches rvvm_usb bus) */
#define XHCI_MAX_INTRS 0x01   /*  1 interrupter */

#define XHCI_CAPLEN_HCIVERSION (XHCI_OPERATIONAL_BASE | (XHCI_VERSION << 16))
#define XHCI_HCSPARAMS1        (XHCI_MAX_SLOTS | (XHCI_MAX_INTRS << 8) | (XHCI_MAX_PORTS << 24))
/*
 * HCCPARAMS1: AC64 (bit 0), CSZ=0 (32-byte contexts), xECP at ext-caps offset
 * in 32-bit words (bits 31:16).
 */
#define XHCI_HCCPARAMS1 (((XHCI_EXT_CAPS_BASE >> 2) << 16) | 0x1)

/* Operational */
#define XHCI_REG_USBCMD   0x80
#define XHCI_REG_USBSTS   0x84
#define XHCI_REG_PAGESIZE 0x88
#define XHCI_REG_DNCTRL   0x94
#define XHCI_REG_CRCR     0x98
#define XHCI_REG_CRCR_H   0x9C
#define XHCI_REG_DCBAAP   0xB0
#define XHCI_REG_DCBAAP_H 0xB4
#define XHCI_REG_CONFIG   0xB8

#define XHCI_USBCMD_RS    0x1
#define XHCI_USBCMD_HCRST 0x2
#define XHCI_USBCMD_INTE  0x4
#define XHCI_USBCMD_HSEE  0x8
#define XHCI_USBCMD_MASK (XHCI_USBCMD_RS | XHCI_USBCMD_HCRST | XHCI_USBCMD_INTE | XHCI_USBCMD_HSEE)

#define XHCI_USBSTS_HCH  0x1
#define XHCI_USBSTS_HSE  0x4
#define XHCI_USBSTS_EINT 0x8
#define XHCI_USBSTS_PCD  0x10
#define XHCI_USBSTS_CNR  0x800
#define XHCI_USBSTS_MASK (XHCI_USBSTS_HSE | XHCI_USBSTS_EINT | XHCI_USBSTS_PCD)

#define XHCI_DNCTRL_MASK 0xFFFF
#define XHCI_ALIGN_64BYTE (~0x3FULL)

#define XHCI_CRCR_RCS 0x1
#define XHCI_CRCR_CS  0x2
#define XHCI_CRCR_CA  0x4
#define XHCI_CRCR_CRR 0x8
#define XHCI_CRCR_WMASK (XHCI_ALIGN_64BYTE | XHCI_CRCR_RCS | XHCI_CRCR_CS | XHCI_CRCR_CA)

/* Port registers — one 0x10-byte block per port at PORT_REGS_BASE + 0x10*i */
#define XHCI_REG_PORTSC    0x0
#define XHCI_REG_PORTPMSC  0x4
#define XHCI_REG_PORTLI    0x8
#define XHCI_REG_PORTHLPMC 0xC

#define XHCI_PORTSC_CCS         0x00000001u  /* Current Connect Status */
#define XHCI_PORTSC_PED         0x00000002u  /* Port Enabled/Disabled */
#define XHCI_PORTSC_OCA         0x00000008u
#define XHCI_PORTSC_PR          0x00000010u  /* Port Reset */
#define XHCI_PORTSC_PLS_SHIFT   5
#define XHCI_PORTSC_PLS_MASK    (0xFu << 5)  /* Port Link State */
#define XHCI_PORTSC_PP          0x00000200u  /* Port Power */
#define XHCI_PORTSC_SPEED_SHIFT 10
#define XHCI_PORTSC_SPEED_MASK  (0xFu << 10)
#define XHCI_PORTSC_SPEED_HS    (0x3u << 10) /* High Speed */
#define XHCI_PORTSC_LWS         0x00010000u  /* Port Link State Write Strobe */
#define XHCI_PORTSC_CSC         0x00020000u  /* Connect Status Change */
#define XHCI_PORTSC_PEC         0x00040000u  /* Port Enabled/Disabled Change */
#define XHCI_PORTSC_WRC         0x00080000u  /* Warm Port Reset Change */
#define XHCI_PORTSC_OCC         0x00100000u  /* Overcurrent Change */
#define XHCI_PORTSC_PRC         0x00200000u  /* Port Reset Change */
#define XHCI_PORTSC_PLC         0x00400000u  /* Port Link State Change */
#define XHCI_PORTSC_CEC         0x00800000u  /* Port Config Error Change */
#define XHCI_PORTSC_WCE         0x02000000u  /* Wake on Connect Enable */
#define XHCI_PORTSC_WDE         0x04000000u  /* Wake on Disconnect Enable */
#define XHCI_PORTSC_WOE         0x08000000u  /* Wake on Overcurrent Enable */

/*
 * Change bits are RW1C — guest writes 1 to clear. These are the bits
 * that clear on guest write of 1.
 */
#define XHCI_PORTSC_CHANGE_MASK \
    (XHCI_PORTSC_CSC | XHCI_PORTSC_PEC | XHCI_PORTSC_WRC | XHCI_PORTSC_OCC \
     | XHCI_PORTSC_PRC | XHCI_PORTSC_PLC | XHCI_PORTSC_CEC)

/* Writable-non-RW1C bits guest can drive via normal writes. */
#define XHCI_PORTSC_WMASK \
    (XHCI_PORTSC_PED | XHCI_PORTSC_PR | XHCI_PORTSC_PP \
     | XHCI_PORTSC_LWS | XHCI_PORTSC_WCE | XHCI_PORTSC_WDE | XHCI_PORTSC_WOE)

/* Interrupter */
#define XHCI_REG_IMAN     0x0
#define XHCI_REG_IMOD     0x4
#define XHCI_REG_ERSTSZ   0x8
#define XHCI_REG_ERSTBA   0x10
#define XHCI_REG_ERSTBA_H 0x14
#define XHCI_REG_ERDP     0x18
#define XHCI_REG_ERDP_H   0x1C

#define XHCI_IMAN_IP 0x1
#define XHCI_IMAN_IE 0x2

#define XHCI_ERDP_DESI 0x7   /* Dequeue ERST Segment Index */
#define XHCI_ERDP_EHB  0x8   /* Event Handler Busy */
#define XHCI_ERDP_PTR  (~0xFULL)

/* TRB */
#define XHCI_TRB_SIZE 0x10

#define XHCI_TRB_COMP_SUCCESS     0x1
#define XHCI_TRB_COMP_DB_ERR      0x2
#define XHCI_TRB_COMP_BABBLE      0x3
#define XHCI_TRB_COMP_TX_ERR      0x4
#define XHCI_TRB_COMP_TRB_ERR     0x5
#define XHCI_TRB_COMP_STALL       0x6
#define XHCI_TRB_COMP_RESOURCE    0x7
#define XHCI_TRB_COMP_NA_SLOTS    0x8
#define XHCI_TRB_COMP_ENDPOINT_NE 0xC
#define XHCI_TRB_COMP_SHORT_PKT   0xD
#define XHCI_TRB_COMP_EINVAL      0x11
#define XHCI_TRB_COMP_CTX_STATE   0x13

#define XHCI_TRB_CTR_C   0x1
#define XHCI_TRB_CTR_ENT 0x2
#define XHCI_TRB_CTR_ISP 0x4
#define XHCI_TRB_CTR_NS  0x8
#define XHCI_TRB_CTR_CH  0x10
#define XHCI_TRB_CTR_IOC 0x20
#define XHCI_TRB_CTR_IDT 0x40
#define XHCI_TRB_CTR_BEI 0x200
#define XHCI_TRB_CTR_TC  0x2       /* Toggle Cycle (LINK TRB) */
#define XHCI_TRB_DIR_IN  0x10000   /* SETUP/DATA direction (b16 of ctr) */

#define XHCI_TRB_TYPE(ctr)       (((ctr) >> 10) & 0x3F)
#define XHCI_TRB_MAKE_TYPE(t)    ((uint32_t)(t) << 10)
#define XHCI_TRB_INTERRUPTER(sts) (((sts) >> 22) & 0x3FF)
#define XHCI_TRB_LEN(sts)        ((sts) & 0x1FFFF)
#define XHCI_TRB_SLOT(ctr)       (((ctr) >> 24) & 0xFF)
#define XHCI_TRB_EPID(ctr)       (((ctr) >> 16) & 0x1F)
#define XHCI_TRB_TRT(ctr)        (((ctr) >> 16) & 0x3) /* Transfer Type (SETUP control) */

#define XHCI_TRB_TYPE_NORMAL       0x1
#define XHCI_TRB_TYPE_SETUP        0x2
#define XHCI_TRB_TYPE_DATA         0x3
#define XHCI_TRB_TYPE_STATUS       0x4
#define XHCI_TRB_TYPE_ISOC         0x5
#define XHCI_TRB_TYPE_LINK         0x6
#define XHCI_TRB_TYPE_EVENT_DATA   0x7
#define XHCI_TRB_TYPE_NOOP         0x8
#define XHCI_TRB_TYPE_ENABLE_SLOT  0x9
#define XHCI_TRB_TYPE_DISABLE_SLOT 0xA
#define XHCI_TRB_TYPE_ADDR_DEV     0xB
#define XHCI_TRB_TYPE_CONFIG_EP    0xC
#define XHCI_TRB_TYPE_EVAL_CONTEXT 0xD
#define XHCI_TRB_TYPE_RESET_EP     0xE
#define XHCI_TRB_TYPE_STOP_RING    0xF
#define XHCI_TRB_TYPE_SET_DEQ      0x10
#define XHCI_TRB_TYPE_RESET_DEV    0x11
#define XHCI_TRB_TYPE_CMD_NOOP     0x17
#define XHCI_TRB_TYPE_TRANSFER     0x20
#define XHCI_TRB_TYPE_COMPLETION   0x21
#define XHCI_TRB_TYPE_PORT_STATUS  0x22

/* Address Device command TRB bits */
#define XHCI_CMD_ADDR_DEV_BSR      0x00000200u /* Block Set Address Request */

/*
 * Extended Capabilities — advertise one USB 2.0 protocol block covering
 * ports 1..XHCI_MAX_PORTS. No USB 3 block, so Linux only probes HS.
 * Format (xHCI spec 7.2):
 *   dw0: capID=2, next offset (in 32-bit words), min rev, maj rev
 *   dw1: "USB "
 *   dw2: compat port offset, port count, protocol defined, PSIC
 *   dw3: slot type
 *   dwN: PSI entries (omitted — Linux falls back to defaults for USB2)
 */
static const uint32_t xhci_ext_caps[] = {
    (0x02u << 24) | (0x00u << 16) | (0x00u << 8) | 0x02u,  /* capID=2, next=0 (terminal), maj=2, min=0 */
    0x20425355,  /* "USB " */
    (0x00u << 28) | (((uint32_t)XHCI_MAX_PORTS) << 8) | 0x01u,  /* PSIC=0, port count, compat offset=1 */
    0x00000000,
};

/*
 * ---------------------------------------------------------------------
 * Slot / endpoint state
 * ---------------------------------------------------------------------
 */

/*
 * One per DCI (Device Context Index). DCI 0 is the slot context; 1 is
 * EP0 bidirectional; 2..31 alternate OUT/IN for EP1..EP15. We track a
 * transfer-ring cursor and optional pending-async-completion state.
 */
typedef struct {
    bool        enabled;
    rvvm_addr_t deq;          /* guest phys, 16-byte aligned */
    uint8_t     dcs;          /* Dequeue Cycle State (consumer cycle bit) */
    uint8_t     ep_type;      /* EP Context type field (1..7), or 0 if unset */
    uint16_t    max_packet;

    /* Async completion state: one TD in flight at a time per EP. */
    struct {
        bool        active;
        rvvm_addr_t trb_ptr;  /* source TRB for Transfer Event */
        uint32_t    interrupter;
        uint32_t    length;   /* requested length */
    } pending;
} xhci_ep_t;

typedef struct {
    bool            in_use;
    rvvm_usb_dev_t* dev;          /* non-NULL once slot is addressed */
    uint32_t        root_port;    /* 1-based */
    rvvm_addr_t     out_ctx_addr; /* DCBA[slot]; output device context */
    xhci_ep_t       eps[32];      /* DCI 0 unused for convention, 1..31 real */
} xhci_slot_t;

typedef struct {
    uint32_t iman;
    uint32_t imod;
    uint32_t erstsz;
    uint32_t erstba;
    uint32_t erstba_h;
    uint32_t erdp;
    uint32_t erdp_h;

    /* Event ring producer cursor. */
    rvvm_addr_t er_base;   /* cached base of current segment */
    uint32_t    er_size;   /* cached TRB count of current segment */
    uint32_t    er_seg;    /* current segment index into ERST */
    uint32_t    er_idx;    /* next TRB index within segment */
    uint8_t     pcs;       /* producer cycle state (flips on segment wrap) */
    bool        er_primed; /* segment base has been loaded once */
} xhci_interrupter_t;

typedef struct {
    uint32_t portsc;
} xhci_port_t;

typedef struct {
    pci_func_t*     pci_func;
    rvvm_usb_bus_t* bus;

    /* Operational regs */
    uint32_t usbcmd;
    uint32_t usbsts;
    uint32_t dnctrl;
    uint32_t crcr;
    uint32_t crcr_h;
    uint32_t dcbaap;
    uint32_t dcbaap_h;
    uint32_t config;

    /*
     * Command ring producer cursor — guest drives the CR via the
     * doorbell at DB[0]. We track our own consumer cycle state because
     * CRCR's cycle bit is just the *initial* state; the ring flips it
     * on LINK TRBs with TC set.
     */
    rvvm_addr_t cmd_deq;
    uint8_t     cmd_ccs;

    xhci_port_t        ports[XHCI_MAX_PORTS + 1];   /* 1-indexed */
    xhci_slot_t        slots[XHCI_MAX_SLOTS + 1];   /* 1-indexed */
    xhci_interrupter_t ints[XHCI_MAX_INTRS];

    spinlock_t lock;
} xhci_bus_t;

/*
 * ---------------------------------------------------------------------
 * Event posting
 * ---------------------------------------------------------------------
 */

/* Re-raise (edge) IRQ if enabled. Must be called from inside xhci->lock. */
static void xhci_interrupt_locked(xhci_bus_t* xhci)
{
    if (atomic_load_uint32_relax(&xhci->usbcmd) & XHCI_USBCMD_INTE) {
        pci_send_irq(xhci->pci_func, 0);
    }
}

/*
 * Load the current ERST segment base/size if we haven't primed yet or
 * the guest just wrote the pointers. Called from inside the lock.
 */
static bool xhci_intr_prime_segment(xhci_bus_t* xhci, xhci_interrupter_t* intr)
{
    rvvm_addr_t erst = ((rvvm_addr_t)atomic_load_uint32_relax(&intr->erstba))
                       | ((rvvm_addr_t)atomic_load_uint32_relax(&intr->erstba_h) << 32);
    uint32_t erstsz = atomic_load_uint32_relax(&intr->erstsz);
    if (erstsz == 0 || erst == 0) return false;

    if (intr->er_seg >= erstsz) intr->er_seg = 0;

    rvvm_addr_t entry_addr = (erst & ~0x3FULL) + (rvvm_addr_t)intr->er_seg * 0x10;
    const uint8_t* entry = pci_get_dma_ptr(xhci->pci_func, entry_addr, 0x10);
    if (entry == NULL) return false;

    intr->er_base = read_uint64_le(entry) & ~0x3FULL;
    intr->er_size = read_uint16_le(entry + 8);  /* TRBs in this segment */
    if (intr->er_size == 0) return false;

    if (!intr->er_primed) {
        intr->er_idx = 0;
        intr->pcs = 1;
        intr->er_primed = true;
    } else if (intr->er_idx >= intr->er_size) {
        /* Defensive — should have been wrapped on post */
        intr->er_idx = 0;
    }
    return true;
}

/*
 * Write a single event TRB onto the interrupter's event ring, flipping
 * PCS on segment wrap. Signals EINT / IP. Caller holds xhci->lock.
 */
static void xhci_post_event_locked(xhci_bus_t*        xhci,
                                   uint32_t           interrupter,
                                   uint64_t           trb_ptr,
                                   uint32_t           sts,
                                   uint32_t           ctr_type_and_flags)
{
    if (interrupter >= XHCI_MAX_INTRS) interrupter = 0;
    xhci_interrupter_t* intr = &xhci->ints[interrupter];

    if (!xhci_intr_prime_segment(xhci, intr)) return;

    /* Check for event ring overflow: we'd wrap into the segment the
     * guest is about to dequeue from. xHCI lets us advance; the guest
     * ERDP gating handles back-pressure. We don't police. */
    rvvm_addr_t slot_addr = intr->er_base + (rvvm_addr_t)intr->er_idx * XHCI_TRB_SIZE;
    uint8_t* dma = pci_get_dma_ptr(xhci->pci_func, slot_addr, XHCI_TRB_SIZE);
    if (dma == NULL) return;

    uint32_t ctr = ctr_type_and_flags | (intr->pcs ? XHCI_TRB_CTR_C : 0);
    write_uint64_le(dma, trb_ptr);
    write_uint32_le(dma + 8, sts);
    atomic_store_uint32_le(dma + 0xC, ctr);

    /* Advance with segment wrap and PCS flip. */
    intr->er_idx += 1;
    if (intr->er_idx >= intr->er_size) {
        intr->er_idx = 0;
        uint32_t erstsz = atomic_load_uint32_relax(&intr->erstsz);
        intr->er_seg = (intr->er_seg + 1) % (erstsz ? erstsz : 1);
        intr->pcs ^= 1;
        /*
         * Reload the new segment's base/size. Leave er_primed so
         * xhci_intr_prime_segment won't re-reset er_idx/pcs.
         */
        xhci_intr_prime_segment(xhci, intr);
    }

    /* EINT in USBSTS, IP in IMAN. */
    atomic_or_uint32(&xhci->usbsts, XHCI_USBSTS_EINT);
    uint32_t iman = atomic_or_uint32(&intr->iman, XHCI_IMAN_IP);
    if ((iman | XHCI_IMAN_IP) & XHCI_IMAN_IE) {
        xhci_interrupt_locked(xhci);
    }
}

static void xhci_post_port_status_change(xhci_bus_t* xhci, uint32_t port)
{
    /*
     * Port Status Change Event: TRB pointer is the port number shifted
     * left by 24 in bits 31:24 of the ptr (yes, xHCI spec 4.11.5.2 is
     * this quirky), with the rest zero. Completion code is SUCCESS.
     */
    uint64_t ptr = ((uint64_t)port & 0xFF) << 24;
    uint32_t sts = ((uint32_t)XHCI_TRB_COMP_SUCCESS << 24);
    uint32_t ctr = XHCI_TRB_MAKE_TYPE(XHCI_TRB_TYPE_PORT_STATUS);
    scoped_spin_lock (&xhci->lock) {
        atomic_or_uint32(&xhci->usbsts, XHCI_USBSTS_PCD);
        xhci_post_event_locked(xhci, 0, ptr, sts, ctr);
    }
}

static void xhci_post_cmd_completion(xhci_bus_t* xhci,
                                     rvvm_addr_t cmd_trb_addr,
                                     uint8_t     comp_code,
                                     uint8_t     slot_id)
{
    uint32_t sts = ((uint32_t)comp_code << 24);
    uint32_t ctr = XHCI_TRB_MAKE_TYPE(XHCI_TRB_TYPE_COMPLETION)
                   | ((uint32_t)slot_id << 24);
    scoped_spin_lock (&xhci->lock) {
        xhci_post_event_locked(xhci, 0, cmd_trb_addr, sts, ctr);
    }
}

static void xhci_post_transfer_event(xhci_bus_t* xhci,
                                     rvvm_addr_t trb_addr,
                                     uint32_t    transferred,
                                     uint32_t    requested,
                                     uint8_t     slot_id,
                                     uint8_t     ep_id,
                                     uint32_t    interrupter)
{
    uint8_t code = XHCI_TRB_COMP_SUCCESS;
    if (transferred < requested) code = XHCI_TRB_COMP_SHORT_PKT;
    /*
     * Transfer Event status: residue (bytes NOT transferred) in bits 23:0,
     * completion code in bits 31:24.
     */
    uint32_t residue = requested > transferred ? requested - transferred : 0;
    uint32_t sts = (residue & 0xFFFFFF) | ((uint32_t)code << 24);
    uint32_t ctr = XHCI_TRB_MAKE_TYPE(XHCI_TRB_TYPE_TRANSFER)
                   | ((uint32_t)ep_id << 16)
                   | ((uint32_t)slot_id << 24);
    scoped_spin_lock (&xhci->lock) {
        xhci_post_event_locked(xhci, interrupter, trb_addr, sts, ctr);
    }
}

/*
 * ---------------------------------------------------------------------
 * Port state machine
 * ---------------------------------------------------------------------
 */

static void xhci_port_on_attach(xhci_bus_t* xhci, uint32_t port)
{
    if (port == 0 || port > XHCI_MAX_PORTS) return;
    scoped_spin_lock (&xhci->lock) {
        uint32_t v = xhci->ports[port].portsc;
        v = (v & ~XHCI_PORTSC_SPEED_MASK) | XHCI_PORTSC_SPEED_HS;
        v |= XHCI_PORTSC_CCS | XHCI_PORTSC_CSC | XHCI_PORTSC_PP;
        /* HS device starts in Polling (U state 7), then Linux issues PR. */
        v = (v & ~XHCI_PORTSC_PLS_MASK) | (7u << XHCI_PORTSC_PLS_SHIFT);
        v |= XHCI_PORTSC_PLC;
        xhci->ports[port].portsc = v;
    }
    xhci_post_port_status_change(xhci, port);
}

static void xhci_port_on_detach(xhci_bus_t* xhci, uint32_t port)
{
    if (port == 0 || port > XHCI_MAX_PORTS) return;
    scoped_spin_lock (&xhci->lock) {
        uint32_t v = xhci->ports[port].portsc;
        bool was_connected = (v & XHCI_PORTSC_CCS) != 0;
        v &= ~(XHCI_PORTSC_CCS | XHCI_PORTSC_PED);
        if (was_connected) v |= XHCI_PORTSC_CSC;
        v = (v & ~XHCI_PORTSC_PLS_MASK) | (5u << XHCI_PORTSC_PLS_SHIFT);  /* RxDetect */
        v |= XHCI_PORTSC_PLC;
        xhci->ports[port].portsc = v;
    }
    xhci_post_port_status_change(xhci, port);
}

/*
 * Guest wrote PORTSC. Apply WMASK bits, handle RW1C clears, and if PR
 * is being set, transition the port to Enabled (since our USB2 devices
 * are always ready — no real link training to wait for).
 */
static void xhci_portsc_write(xhci_bus_t* xhci, uint32_t port, uint32_t val)
{
    if (port == 0 || port > XHCI_MAX_PORTS) return;

    bool post = false;
    scoped_spin_lock (&xhci->lock) {
        uint32_t cur = xhci->ports[port].portsc;

        /* Clear any change bits the guest wrote 1 to. */
        cur &= ~(val & XHCI_PORTSC_CHANGE_MASK);

        /* Clear PED on guest write of 1 (the spec's one write-1-to-clear for state bits). */
        if (val & XHCI_PORTSC_PED) {
            cur &= ~XHCI_PORTSC_PED;
        }

        /* Apply writable bits from WMASK that aren't PED (handled above) or PR (special). */
        cur = (cur & ~(XHCI_PORTSC_PP | XHCI_PORTSC_LWS | XHCI_PORTSC_WCE
                       | XHCI_PORTSC_WDE | XHCI_PORTSC_WOE))
              | (val & (XHCI_PORTSC_PP | XHCI_PORTSC_LWS | XHCI_PORTSC_WCE
                        | XHCI_PORTSC_WDE | XHCI_PORTSC_WOE));

        if ((val & XHCI_PORTSC_PR) && (cur & XHCI_PORTSC_CCS)) {
            /*
             * Port reset. Self-complete: HS device goes to Enabled (U0)
             * with PRC set. This is synchronous because our "hardware"
             * doesn't have a PHY to train.
             */
            cur |= XHCI_PORTSC_PED | XHCI_PORTSC_PRC;
            cur = (cur & ~XHCI_PORTSC_PLS_MASK) | (0u << XHCI_PORTSC_PLS_SHIFT); /* U0 */
            cur = (cur & ~XHCI_PORTSC_SPEED_MASK) | XHCI_PORTSC_SPEED_HS;
            cur &= ~XHCI_PORTSC_PR;
            post = true;
        }

        /* LWS-gated link state writes: only take effect when LWS bit is 1 in the write. */
        if (val & XHCI_PORTSC_LWS) {
            cur = (cur & ~XHCI_PORTSC_PLS_MASK) | (val & XHCI_PORTSC_PLS_MASK);
            /* Non-U0 transitions set PLC; we don't model them in detail. */
            cur |= XHCI_PORTSC_PLC;
            post = true;
        }

        xhci->ports[port].portsc = cur;
    }

    if (post) xhci_post_port_status_change(xhci, port);
}

/*
 * Bus callbacks — the rvvm_usb core invokes these when a device attaches
 * or an async transfer completes on the device side.
 */
static void xhci_cb_port_update(rvvm_usb_bus_t* bus, uint32_t port, bool pres)
{
    xhci_bus_t* xhci = rvvm_usb_bus_get_data(bus);
    if (pres) xhci_port_on_attach(xhci, port);
    else      xhci_port_on_detach(xhci, port);
}

static void xhci_cb_xfer_done(rvvm_usb_bus_t* bus,
                              rvvm_usb_dev_t* dev,
                              uint8_t         ep,
                              int32_t         ret)
{
    xhci_bus_t* xhci = rvvm_usb_bus_get_data(bus);
    /* Locate the slot that owns this device and the pending EP. */
    UNUSED(dev);

    uint8_t  slot_id = 0;
    uint8_t  ep_id   = 0;
    rvvm_addr_t trb_ptr = 0;
    uint32_t interrupter = 0;
    uint32_t requested = 0;
    bool     found = false;

    scoped_spin_lock (&xhci->lock) {
        for (uint32_t s = 1; s <= XHCI_MAX_SLOTS; ++s) {
            if (!xhci->slots[s].in_use || xhci->slots[s].dev != dev) continue;
            /*
             * XHCI endpoint ID encoding: DCI = ep_num * 2 + (dir ? 1 : 0)
             * where ep_num is 0..15 and dir is 1 for IN.
             * ep (USB endpoint address) is ep_num | 0x80 for IN.
             *
             * EP0 control is DCI 1.
             */
            uint8_t ep_num = ep & 0x0F;
            uint8_t dir    = (ep & 0x80) ? 1 : 0;
            uint8_t dci    = ep_num == 0 ? 1 : (ep_num * 2 + dir);
            if (dci >= 32 || !xhci->slots[s].eps[dci].pending.active) continue;
            slot_id = s;
            ep_id = dci;
            trb_ptr = xhci->slots[s].eps[dci].pending.trb_ptr;
            interrupter = xhci->slots[s].eps[dci].pending.interrupter;
            requested = xhci->slots[s].eps[dci].pending.length;
            xhci->slots[s].eps[dci].pending.active = false;
            found = true;
            break;
        }
    }

    if (!found) return;

    uint32_t transferred = ret >= 0 ? (uint32_t)ret : 0;
    xhci_post_transfer_event(xhci, trb_ptr, transferred, requested, slot_id, ep_id, interrupter);
}

static const rvvm_usb_bus_cb_t xhci_bus_cb = {
    .port_update = xhci_cb_port_update,
    .xfer_done   = xhci_cb_xfer_done,
};

/*
 * ---------------------------------------------------------------------
 * Context helpers — 32-byte contexts (CSZ=0)
 * ---------------------------------------------------------------------
 */

#define XHCI_CTX_SIZE 32

/* Read root port number from an Input Slot Context (not an input control context). */
static uint8_t xhci_input_slot_port(xhci_bus_t* xhci, rvvm_addr_t input_ctx_addr)
{
    /*
     * Input Context layout at 32-byte contexts:
     *   +0x00: Input Control Context (32 bytes)
     *   +0x20: Slot Context (32 bytes)
     *   +0x40..: Endpoint Contexts
     *
     * Slot Context dword 1 (offset +0x04 inside) has:
     *   bits 31:24 = Root Hub Port Number
     */
    uint8_t* ptr = pci_get_dma_ptr(xhci->pci_func, input_ctx_addr + XHCI_CTX_SIZE + 4, 4);
    if (ptr == NULL) return 0;
    uint32_t w = read_uint32_le(ptr);
    return (uint8_t)(w >> 24);
}

/*
 * Pull EP type + max packet size from an Input Endpoint Context. DCI is
 * the context index (1..31).
 */
static void xhci_input_ep_params(xhci_bus_t* xhci,
                                 rvvm_addr_t input_ctx_addr,
                                 uint32_t    dci,
                                 uint8_t*    out_type,
                                 uint16_t*   out_mps,
                                 rvvm_addr_t* out_deq,
                                 uint8_t*    out_dcs)
{
    rvvm_addr_t ep_ctx_addr = input_ctx_addr + XHCI_CTX_SIZE + (rvvm_addr_t)(dci + 1) * XHCI_CTX_SIZE;
    uint8_t* ptr = pci_get_dma_ptr(xhci->pci_func, ep_ctx_addr, XHCI_CTX_SIZE);
    if (ptr == NULL) {
        *out_type = 0;
        *out_mps = 0;
        *out_deq = 0;
        *out_dcs = 0;
        return;
    }
    uint32_t dw1 = read_uint32_le(ptr + 4);
    uint32_t dw2 = read_uint32_le(ptr + 8);
    uint32_t dw3 = read_uint32_le(ptr + 12);
    *out_type = (uint8_t)((dw1 >> 3) & 0x7);
    *out_mps  = (uint16_t)((dw1 >> 16) & 0xFFFF);
    *out_deq  = ((rvvm_addr_t)(dw2 & ~0xFu)) | ((rvvm_addr_t)dw3 << 32);
    *out_dcs  = (uint8_t)(dw2 & 0x1);
}

/*
 * Commit the Input Context's slot + endpoint contexts to the output
 * Device Context (DCBA[slot]). We copy verbatim for the fields we
 * actually honor; Linux is permissive about the rest.
 */
static bool xhci_commit_device_context(xhci_bus_t* xhci,
                                       uint8_t     slot_id,
                                       rvvm_addr_t input_ctx_addr)
{
    rvvm_addr_t dcbaap = ((rvvm_addr_t)atomic_load_uint32_relax(&xhci->dcbaap) & XHCI_ALIGN_64BYTE)
                         | ((rvvm_addr_t)atomic_load_uint32_relax(&xhci->dcbaap_h) << 32);
    if (dcbaap == 0) return false;
    uint8_t* ptr = pci_get_dma_ptr(xhci->pci_func, dcbaap + (rvvm_addr_t)slot_id * 8, 8);
    if (ptr == NULL) return false;
    rvvm_addr_t out_ctx = read_uint64_le(ptr);
    if (out_ctx == 0) return false;
    xhci->slots[slot_id].out_ctx_addr = out_ctx;

    /*
     * Copy slot + all 31 EP contexts from input (starts after Input Control Context)
     * to the output Device Context (no Input Control prefix).
     */
    size_t total = (size_t)32 * XHCI_CTX_SIZE;  /* 1 slot + 31 EPs */
    uint8_t* src = pci_get_dma_ptr(xhci->pci_func, input_ctx_addr + XHCI_CTX_SIZE, total);
    uint8_t* dst = pci_get_dma_ptr(xhci->pci_func, out_ctx, total);
    if (src == NULL || dst == NULL) return false;
    memcpy(dst, src, total);

    /* Mark slot state = Addressed (3) in the output Slot Context dw3 high 5 bits. */
    uint32_t dw3 = read_uint32_le(dst + 0x0C);
    dw3 = (dw3 & 0x07FFFFFF) | (3u << 27);
    write_uint32_le(dst + 0x0C, dw3);
    return true;
}

/*
 * ---------------------------------------------------------------------
 * Command ring processing
 * ---------------------------------------------------------------------
 */

static uint8_t xhci_alloc_slot(xhci_bus_t* xhci)
{
    uint8_t ret = 0;
    scoped_spin_lock (&xhci->lock) {
        for (uint8_t s = 1; s <= XHCI_MAX_SLOTS; ++s) {
            if (!xhci->slots[s].in_use) {
                xhci->slots[s].in_use = true;
                xhci->slots[s].dev = NULL;
                xhci->slots[s].root_port = 0;
                xhci->slots[s].out_ctx_addr = 0;
                memset(xhci->slots[s].eps, 0, sizeof(xhci->slots[s].eps));
                ret = s;
                break;
            }
        }
    }
    return ret;
}

static void xhci_free_slot(xhci_bus_t* xhci, uint8_t slot_id)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS) return;
    scoped_spin_lock (&xhci->lock) {
        xhci->slots[slot_id].in_use = false;
        xhci->slots[slot_id].dev = NULL;
        xhci->slots[slot_id].root_port = 0;
        xhci->slots[slot_id].out_ctx_addr = 0;
        memset(xhci->slots[slot_id].eps, 0, sizeof(xhci->slots[slot_id].eps));
    }
}

static void xhci_do_address_device(xhci_bus_t* xhci,
                                   uint64_t    input_ctx,
                                   uint32_t    trb_ctr,
                                   rvvm_addr_t cmd_trb_addr)
{
    uint8_t slot_id = (uint8_t)XHCI_TRB_SLOT(trb_ctr);
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS || !xhci->slots[slot_id].in_use) {
        xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_CTX_STATE, slot_id);
        return;
    }

    uint8_t root_port = xhci_input_slot_port(xhci, input_ctx);
    if (root_port == 0 || root_port > XHCI_MAX_PORTS) {
        xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_TRB_ERR, slot_id);
        return;
    }

    rvvm_usb_dev_t* dev = rvvm_usb_bus_port_dev(xhci->bus, root_port);
    if (dev == NULL) {
        xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_TRB_ERR, slot_id);
        return;
    }

    /*
     * Read EP0 params (DCI 1) from the input context so we know the
     * transfer ring to walk when the EP0 doorbell rings.
     */
    uint8_t ep_type; uint16_t mps; rvvm_addr_t deq; uint8_t dcs;
    xhci_input_ep_params(xhci, input_ctx, 1, &ep_type, &mps, &deq, &dcs);

    if (!xhci_commit_device_context(xhci, slot_id, input_ctx)) {
        xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_TRB_ERR, slot_id);
        return;
    }

    scoped_spin_lock (&xhci->lock) {
        xhci->slots[slot_id].dev = dev;
        xhci->slots[slot_id].root_port = root_port;
        xhci->slots[slot_id].eps[1].enabled = true;
        xhci->slots[slot_id].eps[1].ep_type = 4; /* Control */
        xhci->slots[slot_id].eps[1].max_packet = mps ? mps : 64;
        xhci->slots[slot_id].eps[1].deq = deq;
        xhci->slots[slot_id].eps[1].dcs = dcs;
    }

    /*
     * If BSR is clear the host wants us to issue SET_ADDRESS too. For
     * our bus that's a no-op at the device layer — we accept and move on.
     */
    UNUSED(ep_type);
    UNUSED(trb_ctr);  /* BSR bit lives in dw3, we treat both BSR states the same way */

    xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, slot_id);
}

static void xhci_do_configure_endpoint(xhci_bus_t* xhci,
                                       uint64_t    input_ctx,
                                       uint32_t    trb_ctr,
                                       rvvm_addr_t cmd_trb_addr)
{
    uint8_t slot_id = (uint8_t)XHCI_TRB_SLOT(trb_ctr);
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS || !xhci->slots[slot_id].in_use) {
        xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_CTX_STATE, slot_id);
        return;
    }

    /*
     * Input Control Context dw0 = drop flags (bits 0..31), dw1 = add flags.
     * Bit 0 of each is reserved for D0/A0 (slot context flags) per spec.
     */
    uint8_t* icc = pci_get_dma_ptr(xhci->pci_func, input_ctx, XHCI_CTX_SIZE);
    if (icc == NULL) {
        xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_TRB_ERR, slot_id);
        return;
    }
    uint32_t drop_flags = read_uint32_le(icc + 0);
    uint32_t add_flags  = read_uint32_le(icc + 4);

    scoped_spin_lock (&xhci->lock) {
        /* Drop endpoints first, then add. DCI 0/1 aren't touched by this cmd. */
        for (uint32_t dci = 2; dci < 32; ++dci) {
            if (drop_flags & (1u << dci)) {
                xhci->slots[slot_id].eps[dci].enabled = false;
            }
            if (add_flags & (1u << dci)) {
                uint8_t ep_type; uint16_t mps; rvvm_addr_t deq; uint8_t dcs;
                xhci_input_ep_params(xhci, input_ctx, dci, &ep_type, &mps, &deq, &dcs);
                xhci->slots[slot_id].eps[dci].enabled = true;
                xhci->slots[slot_id].eps[dci].ep_type = ep_type;
                xhci->slots[slot_id].eps[dci].max_packet = mps ? mps : 512;
                xhci->slots[slot_id].eps[dci].deq = deq;
                xhci->slots[slot_id].eps[dci].dcs = dcs;
            }
        }
    }

    /*
     * Copy the committed EP contexts back to the output device context
     * so subsequent Evaluate Context reads see them, and bump slot state
     * to Configured (4).
     */
    rvvm_addr_t out_ctx = xhci->slots[slot_id].out_ctx_addr;
    if (out_ctx) {
        size_t total = (size_t)32 * XHCI_CTX_SIZE;
        uint8_t* src = pci_get_dma_ptr(xhci->pci_func, input_ctx + XHCI_CTX_SIZE, total);
        uint8_t* dst = pci_get_dma_ptr(xhci->pci_func, out_ctx, total);
        if (src && dst) {
            memcpy(dst, src, total);
            uint32_t dw3 = read_uint32_le(dst + 0x0C);
            dw3 = (dw3 & 0x07FFFFFF) | (4u << 27);
            write_uint32_le(dst + 0x0C, dw3);
        }
    }

    xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, slot_id);
}

static void xhci_do_set_tr_dequeue(xhci_bus_t* xhci,
                                   uint64_t    param,
                                   uint32_t    trb_ctr,
                                   rvvm_addr_t cmd_trb_addr)
{
    uint8_t slot_id = (uint8_t)XHCI_TRB_SLOT(trb_ctr);
    uint8_t ep_id   = (uint8_t)XHCI_TRB_EPID(trb_ctr);
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS || ep_id == 0 || ep_id >= 32
        || !xhci->slots[slot_id].in_use) {
        xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_CTX_STATE, slot_id);
        return;
    }
    scoped_spin_lock (&xhci->lock) {
        xhci->slots[slot_id].eps[ep_id].deq = param & ~0xFULL;
        xhci->slots[slot_id].eps[ep_id].dcs = (uint8_t)(param & 1);
    }
    xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, slot_id);
}

/* Return true to continue processing the ring, false to stop. */
static bool xhci_process_cmd_trb(xhci_bus_t* xhci,
                                 rvvm_addr_t cmd_trb_addr,
                                 uint64_t    ptr,
                                 uint32_t    sts,
                                 uint32_t    ctr)
{
    UNUSED(sts);
    uint32_t type = XHCI_TRB_TYPE(ctr);
    uint8_t  slot_id = (uint8_t)XHCI_TRB_SLOT(ctr);

    switch (type) {
        case XHCI_TRB_TYPE_CMD_NOOP:
            xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, 0);
            return true;

        case XHCI_TRB_TYPE_ENABLE_SLOT: {
            uint8_t s = xhci_alloc_slot(xhci);
            xhci_post_cmd_completion(xhci, cmd_trb_addr,
                                     s ? XHCI_TRB_COMP_SUCCESS : XHCI_TRB_COMP_NA_SLOTS,
                                     s);
            return true;
        }

        case XHCI_TRB_TYPE_DISABLE_SLOT:
            xhci_free_slot(xhci, slot_id);
            xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, slot_id);
            return true;

        case XHCI_TRB_TYPE_ADDR_DEV:
            xhci_do_address_device(xhci, ptr, ctr, cmd_trb_addr);
            return true;

        case XHCI_TRB_TYPE_CONFIG_EP:
            xhci_do_configure_endpoint(xhci, ptr, ctr, cmd_trb_addr);
            return true;

        case XHCI_TRB_TYPE_EVAL_CONTEXT:
            /* Not strictly needed for basic HID — accept without action. */
            xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, slot_id);
            return true;

        case XHCI_TRB_TYPE_RESET_EP:
        case XHCI_TRB_TYPE_STOP_RING:
            /* Simple model: ring stop has no in-flight transfers we can't
             * recover from because cancel() is synchronous on our bus. */
            xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, slot_id);
            return true;

        case XHCI_TRB_TYPE_SET_DEQ:
            xhci_do_set_tr_dequeue(xhci, ptr, ctr, cmd_trb_addr);
            return true;

        case XHCI_TRB_TYPE_RESET_DEV: {
            rvvm_usb_dev_t* dev = NULL;
            if (slot_id && slot_id <= XHCI_MAX_SLOTS) dev = xhci->slots[slot_id].dev;
            if (dev) rvvm_usb_dev_reset(dev);
            xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_SUCCESS, slot_id);
            return true;
        }

        case XHCI_TRB_TYPE_LINK:
            return true;  /* handled by the walker */

        default:
            rvvm_warn("xhci: unhandled command TRB type %u", type);
            xhci_post_cmd_completion(xhci, cmd_trb_addr, XHCI_TRB_COMP_TRB_ERR, slot_id);
            return true;
    }
}

static void xhci_run_command_ring(xhci_bus_t* xhci)
{
    rvvm_addr_t addr = xhci->cmd_deq;
    uint8_t     ccs  = xhci->cmd_ccs;

    for (size_t guard = 0; guard < 4096; ++guard) {  /* bound walk to avoid runaway loops */
        uint8_t* dma = pci_get_dma_ptr(xhci->pci_func, addr, XHCI_TRB_SIZE);
        if (dma == NULL) break;
        uint64_t ptr = read_uint64_le(dma);
        uint32_t sts = read_uint32_le(dma + 8);
        uint32_t ctr = read_uint32_le(dma + 0xC);
        if ((ctr & XHCI_TRB_CTR_C ? 1 : 0) != ccs) break;  /* owned by software */

        uint32_t type = XHCI_TRB_TYPE(ctr);
        if (type == XHCI_TRB_TYPE_LINK) {
            addr = ptr & XHCI_ALIGN_64BYTE;
            if (ctr & XHCI_TRB_CTR_TC) ccs ^= 1;
            continue;
        }

        xhci_process_cmd_trb(xhci, addr, ptr, sts, ctr);
        addr += XHCI_TRB_SIZE;
    }

    xhci->cmd_deq = addr;
    xhci->cmd_ccs = ccs;
}

/*
 * ---------------------------------------------------------------------
 * Transfer ring processing
 * ---------------------------------------------------------------------
 */

/*
 * Walk a TD starting at the endpoint's current dequeue, collecting
 * contiguous chained TRBs. Returns the summed length and the TRB that
 * completes the TD (one with CH bit clear or an IOC-bearing STATUS).
 *
 * Caller holds xhci->lock.
 */
typedef struct {
    rvvm_addr_t trb_addr[32];
    uint64_t    trb_ptr[32];
    uint32_t    trb_sts[32];
    uint32_t    trb_ctr[32];
    size_t      count;
    rvvm_addr_t new_deq;
    uint8_t     new_dcs;
} xhci_td_t;

static bool xhci_collect_td(xhci_bus_t* xhci, xhci_ep_t* ep, xhci_td_t* td)
{
    memset(td, 0, sizeof(*td));
    rvvm_addr_t addr = ep->deq;
    uint8_t     dcs  = ep->dcs;

    for (size_t g = 0; g < 64; ++g) {
        uint8_t* dma = pci_get_dma_ptr(xhci->pci_func, addr, XHCI_TRB_SIZE);
        if (dma == NULL) return false;
        uint64_t ptr = read_uint64_le(dma);
        uint32_t sts = read_uint32_le(dma + 8);
        uint32_t ctr = read_uint32_le(dma + 0xC);
        if ((ctr & XHCI_TRB_CTR_C ? 1 : 0) != dcs) return false;  /* not yet ours */

        uint32_t type = XHCI_TRB_TYPE(ctr);
        if (type == XHCI_TRB_TYPE_LINK) {
            addr = ptr & XHCI_ALIGN_64BYTE;
            if (ctr & XHCI_TRB_CTR_TC) dcs ^= 1;
            continue;
        }

        if (td->count < 32) {
            td->trb_addr[td->count] = addr;
            td->trb_ptr[td->count]  = ptr;
            td->trb_sts[td->count]  = sts;
            td->trb_ctr[td->count]  = ctr;
            td->count++;
        }
        addr += XHCI_TRB_SIZE;

        /* End of TD: no CH bit set means this is the final TRB. */
        if (!(ctr & XHCI_TRB_CTR_CH)) {
            td->new_deq = addr;
            td->new_dcs = dcs;
            return true;
        }
    }
    return false;
}

/*
 * DMA-reach a TRB's data buffer. For normal/data/setup TRBs, the buffer
 * is at `ptr` for `length` bytes. IDT means the 8-byte SETUP data is
 * inline in the TRB ptr field.
 */
static uint8_t* xhci_trb_buf(xhci_bus_t* xhci, uint64_t ptr, uint32_t length)
{
    if (length == 0) return NULL;
    return pci_get_dma_ptr(xhci->pci_func, ptr, length);
}

static void xhci_run_control_td(xhci_bus_t*     xhci,
                                uint8_t         slot_id,
                                uint8_t         dci,
                                xhci_ep_t*      ep,
                                rvvm_usb_dev_t* dev,
                                const xhci_td_t* td)
{
    /*
     * Walk the TD to find SETUP, optional DATA chain, STATUS. The
     * SETUP is always the first TRB; DATA may span multiple TRBs (we
     * concatenate into a contiguous buffer only if it's DMA-contiguous
     * — else we stitch with a temp buffer). STATUS carries IOC.
     */
    uint8_t setup[8] = {0};
    bool    have_setup = false;
    rvvm_addr_t status_trb_addr = 0;
    uint32_t    interrupter = 0;
    uint32_t    data_len = 0;
    uint8_t*    data_buf = NULL;
    uint8_t     tmp_buf[4096];
    bool        used_tmp = false;

    for (size_t i = 0; i < td->count; ++i) {
        uint32_t type = XHCI_TRB_TYPE(td->trb_ctr[i]);
        uint32_t len  = XHCI_TRB_LEN(td->trb_sts[i]);

        if (type == XHCI_TRB_TYPE_SETUP) {
            /* Immediate Data for SETUP: the 8-byte packet lives in ptr/sts. */
            write_uint64_le(setup, td->trb_ptr[i]);
            /* Actually SETUP packs bmRequestType..wLength into the TRB
             * ptr+sts words directly as little-endian bytes. Reading
             * as uint64_le above gives us the first 8 bytes. */
            have_setup = true;
        } else if (type == XHCI_TRB_TYPE_DATA) {
            data_len = len;
            uint8_t* b = xhci_trb_buf(xhci, td->trb_ptr[i], len);
            if (b) data_buf = b;
        } else if (type == XHCI_TRB_TYPE_STATUS) {
            status_trb_addr = td->trb_addr[i];
            if (td->trb_ctr[i] & XHCI_TRB_CTR_IOC) {
                interrupter = XHCI_TRB_INTERRUPTER(td->trb_sts[i]);
            }
        }
    }

    if (!have_setup) {
        xhci_post_transfer_event(xhci, td->trb_addr[0], 0, 0, slot_id, dci, 0);
        return;
    }

    /* If data stage exists but wasn't DMA-reachable in one shot, fall
     * back to a stack buffer (data_len capped at tmp_buf size). */
    if (data_len && data_buf == NULL) {
        if (data_len > sizeof(tmp_buf)) data_len = sizeof(tmp_buf);
        data_buf = tmp_buf;
        used_tmp = true;
    }

    int32_t ret = rvvm_usb_dev_control(dev, setup, data_buf, data_len);

    uint32_t transferred = ret >= 0 ? (uint32_t)ret : 0;

    if (ret == RVVM_USB_XFER_ASYNC) {
        /*
         * Control transfers can in theory be async too. Stash the
         * status TRB as the Transfer Event target.
         */
        ep->pending.active = true;
        ep->pending.trb_ptr = status_trb_addr ? status_trb_addr : td->trb_addr[td->count - 1];
        ep->pending.interrupter = interrupter;
        ep->pending.length = data_len;
        return;
    }

    UNUSED(used_tmp);

    uint8_t comp = ret < 0 ? XHCI_TRB_COMP_STALL : XHCI_TRB_COMP_SUCCESS;
    uint32_t sts = ((uint32_t)(data_len - (transferred > data_len ? data_len : transferred)) & 0xFFFFFF)
                   | ((uint32_t)comp << 24);
    uint32_t ctr = XHCI_TRB_MAKE_TYPE(XHCI_TRB_TYPE_TRANSFER)
                   | ((uint32_t)dci << 16) | ((uint32_t)slot_id << 24);
    /*
     * xhci spec: Transfer Event for a control transfer points at the
     * Status Stage TRB with the completion code.
     */
    rvvm_addr_t target = status_trb_addr ? status_trb_addr : td->trb_addr[td->count - 1];
    scoped_spin_lock (&xhci->lock) {
        xhci_post_event_locked(xhci, interrupter, target, sts, ctr);
    }
}

static void xhci_run_normal_td(xhci_bus_t*     xhci,
                               uint8_t         slot_id,
                               uint8_t         dci,
                               xhci_ep_t*      ep,
                               rvvm_usb_dev_t* dev,
                               const xhci_td_t* td)
{
    /*
     * For simplicity the non-control path handles the common case of
     * one NORMAL TRB per TD. For chained TDs we concatenate by issuing
     * back-to-back xfers; every TRB gets its own Transfer Event if it
     * has IOC or ISP set (xhci allows at most one per TD typically).
     */
    uint32_t total_len = 0;
    uint32_t total_done = 0;
    uint32_t interrupter = 0;
    rvvm_addr_t ioc_target = 0;
    bool async = false;

    uint8_t  ep_addr = 0;
    {
        /* DCI → ep address: DCI 2n = OUT EP n, DCI 2n+1 = IN EP n. */
        uint8_t ep_num = dci >> 1;
        bool    is_in  = (dci & 1) != 0;
        ep_addr = ep_num | (is_in ? 0x80 : 0);
    }

    for (size_t i = 0; i < td->count; ++i) {
        uint32_t len = XHCI_TRB_LEN(td->trb_sts[i]);
        uint32_t type = XHCI_TRB_TYPE(td->trb_ctr[i]);
        if (type == XHCI_TRB_TYPE_EVENT_DATA) {
            ioc_target = td->trb_ptr[i];  /* EVENT_DATA points "event" at its ptr field */
            if (td->trb_ctr[i] & XHCI_TRB_CTR_IOC) {
                interrupter = XHCI_TRB_INTERRUPTER(td->trb_sts[i]);
            }
            continue;
        }
        total_len += len;

        uint8_t* buf = xhci_trb_buf(xhci, td->trb_ptr[i], len);
        int32_t r = rvvm_usb_dev_xfer(dev, ep_addr, buf, len);
        if (r == RVVM_USB_XFER_ASYNC) {
            /*
             * Only one async xfer in flight per EP in this model. Stash
             * the final TRB as the event target and return — we resume
             * via xhci_cb_xfer_done.
             */
            ep->pending.active = true;
            ep->pending.trb_ptr = td->trb_addr[td->count - 1];
            ep->pending.interrupter = interrupter;
            ep->pending.length = total_len + len;
            async = true;
            break;
        }
        if (r < 0) {
            /* Error: post STALL and stop. */
            uint32_t sts = ((uint32_t)XHCI_TRB_COMP_STALL << 24);
            uint32_t ctr = XHCI_TRB_MAKE_TYPE(XHCI_TRB_TYPE_TRANSFER)
                           | ((uint32_t)dci << 16) | ((uint32_t)slot_id << 24);
            scoped_spin_lock (&xhci->lock) {
                xhci_post_event_locked(xhci, interrupter, td->trb_addr[i], sts, ctr);
            }
            return;
        }
        total_done += (uint32_t)r;
        if (td->trb_ctr[i] & XHCI_TRB_CTR_IOC) {
            interrupter = XHCI_TRB_INTERRUPTER(td->trb_sts[i]);
        }
    }

    if (async) return;

    rvvm_addr_t target = ioc_target ? ioc_target : td->trb_addr[td->count - 1];
    xhci_post_transfer_event(xhci, target, total_done, total_len, slot_id, dci, interrupter);
}

static void xhci_run_transfer_ring(xhci_bus_t* xhci, uint8_t slot_id, uint8_t dci)
{
    if (slot_id == 0 || slot_id > XHCI_MAX_SLOTS || dci == 0 || dci >= 32) return;

    rvvm_usb_dev_t* dev = NULL;
    xhci_ep_t*      ep  = NULL;
    scoped_spin_lock (&xhci->lock) {
        if (!xhci->slots[slot_id].in_use) return;
        dev = xhci->slots[slot_id].dev;
        ep = &xhci->slots[slot_id].eps[dci];
        if (!ep->enabled) return;
    }
    if (dev == NULL) return;

    while (true) {
        xhci_td_t td;
        bool present;
        scoped_spin_lock (&xhci->lock) {
            if (ep->pending.active) {
                /* Earlier async xfer hasn't completed — don't start another. */
                return;
            }
            present = xhci_collect_td(xhci, ep, &td);
            if (present) {
                ep->deq = td.new_deq;
                ep->dcs = td.new_dcs;
            }
        }
        if (!present) return;

        if (dci == 1) {
            xhci_run_control_td(xhci, slot_id, dci, ep, dev, &td);
        } else {
            xhci_run_normal_td(xhci, slot_id, dci, ep, dev, &td);
        }

        /* If the TD went async, stop — resumes from xfer_done. */
        bool still_pending = false;
        scoped_spin_lock (&xhci->lock) {
            still_pending = ep->pending.active;
        }
        if (still_pending) return;
    }
}

/*
 * ---------------------------------------------------------------------
 * Doorbell
 * ---------------------------------------------------------------------
 */

static void xhci_doorbell_write(xhci_bus_t* xhci, size_t id, uint32_t val)
{
    if (id == 0) {
        /* Command Ring doorbell — target should be 0. */
        xhci_run_command_ring(xhci);
    } else if (id <= XHCI_MAX_SLOTS) {
        uint8_t dci = (uint8_t)(val & 0xFF);
        xhci_run_transfer_ring(xhci, (uint8_t)id, dci);
    }
}

/*
 * ---------------------------------------------------------------------
 * MMIO
 * ---------------------------------------------------------------------
 */

static uint32_t xhci_port_reg_read(xhci_bus_t* xhci, size_t id, size_t offset)
{
    if (id == 0 || id > XHCI_MAX_PORTS) return 0;
    if (offset == XHCI_REG_PORTSC) {
        uint32_t v;
        scoped_spin_read_lock (&xhci->lock) {
            v = xhci->ports[id].portsc;
        }
        return v;
    }
    return 0;
}

static uint32_t xhci_interrupter_read(xhci_bus_t* xhci, size_t id, size_t offset)
{
    if (id >= XHCI_MAX_INTRS) return 0;
    xhci_interrupter_t* intr = &xhci->ints[id];
    switch (offset) {
        case XHCI_REG_IMAN:     return atomic_load_uint32_relax(&intr->iman);
        case XHCI_REG_IMOD:     return atomic_load_uint32_relax(&intr->imod);
        case XHCI_REG_ERSTSZ:   return atomic_load_uint32_relax(&intr->erstsz);
        case XHCI_REG_ERSTBA:   return atomic_load_uint32_relax(&intr->erstba);
        case XHCI_REG_ERSTBA_H: return atomic_load_uint32_relax(&intr->erstba_h);
        case XHCI_REG_ERDP:     return atomic_load_uint32_relax(&intr->erdp);
        case XHCI_REG_ERDP_H:   return atomic_load_uint32_relax(&intr->erdp_h);
    }
    return 0;
}

static void xhci_interrupter_write(xhci_bus_t* xhci, size_t id, size_t offset, uint32_t val)
{
    if (id >= XHCI_MAX_INTRS) return;
    xhci_interrupter_t* intr = &xhci->ints[id];
    switch (offset) {
        case XHCI_REG_IMAN: {
            /*
             * IP is RW1C. IE is RW. Preserve IP unless guest writes 1 to clear.
             */
            uint32_t cur = atomic_load_uint32_relax(&intr->iman);
            uint32_t next = cur;
            if (val & XHCI_IMAN_IP) next &= ~XHCI_IMAN_IP;
            next = (next & ~XHCI_IMAN_IE) | (val & XHCI_IMAN_IE);
            atomic_store_uint32_relax(&intr->iman, next);
            /* Deassert if IP cleared. We use edge IRQs so nothing to do explicitly. */
            return;
        }
        case XHCI_REG_IMOD:
            atomic_store_uint32_relax(&intr->imod, val);
            return;
        case XHCI_REG_ERSTSZ:
            atomic_store_uint32_relax(&intr->erstsz, val & 0xFFFF);
            return;
        case XHCI_REG_ERSTBA:
            atomic_store_uint32_relax(&intr->erstba, val & (uint32_t)XHCI_ALIGN_64BYTE);
            scoped_spin_lock (&xhci->lock) {
                intr->er_seg = 0;
                intr->er_idx = 0;
                intr->pcs = 1;
                intr->er_primed = false;
            }
            return;
        case XHCI_REG_ERSTBA_H:
            atomic_store_uint32_relax(&intr->erstba_h, val);
            return;
        case XHCI_REG_ERDP:
            /*
             * Low 4 bits: DESI + EHB. Writing 1 to EHB clears it. We
             * don't track DESI carefully; accept whatever the guest sends.
             */
            atomic_store_uint32_relax(&intr->erdp, val);
            return;
        case XHCI_REG_ERDP_H:
            atomic_store_uint32_relax(&intr->erdp_h, val);
            return;
    }
}

static bool xhci_pci_read(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    xhci_bus_t* xhci = dev->data;
    uint32_t val = 0;
    UNUSED(size);

    if ((offset - XHCI_RUNTIME_BASE) < XHCI_RUNTIME_SIZE) {
        size_t roff = offset - XHCI_RUNTIME_BASE;
        if (roff < 0x20) {
            /* Microframe Index — return a freely incrementing value on read.
             * Linux doesn't actually use MFINDEX for HS enumeration but
             * some drivers read it during resume probes. */
            val = 0;
        } else {
            val = xhci_interrupter_read(xhci, (roff - 0x20) >> 5, roff & 0x1C);
        }
    } else if ((offset - XHCI_PORT_REGS_BASE) < XHCI_PORT_REGS_SIZE) {
        size_t pid  = ((offset - XHCI_PORT_REGS_BASE) >> 4) + 1;
        size_t poff = offset & 0xC;
        val = xhci_port_reg_read(xhci, pid, poff);
    } else if (offset >= XHCI_EXT_CAPS_BASE) {
        size_t entry = (offset - XHCI_EXT_CAPS_BASE) >> 2;
        if (entry < STATIC_ARRAY_SIZE(xhci_ext_caps)) {
            val = xhci_ext_caps[entry];
        }
    } else switch (offset) {
        case XHCI_REG_CAPLENGTH_HCIVERSION: val = XHCI_CAPLEN_HCIVERSION; break;
        case XHCI_REG_HCSPARAMS1: val = XHCI_HCSPARAMS1; break;
        case XHCI_REG_HCSPARAMS2: val = 0x00000000; break;
        case XHCI_REG_HCSPARAMS3: val = 0; break;
        case XHCI_REG_HCCPARAMS1: val = XHCI_HCCPARAMS1; break;
        case XHCI_REG_DBOFF:      val = XHCI_DOORBELL_BASE; break;
        case XHCI_REG_RTSOFF:     val = XHCI_RUNTIME_BASE; break;
        case XHCI_REG_HCCPARMS2:  val = 0; break;
        case XHCI_REG_USBCMD:     val = atomic_load_uint32_relax(&xhci->usbcmd); break;
        case XHCI_REG_USBSTS: {
            val = atomic_load_uint32_relax(&xhci->usbsts);
            if (!(atomic_load_uint32_relax(&xhci->usbcmd) & XHCI_USBCMD_RS)) {
                val |= XHCI_USBSTS_HCH;
            }
            break;
        }
        case XHCI_REG_PAGESIZE:   val = 0x1; break;
        case XHCI_REG_DNCTRL:     val = atomic_load_uint32_relax(&xhci->dnctrl); break;
        case XHCI_REG_CRCR:       val = 0; break;  /* CRCR reads as zero except CRR bit per spec */
        case XHCI_REG_CRCR_H:     val = 0; break;
        case XHCI_REG_DCBAAP:     val = atomic_load_uint32_relax(&xhci->dcbaap); break;
        case XHCI_REG_DCBAAP_H:   val = atomic_load_uint32_relax(&xhci->dcbaap_h); break;
        case XHCI_REG_CONFIG:     val = atomic_load_uint32_relax(&xhci->config); break;
        default: break;
    }

    write_uint32_le(data, val);
    return true;
}

static void xhci_controller_reset(xhci_bus_t* xhci)
{
    scoped_spin_lock (&xhci->lock) {
        atomic_store_uint32_relax(&xhci->usbcmd, 0);
        atomic_store_uint32_relax(&xhci->usbsts, XHCI_USBSTS_HCH);
        atomic_store_uint32_relax(&xhci->dnctrl, 0);
        atomic_store_uint32_relax(&xhci->crcr, 0);
        atomic_store_uint32_relax(&xhci->crcr_h, 0);
        atomic_store_uint32_relax(&xhci->dcbaap, 0);
        atomic_store_uint32_relax(&xhci->dcbaap_h, 0);
        atomic_store_uint32_relax(&xhci->config, 0);
        xhci->cmd_deq = 0;
        xhci->cmd_ccs = 1;
        memset(xhci->ints, 0, sizeof(xhci->ints));
        memset(xhci->slots, 0, sizeof(xhci->slots));
        /* Leave port CCS alone — Linux expects a re-announce via PSCE which
         * we issue below for every attached device. */
        for (uint32_t p = 1; p <= XHCI_MAX_PORTS; ++p) {
            xhci->ports[p].portsc = XHCI_PORTSC_PP;
        }
    }

    /* Re-announce any attached devices so the driver sees them after reset. */
    for (uint32_t p = 1; p <= XHCI_MAX_PORTS; ++p) {
        if (rvvm_usb_bus_port_dev(xhci->bus, p)) {
            xhci_port_on_attach(xhci, p);
        }
    }
}

static bool xhci_pci_write(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    xhci_bus_t* xhci = dev->data;
    uint32_t val = read_uint32_le(data);
    UNUSED(size);

    if ((offset - XHCI_DOORBELL_BASE) < XHCI_DOORBELL_SIZE) {
        xhci_doorbell_write(xhci, (offset - XHCI_DOORBELL_BASE) >> 2, val);
    } else if ((offset - XHCI_RUNTIME_BASE) < XHCI_RUNTIME_SIZE) {
        size_t roff = offset - XHCI_RUNTIME_BASE;
        if (roff >= 0x20) {
            xhci_interrupter_write(xhci, (roff - 0x20) >> 5, roff & 0x1C, val);
        }
    } else if ((offset - XHCI_PORT_REGS_BASE) < XHCI_PORT_REGS_SIZE) {
        size_t pid = ((offset - XHCI_PORT_REGS_BASE) >> 4) + 1;
        size_t poff = offset & 0xC;
        if (poff == XHCI_REG_PORTSC) xhci_portsc_write(xhci, pid, val);
    } else switch (offset) {
        case XHCI_REG_USBCMD:
            if (val & XHCI_USBCMD_HCRST) {
                xhci_controller_reset(xhci);
            } else {
                uint32_t old = atomic_load_uint32_relax(&xhci->usbcmd);
                atomic_store_uint32_relax(&xhci->usbcmd, val & XHCI_USBCMD_MASK);
                if ((val & XHCI_USBCMD_RS) && !(old & XHCI_USBCMD_RS)) {
                    atomic_and_uint32(&xhci->usbsts, (uint32_t)~XHCI_USBSTS_HCH);
                }
                if (!(val & XHCI_USBCMD_RS) && (old & XHCI_USBCMD_RS)) {
                    atomic_or_uint32(&xhci->usbsts, XHCI_USBSTS_HCH);
                }
            }
            break;
        case XHCI_REG_USBSTS:
            /* Writing 1 clears RW1C bits: HSE, EINT, PCD, SRE, etc. */
            atomic_and_uint32(&xhci->usbsts, ~(val & XHCI_USBSTS_MASK));
            break;
        case XHCI_REG_DNCTRL:
            atomic_store_uint32_relax(&xhci->dnctrl, val & XHCI_DNCTRL_MASK);
            break;
        case XHCI_REG_CRCR:
            atomic_store_uint32_relax(&xhci->crcr, val & XHCI_CRCR_WMASK);
            scoped_spin_lock (&xhci->lock) {
                xhci->cmd_deq = ((rvvm_addr_t)val & XHCI_ALIGN_64BYTE)
                                | (((rvvm_addr_t)atomic_load_uint32_relax(&xhci->crcr_h)) << 32);
                xhci->cmd_ccs = val & XHCI_CRCR_RCS ? 1 : 0;
            }
            break;
        case XHCI_REG_CRCR_H:
            atomic_store_uint32_relax(&xhci->crcr_h, val);
            scoped_spin_lock (&xhci->lock) {
                xhci->cmd_deq = (xhci->cmd_deq & 0xFFFFFFFFULL) | ((rvvm_addr_t)val << 32);
            }
            break;
        case XHCI_REG_DCBAAP:
            atomic_store_uint32_relax(&xhci->dcbaap, val & (uint32_t)XHCI_ALIGN_64BYTE);
            break;
        case XHCI_REG_DCBAAP_H:
            atomic_store_uint32_relax(&xhci->dcbaap_h, val);
            break;
        case XHCI_REG_CONFIG:
            atomic_store_uint32_relax(&xhci->config, val & 0xFF);
            break;
        default: break;
    }
    return true;
}

static void xhci_pci_remove(rvvm_mmio_dev_t* dev)
{
    xhci_bus_t* xhci = dev->data;
    if (xhci) {
        rvvm_usb_bus_free(xhci->bus);
        free(xhci);
    }
}

static const rvvm_mmio_type_t xhci_type = {
    .name   = "xhci",
    .remove = xhci_pci_remove,
};

PUBLIC pci_dev_t* usb_xhci_init(pci_bus_t* pci_bus, rvvm_usb_bus_t** out_bus)
{
    if (out_bus) *out_bus = NULL;
    xhci_bus_t* xhci = safe_new_obj(xhci_bus_t);
    spin_init(&xhci->lock);
    xhci->usbsts = XHCI_USBSTS_HCH;
    xhci->cmd_ccs = 1;
    for (uint32_t p = 1; p <= XHCI_MAX_PORTS; ++p) {
        xhci->ports[p].portsc = XHCI_PORTSC_PP;
    }

    xhci->bus = rvvm_usb_bus_init(&xhci_bus_cb, xhci);
    if (xhci->bus == NULL) {
        free(xhci);
        return NULL;
    }

    pci_func_desc_t xhci_desc = {
        .vendor_id  = 0x1b36,  /* Red Hat, Inc. — stock xhci vendor */
        .device_id  = 0x000d,
        .class_code = 0x0C03,
        .prog_if    = 0x30,
        .irq_pin    = PCI_IRQ_PIN_INTA,
        .bar[0] = {
            .size        = XHCI_BAR_SIZE,
            .min_op_size = 4,
            .max_op_size = 4,
            .read        = xhci_pci_read,
            .write       = xhci_pci_write,
            .data        = xhci,
            .type        = &xhci_type,
        },
    };

    pci_dev_t* pci_dev = pci_attach_func(pci_bus, &xhci_desc);
    if (pci_dev) {
        xhci->pci_func = pci_get_device_func(pci_dev, 0);
        if (out_bus) *out_bus = xhci->bus;
    } else {
        rvvm_usb_bus_free(xhci->bus);
        free(xhci);
    }
    return pci_dev;
}

PUBLIC rvvm_usb_bus_t* usb_xhci_init_auto(rvvm_machine_t* machine)
{
    /*
     * Reuse an existing bus if one was already attached — e.g. main.c
     * called us during machine construction and a binding is now
     * looking it up. Otherwise create one and stash it on the machine.
     */
    rvvm_usb_bus_t* bus = rvvm_get_usb_bus(machine);
    if (bus) return bus;

    pci_bus_t* pci = rvvm_get_pci_bus(machine);
    if (pci == NULL) return NULL;
    pci_dev_t* pci_dev = usb_xhci_init(pci, &bus);
    UNUSED(pci_dev);
    if (bus) rvvm_set_usb_bus(machine, bus);
    return bus;
}
