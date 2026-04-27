/*
parport-pci.c - PCI Parallel Port (NetMos/MosChip MCS9900)
Copyright (C) 2026  Sol Astrius <claude@danielsol.dev>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "parport-pci.h"
#include "compiler.h"
#include "mem_ops.h"
#include "spinlock.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

// NetMos / MosChip MCS9900 PCI parallel port (single SPP port).
//
// PCI: vendor 0x9710 (NetMos / MosChip), device 0x9900,
//      class 0x0701 (Communication / Parallel), prog_if 0x00 (SPP).
//
// Linux's parport_pc.c table:
//     /* netmos_9900 */ { 1, { { 0, -1 }, } }
//   → 1 port, BAR 0 holds the SPP register window, no separate ECP BAR.
//
// Register map (8 bytes at BAR 0), per IEEE 1284 / IBM PC parallel port:
//
//   +0x00 Data    (RW)  D0..D7, drives pins 2-9 of the DB-25 connector
//   +0x01 Status  (RO)  bit 7 nBusy   (active low — reads 1 when idle)
//                       bit 6 nAck    (active low — reads 1 when not ack'ing)
//                       bit 5 PaperOut(active high — 0 = paper present)
//                       bit 4 Select  (active high — 1 = peripheral online)
//                       bit 3 nError  (active low — reads 1 = no error)
//                       bit 2 IRQ     (latched IRQ; we don't fire SPP IRQs)
//                       bits 1:0 reserved
//   +0x02 Control (RW)  bit 0 Strobe  (the "data ready" pulse)
//                       bit 1 AutoLF
//                       bit 2 nInit   (active low — pulse to reset peripheral)
//                       bit 3 SelectIn
//                       bit 4 IRQACK  (IRQ enable on nAck pulse)
//                       bit 5 BIDIR   (data direction; 1 = input mode)
//                       bits 7:6 unused
//   +0x03..0x07         EPP/ECP registers; SPP-only emulation returns 0
//
// Centronics handshake (SPP §5.1):
//   1. Guest waits for nBusy=1 (idle)
//   2. Guest writes data byte to 0x00
//   3. Guest pulses Strobe — control bit 0 transitions 0→1, then 1→0
//   4. Real peripheral pulls nBusy low, latches data, pulses nAck, releases
//
// We model only step 3's rising edge as the "byte send" trigger and treat
// the backend as instantaneous. Status is permanently "idle" — if the
// backend ever needed to push back, we'd add a busy-window timer; today,
// every backend write completes before the guest can re-strobe so it's
// imperceptible.

#define PARPORT_PCI_VENDOR_ID     0x9710  // NetMos / MosChip
#define PARPORT_PCI_DEVICE_ID     0x9900
#define PARPORT_PCI_CLASS         0x0701  // Communication / Parallel
#define PARPORT_PCI_PROG_IF       0x00    // SPP

// Status idle byte: nBusy=1, nAck=1, PaperOut=0, Select=1, nError=1.
// Active-low signals "read as 1 when not asserted" per the OSDev /
// IBM PC convention. The bit-2 IRQ flag stays 0 (we don't latch IRQs).
#define PARPORT_STATUS_IDLE       0xD8u

// Control reset value: bit 2 nInit=1 (peripheral not in reset),
// bit 3 SelectIn=1 (peripheral selected). Strobe=0 idle, BIDIR=0 forward.
#define PARPORT_CONTROL_RESET     0x0Cu

#define PARPORT_CTL_STROBE        (1u << 0)

typedef struct {
    pci_func_t* pci_func;
    spinlock_t  lock;
    uint8_t     data;
    uint8_t     control;

    // Backend
    parport_pci_write_fn write_fn;
    void*                user_data;
} parport_pci_dev_t;

static void parport_pci_remove(rvvm_mmio_dev_t* dev)
{
    parport_pci_dev_t* pp = dev->data;
    free(pp);
}

static rvvm_mmio_type_t parport_pci_type = {
    .name   = "netmos_9900_parport",
    .remove = parport_pci_remove,
};

static bool parport_pci_mmio_read(rvvm_mmio_dev_t* dev, void* data, size_t off, uint8_t size)
{
    UNUSED(size);
    parport_pci_dev_t* pp = dev->data;
    spin_lock(&pp->lock);
    uint8_t val = 0;
    bool ok = true;
    switch (off) {
        case 0x00: val = pp->data;            break;
        case 0x01: val = PARPORT_STATUS_IDLE; break;
        case 0x02: val = pp->control;         break;
        case 0x03: case 0x04: case 0x05: case 0x06: case 0x07:
            val = 0;
            break;
        default:
            ok = false;
            break;
    }
    spin_unlock(&pp->lock);
    if (ok) write_uint8(data, val);
    return ok;
}

static bool parport_pci_mmio_write(rvvm_mmio_dev_t* dev, void* data, size_t off, uint8_t size)
{
    UNUSED(size);
    parport_pci_dev_t* pp = dev->data;
    uint8_t byte = read_uint8(data);

    parport_pci_write_fn fn  = NULL;
    void*                ud  = NULL;
    uint8_t              out = 0;

    spin_lock(&pp->lock);
    bool ok = true;
    switch (off) {
        case 0x00:
            pp->data = byte;
            break;
        case 0x02: {
            uint8_t prev = pp->control;
            pp->control = byte;
            // Centronics strobe rising edge — capture data byte for the
            // backend. Real peripherals latch on the falling edge of
            // nStrobe, but the register-bit semantic in parport_pc.c is
            // the "logical" strobe (XOR'd with the chip's polarity), so
            // a 0→1 register transition is what actually drives the
            // wire transition that latches the byte.
            if (!(prev & PARPORT_CTL_STROBE) && (byte & PARPORT_CTL_STROBE)) {
                fn  = pp->write_fn;
                ud  = pp->user_data;
                out = pp->data;
            }
            break;
        }
        case 0x03: case 0x04: case 0x05: case 0x06: case 0x07:
            // EPP/ECP registers — SPP-only emulation ignores writes
            break;
        default:
            ok = false;
            break;
    }
    spin_unlock(&pp->lock);

    // Call backend outside the lock. Backends may block briefly on a
    // file write or host pipe — don't hold the device lock across that.
    if (ok && fn) {
        fn(ud, out);
    }
    return ok;
}

PUBLIC pci_dev_t* parport_pci_init(pci_bus_t* pci_bus,
                                   parport_pci_write_fn write_fn,
                                   void* user_data)
{
    parport_pci_dev_t* pp = safe_new_obj(parport_pci_dev_t);
    pp->control   = PARPORT_CONTROL_RESET;
    pp->write_fn  = write_fn;
    pp->user_data = user_data;

    pci_func_desc_t desc = {
        .vendor_id        = PARPORT_PCI_VENDOR_ID,
        .device_id        = PARPORT_PCI_DEVICE_ID,
        .class_code       = PARPORT_PCI_CLASS,
        .prog_if          = PARPORT_PCI_PROG_IF,
        // Linux's parport_pc.c netmos_9900 entry requires this exact
        // subsys pair — the table line is `{ 0xA000, 0x2000, ... }`.
        // Without this match parport_pc skips the device and no
        // /dev/parport0 / /dev/lp0 nodes get created, even though
        // lspci correctly shows the MosChip MCS9900 ID.
        .subsys_vendor_id = 0xA000,
        .subsys_device_id = 0x2000,
        .irq_pin          = PCI_IRQ_PIN_INTA,
        .bar[0] = {
            .size        = 8,
            .min_op_size = 1,
            .max_op_size = 4,   // Linux's PCI BAR sizing probe uses 4-byte writes
            .read        = parport_pci_mmio_read,
            .write       = parport_pci_mmio_write,
            .data        = pp,
            .type        = &parport_pci_type,
        },
    };

    pci_dev_t* dev = pci_attach_func(pci_bus, &desc);
    if (dev) pp->pci_func = pci_get_device_func(dev, 0);
    return dev;
}

PUBLIC pci_dev_t* parport_pci_init_auto(rvvm_machine_t* machine,
                                        parport_pci_write_fn write_fn,
                                        void* user_data)
{
    return parport_pci_init(rvvm_get_pci_bus(machine), write_fn, user_data);
}

POP_OPTIMIZATION_SIZE
