/*
unibus-kl11.c - KL11/DL11 console serial line on the PDP-11 Unibus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

The KL11 (and the compatible DL11) is the console serial-line interface: an
ASR-33 Teletype in the 1st Edition UNIX machine. It is a programmed-I/O
character device with four word registers, wired here to an RVVM chardev
backend exactly as ns16550a.c wires the NS16550A UART.

Registers (unix-v1-on-rvvm/port/include/io.inc; Handbook Appendix A p.A-1
lists "DL/DLV11-A/B 777560 ... (console)"):
  tks = 0177560  receiver status  (RX done bit 7, IE bit 6)
  tkb = 0177562  receiver buffer  (read a received char, clears RX done)
  tps = 0177564  transmitter status (TX ready bit 7, IE bit 6)
  tpb = 0177566  transmitter buffer (write a char to transmit)

Interrupts (kernel u0.s ". = orig+60" block, processor level 5 / BR5):
  receiver  -> vector 060
  transmitter -> vector 064

  NOTE: a single Unibus device carries one (BR level, vector) pair, but the
  KL11 has two distinct interrupt sources (RX vector 060, TX vector 064).
  We attach two cooperating Unibus device endpoints sharing one KL11 state:
  a receiver endpoint (tks/tkb, vector 060) and a transmitter endpoint
  (tps/tpb, vector 064), both on BR5. This matches the real hardware, where
  the receiver and transmitter request independently (Handbook p.216:
  "the second word contains the new PS" -- each source has its own vector).

Status-bit layout (DL11):
  bit 7  Done / Ready  (RX: char available; TX: transmitter ready)
  bit 6  Interrupt Enable
*/

#include <rvvm/rvvm_board.h>

#include "chardev.h"
#include "mem_ops.h"
#include "unibus.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

#define DL11_DONE 0x80 // bit 7: Done / Ready
#define DL11_IE   0x40 // bit 6: Interrupt Enable

#define DL11_BR        UNIBUS_BR4 // KL11/DL11 console requests on BR4 (Fourth
                                  // Edition low.s "klin; br4 / klou; br4").
                                  // The ;240 in u0.s "ttyi;240" is the ISR's
                                  // new-PS / run level 5, NOT the bus line.
#define DL11_RX_VECTOR 0060 // receiver interrupt vector
#define DL11_TX_VECTOR 0064 // transmitter interrupt vector

// Register offsets within each 2-word endpoint block. The receiver endpoint
// is based at tks (0177560): offset 0 = tks, 2 = tkb. The transmitter
// endpoint is based at tps (0177564): offset 0 = tps, 2 = tpb.
#define DL11_STATUS 0x0 // status register (tks or tps)
#define DL11_BUFFER 0x2 // buffer register (tkb or tpb)

typedef struct {
    chardev_t* chardev;

    unibus_dev_t* rx_dev; // receiver endpoint (tks/tkb, vector 060)
    unibus_dev_t* tx_dev; // transmitter endpoint (tps/tpb, vector 064)

    uint32_t flags;   // chardev RX/TX availability (CHARDEV_RX|CHARDEV_TX)
    uint32_t rx_ie;   // receiver interrupt enable (tks bit 6)
    uint32_t tx_ie;   // transmitter interrupt enable (tps bit 6)
} kl11_dev_t;

// Re-evaluate the RX/TX interrupt requests from current state
static void kl11_update_irq(kl11_dev_t* kl)
{
    uint32_t flags = atomic_load_uint32_relax(&kl->flags);

    if ((flags & CHARDEV_RX) && atomic_load_uint32_relax(&kl->rx_ie)) {
        unibus_raise_irq(kl->rx_dev);
    } else {
        unibus_lower_irq(kl->rx_dev);
    }

    if ((flags & CHARDEV_TX) && atomic_load_uint32_relax(&kl->tx_ie)) {
        unibus_raise_irq(kl->tx_dev);
    } else {
        unibus_lower_irq(kl->tx_dev);
    }
}

// Chardev -> device notification on RX/TX flag change
static void kl11_notify(void* io_dev, uint32_t flags)
{
    kl11_dev_t* kl = io_dev;
    if (atomic_swap_uint32(&kl->flags, flags) != flags) {
        kl11_update_irq(kl);
    }
}

// Poll the chardev for RX/TX availability
static void kl11_poll_rxtx(kl11_dev_t* kl)
{
    uint32_t flags = chardev_poll(kl->chardev);
    if (flags != atomic_load_uint32_relax(&kl->flags)) {
        kl11_notify(kl, flags);
    }
}

// Receiver endpoint (tks/tkb). The Done bit (bit 7) reflects "a character
// has been received and is in the buffer", which the program polls before
// vectoring (Handbook p.215: "the state of the Done/Ready flag (bit 7) in
// the peripheral interface").
static void kl11_rx_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    kl11_dev_t* kl    = unibus_dev_data(dev);
    uint32_t    flags = chardev_poll(kl->chardev);

    switch (off) {
        case DL11_STATUS: // tks
            *val = (uint16_t)(((flags & CHARDEV_RX) ? DL11_DONE : 0)
                            | (atomic_load_uint32_relax(&kl->rx_ie) ? DL11_IE : 0));
            break;
        case DL11_BUFFER: // tkb: read consumes one char and clears Done
            if (flags & CHARDEV_RX) {
                uint8_t c = 0;
                chardev_read(kl->chardev, &c, 1);
                *val = c;
                kl11_poll_rxtx(kl);
            } else {
                *val = 0;
            }
            break;
        default:
            *val = 0;
            break;
    }
}

static void kl11_rx_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    kl11_dev_t* kl = unibus_dev_data(dev);
    if (off == DL11_STATUS) {
        // Setting tks Interrupt Enable (bit 6) arms the receiver interrupt
        // (Handbook p.215: "The Interrupt Enable bit in the control status
        // register must have been set at some prior time.").
        atomic_store_uint32_relax(&kl->rx_ie, !!(val & DL11_IE));
        kl11_update_irq(kl);
    }
    // tkb is read-only
}

// Transmitter endpoint (tps/tpb). Ready (bit 7) means the transmitter can
// accept a character; writing tpb transmits it.
static void kl11_tx_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    kl11_dev_t* kl    = unibus_dev_data(dev);
    uint32_t    flags = chardev_poll(kl->chardev);

    switch (off) {
        case DL11_STATUS: // tps
            *val = (uint16_t)(((flags & CHARDEV_TX) ? DL11_DONE : 0)
                            | (atomic_load_uint32_relax(&kl->tx_ie) ? DL11_IE : 0));
            break;
        case DL11_BUFFER: // tpb: write-only
        default:
            *val = 0;
            break;
    }
}

static void kl11_tx_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    kl11_dev_t* kl = unibus_dev_data(dev);
    switch (off) {
        case DL11_STATUS: // tps Interrupt Enable
            atomic_store_uint32_relax(&kl->tx_ie, !!(val & DL11_IE));
            kl11_update_irq(kl);
            break;
        case DL11_BUFFER: { // tpb: transmit one character
            uint8_t c = (uint8_t)val;
            chardev_write(kl->chardev, &c, 1);
            kl11_poll_rxtx(kl);
            break;
        }
        default:
            break;
    }
}

// Periodic coalesced chardev polling, driven by the bus poll. Registered on
// the receiver endpoint only (one poll drives the whole KL11 state).
static void kl11_poll(unibus_dev_t* dev)
{
    kl11_dev_t* kl = unibus_dev_data(dev);
    chardev_update(kl->chardev);
    kl11_poll_rxtx(kl);
}

// Cleanup on the owning (receiver) endpoint: release the chardev backend.
static void kl11_cleanup(unibus_dev_t* dev)
{
    kl11_dev_t* kl = unibus_dev_data(dev);
    chardev_free(kl->chardev);
}

RVVM_PUBLIC unibus_dev_t* rvvm_kl11_init(unibus_t* bus, chardev_t* chardev)
{
    if (!bus) {
        return NULL;
    }

    kl11_dev_t* kl = safe_new_obj(kl11_dev_t);
    kl->chardev    = chardev;

    // Receiver endpoint: tks/tkb registers, RX vector 060, BR5. Owns the
    // shared kl11_dev_t (freed via this endpoint's data on cleanup).
    // Receiver endpoint owns the shared kl11_dev_t and drives the poll.
    unibus_dev_desc_t rx_desc = {
        .name     = "kl11-rx",
        .io_addr  = 0xFF70, // tks = 0177560 (offset 0 = tks, 2 = tkb)
        .size     = 0x4,
        .read     = kl11_rx_read,
        .write    = kl11_rx_write,
        .poll     = kl11_poll,
        .cleanup  = kl11_cleanup,
        .min_size = 2,
        .br_level = DL11_BR,
        .vector   = DL11_RX_VECTOR,
        .data     = kl,
    };
    kl->rx_dev = unibus_attach(bus, &rx_desc);
    if (!kl->rx_dev) {
        return NULL;
    }

    // Transmitter endpoint: tps/tpb, TX vector 064, BR5. Shares the
    // kl11_dev_t state but does not own it (shared_data => no double free).
    unibus_dev_desc_t tx_desc = {
        .name        = "kl11-tx",
        .io_addr     = 0xFF74, // tps = 0177564 (offset 0 = tps, 2 = tpb)
        .size        = 0x4,
        .read        = kl11_tx_read,
        .write       = kl11_tx_write,
        .min_size    = 2,
        .br_level    = DL11_BR,
        .vector      = DL11_TX_VECTOR,
        .data        = kl,
        .shared_data = true,
    };
    kl->tx_dev = unibus_attach(bus, &tx_desc);

    if (chardev) {
        chardev->io_dev = kl;
        chardev->notify = kl11_notify;
    }

    return kl->rx_dev;
}

POP_OPTIMIZATION_SIZE
