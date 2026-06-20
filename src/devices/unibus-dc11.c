/*
unibus-dc11.c - DC11 asynchronous serial line multiplexer on the PDP-11 Unibus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

The DC11 is a multi-line asynchronous serial-line interface: N independent
full-duplex lines, each with its own receiver and transmitter, and each
receiver/transmitter with its own interrupt vector. It is the multi-line
generalization of the single-line KL11/DL11 (unibus-kl11.c), and it is wired
here exactly the same way: each receiver and transmitter is a programmed-I/O
character endpoint backed by an RVVM chardev, like ns16550a.c wires a UART.

The ported 1st Edition UNIX kernel wires DC11 as its terminal multiplexer
(unix-v1-on-rvvm/build/u0.s):

  rcsr = 0174000  receiver status reg     (line 0)
  rcbr = 0174002  receiver buffer reg
  tcsr = 0174004  xmtr status reg
  tcbr = 0174006  xmtr buffer reg

Each line occupies 4 words = 8 bytes (010 octal). Line i is at base + i*010:
rcsr(i) = 0174000 + i*010, and the four registers are at offsets 0/2/4/6
within that line block. This matches SIMH pdp11_dc.c, which decodes the line
number as ((PA - base) >> 3) and the register as ((PA >> 1) & 03):
  00 -> dci csr (rcsr)   01 -> dci buf (rcbr)
  02 -> dco csr (tcsr)   03 -> dco buf (tcbr)

Base line-0 rcsr = 0174000 octal == 0xF000.

Interrupts (kernel u0.s ". = orig+300" vector block):

      0*4+trcv; 240; 0*4+txmt; 240   / line 0 input,output vectors
      1*4+trcv; 240; 1*4+txmt; 240   / line 1 ...
      ...
      7*4+trcv; 240; 7*4+txmt; 240   / line 7

  Each vector is a 2-word block (new PC, new PS). The table starts at byte
  address 0300 octal and lays out, per line i, the RX vector block followed
  by the TX vector block, so:

      line i RX vector = 0300 + i*010          (0300, 0310, 0320, ...)
      line i TX vector = 0300 + i*010 + 4      (0304, 0314, 0324, ...)

  i.e. base 0300, per-line stride 010 octal, RX then TX 4 bytes apart. This
  matches SIMH dci_iack() returning (vec + ln*010) and dco_iack() returning
  (vec + ln*010 + 4) off the same device base vector. The "240" words are the
  ISR run-PS (processor level 5), NOT the bus line; see br_level below.

  As with the KL11, a single Unibus device endpoint carries exactly one
  (BR level, vector) pair, but each DC11 line has two interrupt sources
  (receiver vector, transmitter vector). We therefore attach 2*N cooperating
  Unibus device endpoints -- a receiver endpoint and a transmitter endpoint
  per line -- all sharing one DC11 state object. The line-0 receiver endpoint
  owns the shared state (and frees it on cleanup); every other endpoint sets
  shared_data = true so the bus does not double-free.

Bus request level:

  The real DC11 line requests on BR5. The "240" in the V1 vector table is the
  ISR's new-PS / run level (5), which here numerically coincides with the bus
  level but is conceptually the processor priority, not the bus line (cf. the
  same note in unibus-kl11.c / unibus-rf11.c).

Per-line status-bit layout (DL11-compatible subset, matching what the V1
trcv/txmt handlers in u9.s actually use):

  rcsr (receiver status):  bit 7 Done (char received), bit 6 Interrupt Enable
  rcbr (receiver buffer):  reading consumes one char and clears Done
  tcsr (xmtr status):      bit 7 Ready (transmitter ready), bit 6 Int Enable
  tcbr (xmtr buffer):      writing transmits one char

  The V1 receiver ISR reads rcsr into r2 and treats it as an error if the sign
  bit (bit 15) is set ("tst r2; blt error"), and checks parity via bit 040.
  We never set the error/parity bits, so a received char looks clean. SIMH's
  full DC11 also models carrier-detect/ring/overrun bits in the high byte, but
  V1 neither sets DTR nor consults carrier on these lines (it just polls Done),
  so we leave the modem/carrier bits low. Where SIMH and V1 differ, V1 wins for
  the runtime; the discrepancy is noted here.

A line whose chardev is NULL is idle: chardev_poll(NULL) returns CHARDEV_TX,
so RX is never Done (a getty/read simply blocks) and TX is always Ready (a
write completes immediately). This is exactly the "don't spin the kernel on an
unconnected line at boot" behavior, for free.
*/

#include <rvvm/rvvm_board.h>

#include "chardev.h"
#include "unibus.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

#define DC11_DONE 0x80 // bit 7: Done / Ready
#define DC11_IE   0x40 // bit 6: Interrupt Enable

#define DC11_BR        UNIBUS_BR5 // DC11 line requests on BR5 (the bus line;
                                  // the ;240 in u0.s is the ISR run level 5).
#define DC11_IO_BASE   0xF000     // rcsr line 0 = 0174000 octal
#define DC11_VEC_BASE  0300       // line-0 RX vector (octal byte address)
#define DC11_LINE_SIZE 0x8        // 4 words = 8 bytes per line (010 octal)
#define DC11_VEC_STRIDE 0x8       // per-line vector stride (010 octal)

// Register offsets within each 2-word endpoint block. The receiver endpoint
// is based at rcsr: offset 0 = rcsr, 2 = rcbr. The transmitter endpoint is
// based at tcsr: offset 0 = tcsr, 2 = tcbr.
#define DC11_STATUS 0x0 // status register (rcsr or tcsr)
#define DC11_BUFFER 0x2 // buffer register (rcbr or tcbr)

#define DC11_DEFAULT_LINES 8 // 1st Edition UNIX wires 8 DC11 lines

typedef struct dc11_dev dc11_dev_t;

// Per-line state. One chardev backs both the line's receiver and transmitter.
typedef struct {
    dc11_dev_t* dc;   // owning DC11 (for the chardev notify callback)
    size_t      line; // line index (for the chardev notify callback)

    chardev_t* chardev; // line backend, may be NULL (idle line)

    unibus_dev_t* rx_dev; // receiver endpoint (rcsr/rcbr, RX vector)
    unibus_dev_t* tx_dev; // transmitter endpoint (tcsr/tcbr, TX vector)

    uint32_t flags; // chardev RX/TX availability (CHARDEV_RX|CHARDEV_TX)
    uint32_t rx_ie; // receiver interrupt enable (rcsr bit 6)
    uint32_t tx_ie; // transmitter interrupt enable (tcsr bit 6)
} dc11_line_t;

// Shared DC11 state. Allocated once (with the line array inline) and shared by
// all 2*N endpoints; the line-0 receiver endpoint owns and frees it.
struct dc11_dev {
    size_t      nlines;
    dc11_line_t lines[]; // flexible array, nlines entries
};

// Re-evaluate the RX/TX interrupt requests for one line from current state.
static void dc11_update_line_irq(dc11_line_t* ln)
{
    uint32_t flags = atomic_load_uint32_relax(&ln->flags);

    if ((flags & CHARDEV_RX) && atomic_load_uint32_relax(&ln->rx_ie)) {
        unibus_raise_irq(ln->rx_dev);
    } else {
        unibus_lower_irq(ln->rx_dev);
    }

    if ((flags & CHARDEV_TX) && atomic_load_uint32_relax(&ln->tx_ie)) {
        unibus_raise_irq(ln->tx_dev);
    } else {
        unibus_lower_irq(ln->tx_dev);
    }
}

// Chardev -> device notification on RX/TX flag change for one line.
static void dc11_notify(void* io_dev, uint32_t flags)
{
    dc11_line_t* ln = io_dev;
    if (atomic_swap_uint32(&ln->flags, flags) != flags) {
        dc11_update_line_irq(ln);
    }
}

// Poll one line's chardev for RX/TX availability and update its IRQs.
static void dc11_poll_line(dc11_line_t* ln)
{
    uint32_t flags = chardev_poll(ln->chardev);
    if (flags != atomic_load_uint32_relax(&ln->flags)) {
        dc11_notify(ln, flags);
    }
}

// Receiver endpoint (rcsr/rcbr). Done (bit 7) means a character is waiting in
// the buffer, which the V1 trcv ISR polls (it reads rcsr, then rcbr).
static void dc11_rx_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    dc11_line_t* ln    = unibus_dev_data(dev);
    uint32_t     flags = chardev_poll(ln->chardev);

    switch (off) {
        case DC11_STATUS: // rcsr
            *val = (uint16_t)(((flags & CHARDEV_RX) ? DC11_DONE : 0)
                            | (atomic_load_uint32_relax(&ln->rx_ie) ? DC11_IE : 0));
            break;
        case DC11_BUFFER: // rcbr: read consumes one char and clears Done
            if (flags & CHARDEV_RX) {
                uint8_t c = 0;
                chardev_read(ln->chardev, &c, 1);
                *val = c;
                dc11_poll_line(ln);
            } else {
                *val = 0;
            }
            break;
        default:
            *val = 0;
            break;
    }
}

static void dc11_rx_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    dc11_line_t* ln = unibus_dev_data(dev);
    if (off == DC11_STATUS) {
        // Setting rcsr Interrupt Enable (bit 6) arms the receiver interrupt.
        atomic_store_uint32_relax(&ln->rx_ie, !!(val & DC11_IE));
        dc11_update_line_irq(ln);
    }
    // rcbr is read-only
}

// Transmitter endpoint (tcsr/tcbr). Ready (bit 7) means the transmitter can
// accept a character; writing tcbr transmits it.
static void dc11_tx_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    dc11_line_t* ln    = unibus_dev_data(dev);
    uint32_t     flags = chardev_poll(ln->chardev);

    switch (off) {
        case DC11_STATUS: // tcsr
            *val = (uint16_t)(((flags & CHARDEV_TX) ? DC11_DONE : 0)
                            | (atomic_load_uint32_relax(&ln->tx_ie) ? DC11_IE : 0));
            break;
        case DC11_BUFFER: // tcbr: write-only
        default:
            *val = 0;
            break;
    }
}

static void dc11_tx_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    dc11_line_t* ln = unibus_dev_data(dev);
    switch (off) {
        case DC11_STATUS: // tcsr Interrupt Enable
            atomic_store_uint32_relax(&ln->tx_ie, !!(val & DC11_IE));
            dc11_update_line_irq(ln);
            break;
        case DC11_BUFFER: { // tcbr: transmit one character
            uint8_t c = (uint8_t)val;
            chardev_write(ln->chardev, &c, 1);
            dc11_poll_line(ln);
            break;
        }
        default:
            break;
    }
}

// Periodic coalesced chardev polling, driven by the bus poll. Registered on
// the line-0 receiver endpoint only (one poll services every line's chardev).
static void dc11_poll(unibus_dev_t* dev)
{
    dc11_line_t* ln0 = unibus_dev_data(dev);
    dc11_dev_t*  dc  = ln0->dc;
    for (size_t i = 0; i < dc->nlines; i++) {
        dc11_line_t* ln = &dc->lines[i];
        chardev_update(ln->chardev);
        dc11_poll_line(ln);
    }
}

// Cleanup on the owning (line-0 receiver) endpoint: release every chardev.
static void dc11_cleanup(unibus_dev_t* dev)
{
    dc11_line_t* ln0 = unibus_dev_data(dev);
    dc11_dev_t*  dc  = ln0->dc;
    for (size_t i = 0; i < dc->nlines; i++) {
        chardev_free(dc->lines[i].chardev);
    }
    // Every endpoint's desc.data is an interior &dc->lines[i] pointer, so all
    // of them are marked shared_data (the bus must never free an interior
    // pointer). The single allocation base is dc -- free it here.
    safe_free(dc);
}

// Attach a chardev backend to a specific line. Pass the unibus_dev_t* returned
// by rvvm_dc11_init() (the line-0 receiver endpoint, which carries the shared
// state). May be called before the machine starts. Returns success.
RVVM_PUBLIC bool rvvm_dc11_set_chardev(unibus_dev_t* dev, size_t line, chardev_t* chardev)
{
    if (!dev) {
        return false;
    }
    dc11_line_t* ln0 = unibus_dev_data(dev);
    dc11_dev_t*  dc  = ln0->dc;
    if (line >= dc->nlines) {
        return false;
    }

    dc11_line_t* ln = &dc->lines[line];
    // Replace any previous backend (caller owns no prior reference here).
    chardev_free(ln->chardev);
    ln->chardev = chardev;
    if (chardev) {
        chardev->io_dev = ln;
        chardev->notify = dc11_notify;
    }
    dc11_poll_line(ln);
    return true;
}

RVVM_PUBLIC unibus_dev_t* rvvm_dc11_init(unibus_t* bus, size_t nlines)
{
    if (!bus) {
        return NULL;
    }
    if (nlines == 0) {
        nlines = DC11_DEFAULT_LINES;
    }

    dc11_dev_t* dc = safe_calloc(sizeof(dc11_dev_t) + nlines * sizeof(dc11_line_t), 1);
    dc->nlines = nlines;

    for (size_t i = 0; i < nlines; i++) {
        dc11_line_t* ln = &dc->lines[i];
        ln->dc   = dc;
        ln->line = i;

        rvvm_addr_t io_addr = DC11_IO_BASE + i * DC11_LINE_SIZE;
        uint16_t    rx_vec  = (uint16_t)(DC11_VEC_BASE + i * DC11_VEC_STRIDE);
        uint16_t    tx_vec  = (uint16_t)(rx_vec + 4);

        // Receiver endpoint: rcsr/rcbr, RX vector, BR5. The line-0 receiver
        // carries the cleanup hook (frees the allocation and every chardev)
        // and drives the periodic poll for the whole multiplexer. Its data is
        // an interior &dc->lines[0] pointer, so it -- like every endpoint --
        // is shared_data: the bus must not free it; dc11_cleanup frees dc.
        unibus_dev_desc_t rx_desc = {
            .name        = "dc11-rx",
            .io_addr     = io_addr + DC11_STATUS, // rcsr (off 0 = rcsr, 2 = rcbr)
            .size        = 0x4,
            .read        = dc11_rx_read,
            .write       = dc11_rx_write,
            .poll        = (i == 0) ? dc11_poll : NULL,
            .cleanup     = (i == 0) ? dc11_cleanup : NULL,
            .min_size    = 2,
            .br_level    = DC11_BR,
            .vector      = rx_vec,
            .data        = ln,
            .shared_data = true, // data is interior to dc; cleanup frees dc
        };
        ln->rx_dev = unibus_attach(bus, &rx_desc);
        if (!ln->rx_dev) {
            // The line-0 rx endpoint owns the allocation; if it never attached
            // nothing the bus will free it, so free here. For i > 0 the owning
            // endpoint exists and will free on bus teardown.
            if (i == 0) {
                safe_free(dc);
            }
            return NULL;
        }

        // Transmitter endpoint: tcsr/tcbr, TX vector, BR5. Shares the state
        // (shared_data => the bus does not free it).
        unibus_dev_desc_t tx_desc = {
            .name        = "dc11-tx",
            .io_addr     = io_addr + DC11_STATUS + 0x4, // tcsr (off 0 = tcsr, 2 = tcbr)
            .size        = 0x4,
            .read        = dc11_tx_read,
            .write       = dc11_tx_write,
            .min_size    = 2,
            .br_level    = DC11_BR,
            .vector      = tx_vec,
            .data        = ln,
            .shared_data = true,
        };
        ln->tx_dev = unibus_attach(bus, &tx_desc);
    }

    return dc->lines[0].rx_dev;
}

POP_OPTIMIZATION_SIZE
