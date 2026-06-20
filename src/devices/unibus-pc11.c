/*
unibus-pc11.c - PC11 paper-tape reader/punch on the PDP-11 Unibus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

The PC11 is the high-speed paper-tape reader/punch. It is a programmed-I/O
character device with TWO independent interrupt sources, modeled here on the
KL11's two-endpoint (shared_data) pattern: a reader endpoint that consumes
bytes sequentially from a loaded tape image, and a punch endpoint that
appends bytes to an in-memory output buffer. Unlike the KL11 there is no
chardev backend and no full-duplex coupling -- the two halves are wholly
independent; they merely share one state object so the reader endpoint can
own and free the backing buffers.

Registers (kernel build/u0.s equates; Handbook Appendix A "PC11"):
  prs = 0177550  reader status  (Done bit 7, IE bit 6, RDRENB bit 0, ERROR bit 15, BUSY bit 11)
  prb = 0177552  reader buffer  (read a tape byte, clears reader Done)
  pps = 0177554  punch status   (Ready/Done bit 7, IE bit 6)
  ppb = 0177556  punch buffer   (write a byte to punch)

Interrupts (kernel u0.s ". = orig+60" block: ttyi/ttyo/ppti/ppto):
  reader -> vector 070
  punch  -> vector 074

  NOTE: a single Unibus device carries one (BR level, vector) pair, but the
  PC11 has two distinct interrupt sources (reader vector 070, punch vector
  074). As with the KL11 we attach two cooperating Unibus device endpoints
  sharing one pc11 state: a reader endpoint (prs/prb, vector 070) and a punch
  endpoint (pps/ppb, vector 074), both on BR4. The real PC11 requests on BR4
  (cf. KL11). The ";240" in u0.s "ppti;240 / ppto;240" is the ISR's new-PS /
  run level 5, NOT the bus line.

Status-bit layout (SIMH pdp11_pt.c PTRCSR/PTPCSR):
  reader prs: bit 0 RDRENB (reader enable), bit 6 IE, bit 7 Done,
              bit 11 BUSY, bit 15 ERROR.
  punch  pps: bit 6 IE, bit 7 Done/Ready.

Reader-enable handshake (V1 pc.c pcrint(), V4 dmr/pc.c): the guest writes
prs with RDRENB|IENABLE to fetch the next byte. On the real device this sets
BUSY, then ~unit.wait later the service routine either latches a byte and
sets Done (clearing ERROR) or, at end of tape / no tape attached, sets ERROR.
We complete the fetch synchronously: on RDRENB with a byte available, latch
it into prb, set Done and (if IE) raise the reader IRQ; with no byte left,
set ERROR and (if IE) raise the reader IRQ. Reading prb clears Done.

Punch: always Ready (Done set) when idle. Writing ppb latches the byte into
the output buffer; the byte is "punched" immediately and Ready is re-asserted
(SIMH ptp_svc sets DONE), raising the punch IRQ if IE is set.
*/

#include <rvvm/rvvm_board.h>

#include "unibus.h"
#include "utils.h"

#include <string.h> // memcpy (tape image load / punch readback)

PUSH_OPTIMIZATION_SIZE

#define PC11_DONE  0x0080 // bit 7:  Done / Ready
#define PC11_IE    0x0040 // bit 6:  Interrupt Enable
#define PC11_RDRENB 0x0001 // bit 0: reader enable (RDRENB), reader prs only
#define PC11_BUSY  0x0800 // bit 11: reader busy (transient; never latched here)
#define PC11_ERROR 0x8000 // bit 15: error / out of tape (reader prs only)

#define PC11_BR            UNIBUS_BR4 // PC11 reader+punch request on BR4. The
                                      // ;240 in u0.s "ppti;240" is the ISR's
                                      // new-PS / run level 5, NOT the bus line.
#define PC11_READER_VECTOR 0070 // reader interrupt vector (u0.s ppti)
#define PC11_PUNCH_VECTOR  0074 // punch  interrupt vector (u0.s ppto)

// Register offsets within each 2-word endpoint block. The reader endpoint is
// based at prs (0177550): offset 0 = prs, 2 = prb. The punch endpoint is
// based at pps (0177554): offset 0 = pps, 2 = ppb.
#define PC11_STATUS 0x0 // status register (prs or pps)
#define PC11_BUFFER 0x2 // buffer register (prb or ppb)

typedef struct {
    unibus_dev_t* reader_dev; // reader endpoint (prs/prb, vector 070)
    unibus_dev_t* punch_dev;  // punch  endpoint (pps/ppb, vector 074)

    // Reader: in-memory tape image, consumed sequentially.
    uint8_t* tape;     // backing tape image (owned by reader endpoint)
    size_t   tape_len; // total bytes in the tape image
    size_t   tape_pos; // index of the next byte to deliver

    uint32_t reader_buf;   // prb: last latched tape byte
    uint32_t reader_done;  // prs Done (bit 7): byte ready in prb
    uint32_t reader_error; // prs ERROR (bit 15): out of tape / EOF
    uint32_t reader_ie;    // prs IE (bit 6)

    // Punch: in-memory output buffer, grows as bytes are punched.
    uint8_t* punch_buf; // backing output buffer (owned by reader endpoint)
    size_t   punch_cap; // current capacity of punch_buf
    size_t   punch_len; // bytes punched so far

    uint32_t punch_ie; // pps IE (bit 6); punch is always Ready when idle
} pc11_dev_t;

// Re-evaluate the reader/punch interrupt requests from current state. The
// punch is always Ready (idle), so its request is gated purely by IE; this is
// edge-armed by the IE write and the ppb punch (see callers) to avoid storms.
static void pc11_update_irq(pc11_dev_t* pc)
{
    if ((atomic_load_uint32_relax(&pc->reader_done)
         || atomic_load_uint32_relax(&pc->reader_error))
        && atomic_load_uint32_relax(&pc->reader_ie)) {
        unibus_raise_irq(pc->reader_dev);
    } else {
        unibus_lower_irq(pc->reader_dev);
    }

    if (atomic_load_uint32_relax(&pc->punch_ie)) {
        unibus_raise_irq(pc->punch_dev);
    } else {
        unibus_lower_irq(pc->punch_dev);
    }
}

// Reader-enable handshake: fetch the next tape byte (or signal end of tape).
// Models SIMH ptr_wr CSR_GO + ptr_svc, completed synchronously: on a byte
// available, latch it into prb and set Done; on end of tape, set ERROR.
static void pc11_reader_fetch(pc11_dev_t* pc)
{
    if (pc->tape && pc->tape_pos < pc->tape_len) {
        pc->reader_buf   = pc->tape[pc->tape_pos++];
        atomic_store_uint32_relax(&pc->reader_error, 0);
        atomic_store_uint32_relax(&pc->reader_done, 1);
    } else {
        // No tape mounted or exhausted: error, no char (SIMH ptr_svc EOF).
        atomic_store_uint32_relax(&pc->reader_done, 0);
        atomic_store_uint32_relax(&pc->reader_error, 1);
    }
}

// Reader endpoint (prs/prb).
static void pc11_reader_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    pc11_dev_t* pc = unibus_dev_data(dev);
    switch (off) {
        case PC11_STATUS: // prs
            *val = (uint16_t)((atomic_load_uint32_relax(&pc->reader_done) ? PC11_DONE : 0)
                            | (atomic_load_uint32_relax(&pc->reader_ie) ? PC11_IE : 0)
                            | (atomic_load_uint32_relax(&pc->reader_error) ? PC11_ERROR : 0));
            break;
        case PC11_BUFFER: // prb: reading consumes the byte and clears Done
            *val = (uint16_t)(pc->reader_buf & 0xFF);
            atomic_store_uint32_relax(&pc->reader_done, 0);
            pc11_update_irq(pc);
            break;
        default:
            *val = 0;
            break;
    }
}

static void pc11_reader_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    pc11_dev_t* pc = unibus_dev_data(dev);
    if (off == PC11_STATUS) {
        // prs IE (bit 6) arms the reader interrupt; RDRENB (bit 0) starts a
        // fetch of the next tape byte (V1 pc.c: prs = IENABLE|RDRENB).
        atomic_store_uint32_relax(&pc->reader_ie, !!(val & PC11_IE));
        if (val & PC11_RDRENB) {
            pc11_reader_fetch(pc);
        }
        pc11_update_irq(pc);
    }
    // prb is read-only
}

// Punch endpoint (pps/ppb). Idle => Ready (Done bit 7 set, never busy here).
static void pc11_punch_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    pc11_dev_t* pc = unibus_dev_data(dev);
    switch (off) {
        case PC11_STATUS: // pps: always Ready (Done) when idle
            *val = (uint16_t)(PC11_DONE
                            | (atomic_load_uint32_relax(&pc->punch_ie) ? PC11_IE : 0));
            break;
        case PC11_BUFFER: // ppb: write-only
        default:
            *val = 0;
            break;
    }
}

static void pc11_punch_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    pc11_dev_t* pc = unibus_dev_data(dev);
    switch (off) {
        case PC11_STATUS: // pps IE (bit 6)
            atomic_store_uint32_relax(&pc->punch_ie, !!(val & PC11_IE));
            pc11_update_irq(pc);
            break;
        case PC11_BUFFER: { // ppb: "punch" one byte, then re-assert Ready
            if (pc->punch_len == pc->punch_cap) {
                size_t   cap = pc->punch_cap ? pc->punch_cap * 2 : 256;
                uint8_t* buf = safe_realloc(pc->punch_buf, cap);
                pc->punch_buf = buf;
                pc->punch_cap = cap;
            }
            pc->punch_buf[pc->punch_len++] = (uint8_t)val;
            // Punch completes immediately: Ready re-asserted, raise IRQ if IE
            // (SIMH ptp_svc sets DONE and, if IE, SET_INT(PTP)).
            if (atomic_load_uint32_relax(&pc->punch_ie)) {
                unibus_raise_irq(pc->punch_dev);
            }
            break;
        }
        default:
            break;
    }
}

// Cleanup on the owning (reader) endpoint: release the tape + punch buffers.
static void pc11_cleanup(unibus_dev_t* dev)
{
    pc11_dev_t* pc = unibus_dev_data(dev);
    free(pc->tape);
    free(pc->punch_buf);
}

RVVM_PUBLIC unibus_dev_t* rvvm_pc11_init(unibus_t* bus)
{
    if (!bus) {
        return NULL;
    }

    pc11_dev_t* pc = safe_new_obj(pc11_dev_t);
    // No tape mounted yet: the reader reports ERROR until a tape is loaded
    // (SIMH ptr_reset sets CSR_ERR when not attached).
    atomic_store_uint32_relax(&pc->reader_error, 1);

    // Reader endpoint: prs/prb registers, reader vector 070, BR4. Owns the
    // shared pc11_dev_t (freed via this endpoint's data on cleanup) and the
    // backing tape/punch buffers.
    unibus_dev_desc_t reader_desc = {
        .name     = "pc11-reader",
        .io_addr  = 0xFF68, // prs = 0177550 (offset 0 = prs, 2 = prb)
        .size     = 0x4,
        .read     = pc11_reader_read,
        .write    = pc11_reader_write,
        .cleanup  = pc11_cleanup,
        .min_size = 2,
        .br_level = PC11_BR,
        .vector   = PC11_READER_VECTOR,
        .data     = pc,
    };
    pc->reader_dev = unibus_attach(bus, &reader_desc);
    if (!pc->reader_dev) {
        free(pc);
        return NULL;
    }

    // Punch endpoint: pps/ppb, punch vector 074, BR4. Shares the pc11_dev_t
    // state but does not own it (shared_data => no double free).
    unibus_dev_desc_t punch_desc = {
        .name        = "pc11-punch",
        .io_addr     = 0xFF6C, // pps = 0177554 (offset 0 = pps, 2 = ppb)
        .size        = 0x4,
        .read        = pc11_punch_read,
        .write       = pc11_punch_write,
        .min_size    = 2,
        .br_level    = PC11_BR,
        .vector      = PC11_PUNCH_VECTOR,
        .data        = pc,
        .shared_data = true,
    };
    pc->punch_dev = unibus_attach(bus, &punch_desc);

    return pc->reader_dev;
}

// Mount a tape image into the reader. Copies len bytes; replaces any prior
// tape and rewinds. Clears the reader ERROR so a subsequent RDRENB fetch
// delivers the first byte.
RVVM_PUBLIC bool rvvm_pc11_load_reader(unibus_dev_t* dev, const void* data, size_t len)
{
    if (!dev) {
        return false;
    }
    pc11_dev_t* pc = unibus_dev_data(dev);

    uint8_t* tape = NULL;
    if (len) {
        tape = safe_malloc(len);
        memcpy(tape, data, len);
    }
    free(pc->tape);
    pc->tape     = tape;
    pc->tape_len = len;
    pc->tape_pos = 0;

    atomic_store_uint32_relax(&pc->reader_done, 0);
    atomic_store_uint32_relax(&pc->reader_error, len ? 0 : 1);
    pc11_update_irq(pc);
    return true;
}

// Read back up to n bytes of punched output (from the start of the buffer).
// Returns the number of bytes copied.
RVVM_PUBLIC size_t rvvm_pc11_punch_data(unibus_dev_t* dev, void* out, size_t n)
{
    if (!dev) {
        return 0;
    }
    pc11_dev_t* pc = unibus_dev_data(dev);
    size_t copy = (n < pc->punch_len) ? n : pc->punch_len;
    if (copy && out) {
        memcpy(out, pc->punch_buf, copy);
    }
    return copy;
}

POP_OPTIMIZATION_SIZE
