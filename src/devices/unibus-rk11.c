/*
unibus-rk11.c - RK11/RK05 cartridge disk on the PDP-11 Unibus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

The RK11 controller with RK05 cartridge disk is the removable-pack disk that
1st Edition UNIX uses for the file system (u0.s register equates "rkds/rkcs/
rkda"; the disk strategy in u8.s). Like the RF11 drum it is an NPR (DMA)
block device: the program loads the disk address, word count and memory (bus)
address, sets GO in the control status register, and the controller transfers
words directly to/from memory without processor intervention, then interrupts.

This is the Unibus realization of the NPR/DMA path described in the Handbook
(Ch.10 p.268: "The BUS NPR L line is used by a peripheral to request the
data section of the bus for a Direct Memory Access (DMA) transfer. (The
acronym NPR stands for Non Processor Request.)"). We do the whole transfer
synchronously via unibus_dma_read/write() and then raise the completion
interrupt; from the guest's point of view this is indistinguishable from a
very fast disk. Modeled directly on unibus-rf11.c.

Registers (SIMH PDP11/pdp11_rk.c rk_rd/rk_wr decode PA<3:1>; V1 u0.s equates
rkds=0177400, rkcs=0177404, rkda=0177412; V1 u8.s loads them descending from
rkda+2: rkda, rkba, rkwc, then rkcs):
  rkds = 0177400  drive status   (read-only)
  rker = 0177402  error status   (read-only)
  rkcs = 0177404  control & status (GO, function, IE, ready/done, error)
  rkwc = 0177406  word count (two's complement count of 16-bit words)
  rkba = 0177410  bus address (Unibus/bus address, low 16 bits)
  rkda = 0177412  disk address (cylinder / surface / sector encoded)
  rkmr = 0177414  maintenance register (unimplemented)
  rkdb = 0177416  data buffer (unimplemented)

The kernel writes rkcs = 0103 (write+func1+GO+IE) or 0105 (read+func2+GO+IE)
(V1 u8.s "mov $103,...write" / "mov $105,...read", loaded into rkcs last).

Interrupt: bus request BR5 (the RK11's hardwired Unibus level; SIMH IPL_RK 5,
V4 low.s "rkio; br5"), vector 0220 (SIMH autoconfig RK11 "fx CSR, fx VEC"
{017400}{0220}; V1 u0.s ". = orig+214 / tape;300 / disk;300" puts the disk
vector at orig+220). The ISR *runs* at processor level 6 -- the 300 in u0.s
"disk;300" encodes 300 (level 6) as the vector's new-PS, which is the run
level, NOT the bus-request line; do not conflate the two (cf. the KW11-L
clock and the RF11 drum). NOTE: 0220 is the RK11; 0214 is the TC11 DECtape.

rkcs bit layout (SIMH rk_cs_bits):
  bit  0   GO        (start the transfer)
  bits 1-3 function  (1 = write to disk, 2 = read from disk, others below)
  bits 4-5 MEX       (memory extension; UC15/22-bit, unused in the 16b window)
  bit  6   Interrupt Enable
  bit  7   Ready / Done (transfer complete; set by controller)
  bit 13   Search Complete (seek done; not modeled -- we transfer atomically)
  bit 14   Hard Error
  bit 15   Error

rkda bit layout (SIMH rk_da_bits):
  bits 0-3   sector   (0..11)
  bit  4     surface  (head 0/1)
  bits 5-12  cylinder (0..202)
  bits 13-15 drive    (we model one drive, unit 0)
*/

#include <rvvm/rvvm_board.h>

#include "unibus.h"
#include "utils.h"

#include <string.h> // memcpy (disk image load)

PUSH_OPTIMIZATION_SIZE

// rkcs (control & status)
#define RK11_CS_GO     0x0001 // bit 0: start transfer
#define RK11_CS_FUNC   0x000E // bits 1-3: function code
#define RK11_CS_V_FUNC 1      // function shift
#define RK11_CS_IE     0x0040 // bit 6: interrupt enable
#define RK11_CS_DONE   0x0080 // bit 7: control ready / done
#define RK11_CS_HERR   0x4000 // bit 14: hard error
#define RK11_CS_ERR    0x8000 // bit 15: error summary

// rkcs function codes (SIMH rk_cs_bits, V1 u8.s 0103=write / 0105=read)
#define RK11_FUNC_CTLRESET 0 // control reset
#define RK11_FUNC_WRITE    1 // memory -> disk
#define RK11_FUNC_READ     2 // disk -> memory
#define RK11_FUNC_WCHK     3 // write check (treated as a no-op, sets done)
#define RK11_FUNC_SEEK     4 // seek (no data; sets done immediately)
#define RK11_FUNC_RCHK     5 // read check (treated as a no-op, sets done)
#define RK11_FUNC_DRVRESET 6 // drive reset
#define RK11_FUNC_WLK      7 // write lock

// rkda (disk address) field extraction (SIMH rk_da_bits)
#define RK11_DA_SECT(x)  ((x) & 0x000F)        // bits 0-3:  sector
#define RK11_DA_TRACK(x) (((x) >> 4) & 0x01FF) // bits 4-12: track (surf+cyl)
#define RK11_DA_CYL(x)   (((x) >> 5) & 0x00FF) // bits 5-12: cylinder

// rker (error) bits raised on bad geometry (SIMH rk_er_bits)
#define RK11_ER_NXS 0x0040 // nonexistent sector
#define RK11_ER_NXC 0x0080 // nonexistent cylinder
#define RK11_ER_NXM 0x0800 // nonexistent memory (DMA fault)

#define RK11_BR     UNIBUS_BR5 // RK11 controller requests on BR5 (its
                               // hardwired bus line; SIMH IPL_RK 5, V4 low.s
                               // "rkio; br5"). The 300 in u0.s "disk;300" is
                               // the ISR's new-PS / run level 6, NOT the
                               // bus-request line.
#define RK11_VECTOR 0220 // disk interrupt vector (u0.s . = orig+220; SIMH
                         // autoconfig RK11 {0220}). 0214 is the TC11 DECtape.

// Register offsets within the block based at rkds = 0177400 (SIMH PA<3:1>)
#define RK11_RKDS 0x0 // drive status (read-only)
#define RK11_RKER 0x2 // error status (read-only)
#define RK11_RKCS 0x4 // control & status
#define RK11_RKWC 0x6 // word count
#define RK11_RKBA 0x8 // bus address
#define RK11_RKDA 0xA // disk address
#define RK11_RKMR 0xC // maintenance register (unimplemented)
#define RK11_RKDB 0xE // data buffer (unimplemented)

// RK05 geometry (SIMH pdp11_rk.c, V4 NRKBLK=4872)
#define RK11_NUMWD   256  // words per sector
#define RK11_NUMSC   12   // sectors per surface
#define RK11_NUMSF   2    // surfaces per cylinder
#define RK11_NUMCY   203  // cylinders per drive
#define RK11_NUMBL   (RK11_NUMCY * RK11_NUMSF * RK11_NUMSC) // 4872 blocks

#define RK11_BLOCK_BYTES 512 // bytes per sector (256 words)

typedef struct {
    unibus_dev_t* dev;

    uint8_t* store; // Backing image (in memory): the RK05 pack
    size_t   size;  // Backing image size in bytes

    // Atomic register state
    uint32_t rkcs; // control & status
    uint32_t rker; // error status
    uint32_t rkwc; // word count (two's complement)
    uint32_t rkba; // bus address
    uint32_t rkda; // disk address (cyl/surface/sector)

    // Debug counters (boot diagnostics)
    uint64_t xfers;      // total transfers performed
    uint32_t last_block; // last linear block transferred
    uint32_t last_func;  // last function code
    uint32_t hist[32];   // ring of recent block numbers (debug)
} rk11_dev_t;

// Re-evaluate the completion interrupt from rkcs (Done & IE)
static void rk11_update_irq(rk11_dev_t* rk)
{
    uint32_t rkcs = atomic_load_uint32_relax(&rk->rkcs);
    if ((rkcs & RK11_CS_DONE) && (rkcs & RK11_CS_IE)) {
        unibus_raise_irq(rk->dev);
    } else {
        unibus_lower_irq(rk->dev);
    }
}

// Set Done and the error summary from rker, then re-evaluate the interrupt.
// GO is cleared; HERR/ERR follow whether any error bit is set in rker.
static void rk11_set_done(rk11_dev_t* rk, uint32_t rkcs)
{
    uint32_t rker = atomic_load_uint32_relax(&rk->rker);
    rkcs = (rkcs & ~(RK11_CS_GO | RK11_CS_HERR | RK11_CS_ERR)) | RK11_CS_DONE;
    if (rker) {
        rkcs |= RK11_CS_HERR | RK11_CS_ERR;
    }
    atomic_store_uint32_relax(&rk->rkcs, rkcs);
    rk11_update_irq(rk);
}

// Perform the NPR DMA transfer requested by a GO in rkcs (Handbook p.268)
static void rk11_go(rk11_dev_t* rk)
{
    uint32_t rkcs = atomic_load_uint32_relax(&rk->rkcs);
    uint32_t func = (rkcs & RK11_CS_FUNC) >> RK11_CS_V_FUNC;

    // A fresh command clears soft errors / the error summary; we keep only
    // the hard NX* bits we may raise below (SIMH rk_go clears RKER_SOFT).
    atomic_store_uint32_relax(&rk->rker, 0);

    // Control / drive reset: clear errors and address regs, just set Done.
    if (func == RK11_FUNC_CTLRESET || func == RK11_FUNC_DRVRESET) {
        atomic_store_uint32_relax(&rk->rker, 0);
        if (func == RK11_FUNC_CTLRESET) {
            atomic_store_uint32_relax(&rk->rkda, 0);
            atomic_store_uint32_relax(&rk->rkba, 0);
        }
        rk11_set_done(rk, rkcs);
        return;
    }

    // Seek / write-lock / read-check / write-check move no data here: we
    // transfer atomically, so the seek is instantaneous and the checks are
    // satisfied trivially. Just complete with Done (SIMH schedules these,
    // we don't model rotational latency).
    if (func == RK11_FUNC_SEEK || func == RK11_FUNC_WLK ||
        func == RK11_FUNC_RCHK || func == RK11_FUNC_WCHK) {
        rk11_set_done(rk, rkcs);
        return;
    }

    // Word count is the two's complement of the number of words to move.
    uint32_t rkwc  = atomic_load_uint32_relax(&rk->rkwc);
    size_t   words = (uint16_t)(-(int16_t)(uint16_t)rkwc);
    size_t   bytes = words * 2;

    // Disk address: linearize cyl/surface/sector exactly as SIMH does
    // (rk_svc: da = GET_DA(rkda) * RK_NUMWD; GET_DA = GET_TRACK*RK_NUMSC +
    // GET_SECT). track = bits 4-12 (surface in bit 4, cylinder in bits 5-12),
    // sector = bits 0-3. The linear block is track*12 + sector; the byte
    // offset is that block times 512 (256 words/sector * 2 bytes/word). This
    // matches V1's rkaddr encoding (u8.s: block/12 -> cyl/surf bits, block%12
    // -> sector), so V1 and SIMH agree on the geometry.
    uint32_t rkda  = atomic_load_uint32_relax(&rk->rkda);
    uint32_t sect  = RK11_DA_SECT(rkda);
    uint32_t track = RK11_DA_TRACK(rkda);
    uint32_t cyl   = RK11_DA_CYL(rkda);
    uint64_t block = (uint64_t)track * RK11_NUMSC + sect;
    uint64_t disk_off = block * RK11_BLOCK_BYTES;

    // Memory (Unibus) address the data is transferred to/from.
    rvvm_addr_t mem_addr = atomic_load_uint32_relax(&rk->rkba);

    bool ok = false;
    if (sect >= RK11_NUMSC) {
        // Nonexistent sector (SIMH RKER_NXS)
        atomic_store_uint32_relax(&rk->rker, RK11_ER_NXS);
    } else if (cyl >= RK11_NUMCY || disk_off + bytes > rk->size) {
        // Nonexistent cylinder / out-of-bounds (SIMH RKER_NXC)
        atomic_store_uint32_relax(&rk->rker, RK11_ER_NXC);
    } else if (func == RK11_FUNC_READ) {
        // Disk -> memory (NPR DMA write into window RAM)
        ok = unibus_dma_write(rk->dev, mem_addr, rk->store + disk_off, bytes);
        if (!ok) {
            atomic_store_uint32_relax(&rk->rker, RK11_ER_NXM);
        }
    } else if (func == RK11_FUNC_WRITE) {
        // Memory -> disk (NPR DMA read from window RAM)
        ok = unibus_dma_read(rk->dev, mem_addr, rk->store + disk_off, bytes);
        if (!ok) {
            atomic_store_uint32_relax(&rk->rker, RK11_ER_NXM);
        }
    } else {
        // Unknown function: flag a programming error via the summary.
        atomic_store_uint32_relax(&rk->rker, RK11_ER_NXC);
    }

    // Advance bus address / word count to reflect completion (SIMH leaves
    // rkba bumped and rkwc drained after a transfer).
    atomic_store_uint32_relax(&rk->rkba, (uint16_t)(mem_addr + bytes));
    atomic_store_uint32_relax(&rk->rkwc, 0);

    rk->hist[rk->xfers & 31] = (uint32_t)block; // linear block (may be OOB)
    rk->xfers++;
    rk->last_block = (uint32_t)block;
    rk->last_func  = func;

    rk11_set_done(rk, rkcs);
}

static void rk11_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    rk11_dev_t* rk = unibus_dev_data(dev);
    switch (off) {
        case RK11_RKDS:
            // Drive status: report RK05 present and drive ready. We keep this
            // minimal -- the V1 driver polls only CTLRDY (rkcs bit 7) for the
            // data path and SEEKCMP for seeks; RKDS is read for the drive
            // number on seek-complete, which our atomic model never asserts.
            *val = 0;
            break;
        case RK11_RKER:
            *val = (uint16_t)atomic_load_uint32_relax(&rk->rker);
            break;
        case RK11_RKCS: {
            // Reflect the error summary from rker on every read (SIMH rk_rd).
            uint32_t rkcs = atomic_load_uint32_relax(&rk->rkcs);
            if (atomic_load_uint32_relax(&rk->rker)) {
                rkcs |= RK11_CS_ERR;
            }
            *val = (uint16_t)rkcs;
            break;
        }
        case RK11_RKWC:
            *val = (uint16_t)atomic_load_uint32_relax(&rk->rkwc);
            break;
        case RK11_RKBA:
            *val = (uint16_t)atomic_load_uint32_relax(&rk->rkba);
            break;
        case RK11_RKDA:
            *val = (uint16_t)atomic_load_uint32_relax(&rk->rkda);
            break;
        default: // RKMR / RKDB unimplemented
            *val = 0;
            break;
    }
}

static void rk11_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    rk11_dev_t* rk = unibus_dev_data(dev);
    switch (off) {
        case RK11_RKDS:
        case RK11_RKER:
            break; // read-only
        case RK11_RKCS:
            // Writing rkcs with GO set launches the function; clear Done
            // while busy, then rk11_go() sets it again on completion.
            atomic_store_uint32_relax(&rk->rkcs, val & ~RK11_CS_DONE);
            if (val & RK11_CS_GO) {
                rk11_go(rk);
            } else {
                rk11_update_irq(rk);
            }
            break;
        case RK11_RKWC:
            atomic_store_uint32_relax(&rk->rkwc, val);
            break;
        case RK11_RKBA:
            atomic_store_uint32_relax(&rk->rkba, val);
            break;
        case RK11_RKDA:
            atomic_store_uint32_relax(&rk->rkda, val);
            break;
        default: // RKMR / RKDB unimplemented
            break;
    }
}

// No periodic work: the completion interrupt is a one-shot raised by rk11_go
// when a function finishes and cleared when the bus grants its vector (IAK).
// We must NOT re-assert it from a poll -- the RK11 leaves Done/IE set after a
// transfer and the V1 driver clears them only by issuing the next command, so
// re-raising here would produce a spurious-interrupt storm between transfers
// (cf. the RF11 drum).

static void rk11_cleanup(unibus_dev_t* dev)
{
    rk11_dev_t* rk = unibus_dev_data(dev);
    free(rk->store);
}

// Boot diagnostics: total transfers and the last block/function touched.
RVVM_PUBLIC void rvvm_rk11_stats(unibus_dev_t* dev, uint64_t* xfers,
                                 uint32_t* last_block, uint32_t* last_func)
{
    if (!dev) {
        return;
    }
    rk11_dev_t* rk = unibus_dev_data(dev);
    if (xfers)      *xfers      = rk->xfers;
    if (last_block) *last_block = rk->last_block;
    if (last_func)  *last_func  = rk->last_func;
}

// Copy the recent-block ring (most recent last) into out[0..n-1].
RVVM_PUBLIC size_t rvvm_rk11_history(unibus_dev_t* dev, uint32_t* out, size_t n)
{
    if (!dev) {
        return 0;
    }
    rk11_dev_t* rk = unibus_dev_data(dev);
    size_t total = rk->xfers < 32 ? (size_t)rk->xfers : 32;
    if (n > total) {
        n = total;
    }
    for (size_t i = 0; i < n; i++) {
        // walk back from the most recent
        size_t idx = (rk->xfers - n + i) & 31;
        out[i] = rk->hist[idx];
    }
    return n;
}

// Load an initial disk image (file system pack) into the backing store. Bytes
// past the store are dropped; a short image leaves the tail zeroed.
RVVM_PUBLIC bool rvvm_rk11_load(unibus_dev_t* dev, const void* data, size_t len)
{
    if (!dev || !data) {
        return false;
    }
    rk11_dev_t* rk = unibus_dev_data(dev);
    if (len > rk->size) {
        len = rk->size;
    }
    memcpy(rk->store, data, len);
    return true;
}

RVVM_PUBLIC unibus_dev_t* rvvm_rk11_init(unibus_t* bus, size_t image_size_blocks)
{
    if (!bus) {
        return NULL;
    }
    if (!image_size_blocks) {
        image_size_blocks = RK11_NUMBL; // default RK05 pack: 4872 blocks
    }

    rk11_dev_t* rk = safe_new_obj(rk11_dev_t);
    rk->size       = image_size_blocks * RK11_BLOCK_BYTES;
    rk->store      = safe_calloc(rk->size, 1);
    rk->rkcs       = RK11_CS_DONE; // idle controller is ready

    unibus_dev_desc_t desc = {
        .name     = "rk11",
        .io_addr  = 0xFF00, // rkds = 0177400
        .size     = 0x10,   // rkds(0)..rkdb(0xE), 8 word registers (IOLN_RK)
        .read     = rk11_read,
        .write    = rk11_write,
        .cleanup  = rk11_cleanup,
        .min_size = 2,
        .br_level = RK11_BR,
        .vector   = RK11_VECTOR,
        .data     = rk,
    };

    // Save the backing store pointer: on attach failure the bus frees rk
    // (it owns desc.data), but not the separately-allocated store. Capture
    // it first to free without touching the freed rk.
    uint8_t* store = rk->store;

    rk->dev = unibus_attach(bus, &desc);
    if (!rk->dev) {
        free(store);
        return NULL;
    }
    return rk->dev;
}

POP_OPTIMIZATION_SIZE
