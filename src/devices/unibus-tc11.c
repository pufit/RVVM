/*
unibus-tc11.c - TC11/TU56 DECtape on the PDP-11 Unibus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

The TC11 controller with TU56 DECtape transport is the block-addressable
magnetic tape that early UNIX uses for archival and removable storage. Like
the RF11 drum (unibus-rf11.c) it is an NPR (DMA) block device: the program
loads a word count, a bus address and a target block, sets the command, and
the controller transfers words directly to/from memory, then interrupts.
We model the whole transfer synchronously via unibus_dma_read/write() and
raise the completion interrupt, exactly as unibus-rf11.c does.

What makes DECtape different from a drum is that the head is not over an
arbitrary block at random: the tape physically moves and the controller
must *search* (read block-number marks) until the desired block passes
under the head, THEN transfer. Real bidirectional search with deceleration,
reversal and end-zone handling (SIMH PDP11/pdp11_tc.c models all of it) is
elaborate; we implement a faithful-enough subset that satisfies the way the
UNIX drivers actually drive the hardware (see the V1 contract below).

Driver contract (1st Edition UNIX, unix-v1-on-rvvm/build/u8.s "tape:" /
"tape1:" / "tape2:" / "tape3:", and confirmed by the V4 driver
usr/sys/dmr/tc.c "tcstart"/"tcintr"). The driver loop is:

  1. ptc/tcstart issues a SEARCH (read block number): function = SRCH, GO,
     IE, with the unit in bits 8-10 and the direction in bit 11. V1 writes
     tccm = 0103 (octal): GO|IE|FNC=1. (V4: RNUM=02 == FNC=1 in bits 1-3.)
  2. On the resulting interrupt the driver reads TCDT -- the controller has
     placed the block number now under the head there -- and compares it to
     the target block:
        TCDT  < target  -> keep searching forward  (re-issue SRCH)
        TCDT  > target  -> reverse direction and search
        TCDT == target  -> the head is over the block: load TCWC (two's
                           complement word count) and TCBA (bus address)
                           and issue the data transfer:
                             read  : function = READ  (V1 tccm=0105, FNC=2)
                             write : function = WRIT  (V1 tccm=0115, FNC=6)
  3. The transfer interrupt (state tape3 / V4 case SIO) completes the I/O.

So for the search loop to converge with no knowledge of the driver's target,
we model a head position: each SEARCH advances the modeled "current block"
by one in the commanded direction (forward = +1, reverse = -1, clamped to
the tape) and reports it in TCDT. The driver's "< keep going / > reverse / ==
go" logic then walks the head to the target in a bounded number of search
interrupts, precisely as it would against real tape motion. The subsequent
READ/WRITE then transfers that block. READ and WRITE block transfers are
thus fully and correctly modeled; SEARCH is modeled as a one-block-per-step
head advance (enough to position correctly, not a line-accurate transport).

Functions completed-as-noop (the driver may emit them, but they move no
data): STOP (FNC=0) and STOP-SELECTED/SSEL (FNC=4) just set READY -- a tape
that is already "stopped" in our zero-motion model. WMRK/RALL/WALL (mark /
read-all / write-all formatting functions) are not used by the V1/V4 data
path; we complete them with READY and the illegal-op error bit, since they
have no meaning without real mark-track simulation. Where V1 and SIMH/V4
differ, V1's actual register usage is the binding contract.

Registers (V1 u0.s equates; word-spaced from tcst = 0177340):
  tcst = 0177340  control & status (errors, up-to-speed)
  tccm = 0177342  command (GO, function, IE, direction, unit, done, error)
  tcwc = 0177344  word count (two's complement count of 16-bit words)
  tcba = 0177346  bus address (Unibus/bus address, low 16 bits)
  tcdt = 0177350  data / block number

Interrupt: bus request BR6 (the TC11's hardwired Unibus line; SIMH IPL_DTA
== 6, V4 low.s "tcio; br6"), vector 0214 (V1 u0.s ". = orig+214 ... tape;300").
Here the ISR run level encoded in the vector's new-PS (V1 "tape;300" == level
6) HAPPENS to equal the BR line, but br_level below is the *hardware bus line*,
not the run level (cf. the RF11/KW11-L where the two differ).

tccm bit layout (TC11; SIMH CSR_*, V4 tc.c):
  bit  0     GO / DO     (initiate the command)
  bits 1-3   function    (0 STOP, 1 SRCH, 2 READ, 3 RALL, 4 SSEL, 5 WMRK,
                          6 WRIT, 7 WALL)
  bits 4-5   MEX         (extended bus-address bits, unused in 16-bit window)
  bit  6     IE          (interrupt enable)
  bit  7     DONE/READY  (set by controller on completion)
  bits 8-10  unit select
  bit 11     DIR         (1 = reverse)
  bit 15     ERROR

tcst bits we set: STA_ILO (illegal op) on an unsupported function; STA_END
(off the end of tape) on an out-of-bounds block. tccm.ERROR mirrors the OR
of the tcst error bits (SIMH updates it on a TCCM read).
*/

#include <rvvm/rvvm_board.h>

#include "unibus.h"
#include "utils.h"

#include <string.h> // memcpy (tape image load)

PUSH_OPTIMIZATION_SIZE

// tccm (command & status) bits
#define TC11_CM_GO    0x0001 // bit 0: GO / DO, initiate command
#define TC11_CM_FUNC  0x000E // bits 1-3: function code
#define TC11_CM_V_FNC 1      // function field shift
#define TC11_CM_MEX   0x0030 // bits 4-5: extended bus address (unused)
#define TC11_CM_IE    0x0040 // bit 6: interrupt enable
#define TC11_CM_DONE  0x0080 // bit 7: done / ready
#define TC11_CM_UNIT  0x0700 // bits 8-10: unit select
#define TC11_CM_DIR   0x0800 // bit 11: direction (1 = reverse)
#define TC11_CM_ERR   0x8000 // bit 15: error

// Function codes (tccm bits 1-3). V1 tccm=0103/0105/0115 octal decode to
// SRCH/READ/WRIT; V4 RNUM/RDATA/WDATA agree.
#define TC11_FNC_STOP 0 // stop all
#define TC11_FNC_SRCH 1 // search (read block number)
#define TC11_FNC_READ 2 // read data (tape -> memory)
#define TC11_FNC_RALL 3 // read all (formatting; unused by V1/V4 data path)
#define TC11_FNC_SSEL 4 // stop selected
#define TC11_FNC_WMRK 5 // write mark (formatting; unused)
#define TC11_FNC_WRIT 6 // write data (memory -> tape)
#define TC11_FNC_WALL 7 // write all (formatting; unused)

// tcst (status) bits
#define TC11_ST_END   0x8000 // bit 15: end zone (off the end of tape)
#define TC11_ST_ILO   0x1000 // bit 12: illegal operation
#define TC11_ST_UPS   0x0080 // bit 7: up to speed
#define TC11_ST_ALLERR (TC11_ST_END | TC11_ST_ILO) // errors we model

#define TC11_BR     UNIBUS_BR6 // TC11 requests on BR6 (its hardwired bus
                               // line; SIMH IPL_DTA == 6, V4 "tcio; br6").
                               // The 300 in V1 u0.s "tape;300" is the ISR's
                               // new-PS / run level, NOT the bus-request line.
#define TC11_VECTOR 0214 // DECtape interrupt vector (V1 u0.s . = orig+214)

// Register offsets within the block based at tcst = 0177340
#define TC11_TCST 0x0 // control & status
#define TC11_TCCM 0x2 // command
#define TC11_TCWC 0x4 // word count
#define TC11_TCBA 0x6 // bus address
#define TC11_TCDT 0x8 // data / block number

#define TC11_BLOCK_WORDS 256 // 256 16-bit words per block
#define TC11_BLOCK_BYTES (TC11_BLOCK_WORDS * 2) // 512 bytes per block
#define TC11_DEFAULT_BLOCKS 578 // TU56 18b/16b tape: 578 blocks (SIMH D18_TSIZE)

#define TC11_CM_FUNC_OF(cm) (((cm) & TC11_CM_FUNC) >> TC11_CM_V_FNC)

typedef struct {
    unibus_dev_t* dev;

    uint8_t* store; // Backing image (in memory)
    size_t   size;  // Backing image size in bytes
    uint32_t nblk;  // Backing image size in blocks

    // Atomic register state
    uint32_t tcst; // control & status
    uint32_t tccm; // command
    uint32_t tcwc; // word count (two's complement)
    uint32_t tcba; // bus address
    uint32_t tcdt; // data / block number under the head

    // Modeled head position: the block the search reports in TCDT. The
    // driver searches until this matches its target, then transfers it.
    uint32_t cur_block;

    // Debug counters (boot diagnostics)
    uint64_t xfers;      // total data transfers performed
    uint64_t searches;   // total search steps performed
    uint32_t last_block; // last block transferred
    uint32_t last_func;  // last data function (read/write)
    uint32_t hist[32];   // ring of recent block numbers (debug)
} tc11_dev_t;

// Re-evaluate the completion interrupt from tccm (Done & IE)
static void tc11_update_irq(tc11_dev_t* tc)
{
    uint32_t cm = atomic_load_uint32_relax(&tc->tccm);
    if ((cm & TC11_CM_DONE) && (cm & TC11_CM_IE)) {
        unibus_raise_irq(tc->dev);
    } else {
        unibus_lower_irq(tc->dev);
    }
}

// Finish a command: clear GO, set Done, fold the tcst error bits into the
// tccm Error bit, and re-evaluate the IE-gated completion interrupt.
static void tc11_complete(tc11_dev_t* tc, uint32_t cm)
{
    uint32_t st = atomic_load_uint32_relax(&tc->tcst);
    cm = (cm & ~TC11_CM_GO) | TC11_CM_DONE;
    if (st & TC11_ST_ALLERR) {
        cm |= TC11_CM_ERR;
    } else {
        cm &= ~TC11_CM_ERR;
    }
    atomic_store_uint32_relax(&tc->tccm, cm);
    tc11_update_irq(tc);
}

// Search step: advance the modeled head by one block in the commanded
// direction and report the new block number in TCDT. The driver compares
// TCDT to its target and re-issues SEARCH until they match (see header).
static void tc11_search(tc11_dev_t* tc, uint32_t cm)
{
    uint32_t cur = atomic_load_uint32_relax(&tc->cur_block);
    if (cm & TC11_CM_DIR) {
        if (cur > 0) {
            cur--;
        }
    } else {
        if (cur + 1 < tc->nblk) {
            cur++;
        }
    }
    atomic_store_uint32_relax(&tc->cur_block, cur);
    atomic_store_uint32_relax(&tc->tcdt, cur);
    tc->searches++;
    tc11_complete(tc, cm);
}

// Data transfer: DMA the TCWC words synchronously between the block at the
// modeled head position and the TCBA bus address (Handbook p.268 NPR/DMA).
static void tc11_transfer(tc11_dev_t* tc, uint32_t cm, uint32_t func)
{
    // Word count is the two's complement of the number of words to move.
    uint32_t wc    = atomic_load_uint32_relax(&tc->tcwc);
    size_t   words = (uint16_t)(-(int16_t)(uint16_t)wc);
    size_t   bytes = words * 2;

    // The head is over cur_block (the driver searched until TCDT matched its
    // target, so cur_block == the target block to transfer).
    uint32_t block     = atomic_load_uint32_relax(&tc->cur_block);
    uint64_t tape_off  = (uint64_t)block * TC11_BLOCK_BYTES;

    // Memory (Unibus) address the data is transferred to/from.
    rvvm_addr_t mem_addr = atomic_load_uint32_relax(&tc->tcba);

    bool ok = false;
    if (block < tc->nblk && tape_off + bytes <= tc->size) {
        if (func == TC11_FNC_READ) {
            // Tape -> memory (NPR DMA write into window RAM)
            ok = unibus_dma_write(tc->dev, mem_addr, tc->store + tape_off, bytes);
        } else if (func == TC11_FNC_WRIT) {
            // Memory -> tape (NPR DMA read from window RAM)
            ok = unibus_dma_read(tc->dev, mem_addr, tc->store + tape_off, bytes);
        }
    }

    // Advance the bus address / drain the word count to reflect completion.
    atomic_store_uint32_relax(&tc->tcba, (uint16_t)(mem_addr + bytes));
    atomic_store_uint32_relax(&tc->tcwc, 0);

    tc->hist[tc->xfers & 31] = block;
    tc->xfers++;
    tc->last_block = block;
    tc->last_func  = func;

    if (!ok) {
        // Out of bounds: flag end-of-tape (the driver retries / reverses).
        atomic_store_uint32_relax(&tc->tcst,
            atomic_load_uint32_relax(&tc->tcst) | TC11_ST_END);
    }
    tc11_complete(tc, cm);
}

// Decode and execute a command written to tccm with GO set.
static void tc11_go(tc11_dev_t* tc, uint32_t cm)
{
    uint32_t func = TC11_CM_FUNC_OF(cm);

    // Writing GO clears the prior Done and the tcst/tccm error flops (SIMH
    // dt_wr: "GO (DO) set -> clear errors, clear done").
    cm &= ~(TC11_CM_DONE | TC11_CM_ERR);
    atomic_store_uint32_relax(&tc->tcst,
        atomic_load_uint32_relax(&tc->tcst) & ~TC11_ST_ALLERR);
    atomic_store_uint32_relax(&tc->tccm, cm);

    switch (func) {
        case TC11_FNC_SRCH:
            // Read block number: step the head and report it in TCDT.
            tc11_search(tc, cm);
            break;
        case TC11_FNC_READ:
        case TC11_FNC_WRIT:
            // Data transfer (fully modeled): DMA the block.
            atomic_store_uint32_relax(&tc->tcst,
                atomic_load_uint32_relax(&tc->tcst) | TC11_ST_UPS);
            tc11_transfer(tc, cm, func);
            break;
        case TC11_FNC_STOP:
        case TC11_FNC_SSEL:
            // Stop (all / selected): no motion to stop in our model; complete
            // immediately with READY (a no-op seek).
            tc11_complete(tc, cm);
            break;
        case TC11_FNC_RALL:
        case TC11_FNC_WMRK:
        case TC11_FNC_WALL:
        default:
            // Formatting / mark-track functions: not used by the V1/V4 data
            // path and meaningless without mark simulation. Complete with the
            // illegal-op error so the driver's error path runs (V1 "taper").
            atomic_store_uint32_relax(&tc->tcst,
                atomic_load_uint32_relax(&tc->tcst) | TC11_ST_ILO);
            tc11_complete(tc, cm);
            break;
    }
}

static void tc11_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    tc11_dev_t* tc = unibus_dev_data(dev);
    switch (off) {
        case TC11_TCST:
            *val = (uint16_t)atomic_load_uint32_relax(&tc->tcst);
            break;
        case TC11_TCCM: {
            // tccm.ERROR mirrors the OR of the tcst error bits, refreshed on
            // read (SIMH dt_rd case TCCM).
            uint32_t cm = atomic_load_uint32_relax(&tc->tccm);
            uint32_t st = atomic_load_uint32_relax(&tc->tcst);
            if (st & TC11_ST_ALLERR) {
                cm |= TC11_CM_ERR;
            } else {
                cm &= ~TC11_CM_ERR;
            }
            atomic_store_uint32_relax(&tc->tccm, cm);
            *val = (uint16_t)cm;
            break;
        }
        case TC11_TCWC:
            *val = (uint16_t)atomic_load_uint32_relax(&tc->tcwc);
            break;
        case TC11_TCBA:
            *val = (uint16_t)atomic_load_uint32_relax(&tc->tcba);
            break;
        case TC11_TCDT:
            *val = (uint16_t)atomic_load_uint32_relax(&tc->tcdt);
            break;
        default:
            *val = 0;
            break;
    }
}

static void tc11_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    tc11_dev_t* tc = unibus_dev_data(dev);
    switch (off) {
        case TC11_TCST:
            // Only the low read/write status bits are guest-writable; we keep
            // it simple and let the controller own tcst, ignoring writes.
            break;
        case TC11_TCCM:
            // Any TCCM write with GO launches the command; without GO it is
            // just a status/IE update (clear Done while we work, then a
            // completing function re-sets it). Clear Done up front so the
            // driver's trapt "ready bit" test sees a busy controller until
            // the synchronous command finishes.
            atomic_store_uint32_relax(&tc->tccm, val & ~TC11_CM_DONE);
            if (val & TC11_CM_GO) {
                tc11_go(tc, val);
            } else {
                tc11_update_irq(tc);
            }
            break;
        case TC11_TCWC:
            atomic_store_uint32_relax(&tc->tcwc, val);
            break;
        case TC11_TCBA:
            atomic_store_uint32_relax(&tc->tcba, val);
            break;
        case TC11_TCDT:
            atomic_store_uint32_relax(&tc->tcdt, val);
            break;
        default:
            break;
    }
}

// No periodic work: the completion interrupt is a one-shot raised by
// tc11_complete when a command finishes and cleared when the bus grants its
// vector (IAK). We must NOT re-assert it from a poll -- the TC11 leaves
// Done/IE set after a command and the driver clears them only by issuing the
// next command, so re-raising would produce a spurious-interrupt storm
// between commands (same reasoning as unibus-rf11.c).

static void tc11_cleanup(unibus_dev_t* dev)
{
    tc11_dev_t* tc = unibus_dev_data(dev);
    free(tc->store);
}

// Boot diagnostics: total data transfers / search steps and the last
// block/function touched.
RVVM_PUBLIC void rvvm_tc11_stats(unibus_dev_t* dev, uint64_t* xfers,
                                 uint64_t* searches, uint32_t* last_block,
                                 uint32_t* last_func)
{
    if (!dev) {
        return;
    }
    tc11_dev_t* tc = unibus_dev_data(dev);
    if (xfers)      *xfers      = tc->xfers;
    if (searches)   *searches   = tc->searches;
    if (last_block) *last_block = tc->last_block;
    if (last_func)  *last_func  = tc->last_func;
}

// Copy the recent-block ring (most recent last) into out[0..n-1].
RVVM_PUBLIC size_t rvvm_tc11_history(unibus_dev_t* dev, uint32_t* out, size_t n)
{
    if (!dev) {
        return 0;
    }
    tc11_dev_t* tc = unibus_dev_data(dev);
    size_t total = tc->xfers < 32 ? (size_t)tc->xfers : 32;
    if (n > total) {
        n = total;
    }
    for (size_t i = 0; i < n; i++) {
        // walk back from the most recent
        size_t idx = (tc->xfers - n + i) & 31;
        out[i] = tc->hist[idx];
    }
    return n;
}

// Load an initial tape image into the backing store. Bytes past the store are
// dropped; a short image leaves the tail zeroed.
RVVM_PUBLIC bool rvvm_tc11_load(unibus_dev_t* dev, const void* data, size_t len)
{
    if (!dev || !data) {
        return false;
    }
    tc11_dev_t* tc = unibus_dev_data(dev);
    if (len > tc->size) {
        len = tc->size;
    }
    memcpy(tc->store, data, len);
    return true;
}

RVVM_PUBLIC unibus_dev_t* rvvm_tc11_init(unibus_t* bus, size_t image_size_blocks)
{
    if (!bus) {
        return NULL;
    }
    if (!image_size_blocks) {
        image_size_blocks = TC11_DEFAULT_BLOCKS; // TU56 DECtape: 578 blocks
    }

    tc11_dev_t* tc = safe_new_obj(tc11_dev_t);
    tc->nblk       = (uint32_t)image_size_blocks;
    tc->size       = image_size_blocks * TC11_BLOCK_BYTES;
    tc->store      = safe_calloc(tc->size, 1);
    tc->tccm       = TC11_CM_DONE; // idle controller is ready/done
    tc->tcst       = TC11_ST_UPS;  // transport up to speed

    unibus_dev_desc_t desc = {
        .name     = "tc11",
        .io_addr  = 0xFEE0, // tcst = 0177340
        .size     = 0xA,    // tcst(0)..tcdt(8), 5 word registers
        .read     = tc11_read,
        .write    = tc11_write,
        .cleanup  = tc11_cleanup,
        .min_size = 2,
        .br_level = TC11_BR,
        .vector   = TC11_VECTOR,
        .data     = tc,
    };

    // Save the backing store pointer: on attach failure the bus frees tc
    // (it owns desc.data), but not the separately-allocated store. Capture
    // it first to free without touching the freed tc.
    uint8_t* store = tc->store;

    tc->dev = unibus_attach(bus, &desc);
    if (!tc->dev) {
        free(store);
        return NULL;
    }
    return tc->dev;
}

POP_OPTIMIZATION_SIZE
