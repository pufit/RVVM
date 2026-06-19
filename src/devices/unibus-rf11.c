/*
unibus-rf11.c - RF11/RS11 fixed-head disk ("drum") on the PDP-11 Unibus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

The RF11 controller with RS11 fixed-head disk is the "drum" that 1st Edition
UNIX uses for its root and swap (notes/machine.txt: "rf0 - 1024 blocks,
always mounted, has root and swap"). It is an NPR (DMA) block device: the
program loads the disk address, word count and memory (bus) address, sets
GO in the control status register, and the controller transfers words
directly to/from memory without processor intervention, then interrupts.

This is the Unibus realization of the NPR/DMA path described in the Handbook
(Ch.10 p.268: "The BUS NPR L line is used by a peripheral to request the
data section of the bus for a Direct Memory Access (DMA) transfer. (The
acronym NPR stands for Non Processor Request.)"). We do the whole transfer
synchronously via unibus_dma_read/write() and then raise the completion
interrupt; from the guest's point of view this is indistinguishable from a
very fast drum.

Registers (unix-v1-on-rvvm/port/include/io.inc and the driver in
unix-v1-on-rvvm/build/u8.s "prf:" which loads, descending from dae+2:
dae, dar, bus address, word count, then dcs):
  dcs = 0177460  control & status (GO, function, IE, ready, error)
  wc  = 0177462  word count (two's complement count of 16-bit words)
  cma = 0177464  current memory address (Unibus/bus address, low 16 bits)
  dar = 0177466  disk address register (block within the drum)
  dae = 0177470  disk address extension (high block bits)

The kernel writes dcs = 0103 (write+GO+IE) or 0105 (read+GO+IE)
(u8.s "prf:" -> "$103 ... write", "$105 ... read").

Interrupt: BR6 / processor level 6, vector 0204 (build/u0.s ". = orig+204
... drum;300", i.e. priority 300 = level 6, vector at offset 204).
Handbook Appendix A (p.A-1) lists the RF11 CSR family in the I/O page; the
1st Edition UNIX vector table is the binding contract honored here.

dcs bit layout (RF11):
  bit  0  GO        (start the transfer)
  bits 1-2 function (1 = write to drum, 2 = read from drum)
  bit  6  Interrupt Enable
  bit  7  Ready / Done (transfer complete; set by controller)
  bit 15  Error
*/

#include <rvvm/rvvm_board.h>

#include "unibus.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

#define RF11_DCS_GO    0x0001 // bit 0: start transfer
#define RF11_DCS_FUNC  0x0006 // bits 1-2: function
#define RF11_DCS_WRITE 0x0002 // function 1 (bit 1): write to drum
#define RF11_DCS_READ  0x0004 // function 2 (bit 2): read from drum
#define RF11_DCS_IE    0x0040 // bit 6: interrupt enable
#define RF11_DCS_READY 0x0080 // bit 7: ready / done
#define RF11_DCS_ERR   0x8000 // bit 15: error

#define RF11_BR        6    // processor level 6 (u0.s drum;300)
#define RF11_VECTOR    0204 // drum interrupt vector (u0.s . = orig+204)

// Register offsets within the block based at dcs = 0177460
#define RF11_DCS 0x0 // control & status
#define RF11_WC  0x2 // word count
#define RF11_CMA 0x4 // current memory address (bus address)
#define RF11_DAR 0x6 // disk address register
#define RF11_DAE 0x8 // disk address extension

#define RF11_BLOCK_BYTES 512 // bytes per drum block

typedef struct {
    unibus_dev_t* dev;

    uint8_t* store; // Backing image (in memory): root + swap
    size_t   size;  // Backing image size in bytes

    // Atomic register state
    uint32_t dcs; // control & status
    uint32_t wc;  // word count (two's complement)
    uint32_t cma; // current memory address (bus address)
    uint32_t dar; // disk address (low)
    uint32_t dae; // disk address extension (high)
} rf11_dev_t;

// Re-evaluate the completion interrupt from dcs (Done & IE)
static void rf11_update_irq(rf11_dev_t* rf)
{
    uint32_t dcs = atomic_load_uint32_relax(&rf->dcs);
    if ((dcs & RF11_DCS_READY) && (dcs & RF11_DCS_IE)) {
        unibus_raise_irq(rf->dev);
    } else {
        unibus_lower_irq(rf->dev);
    }
}

// Perform the NPR DMA transfer requested by a GO in dcs (Handbook p.268)
static void rf11_go(rf11_dev_t* rf)
{
    uint32_t dcs  = atomic_load_uint32_relax(&rf->dcs);
    uint32_t func = dcs & RF11_DCS_FUNC;

    // Word count is the two's complement of the number of words to move.
    uint32_t wc       = atomic_load_uint32_relax(&rf->wc);
    size_t   words    = (uint16_t)(-(int16_t)(uint16_t)wc);
    size_t   bytes    = words * 2;

    // Disk address: dar low | dae high give the starting block number; the
    // driver loads the physical block number split across dar/dae (u8.s).
    uint32_t dar      = atomic_load_uint32_relax(&rf->dar);
    uint32_t dae      = atomic_load_uint32_relax(&rf->dae);
    uint64_t block    = ((uint64_t)dae << 16) | (uint16_t)dar;
    uint64_t disk_off = block * RF11_BLOCK_BYTES;

    // Memory (Unibus) address the data is transferred to/from.
    rvvm_addr_t mem_addr = atomic_load_uint32_relax(&rf->cma);

    bool ok = false;
    if (disk_off + bytes <= rf->size) {
        if (func == RF11_DCS_READ) {
            // Drum -> memory (NPR DMA write into window RAM)
            ok = unibus_dma_write(rf->dev, mem_addr, rf->store + disk_off, bytes);
        } else if (func == RF11_DCS_WRITE) {
            // Memory -> drum (NPR DMA read from window RAM)
            ok = unibus_dma_read(rf->dev, mem_addr, rf->store + disk_off, bytes);
        }
    }

    // Update bus address / word count to reflect completion, set Ready, and
    // set Error if the transfer was out of bounds or had a bad function.
    atomic_store_uint32_relax(&rf->cma, (uint16_t)(mem_addr + bytes));
    atomic_store_uint32_relax(&rf->wc, 0);

    uint32_t new_dcs = (dcs & ~(RF11_DCS_GO | RF11_DCS_ERR)) | RF11_DCS_READY;
    if (!ok) {
        new_dcs |= RF11_DCS_ERR;
    }
    atomic_store_uint32_relax(&rf->dcs, new_dcs);
    rf11_update_irq(rf);
}

static void rf11_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    rf11_dev_t* rf = unibus_dev_data(dev);
    switch (off) {
        case RF11_DCS:
            *val = (uint16_t)atomic_load_uint32_relax(&rf->dcs);
            break;
        case RF11_WC:
            *val = (uint16_t)atomic_load_uint32_relax(&rf->wc);
            break;
        case RF11_CMA:
            *val = (uint16_t)atomic_load_uint32_relax(&rf->cma);
            break;
        case RF11_DAR:
            *val = (uint16_t)atomic_load_uint32_relax(&rf->dar);
            break;
        case RF11_DAE:
            *val = (uint16_t)atomic_load_uint32_relax(&rf->dae);
            break;
        default:
            *val = 0;
            break;
    }
}

static void rf11_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    rf11_dev_t* rf = unibus_dev_data(dev);
    switch (off) {
        case RF11_DCS:
            // Writing dcs with GO set launches the transfer; clear Ready
            // while busy, then rf11_go() sets it again on completion.
            atomic_store_uint32_relax(&rf->dcs, val & ~RF11_DCS_READY);
            if (val & RF11_DCS_GO) {
                rf11_go(rf);
            } else {
                rf11_update_irq(rf);
            }
            break;
        case RF11_WC:
            atomic_store_uint32_relax(&rf->wc, val);
            break;
        case RF11_CMA:
            atomic_store_uint32_relax(&rf->cma, val);
            break;
        case RF11_DAR:
            atomic_store_uint32_relax(&rf->dar, val);
            break;
        case RF11_DAE:
            atomic_store_uint32_relax(&rf->dae, val);
            break;
        default:
            break;
    }
}

static void rf11_poll(unibus_dev_t* dev)
{
    // Re-assert a level-triggered completion IRQ if the guest re-enabled IE
    // without clearing Done. (No periodic work is otherwise needed.)
    rf11_update_irq(unibus_dev_data(dev));
}

static void rf11_cleanup(unibus_dev_t* dev)
{
    rf11_dev_t* rf = unibus_dev_data(dev);
    free(rf->store);
}

RVVM_PUBLIC unibus_dev_t* rvvm_rf11_init(unibus_t* bus, size_t image_size_blocks)
{
    if (!bus) {
        return NULL;
    }
    if (!image_size_blocks) {
        image_size_blocks = 1024; // 1st Edition UNIX drum: 1024 blocks
    }

    rf11_dev_t* rf = safe_new_obj(rf11_dev_t);
    rf->size       = image_size_blocks * RF11_BLOCK_BYTES;
    rf->store      = safe_calloc(rf->size, 1);
    rf->dcs        = RF11_DCS_READY; // idle controller is ready

    unibus_dev_desc_t desc = {
        .name     = "rf11",
        .io_addr  = 0xFF30, // dcs = 0177460
        .size     = 0xA,    // dcs(0)..dae(8), 5 word registers
        .read     = rf11_read,
        .write    = rf11_write,
        .poll     = rf11_poll,
        .cleanup  = rf11_cleanup,
        .min_size = 2,
        .br_level = RF11_BR,
        .vector   = RF11_VECTOR,
        .data     = rf,
    };

    // Save the backing store pointer: on attach failure the bus frees rf
    // (it owns desc.data), but not the separately-allocated store. Capture
    // it first to free without touching the freed rf.
    uint8_t* store = rf->store;

    rf->dev = unibus_attach(bus, &desc);
    if (!rf->dev) {
        free(store);
        return NULL;
    }
    return rf->dev;
}

POP_OPTIMIZATION_SIZE
