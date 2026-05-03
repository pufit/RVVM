/*
spi-nor.c - Generic JEDEC SPI NOR flash slave (Winbond W25Q-family compatible)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "spi-nor.h"
#include "blk_io.h"
#include "compiler.h"
#include "fdtlib.h"
#include "utils.h"

#include <string.h>

PUSH_OPTIMIZATION_SIZE

// Command set — Winbond W25Q datasheet §8 (matches the JEDEC common
// SPI NOR opcodes implemented by Linux's spi-nor core for chips with
// the no_sfdp_flags = SECT_4K path used by the w25qNN entries).
#define CMD_WREN       0x06  // Write Enable
#define CMD_WRDI       0x04  // Write Disable
#define CMD_RDSR1      0x05  // Read Status Register 1
#define CMD_RDSR2      0x35  // Read Status Register 2
#define CMD_RDSR3      0x15  // Read Status Register 3
#define CMD_WRSR1      0x01  // Write Status Register 1
#define CMD_WRSR2      0x31  // Write Status Register 2
#define CMD_WRSR3      0x11  // Write Status Register 3
#define CMD_READ       0x03  // Read Data
#define CMD_FAST_READ  0x0B  // Fast Read
#define CMD_DOR        0x3B  // Dual Output Fast Read   (1-1-2)
#define CMD_QOR        0x6B  // Quad Output Fast Read   (1-1-4)
#define CMD_DIOR       0xBB  // Dual I/O Fast Read      (1-2-2)
#define CMD_QIOR       0xEB  // Quad I/O Fast Read      (1-4-4)
#define CMD_PP         0x02  // Page Program
#define CMD_SE         0x20  // Sector Erase (4 KB)
#define CMD_BE32       0x52  // Block Erase (32 KB)
#define CMD_BE64       0xD8  // Block Erase (64 KB)
#define CMD_CE_C7      0xC7  // Chip Erase (variant 1)
#define CMD_CE_60      0x60  // Chip Erase (variant 2)
#define CMD_RDID       0x9F  // Read JEDEC ID (mfr + 2-byte device)
#define CMD_REMS       0x90  // Read Mfr/Dev ID (legacy)
#define CMD_RDPD       0xAB  // Release Power-Down / Read Device ID
#define CMD_DP         0xB9  // Deep Power-Down
#define CMD_RSTEN      0x66  // Enable Reset
#define CMD_RST        0x99  // Reset

// Status Register 1 bits.
#define SR1_BUSY       0x01  // Always 0 here — programs/erases finish in 0ns
#define SR1_WEL        0x02  // Write Enable Latch — mirrors the WREN/WRDI state

#define PAGE_SIZE      256u
#define SECTOR_SIZE    4096u
#define BLOCK32_SIZE   (32u * 1024u)
#define BLOCK64_SIZE   (64u * 1024u)

// Maximum device-side state machine length in a single CS-low session.
// Only matters as a sanity ceiling — page programs are bounded at 4 cmd
// + 256 data bytes, reads can be arbitrarily long but stream out from
// the file with no internal buffering.
typedef struct {
    rvfile_t* file;
    uint64_t  size;       // chip capacity in bytes (power of 2)
    uint8_t   jedec[3];   // mfr_id, mem_type, capacity_byte

    // Per-CS-low session state. Reset on falling CS edge; finalized on
    // rising CS edge.
    bool      cs_low;
    uint8_t   cmd;
    uint32_t  byte_idx;   // 0 = cmd byte, 1+ = arg/data bytes
    uint32_t  addr;       // assembled from bytes 1..3
    uint8_t   arg_byte;   // first arg byte for WRSR1/2/3

    // Page-program scratch — bytes the host shifts in are merged into a
    // 256-entry buffer (program wraps within the page on real silicon).
    // `pp_dirty` tracks which offsets were touched so we only write
    // those back on CS deassert; matches the AND-with-existing semantics
    // of NOR (program can only flip 1→0).
    uint8_t   pp_buf[PAGE_SIZE];
    uint8_t   pp_dirty[PAGE_SIZE / 8];

    // Persistent device state.
    uint8_t   sr1;
    uint8_t   sr2;
    uint8_t   sr3;
    bool      wel;
    bool      power_down;
} spi_nor_t;

static const struct {
    uint64_t    size;
    uint8_t     cap_byte;
    const char* name;
} chip_table[] = {
    {2u * 1024u * 1024u,  0x15, "w25q16"},
    {4u * 1024u * 1024u,  0x16, "w25q32"},
    {8u * 1024u * 1024u,  0x17, "w25q64"},
    {16u * 1024u * 1024u, 0x18, "w25q128"},
};

#define CHIP_TABLE_SIZE (sizeof(chip_table) / sizeof(chip_table[0]))

static void erase_range(spi_nor_t* nor, uint64_t addr, uint64_t len)
{
    if (addr >= nor->size) return;
    if (addr + len > nor->size) len = nor->size - addr;

    uint8_t fill[4096];
    memset(fill, 0xFF, sizeof(fill));
    while (len > 0) {
        size_t chunk = len > sizeof(fill) ? sizeof(fill) : (size_t)len;
        rvwrite(nor->file, fill, chunk, addr);
        addr += chunk;
        len  -= chunk;
    }
}

static void program_commit(spi_nor_t* nor)
{
    // Page program: walk the dirty bitmap and AND each touched byte
    // with its current file contents. NOR flash can only flip 1→0
    // without an erase; emulate that so JFFS2/UBIFS see the same
    // semantics they'd get on real hardware.
    uint32_t page_base = nor->addr & ~(PAGE_SIZE - 1u);
    if (page_base >= nor->size) return;

    for (uint32_t off = 0; off < PAGE_SIZE; off++) {
        if (!(nor->pp_dirty[off / 8] & (1u << (off % 8)))) continue;
        uint64_t target = (uint64_t)page_base + off;
        if (target >= nor->size) break;
        uint8_t old = 0xFF;
        rvread(nor->file, &old, 1, target);
        uint8_t new_b = old & nor->pp_buf[off];
        rvwrite(nor->file, &new_b, 1, target);
    }
}

static void spi_nor_select(void* dev, bool asserted)
{
    spi_nor_t* nor = dev;

    if (asserted) {
        // Falling CS: latch a fresh command session.
        nor->cs_low   = true;
        nor->byte_idx = 0;
        nor->cmd      = 0xFF;
        nor->addr     = 0;
        nor->arg_byte = 0;
        memset(nor->pp_dirty, 0, sizeof(nor->pp_dirty));
        return;
    }

    // Rising CS: finalize whatever command was issued.
    if (!nor->cs_low) return;
    nor->cs_low = false;

    switch (nor->cmd) {
        case CMD_WREN:
            nor->wel  = true;
            nor->sr1 |= SR1_WEL;
            break;
        case CMD_WRDI:
            nor->wel  = false;
            nor->sr1 &= (uint8_t)~SR1_WEL;
            break;
        case CMD_WRSR1:
            if (nor->wel && nor->byte_idx >= 2) {
                nor->sr1  = (nor->arg_byte & ~SR1_BUSY) | (nor->sr1 & SR1_WEL);
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_WRSR2:
            if (nor->wel && nor->byte_idx >= 2) {
                nor->sr2  = nor->arg_byte;
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_WRSR3:
            if (nor->wel && nor->byte_idx >= 2) {
                nor->sr3  = nor->arg_byte;
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_PP:
            if (nor->wel && nor->byte_idx >= 5) {
                program_commit(nor);
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_SE:
            if (nor->wel && nor->byte_idx >= 4) {
                erase_range(nor, nor->addr & ~(SECTOR_SIZE - 1u), SECTOR_SIZE);
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_BE32:
            if (nor->wel && nor->byte_idx >= 4) {
                erase_range(nor, nor->addr & ~(BLOCK32_SIZE - 1u), BLOCK32_SIZE);
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_BE64:
            if (nor->wel && nor->byte_idx >= 4) {
                erase_range(nor, nor->addr & ~(BLOCK64_SIZE - 1u), BLOCK64_SIZE);
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_CE_C7:
        case CMD_CE_60:
            if (nor->wel) {
                erase_range(nor, 0, nor->size);
                nor->wel  = false;
                nor->sr1 &= (uint8_t)~SR1_WEL;
            }
            break;
        case CMD_DP:
            nor->power_down = true;
            break;
        case CMD_RDPD:
            nor->power_down = false;
            break;
        case CMD_RSTEN:
        case CMD_RST:
            // Reset is a no-op for the model — there is no in-flight
            // operation to abort and persistent state survives reset.
            break;
        default:
            break;
    }
}

static uint8_t spi_nor_transfer(void* dev, uint8_t tx)
{
    spi_nor_t* nor = dev;

    if (nor->byte_idx == 0) {
        nor->cmd = tx;
        nor->byte_idx++;
        return 0xFF;
    }

    uint32_t i = nor->byte_idx - 1;  // 0-based arg index
    nor->byte_idx++;

    switch (nor->cmd) {
        case CMD_RDID:
            // Three JEDEC ID bytes streamed out; further reads return
            // 0xFF, matching real silicon's "no more data" idle.
            return (i < 3) ? nor->jedec[i] : 0xFF;

        case CMD_REMS:
            // 3 dummy/addr bytes, then alternating mfr/dev forever.
            // The kernel sends address 0 and reads 2 bytes.
            if (i < 3) return 0xFF;
            return ((i - 3) & 1u) ? nor->jedec[2] : nor->jedec[0];

        case CMD_RDPD:
            // 3 dummy bytes, then device ID byte (capacity nibble).
            return (i < 3) ? 0xFF : nor->jedec[2];

        case CMD_RDSR1:
            return nor->sr1;
        case CMD_RDSR2:
            return nor->sr2;
        case CMD_RDSR3:
            return nor->sr3;

        case CMD_WRSR1:
        case CMD_WRSR2:
        case CMD_WRSR3:
            if (i == 0) nor->arg_byte = tx;
            return 0xFF;

        case CMD_READ: {
            // bytes 0..2 of args = address (24-bit big-endian).
            if (i < 3) {
                nor->addr = (nor->addr << 8) | tx;
                return 0xFF;
            }
            // bytes 3+: data. Address advances and wraps at chip size.
            uint8_t b = 0xFF;
            rvread(nor->file, &b, 1, nor->addr);
            nor->addr = (uint32_t)(((uint64_t)nor->addr + 1u) % nor->size);
            return b;
        }

        case CMD_FAST_READ:
        case CMD_DOR:
        case CMD_QOR:
        case CMD_DIOR:
        case CMD_QIOR: {
            // All four "fast read" variants share the same byte-level layout
            // from the slave's POV — only the filler-byte count between the
            // 24-bit address and the data stream differs. Lane width is
            // invisible here: dual/quad just clock more bits per cycle on
            // the wire, but TXDATA/RXDATA still hand us one byte per frame.
            //
            //   0x0B FAST_READ   : addr(3) + dummy(1)            + data
            //   0x3B DOR  (1-1-2): addr(3) + dummy(1)            + data
            //   0x6B QOR  (1-1-4): addr(3) + dummy(1)            + data
            //   0xBB DIOR (1-2-2): addr(3) + mode(1)             + data
            //   0xEB QIOR (1-4-4): addr(3) + mode(1) + dummy(2)  + data
            //
            // The mode byte is consumed but ignored — continuous-read mode
            // (where M[5:4]=0b10 lets the next transaction skip the cmd
            // phase) isn't enabled by Linux's spi-nor core on this path.
            uint32_t fillers;
            switch (nor->cmd) {
                case CMD_DIOR: fillers = 1; break;  // mode only
                case CMD_QIOR: fillers = 3; break;  // mode + 2 dummy
                default:       fillers = 1; break;  // 1 dummy byte
            }
            if (i < 3) {
                nor->addr = (nor->addr << 8) | tx;
                return 0xFF;
            }
            if (i < 3 + fillers) return 0xFF;
            uint8_t b = 0xFF;
            rvread(nor->file, &b, 1, nor->addr);
            nor->addr = (uint32_t)(((uint64_t)nor->addr + 1u) % nor->size);
            return b;
        }

        case CMD_PP: {
            if (i < 3) {
                nor->addr = (nor->addr << 8) | tx;
                return 0xFF;
            }
            // Program data: each byte's offset within the 256-byte page
            // is computed modulo PAGE_SIZE so writes wrap (real silicon
            // increments only A[7:0] across PP).
            uint32_t off = (uint32_t)(((uint64_t)nor->addr + (i - 3)) & (PAGE_SIZE - 1u));
            nor->pp_buf[off]            = tx;
            nor->pp_dirty[off / 8]     |= (uint8_t)(1u << (off % 8));
            return 0xFF;
        }

        case CMD_SE:
        case CMD_BE32:
        case CMD_BE64:
            // 24-bit address, no data bytes.
            if (i < 3) {
                nor->addr = (nor->addr << 8) | tx;
            }
            return 0xFF;

        default:
            return 0xFF;
    }
}

static void spi_nor_remove(void* dev)
{
    spi_nor_t* nor = dev;
    if (nor->file) {
        rvfsync(nor->file);
        rvclose(nor->file);
    }
    free(nor);
}

// Open `path` (creating if absent), grow to `size` if smaller, and pad
// any newly-introduced region with 0xFF (erased-flash state). Returns
// NULL on any failure; caller must rvclose if non-NULL.
static rvfile_t* open_or_create_image(const char* path, uint64_t size)
{
    rvfile_t* file = rvopen(path, RVFILE_RW | RVFILE_CREAT);
    if (!file) return NULL;

    uint64_t cur = rvfilesize(file);
    if (cur < size) {
        if (!rvtruncate(file, size)) {
            rvclose(file);
            return NULL;
        }
        // rvtruncate zero-fills the gap; overwrite it with 0xFF.
        uint8_t fill[4096];
        memset(fill, 0xFF, sizeof(fill));
        uint64_t off = cur;
        while (off < size) {
            size_t chunk = (size - off > sizeof(fill)) ? sizeof(fill) : (size_t)(size - off);
            if (rvwrite(file, fill, chunk, off) != chunk) {
                rvclose(file);
                return NULL;
            }
            off += chunk;
        }
        rvfsync(file);
    }
    return file;
}

PUBLIC uint16_t spi_nor_attach(spi_bus_t* bus, uint16_t cs_id,
                               const char* image_path, uint64_t size_bytes)
{
    if (!bus || !image_path) return SPI_AUTO_CS;

    // Pick the chip variant whose capacity is ≥ requested size. 0
    // defaults to 8 MB (W25Q64). Anything bigger than the table caps
    // at the largest entry.
    if (size_bytes == 0) size_bytes = 8u * 1024u * 1024u;
    size_t chip_idx = CHIP_TABLE_SIZE - 1;
    for (size_t i = 0; i < CHIP_TABLE_SIZE; i++) {
        if (chip_table[i].size >= size_bytes) {
            chip_idx = i;
            break;
        }
    }
    if (size_bytes > chip_table[CHIP_TABLE_SIZE - 1].size) {
        rvvm_warn("spi-nor: requested size %lu exceeds max %lu, capping",
                  (unsigned long)size_bytes,
                  (unsigned long)chip_table[CHIP_TABLE_SIZE - 1].size);
    }

    uint64_t chip_size = chip_table[chip_idx].size;

    rvfile_t* file = open_or_create_image(image_path, chip_size);
    if (!file) {
        rvvm_warn("spi-nor: failed to open %s", image_path);
        return SPI_AUTO_CS;
    }

    spi_nor_t* nor = safe_new_obj(spi_nor_t);
    nor->file      = file;
    nor->size      = chip_size;
    nor->jedec[0]  = 0xEF;  // Winbond
    nor->jedec[1]  = 0x40;  // Standard SPI memory type (W25Q-series)
    nor->jedec[2]  = chip_table[chip_idx].cap_byte;

    spi_dev_t desc = {
        .cs_id    = cs_id,
        .data     = nor,
        .select   = spi_nor_select,
        .transfer = spi_nor_transfer,
        .remove   = spi_nor_remove,
    };

    uint16_t assigned = spi_attach_dev(bus, &desc);
    if (assigned == SPI_AUTO_CS) {
        rvvm_warn("spi-nor: bus attach failed");
        spi_nor_remove(nor);
        return SPI_AUTO_CS;
    }

#ifdef USE_FDT
    // Linux's spi-nor probes any child with compatible="jedec,spi-nor"
    // and reads the JEDEC ID at runtime to look up chip-specific flags.
    // Naming the node "flash@N" so dtc/decompile output reads cleanly.
    struct fdt_node* parent = spi_bus_fdt_node(bus);
    if (parent) {
        struct fdt_node* flash = fdt_node_create_reg("flash", assigned);
        fdt_node_add_prop_u32(flash, "reg", assigned);
        fdt_node_add_prop_str(flash, "compatible", "jedec,spi-nor");
        fdt_node_add_prop_u32(flash, "spi-max-frequency", 50000000);
        fdt_node_add_prop_str(flash, "status", "okay");
        fdt_node_add_child(parent, flash);
    }
#endif

    rvvm_info("spi-nor: attached %s (%lu MiB) on CS%u image=%s",
              chip_table[chip_idx].name,
              (unsigned long)(chip_size >> 20),
              (unsigned)assigned, image_path);

    return assigned;
}

POP_OPTIMIZATION_SIZE
