/*
spi-sifive.c - SiFive FU540/FU740 SPI controller (FIFO mode)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "spi-sifive.h"
#include "compiler.h"
#include "fdtlib.h"
#include "mem_ops.h"
#include "spinlock.h"
#include "utils.h"

#include <string.h>

PUSH_OPTIMIZATION_SIZE

// FU540-C000 §15. Linux driver: drivers/spi/spi-sifive.c.

#define SPI_SIFIVE_REG_SCKDIV   0x00
#define SPI_SIFIVE_REG_SCKMODE  0x04
#define SPI_SIFIVE_REG_CSID     0x10
#define SPI_SIFIVE_REG_CSDEF    0x14
#define SPI_SIFIVE_REG_CSMODE   0x18
#define SPI_SIFIVE_REG_DELAY0   0x28
#define SPI_SIFIVE_REG_DELAY1   0x2C
#define SPI_SIFIVE_REG_FMT      0x40
#define SPI_SIFIVE_REG_TXDATA   0x48
#define SPI_SIFIVE_REG_RXDATA   0x4C
#define SPI_SIFIVE_REG_TXMARK   0x50
#define SPI_SIFIVE_REG_RXMARK   0x54
#define SPI_SIFIVE_REG_FCTRL    0x60
#define SPI_SIFIVE_REG_FFMT     0x64
#define SPI_SIFIVE_REG_IE       0x70
#define SPI_SIFIVE_REG_IP       0x74

#define SPI_SIFIVE_MMIO_SIZE    0x1000

// CSMODE values (§15.5).
#define CSMODE_AUTO  0x0u  // Pulse CS per frame (default)
#define CSMODE_HOLD  0x2u  // Keep CS asserted across multiple frames
#define CSMODE_OFF   0x3u  // CS disabled (no transfer occurs)

// FMT bits (§15.7). LEN=8 is the only width Linux's driver issues.
#define FMT_DIR_BIT     (1u << 3)   // 1 = TX-only, RX bytes discarded

// IP / IE (§15.13/§15.14).
#define IP_TXWM         (1u << 0)
#define IP_RXWM         (1u << 1)

// TXDATA[31] = FULL, RXDATA[31] = EMPTY.
#define DATA_FULL_BIT   (1u << 31)
#define DATA_EMPTY_BIT  (1u << 31)

// We expose a fixed number of CS lines; matches FU540 (4 native CS).
// The driver auto-probes via CSDEF, so this just sets the writable mask.
#define SPI_SIFIVE_NUM_CS 4

#define RX_FIFO_DEPTH SPI_SIFIVE_FIFO_DEPTH
#define RX_FIFO_MASK  (RX_FIFO_DEPTH - 1)

typedef struct {
    spi_bus_t*   bus;
    rvvm_intc_t* intc;
    rvvm_irq_t   irq;
    spinlock_t   lock;

    // Programmed register state. Most are stored verbatim for read-back;
    // semantic effect is limited to FMT.DIR, CSID, CSMODE, TXMARK, RXMARK,
    // CSDEF (probe mask), and IE (IRQ gate).
    uint32_t sckdiv;
    uint32_t sckmode;
    uint32_t csid;
    uint32_t csdef;
    uint32_t csmode;
    uint32_t delay0;
    uint32_t delay1;
    uint32_t fmt;
    uint32_t txmark;
    uint32_t rxmark;
    uint32_t fctrl;
    uint32_t ffmt;
    uint32_t ie;

    // RX FIFO. Producer = TXDATA write path (after slave shift-out);
    // consumer = RXDATA read path. Single shared lock — no real
    // concurrency between producer and consumer here because the kernel
    // serialises register access from one CPU per transfer.
    uint8_t  rx_fifo[RX_FIFO_DEPTH];
    uint32_t rx_head;
    uint32_t rx_tail;

    // Currently-asserted CS, or -1 when no slave is held. Tracked across
    // TXDATA writes so multi-byte HOLD-mode transactions (typical flash
    // command + address + data sequence) keep CS low for the slave.
    int32_t  cs_held;

    // Last raised IRQ level — debounces redundant raise/lower calls.
    bool     irq_level;
} spi_sifive_dev_t;

static uint32_t rx_count(const spi_sifive_dev_t* spi)
{
    return spi->rx_head - spi->rx_tail;
}

static bool rx_empty(const spi_sifive_dev_t* spi)
{
    return rx_count(spi) == 0;
}

static bool rx_full(const spi_sifive_dev_t* spi)
{
    return rx_count(spi) >= RX_FIFO_DEPTH;
}

static void rx_push(spi_sifive_dev_t* spi, uint8_t b)
{
    if (rx_full(spi)) {
        // Real silicon would silently drop or stall here. Drop matches
        // the kernel's sequencing — RXMARK is set so the wait completes
        // exactly when expected entries land; we never overflow if the
        // driver follows the protocol.
        return;
    }
    spi->rx_fifo[spi->rx_head & RX_FIFO_MASK] = b;
    spi->rx_head++;
}

static bool rx_pop(spi_sifive_dev_t* spi, uint8_t* out)
{
    if (rx_empty(spi)) return false;
    *out = spi->rx_fifo[spi->rx_tail & RX_FIFO_MASK];
    spi->rx_tail++;
    return true;
}

// Compute the IP register value from current FIFO state and watermarks.
//
// IP.TXWM asserts when the (modeled) TX FIFO has fewer entries than
// TXMARK. We drain TX synchronously inside the TXDATA write path —
// every byte is transferred to the slave before the write returns — so
// the modeled TX FIFO is always empty. IP.TXWM therefore mirrors
// (TXMARK > 0); the driver's default TXMARK=1 keeps it asserted, which
// is exactly what its TX-only completion wait expects.
//
// IP.RXWM asserts when RX FIFO entries > RXMARK. The driver writes
// RXMARK = N-1 before pushing a chunk of N bytes, so it fires exactly
// once N responses have landed.
static uint32_t compute_ip(const spi_sifive_dev_t* spi)
{
    uint32_t ip = 0;
    if (spi->txmark > 0) {
        ip |= IP_TXWM;
    }
    if (rx_count(spi) > spi->rxmark) {
        ip |= IP_RXWM;
    }
    return ip;
}

// Caller must drop the lock before invoking; raise/lower may take the
// intc's own lock and we don't want nested locking.
static bool update_irq_level(spi_sifive_dev_t* spi)
{
    bool desired = (compute_ip(spi) & spi->ie) != 0;
    if (desired != spi->irq_level) {
        spi->irq_level = desired;
        return true;
    }
    return false;
}

static void apply_irq(spi_sifive_dev_t* spi)
{
    if (spi->irq_level) {
        rvvm_raise_irq(spi->intc, spi->irq);
    } else {
        rvvm_lower_irq(spi->intc, spi->irq);
    }
}

// Drop the currently-held CS, if any. Caller holds the bus lock.
static void release_cs(spi_sifive_dev_t* spi)
{
    if (spi->cs_held >= 0) {
        spi_bus_select(spi->bus, (uint16_t)spi->cs_held, false);
        spi->cs_held = -1;
    }
}

// Run one byte-frame transfer. Caller holds the bus lock; returns the
// MISO byte the slave shifted out so the caller can decide whether to
// enqueue it into the RX FIFO (depending on FMT.DIR).
static uint8_t do_transfer(spi_sifive_dev_t* spi, uint8_t tx)
{
    uint16_t cs   = (uint16_t)(spi->csid & 0x1Fu);
    uint32_t mode = spi->csmode & 0x3u;

    if (mode == CSMODE_OFF) {
        // CS forced inactive — no slave sees the byte. Real silicon
        // still shifts something onto MOSI but with no CS no peripheral
        // latches it. Match that by returning 0xFF (idle bus).
        release_cs(spi);
        return 0xFF;
    }

    if (mode == CSMODE_HOLD) {
        // Continuous-CS transfers. Switching CSID mid-hold deassert
        // the old line first so we don't address two slaves at once;
        // the kernel never does this in practice but it costs nothing.
        if (spi->cs_held != (int32_t)cs) {
            release_cs(spi);
            spi_bus_select(spi->bus, cs, true);
            spi->cs_held = (int32_t)cs;
        }
        return spi_bus_transfer(spi->bus, cs, tx);
    }

    // CSMODE_AUTO — pulse CS per frame. Should never overlap a held
    // session, but if it does (the kernel changing modes between bytes)
    // drop the held line first.
    release_cs(spi);
    spi_bus_select(spi->bus, cs, true);
    uint8_t rx = spi_bus_transfer(spi->bus, cs, tx);
    spi_bus_select(spi->bus, cs, false);
    return rx;
}

static bool spi_sifive_mmio_read(rvvm_mmio_dev_t* mmio, void* data, size_t off, uint8_t size)
{
    spi_sifive_dev_t* spi = mmio->data;
    uint32_t          val = 0;
    UNUSED(size);

    spin_lock(&spi->lock);
    switch (off) {
        case SPI_SIFIVE_REG_SCKDIV:  val = spi->sckdiv;  break;
        case SPI_SIFIVE_REG_SCKMODE: val = spi->sckmode; break;
        case SPI_SIFIVE_REG_CSID:    val = spi->csid;    break;
        case SPI_SIFIVE_REG_CSDEF:   val = spi->csdef;   break;
        case SPI_SIFIVE_REG_CSMODE:  val = spi->csmode;  break;
        case SPI_SIFIVE_REG_DELAY0:  val = spi->delay0;  break;
        case SPI_SIFIVE_REG_DELAY1:  val = spi->delay1;  break;
        case SPI_SIFIVE_REG_FMT:     val = spi->fmt;     break;
        case SPI_SIFIVE_REG_TXDATA:
            // TXDATA reads probe the FULL flag without popping. We drain
            // synchronously so the FIFO is never modeled as full.
            val = 0;
            break;
        case SPI_SIFIVE_REG_RXDATA: {
            uint8_t b = 0;
            if (rx_pop(spi, &b)) {
                val = b;
            } else {
                val = DATA_EMPTY_BIT;
            }
            break;
        }
        case SPI_SIFIVE_REG_TXMARK:  val = spi->txmark;  break;
        case SPI_SIFIVE_REG_RXMARK:  val = spi->rxmark;  break;
        case SPI_SIFIVE_REG_FCTRL:   val = spi->fctrl;   break;
        case SPI_SIFIVE_REG_FFMT:    val = spi->ffmt;    break;
        case SPI_SIFIVE_REG_IE:      val = spi->ie;      break;
        case SPI_SIFIVE_REG_IP:      val = compute_ip(spi); break;
        default: break;
    }

    bool irq_changed = false;
    if (off == SPI_SIFIVE_REG_RXDATA) {
        // Popping a byte may flip IP.RXWM low; recompute.
        irq_changed = update_irq_level(spi);
    }
    spin_unlock(&spi->lock);

    if (irq_changed) apply_irq(spi);
    write_uint32_le(data, val);
    return true;
}

static bool spi_sifive_mmio_write(rvvm_mmio_dev_t* mmio, void* data, size_t off, uint8_t size)
{
    spi_sifive_dev_t* spi = mmio->data;
    uint32_t          val = read_uint32_le(data);
    UNUSED(size);

    spin_lock(&spi->lock);
    bool irq_changed = false;
    switch (off) {
        case SPI_SIFIVE_REG_SCKDIV:  spi->sckdiv  = val & 0xFFFu; break;
        case SPI_SIFIVE_REG_SCKMODE: spi->sckmode = val & 0x3u;   break;
        case SPI_SIFIVE_REG_CSID:
            // Switching CSID while a slave is held is undefined on real
            // hardware; release the line so we don't strand a slave with
            // CS asserted.
            if (spi->cs_held >= 0 && spi->cs_held != (int32_t)(val & 0x1Fu)) {
                release_cs(spi);
            }
            spi->csid = val & 0x1Fu;
            break;
        case SPI_SIFIVE_REG_CSDEF:
            // Mask to physically-present CS bits. The driver probes by
            // writing 0xFFFFFFFF and reading back — we reflect only the
            // bits we actually drive.
            spi->csdef = val & ((1u << SPI_SIFIVE_NUM_CS) - 1u);
            break;
        case SPI_SIFIVE_REG_CSMODE: {
            uint32_t new_mode = val & 0x3u;
            // Leaving HOLD (or moving to OFF) drops the active CS line.
            if (new_mode != CSMODE_HOLD) {
                release_cs(spi);
            }
            spi->csmode = new_mode;
            break;
        }
        case SPI_SIFIVE_REG_DELAY0:  spi->delay0 = val; break;
        case SPI_SIFIVE_REG_DELAY1:  spi->delay1 = val; break;
        case SPI_SIFIVE_REG_FMT:     spi->fmt    = val; break;
        case SPI_SIFIVE_REG_TXDATA: {
            uint8_t tx = (uint8_t)(val & 0xFFu);
            uint8_t rx = do_transfer(spi, tx);
            if (!(spi->fmt & FMT_DIR_BIT)) {
                rx_push(spi, rx);
            }
            irq_changed = update_irq_level(spi);
            break;
        }
        case SPI_SIFIVE_REG_TXMARK:
            spi->txmark = val & 0x7u;
            irq_changed = update_irq_level(spi);
            break;
        case SPI_SIFIVE_REG_RXMARK:
            spi->rxmark = val & 0x7u;
            irq_changed = update_irq_level(spi);
            break;
        case SPI_SIFIVE_REG_FCTRL:
            // XIP-mode select. Driver writes 0 at probe to ensure we're
            // in FIFO mode; we don't model XIP at all, so just store the
            // bit for read-back. Anything non-zero is silently ignored.
            spi->fctrl = val & 0x1u;
            break;
        case SPI_SIFIVE_REG_FFMT:    spi->ffmt = val; break;
        case SPI_SIFIVE_REG_IE:
            spi->ie = val & (IP_TXWM | IP_RXWM);
            irq_changed = update_irq_level(spi);
            break;
        case SPI_SIFIVE_REG_IP:
            // IP is read-only; writes ignored on real silicon.
            break;
        default: break;
    }
    spin_unlock(&spi->lock);

    if (irq_changed) apply_irq(spi);
    return true;
}

static void spi_sifive_remove(rvvm_mmio_dev_t* mmio)
{
    spi_sifive_dev_t* spi = mmio->data;
    spi_bus_free(spi->bus);
    free(spi);
}

static rvvm_mmio_type_t spi_sifive_dev_type = {
    .name   = "spi_sifive",
    .remove = spi_sifive_remove,
};

PUBLIC rvvm_mmio_dev_t* spi_sifive_init(rvvm_machine_t* machine, rvvm_addr_t addr,
                                        rvvm_intc_t* intc, rvvm_irq_t irq)
{
    spi_sifive_dev_t* spi = safe_new_obj(spi_sifive_dev_t);
    spi->bus              = spi_bus_create();
    spi->intc             = intc;
    spi->irq              = irq;
    spi->cs_held          = -1;

    // Power-on defaults that the kernel driver immediately overwrites,
    // but they need to be sane for the CSDEF probe (auto-probe writes
    // 0xFFFFFFFF then reads back; we mask to the implemented CS bits).
    spi->csdef            = (1u << SPI_SIFIVE_NUM_CS) - 1u;
    spi->csmode           = CSMODE_AUTO;
    spi->txmark           = 1;
    spi->rxmark           = 0;

    rvvm_mmio_dev_t mmio_desc = {
        .addr        = addr,
        .size        = SPI_SIFIVE_MMIO_SIZE,
        .data        = spi,
        .type        = &spi_sifive_dev_type,
        .read        = spi_sifive_mmio_read,
        .write       = spi_sifive_mmio_write,
        .min_op_size = 4,
        .max_op_size = 4,
    };

    rvvm_mmio_dev_t* mmio = rvvm_attach_mmio(machine, &mmio_desc);
    if (mmio == NULL) {
        spi_bus_free(spi->bus);
        free(spi);
        return NULL;
    }

#ifdef USE_FDT
    // The driver requires a clock binding (devm_clk_get). Reuse a shared
    // SPI oscillator node, creating one on first attach so multiple SPI
    // controllers can share the same fixed-clock source.
    struct fdt_node* spi_osc = fdt_node_find(rvvm_get_fdt_root(machine), "spi_osc");
    if (spi_osc == NULL) {
        spi_osc = fdt_node_create("spi_osc");
        fdt_node_add_prop_str(spi_osc, "compatible", "fixed-clock");
        fdt_node_add_prop_u32(spi_osc, "#clock-cells", 0);
        fdt_node_add_prop_u32(spi_osc, "clock-frequency", 50000000);
        fdt_node_add_prop_str(spi_osc, "clock-output-names", "spi-clk");
        fdt_node_add_child(rvvm_get_fdt_root(machine), spi_osc);
    }

    struct fdt_node* spi_fdt = fdt_node_create_reg("spi", mmio_desc.addr);
    fdt_node_add_prop_reg(spi_fdt, "reg", mmio_desc.addr, mmio_desc.size);
    fdt_node_add_prop_str(spi_fdt, "compatible", "sifive,spi0");
    rvvm_fdt_describe_irq(spi_fdt, intc, irq);
    fdt_node_add_prop_u32(spi_fdt, "clocks", fdt_node_get_phandle(spi_osc));
    fdt_node_add_prop_str(spi_fdt, "clock-names", "spi-clk");
    fdt_node_add_prop_u32(spi_fdt, "sifive,fifo-depth", SPI_SIFIVE_FIFO_DEPTH);
    fdt_node_add_prop_u32(spi_fdt, "sifive,max-bits-per-word", 8);
    fdt_node_add_prop_u32(spi_fdt, "#address-cells", 1);
    fdt_node_add_prop_u32(spi_fdt, "#size-cells", 0);
    fdt_node_add_prop_str(spi_fdt, "status", "okay");
    fdt_node_add_child(rvvm_get_fdt_soc(machine), spi_fdt);
    spi_bus_set_fdt_node(spi->bus, spi_fdt);
#endif
    return mmio;
}

PUBLIC rvvm_mmio_dev_t* spi_sifive_init_auto(rvvm_machine_t* machine)
{
    rvvm_intc_t* intc = rvvm_get_intc(machine);
    rvvm_addr_t  addr = rvvm_mmio_zone_auto(machine, SPI_SIFIVE_ADDR_DEFAULT, SPI_SIFIVE_MMIO_SIZE);
    return spi_sifive_init(machine, addr, intc, rvvm_alloc_irq(intc));
}

PUBLIC spi_bus_t* spi_sifive_get_bus(rvvm_mmio_dev_t* mmio)
{
    if (!mmio || mmio->type != &spi_sifive_dev_type) return NULL;
    spi_sifive_dev_t* spi = mmio->data;
    return spi->bus;
}

POP_OPTIMIZATION_SIZE
