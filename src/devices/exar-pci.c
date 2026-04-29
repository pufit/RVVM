/*
exar-pci.c - Exar XR17V35x-family PCIe combo serial card
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "exar-pci.h"
#include "atomics.h"
#include "compiler.h"
#include "spinlock.h"
#include "uart16550_core.h"
#include "utils.h"

#include <string.h>

PUSH_OPTIMIZATION_SIZE

#define EXAR_VENDOR_ID    0x13a8 // Exar Corporation

// Per-port window stride. The kernel computes `offset = idx * 0x400`
// and uses port[i].membase = bar0 + offset (8250_exar.c:1343, :924).
#define EXAR_PORT_STRIDE  0x400

// Exar-specific register offsets within each per-port 0x400 window.
// Names match Linux's UART_EXAR_* and UART_XR_* macros.
#define EXAR_REG_FCTR     0x08 // Feature Control
#define EXAR_REG_EFR      0x09 // Enhanced Feature Register (XR17V35x)
#define EXAR_REG_TXTRG    0x0A // TX FIFO trigger level (write-only)
#define EXAR_REG_RXTRG    0x0B // RX FIFO trigger level (write-only)
#define EXAR_REG_DLD      0x02 // Fractional divisor (DLAB=1 banks this over IIR/FCR)

// Chip-global registers projected into every per-port window. The
// kernel uses port 0's membase to reach the "globals" at +0x80..+0x9A,
// and additionally reads INT0 at port 8's window for >8-port chips
// (8250_exar.c:1399). We respond identically from any port window so
// the slave-bridge sweep just works.
#define EXAR_REG_INT0     0x80 // Misc IRQ ack (read-only, returns 0)
#define EXAR_REG_8XMODE   0x88 // 8x sample-rate select (per-port)
#define EXAR_REG_SLEEP    0x8B // Sleep mode (per-port)
#define EXAR_REG_DVID     0x8D // Device identification (per-port, read-only)
#define EXAR_REG_REGB     0x8E // EEPROM bit-bang (sink — no EEPROM)
#define EXAR_REG_MPIO_LO  0x8F // 0x8F..0x94: MPIO[7:0] regs
#define EXAR_REG_MPIO_HI  0x95 // 0x95..0x9A: MPIO[15:8] regs
#define EXAR_REG_MPIO_END 0x9B

typedef struct exar_pci_dev exar_pci_dev_t;

typedef struct {
    exar_pci_dev_t*  parent;
    uint32_t         index;
    uart16550_core_t core;
    spinlock_t       lock;

    // Standard 16550 banked extension: when LCR.DLAB=1, register 0x02
    // exposes DLD (4-bit fractional divisor low nibble + status bits).
    // Read-modify-write via xr17v35x_set_divisor (8250_exar.c:454).
    uint8_t          dld;

    // Exar register state. Most are write-sinks the kernel programs
    // during port setup; we store them so reads round-trip cleanly.
    uint8_t          fctr;     // 0x08
    uint8_t          efr;      // 0x09
    uint8_t          txtrg;    // 0x0A (kernel writes 128, never reads)
    uint8_t          rxtrg;    // 0x0B
    uint8_t          mode_8x;  // 0x88
    uint8_t          sleep;    // 0x8B
    uint8_t          regb;     // 0x8E
    uint8_t          mpio[EXAR_REG_MPIO_END - EXAR_REG_MPIO_LO];

    // Per-port DVID. For homogeneous chips (V352/V354/V358) every port
    // returns the same value. For heterogeneous combos (V4358 = V354 +
    // V358 expansion, V8358 = V358 + V358 expansion) ports on each
    // half identify their own die: ports 0-3 of V4358 see 0x84, ports
    // 4-11 see 0x88. The default_setup matcher in 8250_exar.c:520
    // accepts {0x82, 0x84, 0x88}; anything else falls back to the
    // older PORT_XR17D15X type and skips fractional-divisor wiring.
    uint8_t          dvid;
} exar_port_t;

struct exar_pci_dev {
    pci_func_t*    pci_func;
    exar_model_t   model;
    uint32_t       n_ports;
    exar_port_t*   ports;

    // INTx fan-in. Each port sets/clears bit `index`. The PCI line
    // is wired-OR: rising edge of (irq_active != 0) raises, falling
    // edge lowers. Atomic compare-and-swap over the bitmask gives
    // correct edge detection without the lock; the per-port lock
    // already serialises concurrent core-state changes.
    uint32_t       irq_active;  // bitmask of asserted ports
};

static const struct {
    uint16_t device_id;
    uint8_t  dvid_main;
    uint8_t  dvid_slave;
    uint32_t main_ports;
} exar_chip_table[] = {
    [EXAR_MODEL_XR17V352]  = {0x0352, 0x82, 0x82, 2},
    [EXAR_MODEL_XR17V354]  = {0x0354, 0x84, 0x84, 4},
    [EXAR_MODEL_XR17V358]  = {0x0358, 0x88, 0x88, 8},
    [EXAR_MODEL_XR17V4358] = {0x4358, 0x84, 0x88, 4},
    [EXAR_MODEL_XR17V8358] = {0x8358, 0x88, 0x88, 8},
};

// Wire-OR the shared INTx line. Atomically flip bit `port_idx`; raise
// the host IRQ on the 0→nonzero edge, lower on the nonzero→0 edge.
static void exar_irq_fn(void* ctx, bool level)
{
    exar_port_t*    port = ctx;
    exar_pci_dev_t* dev  = port->parent;
    uint32_t        bit  = 1u << port->index;
    uint32_t        prev;

    if (level) {
        prev = atomic_or_uint32(&dev->irq_active, bit);
        if (prev == 0) {
            pci_raise_irq(dev->pci_func, 0);
        }
    } else {
        prev = atomic_and_uint32(&dev->irq_active, ~bit);
        if (prev == bit) {
            pci_lower_irq(dev->pci_func, 0);
        }
    }
}

static uint8_t exar_port_read(exar_port_t* port, uint32_t reg)
{
    // 16550 register window. Offset 0x02 with DLAB=1 banks DLD instead
    // of IIR/FCR; keep that logic here so the core stays oblivious to
    // Exar fractional-divisor extensions.
    if (reg < 8) {
        if (reg == EXAR_REG_DLD && (atomic_load_uint32_relax(&port->core.lcr) & 0x80)) {
            return port->dld;
        }
        return uart16550_core_read(&port->core, reg);
    }

    switch (reg) {
        case EXAR_REG_FCTR:    return port->fctr;
        case EXAR_REG_EFR:     return port->efr;
        case EXAR_REG_TXTRG:   return port->txtrg;
        case EXAR_REG_RXTRG:   return port->rxtrg;
        case EXAR_REG_INT0:    return 0; // misc-irq sweep, no-op (8250_exar.c:1395)
        case EXAR_REG_8XMODE:  return port->mode_8x;
        case EXAR_REG_SLEEP:   return port->sleep;
        case EXAR_REG_DVID:    return port->dvid;
        case EXAR_REG_REGB:    return port->regb;
        default:
            if (reg >= EXAR_REG_MPIO_LO && reg < EXAR_REG_MPIO_END) {
                return port->mpio[reg - EXAR_REG_MPIO_LO];
            }
            return 0;
    }
}

static void exar_port_write(exar_port_t* port, uint32_t reg, uint8_t val)
{
    if (reg < 8) {
        if (reg == EXAR_REG_DLD && (atomic_load_uint32_relax(&port->core.lcr) & 0x80)) {
            port->dld = val;
            return;
        }
        uart16550_core_write(&port->core, reg, val);
        return;
    }

    switch (reg) {
        case EXAR_REG_FCTR:    port->fctr    = val; break;
        case EXAR_REG_EFR:     port->efr     = val; break;
        case EXAR_REG_TXTRG:   port->txtrg   = val; break;
        case EXAR_REG_RXTRG:   port->rxtrg   = val; break;
        case EXAR_REG_8XMODE:  port->mode_8x = val; break;
        case EXAR_REG_SLEEP:   port->sleep   = val; break;
        case EXAR_REG_REGB:    port->regb    = val; break;
        case EXAR_REG_INT0:    /* read-clear; ignore writes */ break;
        case EXAR_REG_DVID:    /* read-only */ break;
        default:
            if (reg >= EXAR_REG_MPIO_LO && reg < EXAR_REG_MPIO_END) {
                port->mpio[reg - EXAR_REG_MPIO_LO] = val;
            }
            break;
    }
}

static bool exar_mmio_read(rvvm_mmio_dev_t* mmio, void* data, size_t offset, uint8_t size)
{
    exar_pci_dev_t* dev      = mmio->data;
    uint32_t        port_idx = offset / EXAR_PORT_STRIDE;
    uint32_t        reg      = offset & (EXAR_PORT_STRIDE - 1);

    UNUSED(size);
    if (port_idx >= dev->n_ports) {
        *(uint8_t*)data = 0;
        return true;
    }
    *(uint8_t*)data = exar_port_read(&dev->ports[port_idx], reg);
    return true;
}

static bool exar_mmio_write(rvvm_mmio_dev_t* mmio, void* data, size_t offset, uint8_t size)
{
    exar_pci_dev_t* dev      = mmio->data;
    uint32_t        port_idx = offset / EXAR_PORT_STRIDE;
    uint32_t        reg      = offset & (EXAR_PORT_STRIDE - 1);

    UNUSED(size);
    if (port_idx < dev->n_ports) {
        exar_port_write(&dev->ports[port_idx], reg, *(uint8_t*)data);
    }
    return true;
}

static void exar_update(rvvm_mmio_dev_t* mmio)
{
    exar_pci_dev_t* dev = mmio->data;
    for (uint32_t i = 0; i < dev->n_ports; ++i) {
        uart16550_core_update(&dev->ports[i].core);
    }
}

static void exar_remove(rvvm_mmio_dev_t* mmio)
{
    exar_pci_dev_t* dev = mmio->data;
    for (uint32_t i = 0; i < dev->n_ports; ++i) {
        uart16550_core_cleanup(&dev->ports[i].core);
    }
    free(dev->ports);
    free(dev);
}

static const rvvm_mmio_type_t exar_dev_type = {
    .name   = "exar-xr17v35x",
    .update = exar_update,
    .remove = exar_remove,
};

PUBLIC pci_dev_t* exar_pci_init(pci_bus_t* bus, exar_model_t model, chardev_t** chardevs)
{
    if (model != EXAR_MODEL_XR17V352 && model != EXAR_MODEL_XR17V354 //
        && model != EXAR_MODEL_XR17V358 && model != EXAR_MODEL_XR17V4358 && model != EXAR_MODEL_XR17V8358) {
        return NULL;
    }

    exar_pci_dev_t* dev   = safe_new_obj(exar_pci_dev_t);
    dev->model            = model;
    dev->n_ports          = (uint32_t)model;
    dev->ports            = safe_new_arr(exar_port_t, dev->n_ports);

    uint8_t  dvid_main  = exar_chip_table[model].dvid_main;
    uint8_t  dvid_slave = exar_chip_table[model].dvid_slave;
    uint32_t main_ports = exar_chip_table[model].main_ports;

    for (uint32_t i = 0; i < dev->n_ports; ++i) {
        exar_port_t* port = &dev->ports[i];
        port->parent      = dev;
        port->index       = i;
        port->dvid        = (i < main_ports) ? dvid_main : dvid_slave;
        uart16550_core_init(&port->core, chardevs ? chardevs[i] : NULL, exar_irq_fn, port);
    }

    pci_func_desc_t desc = {
        .vendor_id  = EXAR_VENDOR_ID,
        .device_id  = exar_chip_table[model].device_id,
        // Class 0x0700 = Simple Communications / Generic XT-compatible
        // serial. prog_if 0x02 = 16550-compatible, which is what the
        // 8250 layer treats as the lowest-common-denominator type.
        .class_code = 0x0700,
        .prog_if    = 0x02,
        .irq_pin    = PCI_IRQ_PIN_INTA,
        .bar[0] = {
            .size        = dev->n_ports * EXAR_PORT_STRIDE,
            .min_op_size = 1,
            .max_op_size = 1,
            .read        = exar_mmio_read,
            .write       = exar_mmio_write,
            .data        = dev,
            .type        = &exar_dev_type,
        },
    };

    pci_dev_t* pci_dev = pci_attach_func(bus, &desc);
    if (pci_dev == NULL) {
        for (uint32_t i = 0; i < dev->n_ports; ++i) {
            uart16550_core_cleanup(&dev->ports[i].core);
        }
        free(dev->ports);
        free(dev);
        return NULL;
    }
    dev->pci_func = pci_get_device_func(pci_dev, 0);

    // Bootstrap notify, same dance as the JNI bridge's bridge_init
    // (rvvm_jni.c:921). Without firing one notify per port at attach
    // time, the core's cached `flags` field stays at zero-init until
    // the kernel does a polled THR write. The kernel's IRQ-driven 8250
    // path enables THRI in IER and waits for a TX IRQ that never comes
    // because update_irq sees flags=0. ttyS0 hides this via printk's
    // polled console writes; ttyS1+ deadlock at first user-space TX.
    for (uint32_t i = 0; i < dev->n_ports; ++i) {
        exar_port_t* port = &dev->ports[i];
        if (port->core.chardev) {
            uint32_t flags = chardev_poll(port->core.chardev);
            chardev_notify(port->core.chardev, flags);
        }
    }

    return pci_dev;
}

PUBLIC pci_dev_t* exar_pci_init_auto(rvvm_machine_t* machine, exar_model_t model, chardev_t** chardevs)
{
    return exar_pci_init(rvvm_get_pci_bus(machine), model, chardevs);
}

POP_OPTIMIZATION_SIZE
