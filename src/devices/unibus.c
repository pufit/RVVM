/*
unibus.c - PDP-11 Unibus, modeled on RVVM's PCI bus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

Common-bus abstraction for PDP-11 peripherals, mirroring pci-bus.c. See
unibus.h for the full API and the interrupt / IAK / priority / NPR contract.

All Handbook references are to the "PDP-11 Architecture Handbook" (Digital
Equipment Corporation, 1983), by *printed* page number (PDF index +9):
  - Ch.8 "Traps and Interrupts" pp.215-216 (vector = 2 words: new PC, new PS)
  - Ch.9 "Mapping to Memory and Busses" p.258 fig.9-37 (I/O page at 760000)
  - Ch.10 "PDP-11 Bus Structures" pp.266-271 (Unibus, BRx, NPR, arbitration)
*/

#include "unibus.h"

#include <rvvm/rvvm_fdt.h>
#include <rvvm/rvvm_irq.h>

#include "mem_ops.h"
#include "spinlock.h"
#include "utils.h"
#include "vector.h"

PUSH_OPTIMIZATION_SIZE

struct rvvm_unibus_dev {
    unibus_t* bus;

    const char* name;
    rvvm_addr_t io_off; // Offset within the I/O page
    size_t      size;

    void (*read)(unibus_dev_t* dev, uint16_t* val, size_t off);
    void (*write)(unibus_dev_t* dev, uint16_t val, size_t off);
    void (*poll)(unibus_dev_t* dev);
    void (*cleanup)(unibus_dev_t* dev);

    uint32_t br_level; // BR4..BR7 (Handbook p.268)
    uint16_t vector;   // Interrupt vector (Handbook p.216)

    // Atomic: nonzero while this device is asserting its BR line.
    uint32_t irq_pending;

    void* data;
    bool   shared_data; // If set, data is owned by another endpoint
};

struct rvvm_unibus {
    rvvm_machine_t* machine;
    rvvm_intc_t*    intc;
    rvvm_irq_t      irq; // The single shared RISC-V external IRQ line
    rvvm_addr_t     mem_base;

    // IAK grant latch: a write to UNIBUS_IAK_PRI performs the grant and
    // latches the resulting vector here; the following UNIBUS_IAK_VEC read
    // returns it. See the contract in unibus.h.
    uint32_t iak_grant;

    vector_t(unibus_dev_t*) dev;
    spinlock_t lock;
};

/*
 * Re-evaluate the shared RISC-V external line: assert it while any device
 * is asserting a BR line, deassert it when none are.
 *
 * The real Unibus has four separate BRx lines arbitrated in hardware; the
 * RISC-V hart has one external line, so we OR all pending BR requests onto
 * it. Per-level arbitration is resolved later, in the IAK read.
 */
static void unibus_update_irq_line(unibus_t* bus)
{
    bool pending = false;
    scoped_spin_lock (&bus->lock) {
        vector_foreach (bus->dev, i) {
            unibus_dev_t* dev = vector_at(bus->dev, i);
            if (dev && atomic_load_uint32_relax(&dev->irq_pending)) {
                pending = true;
                break;
            }
        }
    }
    rvvm_irq_set(bus->intc, bus->irq, pending);
}

/*
 * Interrupt acknowledge (IAK), the software stand-in for BGx grant + the
 * BUS INTR vector pass (Handbook p.268-269).
 *
 * Given the processor priority P (PSW bits 5:7), grant the highest-BR-level
 * pending device strictly above P, clear its request (the grant), and return
 * its vector. Ties at the same level are broken by attach order, modeling
 * the daisy-chain "nearest device wins" rule (Handbook p.267). Returns 0 if
 * nothing outranks P.
 */
static uint16_t unibus_iak(unibus_t* bus, uint32_t prio)
{
    unibus_dev_t* winner = NULL;
    scoped_spin_lock (&bus->lock) {
        vector_foreach (bus->dev, i) {
            unibus_dev_t* dev = vector_at(bus->dev, i);
            if (dev && atomic_load_uint32_relax(&dev->irq_pending) && dev->br_level > prio) {
                // Strictly higher BR level wins; equal level keeps the
                // earlier-attached (electrically nearer) device.
                if (!winner || dev->br_level > winner->br_level) {
                    winner = dev;
                }
            }
        }
        if (winner) {
            // Grant: clear this one request (BGx daisy-chained to it)
            atomic_store_uint32_relax(&winner->irq_pending, 0);
        }
    }

    uint16_t vector = winner ? winner->vector : 0;

    // The line stays asserted while other requests remain pending.
    unibus_update_irq_line(bus);

    return vector;
}

// Find the device owning an I/O-page offset (called under bus->lock)
static unibus_dev_t* unibus_dev_at(unibus_t* bus, rvvm_addr_t io_off, size_t size)
{
    vector_foreach (bus->dev, i) {
        unibus_dev_t* dev = vector_at(bus->dev, i);
        if (dev && io_off >= dev->io_off && (io_off + size) <= (dev->io_off + dev->size)) {
            return dev;
        }
    }
    return NULL;
}

static bool unibus_mmio_read(rvvm_mmio_dev_t* mmio, void* data, size_t offset, uint8_t size)
{
    unibus_t* bus = mmio->data;
    uint16_t  val = 0;

    if (offset == UNIBUS_IAK_VEC) {
        // IAK vector grant read (Handbook p.268-269): return the vector
        // latched by the preceding UNIBUS_IAK_PRI write.
        write_uint16_le(data, atomic_load_uint32_relax(&bus->iak_grant));
        return true;
    }
    if (offset == UNIBUS_IAK_PRI) {
        write_uint16_le(data, 0);
        return true;
    }

    unibus_dev_t* dev = NULL;
    scoped_spin_lock (&bus->lock) {
        dev = unibus_dev_at(bus, offset, size);
    }
    if (dev && dev->read) {
        dev->read(dev, &val, offset - dev->io_off);
    } else {
        // Nonexistent I/O-page locations time out on the real Unibus
        // (Handbook p.272 "BUS ERRORS"); we read as zero for simplicity.
        val = 0;
    }

    if (size == 1) {
        write_uint8(data, (offset & 1) ? (val >> 8) : (val & 0xFF));
    } else {
        write_uint16_le(data, val);
    }
    return true;
}

static bool unibus_mmio_write(rvvm_mmio_dev_t* mmio, void* data, size_t offset, uint8_t size)
{
    unibus_t* bus = mmio->data;

    if (offset == UNIBUS_IAK_PRI) {
        // Processor priority P (PSW 5:7); grant the next vector now so the
        // following UNIBUS_IAK_VEC read returns it (Handbook p.216, p.268).
        atomic_store_uint32_relax(&bus->iak_grant, unibus_iak(bus, read_uint16_le(data) & 0x7));
        return true;
    }
    if (offset == UNIBUS_IAK_VEC) {
        return true;
    }

    unibus_dev_t* dev = NULL;
    scoped_spin_lock (&bus->lock) {
        dev = unibus_dev_at(bus, offset, size);
    }
    if (dev && dev->write) {
        uint16_t val;
        if (size == 1) {
            // Byte write: read-modify-write the 16-bit register
            uint16_t cur = 0;
            if (dev->read) {
                dev->read(dev, &cur, (offset & ~(size_t)1) - dev->io_off);
            }
            if (offset & 1) {
                val = (cur & 0x00FF) | ((uint16_t)read_uint8(data) << 8);
            } else {
                val = (cur & 0xFF00) | read_uint8(data);
            }
            dev->write(dev, val, (offset & ~(size_t)1) - dev->io_off);
        } else {
            val = read_uint16_le(data);
            dev->write(dev, val, offset - dev->io_off);
        }
    }
    return true;
}

/*
 * Periodic update from the RVVM event thread. Drives every attached device's
 * optional poll hook -- this is how time-driven devices (KW11-L line clock)
 * advance and how chardev-backed devices coalesce RX/TX polling.
 */
static void unibus_mmio_update(rvvm_mmio_dev_t* mmio)
{
    unibus_t* bus = mmio->data;
    // Devices are attached at setup and only freed at machine teardown (no
    // hot-unplug), so the device set is stable here. poll() may raise/lower
    // IRQs, which take bus->lock, so we must not hold it across the calls.
    size_t count = vector_size(bus->dev);
    for (size_t i = 0; i < count; ++i) {
        unibus_dev_t* dev = vector_at(bus->dev, i);
        if (dev && dev->poll) {
            dev->poll(dev);
        }
    }
}

static void unibus_mmio_remove(rvvm_mmio_dev_t* mmio)
{
    unibus_t* bus = mmio->data;
    vector_foreach (bus->dev, i) {
        unibus_dev_t* dev = vector_at(bus->dev, i);
        if (dev) {
            // Per-device cleanup (release backing store, chardev, etc),
            // then free private data unless it is shared with another
            // endpoint (the owning endpoint frees it). See unibus_dev_desc_t.
            if (dev->cleanup) {
                dev->cleanup(dev);
            }
            if (dev->data && !dev->shared_data) {
                free(dev->data);
            }
            free(dev);
        }
    }
    vector_free(bus->dev);
    if (bus->intc) {
        rvvm_irq_dealloc(bus->intc, bus->irq);
    }
    free(bus);
}

static const rvvm_mmio_type_t unibus_mmio_type = {
    .name   = "unibus",
    .remove = unibus_mmio_remove,
    .update = unibus_mmio_update,
};

PUBLIC unibus_t* unibus_init(rvvm_machine_t* machine, rvvm_addr_t mem_base)
{
    unibus_t* bus = safe_new_obj(unibus_t);
    bus->machine  = machine;
    bus->mem_base = mem_base;
    bus->intc     = rvvm_get_intc(machine);

    if (!bus->intc) {
        rvvm_error("Unibus requires an interrupt controller!");
        free(bus);
        return NULL;
    }
    bus->irq = rvvm_alloc_irq(bus->intc);

    rvvm_mmio_dev_t io_page = {
        // I/O page at the top of the window (Handbook p.258 fig.9-37,
        // p.266 "top 8 Kbytes ... for I/O").
        .addr        = mem_base + UNIBUS_IO_PAGE_BASE,
        .size        = UNIBUS_IO_PAGE_SIZE,
        .data        = bus,
        .type        = &unibus_mmio_type,
        .read        = unibus_mmio_read,
        .write       = unibus_mmio_write,
        // PDP-11 registers are 16-bit words; allow byte access too, the
        // handlers promote bytes to RMW (Handbook p.269 DATO/DATOB).
        .min_op_size = 1,
        .max_op_size = 2,
    };

    if (!rvvm_attach_mmio(machine, &io_page)) {
        rvvm_error("Failed to attach Unibus I/O page!");
        rvvm_irq_dealloc(bus->intc, bus->irq);
        free(bus);
        return NULL;
    }

#if defined(USE_FDT)
    rvvm_fdt_node_t* soc = rvvm_get_fdt_soc(machine);
    if (soc) {
        rvvm_fdt_node_t* fdt = rvvm_fdt_init_reg("unibus", io_page.addr);
        rvvm_fdt_prop_set_reg(fdt, "reg", io_page.addr, io_page.size);
        rvvm_fdt_prop_set_str(fdt, "compatible", "dec,unibus");
        rvvm_fdt_attach(soc, fdt);
    }
#endif

    return bus;
}

PUBLIC unibus_t* unibus_init_auto(rvvm_machine_t* machine)
{
    // Natural MB == 0: PDP-11 address equals host physical address.
    return unibus_init(machine, 0x0);
}

PUBLIC unibus_dev_t* unibus_attach(unibus_t* bus, const unibus_dev_desc_t* desc)
{
    if (!bus || !desc) {
        return NULL;
    }
    if (desc->io_addr < UNIBUS_IO_PAGE_BASE || (desc->io_addr + desc->size) > UNIBUS_WINDOW_SIZE) {
        rvvm_error("Unibus device %s outside the I/O page!", desc->name ? desc->name : "?");
        if (!desc->shared_data) {
            free(desc->data);
        }
        return NULL;
    }
    if (desc->size > UNIBUS_DEV_MAX_SIZE) {
        rvvm_error("Unibus device %s register block too large!", desc->name ? desc->name : "?");
        if (!desc->shared_data) {
            free(desc->data);
        }
        return NULL;
    }

    unibus_dev_t* dev = safe_new_obj(unibus_dev_t);
    dev->bus          = bus;
    dev->name         = desc->name;
    dev->io_off       = desc->io_addr - UNIBUS_IO_PAGE_BASE;
    dev->size         = desc->size;
    dev->read         = desc->read;
    dev->write        = desc->write;
    dev->poll         = desc->poll;
    dev->cleanup      = desc->cleanup;
    dev->br_level     = desc->br_level;
    dev->vector       = desc->vector;
    dev->data         = desc->data;
    dev->shared_data  = desc->shared_data;

    scoped_spin_lock (&bus->lock) {
        // Reject overlap with an already-attached device
        if (unibus_dev_at(bus, dev->io_off, dev->size)) {
            rvvm_error("Unibus address conflict for device %s!", desc->name ? desc->name : "?");
            if (dev->data && !dev->shared_data) {
                free(dev->data);
            }
            free(dev);
            dev = NULL;
        } else {
            vector_push_back(bus->dev, dev);
        }
    }

    return dev;
}

PUBLIC rvvm_machine_t* unibus_machine(unibus_t* bus)
{
    return bus ? bus->machine : NULL;
}

PUBLIC rvvm_addr_t unibus_mem_base(unibus_t* bus)
{
    return bus ? bus->mem_base : 0;
}

PUBLIC void* unibus_dev_data(unibus_dev_t* dev)
{
    return dev ? dev->data : NULL;
}

PUBLIC unibus_t* unibus_dev_bus(unibus_dev_t* dev)
{
    return dev ? dev->bus : NULL;
}

PUBLIC uint16_t unibus_ack(unibus_t* bus, uint32_t prio)
{
    // Programmatic IAK grant: identical to the MMIO PRI-write/VEC-read pair.
    return bus ? unibus_iak(bus, prio & 0x7) : 0;
}

PUBLIC void unibus_raise_irq(unibus_dev_t* dev)
{
    if (likely(dev)) {
        // Assert BRx (Handbook p.268). Idempotent while already asserted.
        atomic_store_uint32_relax(&dev->irq_pending, 1);
        unibus_update_irq_line(dev->bus);
    }
}

PUBLIC void unibus_lower_irq(unibus_dev_t* dev)
{
    if (likely(dev)) {
        atomic_store_uint32_relax(&dev->irq_pending, 0);
        unibus_update_irq_line(dev->bus);
    }
}

PUBLIC bool unibus_dma_read(unibus_dev_t* dev, rvvm_addr_t pdp_addr, void* buf, size_t len)
{
    // NPR DMA: window RAM at (MB + pdp_addr) -> host buffer (Handbook p.268)
    if (likely(dev)) {
        return rvvm_read_ram(dev->bus->machine, buf, dev->bus->mem_base + pdp_addr, len);
    }
    return false;
}

PUBLIC bool unibus_dma_write(unibus_dev_t* dev, rvvm_addr_t pdp_addr, const void* buf, size_t len)
{
    // NPR DMA: host buffer -> window RAM at (MB + pdp_addr) (Handbook p.268)
    if (likely(dev)) {
        return rvvm_write_ram(dev->bus->machine, dev->bus->mem_base + pdp_addr, buf, len);
    }
    return false;
}

PUBLIC void* unibus_dma_ptr(unibus_dev_t* dev, rvvm_addr_t pdp_addr, size_t len)
{
    // Zero-copy NPR pointer into window RAM (Handbook p.268)
    if (likely(dev)) {
        return rvvm_get_dma_ptr(dev->bus->machine, dev->bus->mem_base + pdp_addr, len);
    }
    return NULL;
}

POP_OPTIMIZATION_SIZE
