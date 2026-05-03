/*
spi-bus.c - SPI bus abstraction (slave registration + dispatch)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "spi-bus.h"
#include "compiler.h"
#include "utils.h"

#include <string.h>

PUSH_OPTIMIZATION_SIZE

struct spi_bus_t {
    // Flat CS-indexed table. Slot empty when `transfer` is NULL.
    spi_dev_t        slaves[SPI_BUS_MAX_CS];
    uint32_t         cs_present;  // bitmask of populated CS slots
    struct fdt_node* fdt_node;    // controller's FDT node (for child registration)
};

PUBLIC spi_bus_t* spi_bus_create(void)
{
    return safe_new_obj(struct spi_bus_t);
}

PUBLIC void spi_bus_free(spi_bus_t* bus)
{
    if (!bus) return;
    for (size_t i = 0; i < SPI_BUS_MAX_CS; ++i) {
        spi_dev_t* dev = &bus->slaves[i];
        if (dev->remove) {
            dev->remove(dev->data);
        }
    }
    free(bus);
}

PUBLIC uint16_t spi_attach_dev(spi_bus_t* bus, const spi_dev_t* dev_desc)
{
    if (!bus || !dev_desc || !dev_desc->transfer) return SPI_AUTO_CS;

    uint16_t cs = dev_desc->cs_id;
    if (cs == SPI_AUTO_CS) {
        for (uint16_t i = 0; i < SPI_BUS_MAX_CS; ++i) {
            if (bus->slaves[i].transfer == NULL) {
                cs = i;
                break;
            }
        }
        if (cs == SPI_AUTO_CS) return SPI_AUTO_CS;
    } else if (cs >= SPI_BUS_MAX_CS || bus->slaves[cs].transfer != NULL) {
        return SPI_AUTO_CS;
    }

    bus->slaves[cs]       = *dev_desc;
    bus->slaves[cs].cs_id = cs;
    bus->cs_present      |= (1u << cs);
    return cs;
}

PUBLIC uint32_t spi_bus_cs_count(spi_bus_t* bus)
{
    if (!bus || !bus->cs_present) return 0;
    // Highest occupied slot + 1. Sparse holes count toward the line-count
    // probe — the controller drives a full row of CS pins out, so the
    // kernel sees them all even if some are unbound on our side.
    uint32_t mask = bus->cs_present;
    uint32_t hi   = 0;
    while (mask) {
        hi++;
        mask >>= 1;
    }
    return hi;
}

PUBLIC void spi_bus_select(spi_bus_t* bus, uint16_t cs_id, bool asserted)
{
    if (!bus || cs_id >= SPI_BUS_MAX_CS) return;
    spi_dev_t* dev = &bus->slaves[cs_id];
    if (dev->select) {
        dev->select(dev->data, asserted);
    }
}

PUBLIC uint8_t spi_bus_transfer(spi_bus_t* bus, uint16_t cs_id, uint8_t tx)
{
    // Floating MISO returns 0xFF on a real bus when no slave is driving
    // it; matches what an unpopulated CS line would look like.
    if (!bus || cs_id >= SPI_BUS_MAX_CS) return 0xFF;
    spi_dev_t* dev = &bus->slaves[cs_id];
    if (!dev->transfer) return 0xFF;
    return dev->transfer(dev->data, tx);
}

PUBLIC void spi_bus_set_fdt_node(spi_bus_t* bus, struct fdt_node* node)
{
    if (bus) bus->fdt_node = node;
}

PUBLIC struct fdt_node* spi_bus_fdt_node(spi_bus_t* bus)
{
    return bus ? bus->fdt_node : NULL;
}

POP_OPTIMIZATION_SIZE
