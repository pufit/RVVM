/*
spi-bus.h - SPI bus abstraction (slave registration + dispatch)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_SPI_BUS_H
#define RVVM_SPI_BUS_H

#include "rvvmlib.h"

/*
 * SPI master ↔ slave abstraction. The controller (e.g. spi-sifive) owns
 * the spi_bus_t and routes transfers to slaves indexed by chip-select
 * line. Slaves are pure software peripherals (NOR flash, sensors, ...)
 * that respond synchronously to byte transfers — same byte-in / byte-out
 * model as a real shift-register SPI link, but with no clock or timing
 * to model since transfers are atomic from the guest's perspective.
 *
 * Mirrors the i2c-oc.h `i2c_dev_t` style.
 */

typedef struct spi_bus_t spi_bus_t;

#define SPI_AUTO_CS 0xFFFFu // Auto-pick lowest free CS line

typedef struct {
    // CS line this slave responds to. SPI_AUTO_CS picks the lowest free
    // slot at attach time; the assigned value is returned.
    uint16_t cs_id;
    // Slave-private state pointer
    void*    data;

    // Chip-select assert/deassert. `asserted=true` corresponds to the
    // wire-level active-low CS being driven low. Slaves typically reset
    // their command parser on each falling edge.
    void    (*select)(void* dev, bool asserted);

    // Full-duplex byte exchange. Returns the byte the slave shifts out
    // on MISO during the same SCK frame the master shifts `tx` on MOSI.
    // CS is guaranteed asserted when this is called.
    uint8_t (*transfer)(void* dev, uint8_t tx);

    // Optional cleanup hook called when the bus is destroyed.
    void    (*remove)(void* dev);
} spi_dev_t;

// Maximum chip-select ID the bus tracks. The SiFive controller advertises
// up to 32 CS lines; one entry per line is cheap enough to allocate flat.
#define SPI_BUS_MAX_CS 32

// Construct a bare spi_bus_t. Caller (the controller) keeps ownership and
// passes the bus pointer back into the dispatch helpers below.
PUBLIC spi_bus_t* spi_bus_create(void);

// Tear down the bus, calling each attached slave's `remove` hook.
PUBLIC void       spi_bus_free(spi_bus_t* bus);

// Attach a slave. `dev_desc` is copied by value; if `cs_id == SPI_AUTO_CS`
// the lowest free CS slot is picked. Returns the assigned CS, or
// SPI_AUTO_CS on failure (slot occupied / out of range / out of slots).
PUBLIC uint16_t   spi_attach_dev(spi_bus_t* bus, const spi_dev_t* dev_desc);

// How many CS lines the bus is willing to reflect back to the host. The
// SiFive controller probes its CS count by writing 0xFFFFFFFF to CSDEF
// and reading back; we reflect (1 << count) - 1 worth of bits.
PUBLIC uint32_t   spi_bus_cs_count(spi_bus_t* bus);

// Dispatch hooks called by the controller from its register write path.
// Both are no-ops if no slave is attached at `cs_id` (bus stays usable
// but bytes shifted into MOSI go to a floating MISO that returns 0xFF —
// matches an unconnected SPI line).
PUBLIC void       spi_bus_select(spi_bus_t* bus, uint16_t cs_id, bool asserted);
PUBLIC uint8_t    spi_bus_transfer(spi_bus_t* bus, uint16_t cs_id, uint8_t tx);

#endif
