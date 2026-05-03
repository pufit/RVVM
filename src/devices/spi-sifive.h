/*
spi-sifive.h - SiFive FU540/FU740 SPI controller (FIFO mode)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_SPI_SIFIVE_H
#define RVVM_SPI_SIFIVE_H

#include "rvvmlib.h"
#include "spi-bus.h"

/*
 * Emulates the SiFive "spi0" controller as documented in the FU540-C000
 * manual (https://static.dev.sifive.com/FU540-C000-v1.0.pdf §15) and
 * matched by Linux's drivers/spi/spi-sifive.c (compatible
 * "sifive,spi0"). FIFO mode only — the memory-mapped XIP path
 * (FCTRL/FFMT plus a second BAR) is stubbed: writes are accepted, the
 * second region is not mapped, and Linux falls back to FIFO transfers.
 *
 * The controller owns an spi_bus_t internally; attach slaves to it
 * through `spi_sifive_get_bus()` after init.
 */

#define SPI_SIFIVE_ADDR_DEFAULT 0x10040000
#define SPI_SIFIVE_FIFO_DEPTH   8

PUBLIC rvvm_mmio_dev_t* spi_sifive_init(rvvm_machine_t* machine, rvvm_addr_t addr,
                                        rvvm_intc_t* intc, rvvm_irq_t irq);

PUBLIC rvvm_mmio_dev_t* spi_sifive_init_auto(rvvm_machine_t* machine);

// Bus handle for slave attachment. Returned bus is owned by the
// controller; lifetime tied to the MMIO device.
PUBLIC spi_bus_t* spi_sifive_get_bus(rvvm_mmio_dev_t* mmio);

#endif
