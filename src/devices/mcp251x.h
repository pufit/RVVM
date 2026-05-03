/*
mcp251x.h - Microchip MCP2515/MCP25625 CAN controller (SPI slave)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_MCP251X_H
#define RVVM_MCP251X_H

#include "rvvmlib.h"
#include "spi-bus.h"
#include "can-bus.h"

/*
 * Emulates the SPI side of a Microchip MCP2515 / MCP25625 CAN
 * controller (DS21801G §11–§12). Linux's drivers/net/can/spi/mcp251x.c
 * binds against `microchip,mcp2515` and only exercises a handful of
 * SPI opcodes (READ / WRITE / BIT_MODIFY / LOAD_TXB / RTS / READ_RXB /
 * RESET); we model exactly those plus the bare minimum of the register
 * file the driver inspects (CANCTRL/CANSTAT mode bits, CANINTE/CANINTF
 * IRQ gate, RXB/TXB buffers).
 *
 * The chip's INT line is wired to the supplied (intc, irq) pair. A
 * frame issued via RTS while CANCTRL.REQOP = LOOPBACK is delivered
 * immediately into RXB0 (or RXB1 if BUKT is set and RXB0 is full),
 * which is the configuration Linux uses for self-tests. NORMAL-mode
 * transmits are accepted (TXnIF set) but go to /dev/null — there is
 * no shared bus between multiple emulated chips yet.
 *
 * Bit timing, error counters, acceptance filters/masks, and the
 * BFPCTRL / TXRTSCTRL GPIO functions are stored verbatim and have no
 * semantic effect.
 */

//! \brief  Attach an MCP2515 CAN controller to a SPI bus.
//! \param  spi_bus  Controller's spi_bus_t (e.g. from spi_sifive_get_bus)
//! \param  cs_id    Chip-select index, or SPI_AUTO_CS to pick lowest free
//! \param  intc     Interrupt controller for the chip's INT line
//! \param  irq      IRQ line on `intc` driven by INT
//! \param  can_bus  Optional CAN segment for inter-chip frames; pass NULL
//!                  for loopback-only operation (NORMAL-mode TX silently
//!                  drops, no RX path).
//! \return Assigned CS, or SPI_AUTO_CS on failure.
PUBLIC uint16_t mcp251x_attach(spi_bus_t* spi_bus, uint16_t cs_id,
                               rvvm_intc_t* intc, rvvm_irq_t irq,
                               can_bus_t* can_bus);

#endif
