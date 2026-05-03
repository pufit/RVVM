/*
can-echo.h - Test responder for CAN bus end-to-end smoke testing
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_CAN_ECHO_H
#define RVVM_CAN_ECHO_H

#include "rvvmlib.h"
#include "can-bus.h"

/*
 * Tiny test slave: echoes any standard-format (11-bit ID), non-RTR,
 * non-error frame whose ID is below `id_offset` back onto the bus with
 * its `can_id` increased by `id_offset` and the same payload. Frames
 * already in the [id_offset, ...) range are not echoed, so a single
 * round-trip terminates without ping-pong.
 *
 * Pairs with an MCP2515 in NORMAL mode to give a guest a real RX path:
 *   guest$ cansend can0 050#deadbeef
 *   guest$ candump can0
 *      can0  150   [4]  DE AD BE EF
 */

//! \brief  Attach an echo responder to a CAN bus.
//! \param  bus        Bus to attach to.
//! \param  id_offset  ID bump for echoed frames (0 → 0x100).
PUBLIC void can_echo_attach(can_bus_t* bus, uint32_t id_offset);

#endif
