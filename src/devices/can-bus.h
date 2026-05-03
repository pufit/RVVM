/*
can-bus.h - Minimal CAN bus fan-out abstraction
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_CAN_BUS_H
#define RVVM_CAN_BUS_H

#include "rvvmlib.h"

/*
 * Software CAN segment. Slaves (controllers, test responders, future
 * host bridges) attach with an rx callback; broadcasting a frame
 * delivers it to every attached slave except the originator — same
 * semantics a real CAN node sees on the wire (its own transmissions
 * are not received via the RX path; TX completion is signalled
 * separately).
 *
 * Mirrors the spi_bus_t / spi_dev_t pattern. No clock or arbitration
 * model — frames are atomic from the guest's POV.
 */

#define CAN_FRAME_MAX_DATA 8

// SocketCAN-compatible flags occupying the high bits of can_id.
// Matches Linux's <linux/can.h> so a future host-side SocketCAN bridge
// can pass frames through verbatim.
#define CAN_EFF_FLAG       0x80000000u  // Extended (29-bit) frame
#define CAN_RTR_FLAG       0x40000000u  // Remote-transmission request
#define CAN_ERR_FLAG       0x20000000u  // Error frame (out-of-band)
#define CAN_EFF_MASK       0x1FFFFFFFu  // 29-bit ID mask
#define CAN_SFF_MASK       0x000007FFu  // 11-bit ID mask

typedef struct {
    uint32_t can_id;                      // ID with high-bit flags above
    uint8_t  dlc;                         // 0..CAN_FRAME_MAX_DATA
    uint8_t  data[CAN_FRAME_MAX_DATA];
} can_frame_t;

typedef struct can_bus_t can_bus_t;

typedef struct {
    void* data;                           // slave-private state pointer
    void  (*rx)(void* dev, const can_frame_t* frame);
    void  (*remove)(void* dev);           // optional cleanup on bus_free
} can_dev_t;

PUBLIC can_bus_t* can_bus_create(void);
PUBLIC void       can_bus_free(can_bus_t* bus);

// Attach a slave. The descriptor is copied by value. No detach API —
// node lifetimes are tied to the bus's lifetime.
PUBLIC void       can_bus_attach(can_bus_t* bus, const can_dev_t* desc);

// Deliver `frame` to every attached slave except `from`. Pass NULL for
// `from` to deliver to all (e.g. host-side injection). The rx callback
// runs synchronously on the caller's thread; rx implementations that
// re-broadcast (echo responders) must be reentrancy-safe and must not
// hold their own locks across can_bus_broadcast.
PUBLIC void       can_bus_broadcast(can_bus_t* bus, void* from,
                                    const can_frame_t* frame);

#endif
