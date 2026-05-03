/*
can-echo.c - Test responder for CAN bus end-to-end smoke testing
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "can-echo.h"
#include "compiler.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

typedef struct {
    can_bus_t* bus;
    uint32_t   id_offset;
} can_echo_t;

static void can_echo_rx(void* dev, const can_frame_t* frame)
{
    can_echo_t* echo = dev;

    // Standard-frame, non-RTR, non-error only.
    if (frame->can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) return;
    uint32_t id = frame->can_id & CAN_SFF_MASK;
    // Skip frames already in (or past) the echo range so a single bump
    // round-trips without further ping-pong.
    if (id >= echo->id_offset) return;

    can_frame_t reply = *frame;
    reply.can_id = id + echo->id_offset;
    can_bus_broadcast(echo->bus, echo, &reply);
}

static void can_echo_remove(void* dev)
{
    free(dev);
}

PUBLIC void can_echo_attach(can_bus_t* bus, uint32_t id_offset)
{
    if (!bus) return;
    if (id_offset == 0) id_offset = 0x100;

    can_echo_t* echo = safe_new_obj(can_echo_t);
    echo->bus       = bus;
    echo->id_offset = id_offset;

    can_dev_t desc = {
        .data   = echo,
        .rx     = can_echo_rx,
        .remove = can_echo_remove,
    };
    can_bus_attach(bus, &desc);

    rvvm_info("can-echo: attached (offset=+0x%x)", (unsigned)id_offset);
}

POP_OPTIMIZATION_SIZE
