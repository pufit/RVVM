/*
can-bus.c - Minimal CAN bus fan-out abstraction
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "can-bus.h"
#include "compiler.h"
#include "spinlock.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

#define CAN_BUS_MAX_NODES 16

struct can_bus_t {
    spinlock_t lock;                       // protects nodes/count on attach
    can_dev_t  nodes[CAN_BUS_MAX_NODES];
    size_t     count;
};

PUBLIC can_bus_t* can_bus_create(void)
{
    return safe_new_obj(struct can_bus_t);
}

PUBLIC void can_bus_free(can_bus_t* bus)
{
    if (!bus) return;
    for (size_t i = 0; i < bus->count; i++) {
        if (bus->nodes[i].remove) bus->nodes[i].remove(bus->nodes[i].data);
    }
    free(bus);
}

PUBLIC void can_bus_attach(can_bus_t* bus, const can_dev_t* desc)
{
    if (!bus || !desc || !desc->rx) return;
    spin_lock(&bus->lock);
    if (bus->count < CAN_BUS_MAX_NODES) {
        bus->nodes[bus->count++] = *desc;
    }
    spin_unlock(&bus->lock);
}

PUBLIC void can_bus_broadcast(can_bus_t* bus, void* from, const can_frame_t* frame)
{
    if (!bus || !frame) return;

    // Snapshot the count under lock then iterate without holding it. The
    // node table is append-only, so entries [0..n) are stable for the
    // duration of this call. Iterating without the lock matters because
    // an rx callback can re-enter can_bus_broadcast (echo responders),
    // and slaves take their own internal locks inside rx.
    spin_lock(&bus->lock);
    size_t n = bus->count;
    spin_unlock(&bus->lock);

    for (size_t i = 0; i < n; i++) {
        if (bus->nodes[i].data == from) continue;
        bus->nodes[i].rx(bus->nodes[i].data, frame);
    }
}

POP_OPTIMIZATION_SIZE
