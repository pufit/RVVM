/*
unibus-pdp11.c - Standard PDP-11 (1st Edition UNIX) Unibus device set

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

Convenience attach mirroring pci_bus_init_auto(): builds a Unibus with the
device set the ported 1st Edition UNIX kernel expects -- KW11-L line clock,
KL11/DL11 console on a stdio terminal, and the RF11/RS11 drum (root+swap).

See docs/unibus.md for the per-device address/vector/BR-level table.
*/

#include <rvvm/rvvm_board.h>

#include "chardev.h"
#include "unibus-devices.h"
#include "unibus.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

PUBLIC unibus_t* unibus_pdp11_init_auto(rvvm_machine_t* machine)
{
    unibus_t* bus = unibus_init_auto(machine);
    if (!bus) {
        return NULL;
    }

    // KW11-L line clock at the AC line frequency (60 Hz default).
    rvvm_kw11l_init(bus, 0);

    // KL11/DL11 console wired to a stdio terminal, like ns16550a's
    // init_term_auto().
    rvvm_kl11_init(bus, chardev_term_create());

    // RF11/RS11 drum: 1024 blocks (1st Edition UNIX root+swap).
    rvvm_rf11_init(bus, 1024);

    // RK11/RK05 cartridge disk (empty pack; image via rvvm_rk11_load).
    rvvm_rk11_init(bus, 0);

    // PC11 paper-tape reader/punch (no tape; mount via rvvm_pc11_load_reader).
    rvvm_pc11_init(bus);

    return bus;
}

POP_OPTIMIZATION_SIZE
