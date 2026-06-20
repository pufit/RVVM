/*
unibus-devices.h - PDP-11 Unibus peripheral constructors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

Constructors for the concrete PDP-11 devices that sit on top of the Unibus
(unibus.{c,h}), analogous to the device drivers built on top of pci-bus.

Each device wires itself onto the bus with its fixed (BR level, vector) pair.
The addresses, levels and vectors match the ported 1st Edition UNIX kernel's
expectations (unix-v1-on-rvvm/port/include/io.inc and build/u0.s); Handbook
Appendix A values are noted in the implementation files.

Handbook = "PDP-11 Architecture Handbook" (DEC, 1983), printed page numbers.
*/

#ifndef RVVM_UNIBUS_DEVICES_H
#define RVVM_UNIBUS_DEVICES_H

#include "chardev.h"
#include "unibus.h"

/*
 * KW11-L line time clock (lks = 0177546, BR6, vector 0100).
 *
 * line_hz is the AC line frequency in Hz (60 US / 50 EU); 0 selects 60.
 * Ticks at line frequency, setting the Monitor bit and raising a BR6
 * interrupt when interrupt-enable is set. See unibus-kw11l.c.
 */
PUBLIC unibus_dev_t* rvvm_kw11l_init(unibus_t* bus, uint64_t line_hz);

// Force a single line tick (for deterministic testing of the IRQ path).
PUBLIC void rvvm_kw11l_tick(unibus_dev_t* dev);

/*
 * KL11/DL11 console (tks/tkb/tps/tpb @ 0177560, BR4, vectors 060/064).
 *
 * PIO character device wired to an RVVM chardev backend (like ns16550a).
 * Receiver done -> RX interrupt (vector 060); transmitter ready -> TX
 * interrupt (vector 064). See unibus-kl11.c.
 *
 * chardev may be NULL to operate purely on the internal FIFO.
 */
PUBLIC unibus_dev_t* rvvm_kl11_init(unibus_t* bus, chardev_t* chardev);

/*
 * RF11/RS11 fixed-head disk ("drum"), the root+swap device
 * (dcs = 0177460, BR5, vector 0204). NPR block DMA between a backing image
 * and the window RAM. See unibus-rf11.c.
 *
 * image_size_blocks is the drum size in 512-byte blocks; backing is an
 * in-memory store (no file I/O) sufficient for the port's root+swap.
 */
PUBLIC unibus_dev_t* rvvm_rf11_init(unibus_t* bus, size_t image_size_blocks);

/*
 * Convenience: attach a Unibus (MB == 0) populated with the standard 1st
 * Edition UNIX device set -- KW11-L clock, KL11 console (on a stdio
 * terminal) and RF11 drum. Mirrors pci_bus_init_auto() for the PCI side.
 * Returns the bus, or NULL on failure. Not wired into the default machine
 * boot; intended for an explicit CLI/option path.
 */
PUBLIC unibus_t* unibus_pdp11_init_auto(rvvm_machine_t* machine);

#endif
