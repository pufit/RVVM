/*
parport-pci.h - PCI Parallel Port (NetMos/MosChip MCS9900)
Copyright (C) 2026  Sol Astrius <claude@danielsol.dev>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef PARPORT_PCI_H
#define PARPORT_PCI_H

#include "rvvmlib.h"
#include "pci-bus.h"

/*
 * Backend callback. The HDA-style two-arg shape: caller hands the
 * device a function pointer + user data, the device calls it with
 * captured PCM-equivalent... actually with each guest-strobed byte.
 *
 * Called from the MMIO write path (no spinlock held), so the backend
 * may safely block briefly on a host file/pipe write.
 */
typedef void (*parport_pci_write_fn)(void *user_data, uint8_t byte);

/*
 * Attach a NetMos 9900 PCI parport to the given PCI bus. Single port,
 * SPP-only (no EPP/ECP). Linux's parport_pc.c picks it up via PCI ID
 * match (vendor 0x9710 device 0x9900) and exposes /dev/parport0 +
 * /dev/lp0 + supports PLIP and ppdev.
 *
 * write_fn / user_data: optional. If NULL, the device is "headless" —
 * registers respond to MMIO but bytes are silently dropped after the
 * Centronics strobe. Useful for testing PCI enumeration without a
 * real backend.
 */
PUBLIC pci_dev_t* parport_pci_init(pci_bus_t* pci_bus,
                                   parport_pci_write_fn write_fn,
                                   void* user_data);

PUBLIC pci_dev_t* parport_pci_init_auto(rvvm_machine_t* machine,
                                        parport_pci_write_fn write_fn,
                                        void* user_data);

#endif
