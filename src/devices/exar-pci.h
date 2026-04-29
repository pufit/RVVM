/*
exar-pci.h - Exar XR17V35x-family PCIe combo serial card
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_EXAR_PCI_H
#define RVVM_EXAR_PCI_H

#include "rvvmlib.h"
#include "chardev.h"
#include "pci-bus.h"

/*
 * Exar XR17V35x-family PCIe multi-port serial. The kernel driver
 * (drivers/tty/serial/8250/8250_exar.c) attaches to all members below
 * via a single PCI ID; port count is encoded in the device ID's last
 * nibble (`exar_get_nr_ports`).
 *
 * BAR0 is one flat MMIO region of `n_ports * 0x400` bytes. Standard
 * 16550 registers live at +0x00..+0x07 of each per-port 0x400 window;
 * Exar-specific FCTR/EFR/TXTRG/RXTRG sit at +0x08..+0x0B; chip-global
 * MPIO/INT0/DVID/REGB are projected into every per-port window at
 * +0x80..+0x9A. INTx is shared across all ports.
 */

typedef enum {
    EXAR_MODEL_XR17V352  = 2,
    EXAR_MODEL_XR17V354  = 4,
    EXAR_MODEL_XR17V358  = 8,
    EXAR_MODEL_XR17V4358 = 12,
    EXAR_MODEL_XR17V8358 = 16,
} exar_model_t;

//! \brief  Attach an Exar XR17V35x PCIe combo card to a PCI bus.
//! \param  bus       Valid PCI bus handle
//! \param  model     Chip variant (selects PCI device ID + port count)
//! \param  chardevs  Array of `model` chardev pointers (one per port).
//!                   Entries may be NULL — those ports decode reads/writes
//!                   but discard data. Chardev ownership transfers to the
//!                   card; freed when the card is unplugged.
//! \return PCI device handle, or NULL on failure
PUBLIC pci_dev_t* exar_pci_init(pci_bus_t* bus, exar_model_t model, chardev_t** chardevs);

//! \brief Convenience wrapper that uses the host PCI bus from a machine.
PUBLIC pci_dev_t* exar_pci_init_auto(rvvm_machine_t* machine, exar_model_t model, chardev_t** chardevs);

#endif
