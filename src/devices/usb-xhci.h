/*
usb-xhci.h - USB Extensible Host Controller Interface
Copyright (C) 2024  LekKit <github.com/LekKit>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_USB_XHCI_H
#define RVVM_USB_XHCI_H

#include "pci-bus.h"
#include "rvvmlib.h"

#include <rvvm/rvvm_usb.h>

/*
 * Attach an XHCI controller to a PCI bus.
 *
 * If `out_bus` is non-NULL, receives the rvvm_usb_bus_t handle that
 * callers can plug devices into via rvvm_usb_dev_init_at(). The bus
 * lifetime is tied to the PCI device (freed when the machine is torn
 * down).
 */
PUBLIC pci_dev_t* usb_xhci_init(pci_bus_t* pci_bus, rvvm_usb_bus_t** out_bus);

/*
 * Machine-scoped convenience init. Locates the machine's PCI bus,
 * attaches an XHCI, and returns the usable USB bus handle. Returns
 * NULL on failure.
 */
PUBLIC rvvm_usb_bus_t* usb_xhci_init_auto(rvvm_machine_t* machine);

#endif
