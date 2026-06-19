/*
unibus.h - PDP-11 Unibus, modeled on RVVM's PCI bus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

This is a common-bus abstraction with PDP-11 peripherals attached on top of
it, modeled directly on how RVVM implements PCI (see pci-bus.{c,h}). It lets
a hand-ported 1st Edition UNIX kernel (PDP-11 asm -> RISC-V) talk to
PDP-11-shaped hardware.

All references below are to the "PDP-11 Architecture Handbook" (Digital
Equipment Corporation, 1983 edition). Page numbers are the *printed* page
numbers shown at the bottom of each page (the PDF page index is +9).

The Unibus model (Handbook Ch.10 "PDP-11 Bus Structures" pp.266-271,
Ch.9 "Mapping to Memory and Busses" p.258 fig.9-37):

  - The bus is asynchronous, strict master/slave. The processor and the
    peripherals share a single 18-bit Unibus address space, of which a
    16-bit window (000000..177777 octal) is what 1st Edition UNIX sees.
  - "Each bus reserves the top 8 Kbytes of its address space for I/O and
    peripheral devices." (Handbook p.266). That is the I/O page,
    760000..777777 octal (Handbook Ch.9 p.258, fig.9-37). In the 16-bit
    window this is 160000..177777 octal == 0xE000..0xFFFF.
  - Everything below the I/O page is "YOUR MEMORY OR I/O" (Handbook p.258),
    i.e. plain RAM as far as the kernel is concerned.

The RVVM mapping (the "window / MB" model):

  A PDP-11 address A is reached at host physical (MB + A). With the natural
  choice MB == 0, a PDP-11 address equals its host physical address, but MB
  is configurable. The window is laid out exactly like the Unibus:

      MB + 0x0000 .. MB + 0xDFFF   RAM            (000000..157777 octal)
      MB + 0xE000 .. MB + 0xFFFF   I/O page       (160000..177777 octal)

  rvvm_attach_mmio() rejects overlap with main RAM, so the I/O page is the
  Unibus's MMIO region (this object), while the RAM portion is ordinary
  machine RAM that the kernel and DMA both reach via MB + A.

Interrupts (Handbook Ch.8 "Traps and Interrupts" pp.215-216, Ch.10
"Arbitration" p.268):

  - A device requests service by raising one of four bus-request lines
    BR4..BR7 (Handbook p.268: "The BUS BRx L lines tell the processor that
    a peripheral would like to interrupt at level x", x = 4..7). BR7 is the
    highest priority, BR4 the lowest.
  - The processor grants the bus (BGx) to the highest-priority requester
    whose level is above the processor's current priority. The grant is a
    daisy chain (Handbook p.267: grant lines "are passed from one I/O
    module to the next in daisy-chained fashion"), so among devices at the
    same level the electrically-nearest device wins.
  - On grant, the device puts an interrupt *vector* on the bus (Handbook
    p.269: "BUS INTR L ... the master ... has placed the address of an
    interrupt vector on the bus. The processor will respond ... and will
    interrupt through that vector."). The vector is a 2-word block in low
    memory: word 0 is the new PC, word 1 is the new PS (Handbook p.216).
    The new PS sets the new processor priority, which is how an ISR masks
    same-or-lower-level interrupts while it runs.

  RVVM has no Unibus arbitration hardware and the RISC-V hart has a single
  external-interrupt line, so this object models arbitration in software:

    * unibus_raise_irq(dev) records dev's (BR level, vector) as a pending
      request and raises ONE RISC-V external IRQ on the machine intc to
      wake the hart. unibus_lower_irq(dev) clears it.
    * The bus exposes an interrupt-acknowledge (IAK) register pair in its
      own MMIO space (UNIBUS_IAK_PRI / UNIBUS_IAK_VEC, see below). This is
      the software stand-in for the BGx grant + BUS INTR vector pass.

  The IAK contract (what the port's RISC-V mtvec dispatcher must implement):

    1. On taking the RISC-V external interrupt, the dispatcher writes its
       current PDP-11 processor priority P (0..7, the PSW bits 5:7) to
       UNIBUS_IAK_PRI (a 16-bit write).
    2. It then reads UNIBUS_IAK_VEC (a 16-bit read). The bus returns the
       interrupt vector of the highest-BR-level pending device whose level
       is strictly greater than P, atomically clearing that one pending
       request (this is the grant). Ties at the same level are broken by
       attach order (the daisy-chain nearest-wins rule).
    3. If no pending request outranks P, the read returns 0 (no device).
       The dispatcher treats vector 0 as "spurious / nothing to do".
    4. The dispatcher then fetches the 2-word vector block at (MB + vector)
       from RAM -- word 0 -> new PC, word 1 -> new PS -- and dispatches,
       exactly as the real processor does (Handbook p.216). Honoring the
       new PS priority (spl) is the dispatcher's job; the bus only enforces
       "level must be > P" at grant time.

    The bus keeps the RISC-V external line asserted as long as any pending
    request exists, so after servicing one device the dispatcher loops
    (write P, read vector) until it reads 0.

NPR / DMA (Handbook p.268: "The BUS NPR L line is used by a peripheral to
request the data section of the bus for a Direct Memory Access (DMA)
transfer. (The acronym NPR stands for Non Processor Request.)"):

  Block devices (drum, disk, DECtape) move data between their backing store
  and the window RAM without processor involvement. unibus_dma_read/write()
  copy between a host buffer and (MB + pdp_addr) in machine RAM, and
  unibus_dma_ptr() exposes a direct pointer for zero-copy transfers. No
  bus-master enable gate is modeled (unlike PCI); NPR is always available,
  matching the Unibus where any device may arbitrate for NPR.

How to add a new device: fill a unibus_dev_desc_t (I/O-page offset, size,
read/write handlers, private data, BR level, vector), call unibus_attach(),
and use unibus_raise_irq()/unibus_lower_irq() for interrupts and the
unibus_dma_*() helpers for NPR transfers. See unibus-kw11l.c (clock),
unibus-kl11.c (console) and unibus-rf11.c (drum) for worked examples.

See docs/unibus.md for the full design notes and per-device register table.
*/

#ifndef RVVM_UNIBUS_H
#define RVVM_UNIBUS_H

#include "rvvmlib.h"

/*
 * PDP-11 Unibus geometry (octal addresses from the Handbook, here in hex).
 *
 * The 16-bit window the kernel sees, and the I/O page at its top
 * (Handbook p.266 "top 8 Kbytes", p.258 fig.9-37 "760000 I/O PAGE").
 */
#define UNIBUS_WINDOW_SIZE   0x10000 // 16-bit PDP-11 window (64 KiB)
#define UNIBUS_IO_PAGE_BASE  0xE000  // 160000 octal, start of the I/O page
#define UNIBUS_IO_PAGE_SIZE  0x2000  // 8 KiB I/O page (760000..777777 octal)

/*
 * Bus-request priority levels (Handbook Ch.10 p.268). BR7 highest, BR4
 * lowest. Levels 1..3 exist as processor priorities but have no bus-request
 * line; peripherals only ever request on BR4..BR7.
 */
#define UNIBUS_BR4 4 // Lowest device interrupt priority
#define UNIBUS_BR5 5
#define UNIBUS_BR6 6
#define UNIBUS_BR7 7 // Highest device interrupt priority

/*
 * Interrupt-acknowledge (IAK) register pair, in the bus's own MMIO space,
 * placed just above the modeled I/O-page device area. These are NOT real
 * PDP-11 registers; they are the RVVM software stand-in for the BGx grant
 * and the BUS INTR vector pass (Handbook p.268-269). See the contract in
 * the file header above. Offsets are within the bus MMIO region.
 */
#define UNIBUS_IAK_PRI 0x1FF0 // W: current processor priority P (PSW 5:7)
#define UNIBUS_IAK_VEC 0x1FF2 // R: granted vector (level > P), 0 if none

/*
 * Processor status word (PSW), window address 0177776 octal == 0xFFFE, i.e.
 * I/O-page offset 0x1FFE. Unlike the IAK pair this IS a real PDP-11 register:
 * the PDP-11/20 exposes the PSW at 777776 (Handbook Ch.8 p.216), and 1st
 * Edition UNIX raises/lowers the processor priority by writing it directly
 * ("mov $340,*$ps" / "clr *$ps", ps = 0177776).
 *
 * Modeling it on the bus closes the delivery gap in the IAK contract above:
 * the bus asserts the single RISC-V external line only while a pending device
 * outranks the current PSW priority (bits 5:7), so a device that completes
 * while the kernel is at high priority (e.g. the RF11 finishing its DMA inside
 * the GO store while ppoke holds level 7) is deferred until the kernel lowers
 * priority -- exactly as the real processor defers BRx below P. Without this,
 * the RISC-V interrupt would be taken the instant MIE allows it, reentering
 * the very critical section the spl was meant to protect.
 */
#define UNIBUS_PSW 0x1FFE // R/W: processor status word (priority in bits 5:7)

// Maximum register-block size of a single Unibus device
#define UNIBUS_DEV_MAX_SIZE 0x40

// Opaque handles
typedef struct rvvm_unibus     unibus_t;
typedef struct rvvm_unibus_dev unibus_dev_t;

/*
 * Unibus device description.
 *
 * Mirrors pci_func_desc_t: a register block (here addressed by I/O-page
 * offset rather than a BAR), plus per-device interrupt routing. Unlike PCI,
 * the interrupt identity is a fixed (BR level, vector) pair set by the
 * device, matching the wired-down nature of Unibus peripherals.
 */
typedef struct {
    const char* name; // Device identifier (logger / FDT)

    // Register block within the I/O page, addressed by *window* address,
    // e.g. lks = 0177546 octal == 0xFF66. The bus translates this to an
    // offset within the I/O page (io_addr - UNIBUS_IO_PAGE_BASE).
    rvvm_addr_t io_addr;
    size_t      size;

    // 16-bit register read/write. off is relative to io_addr.
    // PDP-11 registers are word-sized; byte access is supported where the
    // device opts in via min_size == 1.
    void (*read)(unibus_dev_t* dev, uint16_t* val, size_t off);
    void (*write)(unibus_dev_t* dev, uint16_t val, size_t off);

    // Optional periodic service hook, called from the bus event-thread
    // poll. Used for time-driven devices (e.g. the KW11-L line clock) and
    // for coalesced chardev polling. May be NULL.
    void (*poll)(unibus_dev_t* dev);

    // Optional cleanup hook, called on bus teardown before the device's
    // data is freed. Use it to release auxiliary resources (backing store,
    // chardev handle, etc). May be NULL.
    void (*cleanup)(unibus_dev_t* dev);

    uint32_t min_size; // 1 to allow byte access, else 2 (word only)

    // Interrupt routing (Handbook p.268 BRx, p.216 vector)
    uint32_t br_level; // UNIBUS_BR4..UNIBUS_BR7
    uint16_t vector;   // Interrupt vector, low-memory byte address

    void* data; // Private device data, see shared_data below

    // If true, this device's data is shared with another device endpoint and
    // is NOT freed when this device is cleaned up (the owning endpoint frees
    // it). Used by multi-vector devices like the KL11 console, where the
    // receiver and transmitter endpoints share one state object. Default
    // false: the bus free()s data on cleanup.
    bool shared_data;
} unibus_dev_desc_t;

/**
 * @defgroup rvvm_unibus_api Unibus Management API
 * @addtogroup rvvm_unibus_api
 * @{
 */

//! \brief  Attach a Unibus to a machine, owning the I/O-page MMIO region
//! \param  machine  RVVM machine handle with a working interrupt controller
//! \param  mem_base Host physical base (MB) of the 16-bit PDP-11 window
//! \return Unibus handle, or NULL on failure
//!
//! The I/O page is mapped at (mem_base + UNIBUS_IO_PAGE_BASE). The RAM
//! portion of the window (mem_base + 0 .. mem_base + UNIBUS_IO_PAGE_BASE)
//! must be backed by machine RAM by the caller; this is where DMA lands.
PUBLIC unibus_t* unibus_init(rvvm_machine_t* machine, rvvm_addr_t mem_base);

//! \brief  Attach a Unibus with the natural MB == 0 layout
//! \param  machine RVVM machine handle with a working interrupt controller
//! \return Unibus handle, or NULL on failure
PUBLIC unibus_t* unibus_init_auto(rvvm_machine_t* machine);

//! \brief  Attach a device to a Unibus
//! \param  bus  Valid Unibus handle
//! \param  desc Device description, device data is cleaned up automatically
//! \return Unibus device handle, or NULL on failure
PUBLIC unibus_dev_t* unibus_attach(unibus_t* bus, const unibus_dev_desc_t* desc);

//! \brief  Get the owning machine of a Unibus
PUBLIC rvvm_machine_t* unibus_machine(unibus_t* bus);

//! \brief  Get the host physical base (MB) of the window
PUBLIC rvvm_addr_t unibus_mem_base(unibus_t* bus);

/** @}*/

/**
 * @defgroup rvvm_unibus_dev_api Unibus Device API
 * @addtogroup rvvm_unibus_dev_api
 * @{
 */

//! \brief  Get private device data
PUBLIC void* unibus_dev_data(unibus_dev_t* dev);

//! \brief  Get the Unibus a device is attached to
PUBLIC unibus_t* unibus_dev_bus(unibus_dev_t* dev);

//! \brief Interrupt-acknowledge grant at processor priority prio
//! \param bus  Valid Unibus handle
//! \param prio Current processor priority P (PSW bits 5:7, range 0..7)
//! \return Vector of the granted device, or 0 if none outranks prio
//!
//! This is the programmatic form of the IAK register contract: it returns
//! the interrupt vector of the highest-BR-level pending device whose level
//! is strictly greater than prio, atomically clearing that one pending
//! request (the grant), or 0 if no pending request outranks prio. Ties at
//! the same level are broken by attach order (daisy-chain nearest-wins,
//! Handbook p.267). The MMIO IAK register pair performs exactly this on a
//! UNIBUS_IAK_PRI write followed by a UNIBUS_IAK_VEC read; this entry point
//! exposes the same grant to host code and tests. (Handbook pp.216, 268)
PUBLIC uint16_t unibus_ack(unibus_t* bus, uint32_t prio);

//! \brief Raise a device interrupt request (assert BRx; Handbook p.268)
//!
//! Records this device's (BR level, vector) as pending and raises the
//! single RISC-V external IRQ. Idempotent while already pending.
PUBLIC void unibus_raise_irq(unibus_dev_t* dev);

//! \brief Lower a device interrupt request (deassert BRx)
PUBLIC void unibus_lower_irq(unibus_dev_t* dev);

//! \brief NPR DMA read: copy len bytes from window RAM at pdp_addr to buf
//! \return Success (Handbook p.268 NPR / DMA)
PUBLIC bool unibus_dma_read(unibus_dev_t* dev, rvvm_addr_t pdp_addr, void* buf, size_t len);

//! \brief NPR DMA write: copy len bytes from buf into window RAM at pdp_addr
//! \return Success (Handbook p.268 NPR / DMA)
PUBLIC bool unibus_dma_write(unibus_dev_t* dev, rvvm_addr_t pdp_addr, const void* buf, size_t len);

//! \brief Direct DMA pointer into window RAM at pdp_addr (zero-copy NPR)
//! \return Host pointer, or NULL on failure
PUBLIC void* unibus_dma_ptr(unibus_dev_t* dev, rvvm_addr_t pdp_addr, size_t len);

/** @}*/

#endif
