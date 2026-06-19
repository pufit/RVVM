/*
unibus-kw11l.c - KW11-L Line Time Clock on the PDP-11 Unibus

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

The KW11-L is the line-frequency real-time clock. It has a single CSR (LKS)
and ticks at the AC line frequency (Handbook Ch.10 p.271: "The LTC line
provides a realtime clock input for the system. It pulses with each cycle of
the line current."). Each tick sets the Monitor (done) bit; if interrupt
enable is set, it requests a Unibus interrupt.

Addresses / vector (the binding contract is the ported kernel; the Handbook
values are noted for reference):
  - LKS  = 0177546 octal  (Handbook Appendix A p.A-1: "DL11-W(LTC) 777546
           ... line clock", and BDV11-LTC 777546).
  - BR7 / processor level 7 -- the kernel's u0.s vector table
    (build/u0.s: "clock;340  / clock interrupt vector ; processor level 7").
  - Vector 064 octal -- per the kernel's u0.s (". = orig+60" block, the
    fifth entry "clock;340" lands at offset 64). The Handbook's Appendix A
    fixed assignment for KW11-L is vector 100 (p.A-5); 1st Edition UNIX uses
    its own table, and that table is what we honor here.

LKS bit layout (KW11-L):
  bit  7  Monitor / Done  -- set each line tick, cleared by writing 0
  bit  6  Interrupt Enable
*/

#include <rvvm/rvvm_board.h>

#include "rvtimer.h"
#include "unibus.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

#define KW11L_LKS_DONE   0x80 // bit 7: Monitor / Done flag (set each tick)
#define KW11L_LKS_IE     0x40 // bit 6: Interrupt Enable
#define KW11L_LKS_RW     (KW11L_LKS_DONE | KW11L_LKS_IE)

#define KW11L_DEFAULT_HZ 60 // AC line frequency (60 Hz US, 50 Hz elsewhere)

typedef struct {
    unibus_dev_t* dev;

    uint64_t freq;       // Line frequency in Hz
    uint64_t last_tick;  // Clocksource value at the last serviced tick

    uint32_t lks; // Atomic: LKS register contents (DONE | IE)
} kw11l_dev_t;

// Re-evaluate the interrupt request from the current LKS state
static void kw11l_update_irq(kw11l_dev_t* clk)
{
    uint32_t lks = atomic_load_uint32_relax(&clk->lks);
    if ((lks & KW11L_LKS_DONE) && (lks & KW11L_LKS_IE)) {
        unibus_raise_irq(clk->dev);
    } else {
        unibus_lower_irq(clk->dev);
    }
}

// Advance the clock by however many line ticks have elapsed in real time
static void kw11l_advance(kw11l_dev_t* clk)
{
    uint64_t now = rvtimer_clocksource(clk->freq);
    if (now != clk->last_tick) {
        // At least one line tick elapsed; latch the Monitor bit. The bit is
        // sticky until the guest clears it, matching real hardware where a
        // tick during an unserviced previous tick is simply coalesced.
        clk->last_tick = now;
        atomic_or_uint32(&clk->lks, KW11L_LKS_DONE);
        kw11l_update_irq(clk);
    }
}

static void kw11l_read(unibus_dev_t* dev, uint16_t* val, size_t off)
{
    kw11l_dev_t* clk = unibus_dev_data(dev);
    UNUSED(off);
    // Reading LKS reflects the current Monitor/IE bits.
    *val = (uint16_t)atomic_load_uint32_relax(&clk->lks);
}

static void kw11l_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    kw11l_dev_t* clk = unibus_dev_data(dev);
    UNUSED(off);
    // Writing LKS sets Interrupt Enable and clears Monitor when the guest
    // writes a 0 to bit 7 (the standard ack on the KW11-L).
    atomic_store_uint32_relax(&clk->lks, val & KW11L_LKS_RW);
    kw11l_update_irq(clk);
}

// Periodic service from the bus event-thread poll: advance the line clock.
static void kw11l_poll(unibus_dev_t* dev)
{
    kw11l_advance(unibus_dev_data(dev));
}

RVVM_PUBLIC unibus_dev_t* rvvm_kw11l_init(unibus_t* bus, uint64_t line_hz)
{
    if (!bus) {
        return NULL;
    }
    if (!line_hz) {
        line_hz = KW11L_DEFAULT_HZ;
    }

    kw11l_dev_t* clk = safe_new_obj(kw11l_dev_t);
    clk->freq        = line_hz;
    clk->last_tick   = rvtimer_clocksource(line_hz);

    unibus_dev_desc_t desc = {
        .name     = "kw11-l",
        .io_addr  = 0xFF66, // lks = 0177546 octal
        .size     = 0x2,
        .read     = kw11l_read,
        .write    = kw11l_write,
        .poll     = kw11l_poll,
        .min_size = 2,
        .br_level = UNIBUS_BR7, // processor level 7 (u0.s: clock;340)
        .vector   = 0100,       // clock is the 5th vector from orig+60 -> 0100
                                // octal (0064 is ttyo, used by the KL11 TX)
        .data     = clk,
    };

    clk->dev = unibus_attach(bus, &desc);
    return clk->dev;
}

// Force a single line tick (one LTC pulse), independent of wall-clock time.
// Exposed for deterministic testing of the interrupt path; normal operation
// is driven by the time-based bus poll above.
RVVM_PUBLIC void rvvm_kw11l_tick(unibus_dev_t* dev)
{
    if (dev) {
        kw11l_dev_t* clk = unibus_dev_data(dev);
        atomic_or_uint32(&clk->lks, KW11L_LKS_DONE);
        kw11l_update_irq(clk);
    }
}

POP_OPTIMIZATION_SIZE
