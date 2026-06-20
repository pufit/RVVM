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

Addresses / vector / BR level (the binding contract is the ported kernel;
the Handbook and Fourth Edition source are noted for cross-reference):
  - LKS  = 0177546 octal  (Handbook Appendix A p.A-1: "DL11-W(LTC) 777546
           ... line clock", and BDV11-LTC 777546).
  - Vector 0100 octal -- the kernel's u0.s ". = orig+60" block holds five
    two-word entries (ttyi 060, ttyo 064, ppti 070, ppto 074, clock 0100),
    so the clock lands at 0100, NOT 064 (064 is ttyo, used by the KL11 TX).
    This matches the Handbook's fixed KW11-L vector (p.A-5) and Fourth
    Edition (low.s ". = 100^." / "kwlp").
  - BR6 -- the KW11-L hardwires its bus request on BR6. This is a property
    of the backplane wiring, not of the kernel, and the vector table does
    NOT encode it. Fourth Edition states it outright (low.s: "kwlp; br6",
    where br6 = 0300). DO NOT read it off the "340" in u0.s "clock;340":
    that 340 is the *new PS* (second vector word) the dispatcher loads into
    the PSW on taking the interrupt -- the priority the clock ISR *runs at*
    (level 7), so the tick handler masks every device while it runs. It is
    a software run-level, not the bus-request line: the very same KW11-L
    runs its ISR at level 6 (PS 300) in Fourth Edition and level 7 (PS 340)
    in First Edition. Modeling the request at BR7 would let the clock
    preempt the kernel's spl6 critical sections (u3.s "swap", u4.s runq
    manipulation), which on real hardware a BR6 clock cannot.

LKS bit layout (KW11-L):
  bit  7  Monitor / Done  -- set by hardware each line tick; cannot be set
          under program control. Cleared by EITHER reading LKS (the First
          Edition ISR restarts the clock with a bare "tst *$lks", u4.s) OR
          by writing 0 to bit 7 (Fourth Edition acks with "*lks = 0115",
          clock.c -- bit 7 clear, IE set). Clearing it drops the interrupt
          request; both clear paths are honored here so either kernel works.
  bit  6  Interrupt Enable (read/write)
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
    // Read returns the current Monitor/IE bits, then clears Monitor as a
    // side effect (KW11-L read-to-restart). This is how the First Edition
    // ISR dismisses the tick interrupt: a bare "tst *$lks" with no write
    // (u4.s "clock:"). atomic_and returns the pre-clear value, so the guest
    // still observes the Monitor bit that was set.
    uint32_t old = atomic_and_uint32(&clk->lks, ~(uint32_t)KW11L_LKS_DONE);
    *val = (uint16_t)old;
    kw11l_update_irq(clk); // Monitor now clear -> drop the request
}

static void kw11l_write(unibus_dev_t* dev, uint16_t val, size_t off)
{
    kw11l_dev_t* clk = unibus_dev_data(dev);
    UNUSED(off);
    // Interrupt Enable follows the written bit 6. Monitor cannot be SET
    // under program control (only the line tick sets it), and writing a 0
    // to bit 7 CLEARS it -- the Fourth Edition ack "*lks = 0115" (IE set,
    // bit 7 clear). A CAS keeps a concurrent tick (poll thread) from being
    // lost between load and store.
    uint32_t cur;
    uint32_t next;
    do {
        cur  = atomic_load_uint32_relax(&clk->lks);
        next = (uint32_t)(val & KW11L_LKS_IE); // IE from the write
        if (val & KW11L_LKS_DONE) {
            next |= (cur & KW11L_LKS_DONE);    // bit 7 not cleared: keep Monitor
        }                                      // bit 7 == 0: Monitor cleared
    } while (!atomic_cas_uint32(&clk->lks, cur, next));
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
        .br_level = UNIBUS_BR6, // KW11-L requests on BR6 (Fourth Edition
                                // low.s "kwlp; br6"). The 340 in u0.s
                                // "clock;340" is the ISR's new PS / run
                                // level 7, NOT the bus-request line; see the
                                // file header. BR7 here would defeat the
                                // kernel's spl6 critical sections.
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
