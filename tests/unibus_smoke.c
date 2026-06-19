/*
unibus_smoke.c - Self-contained smoke test for the PDP-11 Unibus subsystem

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

----------------------------------------------------------------------------

Builds against librvvm using only its public ABI. It:

  1. Creates a machine and installs a recording interrupt controller that
     captures the level of the single RISC-V external IRQ line.
  2. Attaches a Unibus (MB == 0) and a KW11-L line clock (BR7, vector 064).
  3. Runs a tiny hand-encoded RV64 firmware that enables the clock's
     interrupt by writing the IE bit to LKS (0177546) over real MMIO, then
     signals readiness in RAM and spins.
  4. Forces a line tick and verifies:
       - the bus asserts the RISC-V external IRQ line (recording intc), and
       - unibus_ack(P=0) returns vector 064 (the granted clock interrupt),
       - unibus_ack(P=7) returns 0 (level 7 is not strictly above P=7),
     matching the IAK / BR-priority contract in unibus.h.

Build (from the repo root, after `make USE_LIB=1 lib`):
  cc -Iinclude tests/unibus_smoke.c -Lrelease.<os>.<arch> -lrvvm \
     -Wl,-rpath,release.<os>.<arch> -o /tmp/unibus_smoke && /tmp/unibus_smoke
*/

#include <rvvm/rvvm.h>
#include <rvvm/rvvm_irq.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Unibus public ABI (declared here to keep the test free of internal
// headers; signatures mirror src/devices/unibus.h and unibus-devices.h).
typedef struct rvvm_unibus     unibus_t;
typedef struct rvvm_unibus_dev unibus_dev_t;

extern unibus_t*     unibus_init_auto(rvvm_machine_t* machine);
extern unibus_t*     unibus_init(rvvm_machine_t* machine, rvvm_addr_t mem_base);
extern uint16_t      unibus_ack(unibus_t* bus, uint32_t prio);
extern unibus_dev_t* rvvm_kw11l_init(unibus_t* bus, uint64_t line_hz);
extern void          rvvm_kw11l_tick(unibus_dev_t* dev);
extern unibus_dev_t* rvvm_rf11_init(unibus_t* bus, size_t image_size_blocks);

// PDP-11 window addresses (MB == 0)
#define LKS_ADDR     0xFF66 // lks = 0177546
#define LKS_IE       0x40   // interrupt enable
#define READY_MARKER 0x1000 // RAM offset where the firmware signals ready

// RAM layout
#define MEM_BASE 0x80000000ULL
#define MEM_SIZE (16 * 1024 * 1024)

// --- Recording interrupt controller -----------------------------------------

static volatile int g_irq_level = -1; // last observed external line level

static void rec_set_irq(rvvm_irq_dev_t* irq_dev, rvvm_irq_t irq, bool lvl)
{
    (void)irq_dev;
    (void)irq;
    g_irq_level = lvl ? 1 : 0;
}

static const rvvm_irq_dev_cb_t rec_intc_cb = {
    .set_irq = rec_set_irq,
};

// --- Tiny RV64 firmware -----------------------------------------------------
//
// Hand-encoded RV64I. Reset PC defaults to MEM_BASE (0x80000000).
//
//       lui   t0, 0x10           ; t0 = 0x10000
//       addi  t0, t0, -0x9A      ; t0 = 0x0FF66  (LKS address)
//       li    t1, 0x40           ; t1 = LKS_IE
//       sh    t1, 0(t0)          ; *(u16*)0xFF66 = 0x40  (enable clock IRQ)
//       li    t2, 0x80001000     ; t2 = MEM_BASE + READY_MARKER
//       li    t3, 1
//       sw    t3, 0(t2)          ; *(u32*)0x80001000 = 1  (signal ready)
//   1:  j 1b                     ; spin forever
//
// The encodings below are the exact output of:
//   zig cc -target riscv64-freestanding -mcpu=generic_rv64 -nostdlib \
//          -Wl,-T,fw.ld -o fw.elf fw.s   (then `zig objcopy -O binary`)
// They are valid base RV64I (no compressed instructions, matching the
// "rv64i" machine ISA below). Note `li t2, 0x80001000` MUST be the
// lui+addi+slli sequence the assembler emits, NOT a single `lui t2,0x80001`:
// on RV64 lui sign-extends bit 31, so a single lui would yield the
// nonexistent address 0xFFFFFFFF80001000.
static const uint32_t firmware[] = {
    0x000102B7, // lui   t0, 0x10
    0xF6628293, // addi  t0, t0, -0x9A
    0x04000313, // li    t1, 0x40
    0x00629023, // sh    t1, 0(t0)
    0x000803B7, // lui   t2, 0x80
    0x00138393, // addi  t2, t2, 1
    0x00C39393, // slli  t2, t2, 12   -> t2 = 0x80001000
    0x00100E13, // li    t3, 1
    0x01C3A023, // sw    t3, 0(t2)
    0x0000006F, // j     . (self loop)
};

static int g_failures = 0;

static void check(const char* what, bool ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) {
        g_failures++;
    }
}

int main(void)
{
    if (!rvvm_check_abi(RVVM_ABI_VERSION)) {
        printf("[FAIL] librvvm ABI mismatch\n");
        return 1;
    }

    rvvm_machine_t* machine = rvvm_create_machine(MEM_SIZE, 1, "rv64i");
    if (!machine) {
        printf("[FAIL] could not create machine\n");
        return 1;
    }

    // Install the recording interrupt controller before the bus is attached.
    rvvm_intc_t* intc = rvvm_irq_dev_init(&rec_intc_cb, NULL);
    rvvm_set_intc(machine, intc);

    unibus_t* bus = unibus_init_auto(machine);
    check("unibus_init_auto", bus != NULL);

    unibus_dev_t* clk = rvvm_kw11l_init(bus, 60);
    check("kw11l attach", clk != NULL);

    // Load and run the firmware that enables the clock interrupt over MMIO.
    rvvm_write_ram(machine, MEM_BASE, firmware, sizeof(firmware));
    rvvm_set_opt(machine, RVVM_OPT_RESET_PC, MEM_BASE);
    rvvm_start_machine(machine);

    // Wait for the firmware to signal it has enabled IE over real MMIO.
    bool ready = false;
    for (int i = 0; i < 100000 && !ready; ++i) {
        uint32_t marker = 0;
        rvvm_read_ram(machine, &marker, MEM_BASE + READY_MARKER, sizeof(marker));
        ready = (marker == 1);
    }
    check("firmware enabled LKS IE over MMIO", ready);

    // Before any tick, the line must be low and IAK must grant nothing.
    check("no IRQ before tick", g_irq_level != 1);
    check("no vector before tick (P=0)", unibus_ack(bus, 0) == 0);

    // Force one line tick (one LTC pulse). With IE set, the bus must raise
    // the RISC-V external line.
    rvvm_kw11l_tick(clk);
    check("bus raised RISC-V IRQ after tick", g_irq_level == 1);

    // IAK at P=7: BR7 is not strictly greater than 7, so nothing is granted.
    check("IAK P>=7 grants nothing", unibus_ack(bus, 7) == 0);

    // IAK at P=0: the clock (BR7) outranks P, grant returns vector 064.
    uint16_t vec = unibus_ack(bus, 0);
    check("IAK P=0 grants clock vector 064", vec == 0064);

    // The grant cleared the request, so a second IAK grants nothing and the
    // bus drops the RISC-V line.
    check("second IAK grants nothing", unibus_ack(bus, 0) == 0);
    check("RISC-V IRQ dropped after grant", g_irq_level == 0);

    rvvm_pause_machine(machine);
    rvvm_free_machine(machine);

    // --- RF11 drum NPR/DMA test ---------------------------------------------
    //
    // The RF11 transfers blocks between its backing store and the window RAM
    // via NPR DMA at (MB + pdp_addr). For the window RAM to be real memory at
    // low PDP-11 addresses, this machine is based at physical 0 (MB == 0, so
    // the window RAM coincides with machine RAM). We validate the NPR
    // transport with a host-side round trip over the public DMA helpers --
    // the same calls rf11_go() issues internally.
    {
        // Size RAM to exactly the window-RAM region (0..0xDFFF) so it does
        // not overlap the I/O page at 0xE000 -- the Unibus layout, and the
        // condition rvvm_attach_mmio() requires (MMIO must not overlap RAM).
        rvvm_machine_t* dm = rvvm_create_machine(0xE000, 1, "rv64i");
        rvvm_set_opt(dm, RVVM_OPT_MEM_BASE, 0x0); // MB == 0: window RAM at phys 0
        rvvm_set_intc(dm, rvvm_irq_dev_init(&rec_intc_cb, NULL));

        unibus_t*     dbus = unibus_init(dm, 0x0);
        unibus_dev_t* rf   = rvvm_rf11_init(dbus, 64);
        check("rf11 attach", rf != NULL);

        // A drum block is 512 bytes (256 words). Put a pattern in window RAM
        // at PDP-11 address 0x2000 (well inside RAM, below the I/O page).
        const rvvm_addr_t buf = 0x2000;
        uint8_t           pattern[512];
        for (size_t i = 0; i < sizeof(pattern); ++i) {
            pattern[i] = (uint8_t)(i * 7 + 3);
        }
        rvvm_write_ram(dm, buf, pattern, sizeof(pattern));

        extern bool unibus_dma_read(unibus_dev_t*, rvvm_addr_t, void*, size_t);
        extern bool unibus_dma_write(unibus_dev_t*, rvvm_addr_t, const void*, size_t);

        uint8_t block[512] = {0};
        bool    ok         = unibus_dma_read(rf, buf, block, sizeof(block));
        check("NPR DMA read from window RAM", ok && memcmp(block, pattern, sizeof(block)) == 0);

        // Write it back to a different window RAM address and verify via RAM.
        const rvvm_addr_t buf2 = 0x4000;
        ok                     = unibus_dma_write(rf, buf2, block, sizeof(block));
        uint8_t check_buf[512] = {0};
        rvvm_read_ram(dm, check_buf, buf2, sizeof(check_buf));
        check("NPR DMA write to window RAM", ok && memcmp(check_buf, pattern, sizeof(check_buf)) == 0);

        rvvm_free_machine(dm);
    }

    printf("\n%s (%d failure%s)\n", g_failures ? "SMOKE TEST FAILED" : "SMOKE TEST PASSED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
