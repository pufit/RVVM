# PDP-11 Unibus subsystem

A common-bus abstraction with PDP-11 peripherals attached on top, modeled
directly on how RVVM implements PCI (`src/devices/pci-bus.{c,h}`). It is the
device layer that lets a hand-ported 1st Edition UNIX kernel (PDP-11 asm →
RISC-V) talk to PDP-11-shaped hardware.

All Handbook references are to the **PDP-11 Architecture Handbook**, Digital
Equipment Corporation, 1983 edition, by **printed** page number (the number
at the bottom of the page; the PDF page index is +9).

## Files

| File | Role |
|------|------|
| `src/devices/unibus.h` | Public bus + device API, and the full interrupt/IAK/NPR contract |
| `src/devices/unibus.c` | The bus: owns the I/O-page MMIO region, routes register access, arbitration/IAK, NPR/DMA |
| `src/devices/unibus-devices.h` | Constructors for the concrete devices |
| `src/devices/unibus-kw11l.c` | KW11-L line time clock (fully working) |
| `src/devices/unibus-kl11.c` | KL11/DL11 console (fully working) |
| `src/devices/unibus-rf11.c` | RF11/RS11 "drum" — NPR block DMA (fully working) |
| `tests/unibus_smoke.c` | Self-contained librvvm smoke test (real-MMIO + IAK contract) |

## The window / MB model

The ported kernel runs on RISC-V (M-mode) and accesses a 16-bit PDP-11
window. A PDP-11 address `A` is reached at host physical `MB + A`, where `MB`
is the configurable window base (natural choice `MB == 0`, so PDP-11 address
== host physical address). The window is laid out exactly like the Unibus
(Handbook Ch.9 p.258 fig.9-37 "760000 I/O PAGE … YOUR MEMORY OR I/O";
Ch.10 p.266 "Each bus reserves the top 8 Kbytes of its address space for I/O
and peripheral devices"):

```
   MB + 0x0000 .. MB + 0xDFFF   RAM        (000000..157777 octal)
   MB + 0xE000 .. MB + 0xFFFF   I/O page   (160000..177777 octal == 760000..777777 on the 18-bit Unibus)
```

`rvvm_attach_mmio()` rejects overlap with main RAM, so the I/O page is the
Unibus's MMIO region (`unibus.c` owns it), while the RAM portion is ordinary
machine RAM that the kernel and DMA both reach through `MB + A`.

The bus is created with `unibus_init(machine, MB)` (or `unibus_init_auto()`
for `MB == 0`). It owns the 8 KiB I/O-page MMIO region and routes 16-bit
register accesses to attached devices by I/O-page offset, supporting both
word (the PDP-11 norm) and byte access (byte writes become read-modify-write,
matching the Unibus DATOB transfer, Handbook p.269).

## Bus API (vs the PCI pattern)

| PCI (`pci-bus.h`) | Unibus (`unibus.h`) | Note |
|-------------------|---------------------|------|
| `pci_bus_init(machine, …)` | `unibus_init(machine, mem_base)` | Bus owns an address region |
| `pci_bus_init_auto(machine)` | `unibus_init_auto(machine)` | Convenience constructor |
| `pci_func_desc_t` (BARs as `rvvm_mmio_dev_t`) | `unibus_dev_desc_t` (register block at I/O-page offset) | Device descriptor |
| `pci_attach_func(bus, desc)` | `unibus_attach(bus, desc)` | Attach a device |
| `pci_raise_irq/lower_irq/send_irq(func, id)` | `unibus_raise_irq/lower_irq(dev)` | Per-device interrupt |
| `pci_get_dma_ptr(func, addr, size)` | `unibus_dma_ptr/read/write(dev, pdp_addr, …)` | DMA |

A `unibus_dev_desc_t` carries: a name, the register-block window address +
size, `read`/`write`/`poll`/`cleanup` handlers, private `data`, and the
device's fixed **BR level** and **interrupt vector**. The fixed
(level, vector) pair reflects the wired-down nature of Unibus peripherals,
unlike PCI's programmable INTx/MSI routing.

## Interrupt + IAK + priority contract

This is the part the port's RISC-V `mtvec` dispatcher must implement against.

### Hardware model (Handbook)

- A device requests service by raising one of four bus-request lines
  **BR4..BR7** (Handbook Ch.10 p.268: "The BUS BRx L lines tell the
  processor that a peripheral would like to interrupt at level x", x = 4..7).
  BR7 is highest priority, BR4 lowest.
- The processor grants the bus (BGx) to the highest-priority requester whose
  level is above the processor's current priority. The grant is a daisy
  chain (Handbook p.267: grant signals "are passed from one I/O module to
  the next in daisy-chained fashion"), so among devices at the same level the
  electrically-nearest one wins.
- On grant, the device puts an interrupt **vector** on the bus (Handbook
  p.269: "The BUS INTR L line is asserted by the master to indicate that it
  has placed the address of an interrupt vector on the bus. The processor
  will respond … and will interrupt through that vector."). The vector is a
  2-word block in low memory: word 0 is the new PC, word 1 is the new PS
  (Handbook p.216: "The first word contains the interrupt service routine
  entry address (the new PC), and the second word contains the new PS").
  The new PS sets the new processor priority (PSW bits 5:7), which is how an
  ISR masks same-or-lower-level interrupts while it runs (the `spl` mechanism;
  the program polls the Done/Ready flag bit 7 and arms the Interrupt Enable
  bit in the CSR — Handbook p.215).

### RVVM realization

RVVM has no Unibus arbitration hardware and the RISC-V hart has a single
external-interrupt line, so the bus models arbitration in software:

1. `unibus_raise_irq(dev)` records the device's `(BR level, vector)` as a
   pending request and raises the **one** RISC-V external IRQ on the machine
   intc (it stays asserted while any request is pending).
   `unibus_lower_irq(dev)` clears it.
2. The bus exposes an **interrupt-acknowledge (IAK)** register pair in its own
   MMIO space (offsets within the I/O-page region):
   - `UNIBUS_IAK_PRI` (0x1FF0, **write**): the dispatcher writes its current
     PDP-11 processor priority `P` (PSW bits 5:7, 0..7).
   - `UNIBUS_IAK_VEC` (0x1FF2, **read**): returns the vector latched by the
     preceding PRI write.

   A PRI write performs the grant: it selects the highest-BR-level pending
   device whose level is **strictly greater than P**, atomically clears that
   one request (the grant; ties broken by attach order = daisy-chain
   nearest-wins), and latches its vector. If nothing outranks P, the latched
   vector is 0.

   The same grant is available programmatically as
   `unibus_ack(bus, prio)` (returns the vector or 0), for host code or an
   alternative dispatcher.

### What the port's mtvec/spl dispatcher must do

On taking the RISC-V external interrupt:

```
loop:
    write P  -> UNIBUS_IAK_PRI      ; P = current PDP-11 priority (PSW 5:7)
    read  vec <- UNIBUS_IAK_VEC
    if vec == 0: done               ; spurious / nothing outranks P
    newPC = mem[MB + vec + 0]        ; vector word 0 (Handbook p.216)
    newPS = mem[MB + vec + 2]        ; vector word 1 -> new priority (spl)
    push old PC, old PS; set PSW from newPS; jump newPC
    ; ISR runs at the new (higher) priority, acks its device's Done bit,
    ; then RTI restores the old PC/PS, lowering priority again
    goto loop                        ; service any remaining pending request
```

The bus only enforces "level must be > P" at grant time and keeps the RISC-V
line asserted while requests remain; honoring the new PS priority (spl) and
the 2-word vector fetch is the dispatcher's job, exactly as the real
processor does it.

## NPR / DMA model

Block devices move data between their backing store and the window RAM
without processor involvement — the Unibus Non-Processor Request path
(Handbook p.268: "The BUS NPR L line is used by a peripheral to request the
data section of the bus for a Direct Memory Access (DMA) transfer. (The
acronym NPR stands for Non Processor Request.)").

- `unibus_dma_read(dev, pdp_addr, buf, len)` — copy from window RAM at
  `MB + pdp_addr` into a host buffer.
- `unibus_dma_write(dev, pdp_addr, buf, len)` — copy a host buffer into
  window RAM at `MB + pdp_addr`.
- `unibus_dma_ptr(dev, pdp_addr, len)` — direct host pointer into window RAM
  for zero-copy transfers.

There is no bus-master enable gate (unlike PCI): NPR is always available,
matching the Unibus where any device may arbitrate for NPR.

## Per-device register / vector / BR-level table

Addresses are the kernel's I/O-page octal addresses
(`unix-v1-on-rvvm/port/include/io.inc`); levels and vectors are from the
kernel's low-memory vector table (`unix-v1-on-rvvm/build/u0.s`). Handbook
Appendix A fixed assignments are noted where they differ; the **kernel's
table is the binding contract** honored by the code.

| Device | Register(s) (octal) | BR level | Vector (octal) | Status | Handbook ref |
|--------|---------------------|----------|----------------|--------|--------------|
| KW11-L line clock | `lks` 0177546 | BR7 (level 340) | 064 | **working** | App.A p.A-1 (CSR 777546), p.A-5 (App.A fixed vector 100; kernel uses 064); LTC p.271 |
| KL11/DL11 console RX | `tks` 0177560 / `tkb` 0177562 | BR5 (level 240) | 060 | **working** | App.A p.A-1 ("DL/DLV11 777560 … console"); Done/IE p.215 |
| KL11/DL11 console TX | `tps` 0177564 / `tpb` 0177566 | BR5 (level 240) | 064 | **working** | App.A p.A-1; vector pass p.269 |
| RF11/RS11 drum | `dcs` 0177460, `wc` 0177462, `cma` 0177464, `dar` 0177466, `dae` 0177470 | BR6 (level 300) | 0204 | **working** | NPR/DMA p.268; CSR family App.A p.A-1 |
| RK11 disk | `rkds` 0177400 … `rkda` 0177412 | BR6 (level 300) | 0214 | *not implemented* | App.A p.A-1 |
| TC11 DECtape | `tcst` 0177340 … `tcdt` 0177350 | BR6 (level 300) | 0214 | *not implemented* | App.A p.A-1 |
| DC11 serial lines | 0174000.. | BR5 | 0300.. | *not implemented* | App.A p.A-1 ("DC11 774000") |

Notes:
- The KL11 console is two cooperating Unibus device endpoints (receiver and
  transmitter) sharing one state object, because it has two interrupt sources
  with distinct vectors (060 RX, 064 TX) — each Unibus device endpoint
  carries exactly one (level, vector) pair.
- The KW11-L's kernel vector 064 collides numerically with the console TX
  vector 064; this is the kernel's own table (they are distinguished by which
  device requests). The code reproduces the kernel table verbatim.

## How to add a new device

1. Allocate a private state struct and fill a `unibus_dev_desc_t`:
   - `io_addr` (window address, e.g. `0xFF66` for `lks = 0177546`), `size`.
   - `read`/`write` 16-bit register handlers (`off` is relative to `io_addr`;
     set `min_size = 1` to also accept byte access).
   - optional `poll` (called from the bus event-thread for time- or
     chardev-driven work) and `cleanup` (free auxiliary resources).
   - `br_level` (`UNIBUS_BR4..UNIBUS_BR7`) and `vector`.
   - `data` (owned by the bus, freed on teardown unless `shared_data`).
2. `unibus_attach(bus, &desc)`.
3. Use `unibus_raise_irq(dev)` / `unibus_lower_irq(dev)` for interrupts, and
   `unibus_dma_read/write/ptr()` for NPR transfers.
4. Add a constructor prototype to `unibus-devices.h`. See `unibus-kw11l.c`
   (clock), `unibus-kl11.c` (console, multi-vector) and `unibus-rf11.c`
   (drum, NPR DMA) as worked examples.

## Build & smoke test

The build globs `src/**/*.c`, so the four new device sources are picked up
automatically by both CMake and the Makefile — no build-file edits needed.
The library (`make USE_LIB=1 lib`) and the binary (`make`) both stay green.

The smoke test links against `librvvm` using only the public ABI:

```sh
make USE_LIB=1 lib
cc -Iinclude tests/unibus_smoke.c -Lrelease.<os>.<arch> -lrvvm \
   -Wl,-rpath,@executable_path/release.<os>.<arch> -o /tmp/unibus_smoke
DYLD_LIBRARY_PATH=release.<os>.<arch> /tmp/unibus_smoke   # (LD_LIBRARY_PATH on Linux)
```

It runs two scenarios:

1. **Clock + IAK contract.** A machine with a recording interrupt controller,
   the bus, and the KW11-L clock. A tiny RV64I firmware enables the clock
   interrupt **over real MMIO** (writing the IE bit to `lks`); the test forces
   one line tick and verifies the bus asserts the RISC-V external line, that
   `unibus_ack(P=0)` returns vector 064 while `unibus_ack(P=7)` returns 0, and
   that the grant clears the request and drops the line — the full IAK /
   BR-priority contract. The firmware encodings are the verified output of
   `zig cc -target riscv64-freestanding -mcpu=generic_rv64`.

2. **RF11 NPR/DMA.** A machine based at physical 0 with RAM sized to exactly
   the window-RAM region (`0..0xDFFF`, so it does not overlap the I/O page at
   `0xE000`). The test round-trips a 512-byte block through the drum's
   `unibus_dma_read`/`unibus_dma_write` helpers and checks the data survives,
   exercising the NPR transport end to end.

All 13 checks pass.

## Handbook pages cited

- Ch.8 "Traps and Interrupts" pp.215–216 — Done/Ready flag + Interrupt
  Enable; the 2-word interrupt vector (new PC, new PS).
- Ch.9 "Mapping to Memory and Busses" p.258 (fig.9-37) — the I/O page at the
  top of the address space.
- Ch.10 "PDP-11 Bus Structures" p.266 — top 8 Kbytes reserved for I/O;
  p.267 — daisy-chained bus grants; p.268 — BR4..BR7 request lines and NPR
  (DMA); p.269 — DATO/DATOB transfers and the BUS INTR vector pass; p.271 —
  the LTC line-time-clock pulse; p.272 — bus errors / time-outs.
- Appendix A "Assignment of Bus Addresses and Vectors" p.A-1 (device I/O-page
  addresses) and p.A-5 (interrupt vectors).
