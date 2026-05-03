/*
mcp251x.c - Microchip MCP2515/MCP25625 CAN controller (SPI slave)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "mcp251x.h"
#include "compiler.h"
#include "fdtlib.h"
#include "spinlock.h"
#include "utils.h"

#include <string.h>

PUSH_OPTIMIZATION_SIZE

// MCP2515 datasheet (DS21801G). Linux: drivers/net/can/spi/mcp251x.c.

// ─── SPI opcodes (§12) ──────────────────────────────────────────────────────
#define OP_RESET              0xC0
#define OP_READ               0x03
#define OP_WRITE              0x02
#define OP_BIT_MODIFY         0x05
#define OP_LOAD_TXB0_SIDH     0x40
#define OP_LOAD_TXB1_SIDH     0x42
#define OP_LOAD_TXB2_SIDH     0x44
#define OP_RTS_BASE           0x80   // 0x80 | (mask & 0x07)
#define OP_RTS_MASK           0xF8
#define OP_READ_RXB0_SIDH     0x90
#define OP_READ_RXB1_SIDH     0x94

// ─── Register layout (§11) ──────────────────────────────────────────────────
// We keep a flat 128-byte register file. Most addresses are pure storage;
// only the ones below have semantic effects.
#define REG_CANSTAT           0x0E
#define REG_CANCTRL           0x0F
#define REG_CANINTE           0x2B
#define REG_CANINTF           0x2C
#define REG_TXBCTRL(n)        (0x30 + (n) * 0x10)
#define REG_TXBSIDH(n)        (REG_TXBCTRL(n) + 1)
#define REG_RXBCTRL(n)        (0x60 + (n) * 0x10)
#define REG_RXBSIDH(n)        (REG_RXBCTRL(n) + 1)

// CANCTRL bits
#define CANCTRL_REQOP_MASK     0xE0
#define CANCTRL_REQOP_NORMAL   0x00
#define CANCTRL_REQOP_SLEEP    0x20
#define CANCTRL_REQOP_LOOPBACK 0x40
#define CANCTRL_REQOP_LISTEN   0x60
#define CANCTRL_REQOP_CONF     0x80
#define CANCTRL_ABAT           0x10

// CANSTAT bits
#define CANSTAT_OPMOD_MASK    0xE0

// CANINTF bits
#define CANINTF_RX0IF         0x01
#define CANINTF_RX1IF         0x02
#define CANINTF_TX0IF         0x04

// TXBnCTRL bits
#define TXBCTRL_TXREQ         0x08

// RXBnCTRL bits
#define RXBCTRL_BUKT          0x04

// Power-on defaults. The Linux probe checks (CANCTRL & 0x17) == 0x07,
// which corresponds to CLKEN=1 + CLKPRE=11 (the reset value). CANSTAT's
// upper 3 bits mirror CANCTRL.REQOP — Configuration mode after reset.
#define POR_CANCTRL           0x87
#define POR_CANSTAT           0x80

#define MCP_REGS_SIZE         128

// Length of the LOAD_TXB / READ_RXB byte stream after the opcode
// (SIDH..DLC + 8 data = 13 bytes).
#define BUF_DATA_LEN          13

typedef struct {
    rvvm_intc_t* intc;
    rvvm_irq_t   irq;
    can_bus_t*   can_bus;   // NULL = loopback-only / silent NORMAL TX
    spinlock_t   lock;

    uint8_t      regs[MCP_REGS_SIZE];
    bool         irq_level;

    // Per-CS-low session state — reset on falling CS, finalized on rising CS.
    bool         cs_low;
    uint8_t      cmd;
    uint32_t     byte_idx;
    uint8_t      addr;       // for READ / WRITE / BIT_MODIFY
    uint8_t      mod_mask;   // for BIT_MODIFY
    uint8_t      buf_idx;    // 0..2 for LOAD_TXB; 0..1 for READ_RXB
} mcp251x_t;

// CANSTAT.OPMOD must mirror CANCTRL.REQOP. The Linux driver polls
// (CANSTAT & 0xE0) == requested-mode after every mode change, so the
// reflection has to happen synchronously on the CANCTRL write.
static void update_canstat_opmod(mcp251x_t* mcp)
{
    uint8_t opmod = mcp->regs[REG_CANCTRL] & CANCTRL_REQOP_MASK;
    mcp->regs[REG_CANSTAT] = (mcp->regs[REG_CANSTAT] & ~CANSTAT_OPMOD_MASK) | opmod;
}

static void mcp_reset_state(mcp251x_t* mcp)
{
    memset(mcp->regs, 0, sizeof(mcp->regs));
    mcp->regs[REG_CANCTRL] = POR_CANCTRL;
    mcp->regs[REG_CANSTAT] = POR_CANSTAT;
}

// Returns true if irq_level changed and apply_irq must be called.
static bool mcp_recompute_irq(mcp251x_t* mcp)
{
    bool desired = (mcp->regs[REG_CANINTF] & mcp->regs[REG_CANINTE]) != 0;
    if (desired != mcp->irq_level) {
        mcp->irq_level = desired;
        return true;
    }
    return false;
}

// Caller must NOT hold mcp->lock — raise/lower may grab the intc's lock.
static void mcp_apply_irq(mcp251x_t* mcp)
{
    if (mcp->irq_level) {
        rvvm_raise_irq(mcp->intc, mcp->irq);
    } else {
        rvvm_lower_irq(mcp->intc, mcp->irq);
    }
}

// Pick a free RX buffer for an incoming loopback frame. Caller holds lock.
static int8_t mcp_pick_rxb(mcp251x_t* mcp)
{
    if (!(mcp->regs[REG_CANINTF] & CANINTF_RX0IF)) return 0;
    // RXB0 full — roll into RXB1 if BUKT is set (driver enables it in setup).
    if ((mcp->regs[REG_RXBCTRL(0)] & RXBCTRL_BUKT) &&
        !(mcp->regs[REG_CANINTF] & CANINTF_RX1IF)) {
        return 1;
    }
    return -1;  // overflow — drop frame
}

// Decode a TXBn/RXBn 13-byte buffer (SIDH..DLC + 8 data) into a
// SocketCAN-flavored can_frame_t. Bit positions per Linux's mcp251x_hw_tx
// (mirror image of mcp251x_hw_rx).
static void mcp_buf_to_frame(const uint8_t buf[BUF_DATA_LEN], can_frame_t* f)
{
    uint8_t sidh = buf[0];
    uint8_t sidl = buf[1];
    uint8_t eid8 = buf[2];
    uint8_t eid0 = buf[3];
    uint8_t dlcb = buf[4];

    if (sidl & 0x08) {
        // Extended: SID[10:0] = (SIDH << 3) | (SIDL >> 5). EID[17:0] =
        // ((SIDL & 3) << 16) | (EID8 << 8) | EID0. can_id = SID<<18|EID.
        uint32_t sid = ((uint32_t)sidh << 3) | ((uint32_t)sidl >> 5);
        uint32_t eid = ((uint32_t)(sidl & 0x03) << 16)
                     | ((uint32_t)eid8 << 8) | eid0;
        f->can_id = ((sid << 18) | eid) | CAN_EFF_FLAG;
        if (dlcb & 0x40) f->can_id |= CAN_RTR_FLAG;
    } else {
        f->can_id = ((uint32_t)sidh << 3) | ((uint32_t)sidl >> 5);
        // For standard frames the TX RTR lives in DLC[6] (TXBnDLC.RTR).
        if (dlcb & 0x40) f->can_id |= CAN_RTR_FLAG;
    }
    f->dlc = dlcb & 0x0F;
    if (f->dlc > CAN_FRAME_MAX_DATA) f->dlc = CAN_FRAME_MAX_DATA;
    memcpy(f->data, &buf[5], f->dlc);
}

// Encode a frame into RXBn buffer layout (SIDH..DLC + 8 data). RXBnSIDL
// uses bit 4 (SRR) for standard-frame RTR and bit 6 of RXBnDLC for
// extended-frame RTR — mirrors what the Linux RX decode expects.
static void mcp_frame_to_rxbuf(uint8_t buf[BUF_DATA_LEN], const can_frame_t* f)
{
    memset(buf, 0, BUF_DATA_LEN);
    if (f->can_id & CAN_EFF_FLAG) {
        uint32_t id  = f->can_id & CAN_EFF_MASK;
        uint32_t sid = (id >> 18) & 0x7FF;
        uint32_t eid = id & 0x3FFFFu;
        buf[0] = (uint8_t)(sid >> 3);
        buf[1] = (uint8_t)(((sid & 0x07) << 5) | 0x08 | ((eid >> 16) & 0x03));
        buf[2] = (uint8_t)((eid >> 8) & 0xFF);
        buf[3] = (uint8_t)(eid & 0xFF);
        if (f->can_id & CAN_RTR_FLAG) buf[4] = 0x40;  // RXBnDLC.RTR
    } else {
        uint32_t sid = f->can_id & CAN_SFF_MASK;
        buf[0] = (uint8_t)(sid >> 3);
        buf[1] = (uint8_t)((sid & 0x07) << 5);
        if (f->can_id & CAN_RTR_FLAG) buf[1] |= 0x10;  // RXBnSIDL.SRR
    }
    uint8_t len = f->dlc > CAN_FRAME_MAX_DATA ? CAN_FRAME_MAX_DATA : f->dlc;
    buf[4] = (uint8_t)((buf[4] & 0xF0) | (len & 0x0F));
    memcpy(&buf[5], f->data, len);
}

// Fire TXBn: in LOOPBACK mode deliver locally; in NORMAL mode snapshot
// the frame for outside-lock broadcast (caller does the actual fan-out
// after dropping mcp->lock — re-entering bus_broadcast under the lock
// would deadlock against an echo responder bouncing the frame back).
static void mcp_fire_txb(mcp251x_t* mcp, uint8_t n,
                         can_frame_t* pending, uint8_t* pending_count)
{
    uint8_t mode = mcp->regs[REG_CANCTRL] & CANCTRL_REQOP_MASK;

    if (mode == CANCTRL_REQOP_LOOPBACK) {
        int8_t rxn = mcp_pick_rxb(mcp);
        if (rxn >= 0) {
            // Layout matches TXB at this offset, so a memcpy is enough.
            memcpy(&mcp->regs[REG_RXBSIDH(rxn)],
                   &mcp->regs[REG_TXBSIDH(n)],
                   BUF_DATA_LEN);
            mcp->regs[REG_CANINTF] |= (rxn == 0) ? CANINTF_RX0IF : CANINTF_RX1IF;
        }
    } else if (mode == CANCTRL_REQOP_NORMAL && mcp->can_bus) {
        if (*pending_count < 3) {
            mcp_buf_to_frame(&mcp->regs[REG_TXBSIDH(n)], &pending[*pending_count]);
            (*pending_count)++;
        }
    }
    // LISTEN_ONLY / CONFIG / SLEEP: silently drop. NORMAL without a bus:
    // also silently dropped (preserves single-chip "TX into the void"
    // behavior so the driver still sees TX completion).
    mcp->regs[REG_TXBCTRL(n)] &= (uint8_t)~TXBCTRL_TXREQ;
    mcp->regs[REG_CANINTF]    |= (uint8_t)(CANINTF_TX0IF << n);
}

// Apply rising-CS effects: RESET, RTS fan-out, and READ_RXB auto-clear of
// the corresponding RXnIF (documented MCP2515 behavior — driver relies on
// it for the MCP2515/25625 path; only the ancient MCP2510 manually clears).
//
// `pending`/`pending_count` collect any NORMAL-mode TX frames; the caller
// drops the lock and broadcasts them after this returns.
static void mcp_finalize_session(mcp251x_t* mcp, can_frame_t* pending,
                                 uint8_t* pending_count)
{
    if (mcp->cmd == OP_RESET) {
        mcp_reset_state(mcp);
        return;
    }
    if (mcp->cmd == OP_READ_RXB0_SIDH) {
        mcp->regs[REG_CANINTF] &= (uint8_t)~CANINTF_RX0IF;
        return;
    }
    if (mcp->cmd == OP_READ_RXB1_SIDH) {
        mcp->regs[REG_CANINTF] &= (uint8_t)~CANINTF_RX1IF;
        return;
    }
    if ((mcp->cmd & OP_RTS_MASK) == OP_RTS_BASE) {
        uint8_t mask = mcp->cmd & 0x07;
        for (uint8_t i = 0; i < 3; i++) {
            if (mask & (1u << i)) mcp_fire_txb(mcp, i, pending, pending_count);
        }
        return;
    }
}

static void mcp251x_select(void* dev, bool asserted)
{
    mcp251x_t*  mcp = dev;
    bool        irq_changed;
    can_frame_t pending[3];
    uint8_t     pending_count = 0;

    spin_lock(&mcp->lock);
    if (asserted) {
        mcp->cs_low   = true;
        mcp->byte_idx = 0;
        mcp->cmd      = 0xFF;
    } else if (mcp->cs_low) {
        mcp->cs_low = false;
        mcp_finalize_session(mcp, pending, &pending_count);
    }
    irq_changed = mcp_recompute_irq(mcp);
    spin_unlock(&mcp->lock);

    if (irq_changed) mcp_apply_irq(mcp);

    // Broadcast any NORMAL-mode TX outside the lock. An echo responder
    // bouncing a frame back will re-enter our rx callback, which takes
    // mcp->lock — must not be held here.
    for (uint8_t i = 0; i < pending_count; i++) {
        can_bus_broadcast(mcp->can_bus, mcp, &pending[i]);
    }
}

// Bus-side RX callback: a frame arrived from another node. Place it in
// the next free RXBn (RXB0 first, RXB1 if BUKT and RXB0 full); set
// RXnIF; raise IRQ. Frames received while the chip is in CONFIG or
// SLEEP are dropped — real silicon doesn't sample the bus there.
static void mcp251x_can_rx(void* dev, const can_frame_t* frame)
{
    mcp251x_t* mcp = dev;
    bool       irq_changed = false;

    spin_lock(&mcp->lock);
    uint8_t mode = mcp->regs[REG_CANCTRL] & CANCTRL_REQOP_MASK;
    if (mode != CANCTRL_REQOP_CONF && mode != CANCTRL_REQOP_SLEEP) {
        int8_t rxn = mcp_pick_rxb(mcp);
        if (rxn >= 0) {
            mcp_frame_to_rxbuf(&mcp->regs[REG_RXBSIDH(rxn)], frame);
            mcp->regs[REG_CANINTF] |= (rxn == 0) ? CANINTF_RX0IF : CANINTF_RX1IF;
        }
        // Else: overflow — real silicon would set EFLG.RXnOVR. Skipped
        // here; driver only flags it for stats and we don't model bus
        // saturation pressure.
        irq_changed = mcp_recompute_irq(mcp);
    }
    spin_unlock(&mcp->lock);

    if (irq_changed) mcp_apply_irq(mcp);
}

static uint8_t mcp251x_transfer(void* dev, uint8_t tx)
{
    mcp251x_t* mcp = dev;
    uint8_t    out = 0xFF;
    bool       irq_changed = false;

    spin_lock(&mcp->lock);

    if (mcp->byte_idx == 0) {
        mcp->cmd = tx;
        // Stash buffer index for the few opcodes that need it later.
        switch (tx) {
            case OP_LOAD_TXB0_SIDH: mcp->buf_idx = 0; break;
            case OP_LOAD_TXB1_SIDH: mcp->buf_idx = 1; break;
            case OP_LOAD_TXB2_SIDH: mcp->buf_idx = 2; break;
            case OP_READ_RXB0_SIDH: mcp->buf_idx = 0; break;
            case OP_READ_RXB1_SIDH: mcp->buf_idx = 1; break;
            default: break;
        }
        mcp->byte_idx = 1;
        spin_unlock(&mcp->lock);
        return 0xFF;
    }

    uint32_t i = mcp->byte_idx - 1;  // 0-based index into the post-opcode stream
    mcp->byte_idx++;

    switch (mcp->cmd) {
        case OP_READ:
            // byte 0 = address. Subsequent bytes shift out reg[addr++],
            // wrapping inside the 128-byte register file.
            if (i == 0) {
                mcp->addr = tx;
            } else {
                out = mcp->regs[(mcp->addr + (i - 1)) & 0x7F];
            }
            break;

        case OP_WRITE:
            if (i == 0) {
                mcp->addr = tx;
            } else {
                uint8_t a = (uint8_t)((mcp->addr + (i - 1)) & 0x7F);
                mcp->regs[a] = tx;
                if (a == REG_CANCTRL) {
                    update_canstat_opmod(mcp);
                    // ABAT auto-clears: nothing to abort in this model.
                    mcp->regs[REG_CANCTRL] &= (uint8_t)~CANCTRL_ABAT;
                }
                if (a == REG_CANINTE || a == REG_CANINTF) {
                    irq_changed = mcp_recompute_irq(mcp);
                }
            }
            break;

        case OP_BIT_MODIFY:
            // 3 args: addr, mask, val. Apply RMW on the val byte.
            if (i == 0) {
                mcp->addr = tx;
            } else if (i == 1) {
                mcp->mod_mask = tx;
            } else if (i == 2) {
                uint8_t a = mcp->addr & 0x7F;
                mcp->regs[a] = (uint8_t)((mcp->regs[a] & ~mcp->mod_mask) |
                                         (tx & mcp->mod_mask));
                if (a == REG_CANCTRL) {
                    update_canstat_opmod(mcp);
                    mcp->regs[REG_CANCTRL] &= (uint8_t)~CANCTRL_ABAT;
                }
                if (a == REG_CANINTE || a == REG_CANINTF) {
                    irq_changed = mcp_recompute_irq(mcp);
                }
            }
            break;

        case OP_READ_RXB0_SIDH:
        case OP_READ_RXB1_SIDH: {
            // Stream out 13 bytes starting at RXBnSIDH (offset 1 of buffer).
            uint8_t base = REG_RXBSIDH(mcp->buf_idx);
            if (i < BUF_DATA_LEN) {
                out = mcp->regs[(uint8_t)((base + i) & 0x7F)];
            }
            break;
        }

        case OP_LOAD_TXB0_SIDH:
        case OP_LOAD_TXB1_SIDH:
        case OP_LOAD_TXB2_SIDH: {
            // Mirror of READ_RXB but for writes, into TXBnSIDH..DAT[7].
            uint8_t base = REG_TXBSIDH(mcp->buf_idx);
            if (i < BUF_DATA_LEN) {
                mcp->regs[(uint8_t)((base + i) & 0x7F)] = tx;
            }
            break;
        }

        // OP_RESET / OP_RTS_BASE..OP_RTS_BASE+7 / READ_STATUS / RX_STATUS:
        // no further byte-level effect — finalized on rising CS. Driver
        // never reads the response bytes for these.
        default: break;
    }

    spin_unlock(&mcp->lock);

    if (irq_changed) mcp_apply_irq(mcp);

    return out;
}

static void mcp251x_remove(void* dev)
{
    free(dev);
}

PUBLIC uint16_t mcp251x_attach(spi_bus_t* spi_bus, uint16_t cs_id,
                               rvvm_intc_t* intc, rvvm_irq_t irq,
                               can_bus_t* can_bus)
{
    if (!spi_bus || !intc) return SPI_AUTO_CS;

    mcp251x_t* mcp = safe_new_obj(mcp251x_t);
    mcp->intc      = intc;
    mcp->irq       = irq;
    mcp->can_bus   = can_bus;
    mcp_reset_state(mcp);

    spi_dev_t desc = {
        .cs_id    = cs_id,
        .data     = mcp,
        .select   = mcp251x_select,
        .transfer = mcp251x_transfer,
        .remove   = mcp251x_remove,
    };

    uint16_t assigned = spi_attach_dev(spi_bus, &desc);
    if (assigned == SPI_AUTO_CS) {
        rvvm_warn("mcp251x: bus attach failed");
        free(mcp);
        return SPI_AUTO_CS;
    }

    if (can_bus) {
        // Register on the CAN segment for incoming frames. The chip's
        // private data is owned by the SPI slave entry above, so the
        // CAN-side `remove` is a no-op; it just keeps the can_dev_t
        // shape symmetric.
        can_dev_t can_desc = {
            .data   = mcp,
            .rx     = mcp251x_can_rx,
            .remove = NULL,
        };
        can_bus_attach(can_bus, &can_desc);
    }

#ifdef USE_FDT
    struct fdt_node* parent = spi_bus_fdt_node(spi_bus);
    if (parent) {
        // Linux's mcp251x driver requires either a `clocks` reference or a
        // `clock-frequency` property in the 1–25 MHz range. Inline frequency
        // sidesteps the need for a shared clock-source node.
        struct fdt_node* node = fdt_node_create_reg("can", assigned);
        fdt_node_add_prop_u32(node, "reg", assigned);
        fdt_node_add_prop_str(node, "compatible", "microchip,mcp2515");
        fdt_node_add_prop_u32(node, "spi-max-frequency", 10000000);
        fdt_node_add_prop_u32(node, "clock-frequency", 16000000);
        rvvm_fdt_describe_irq(node, intc, irq);
        fdt_node_add_prop_str(node, "status", "okay");
        fdt_node_add_child(parent, node);
    }
#endif

    rvvm_info("mcp251x: attached on CS%u (irq=%u)",
              (unsigned)assigned, (unsigned)irq);
    return assigned;
}

POP_OPTIMIZATION_SIZE
