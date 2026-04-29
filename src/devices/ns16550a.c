/*
ns16550a.c - NS16550A UART
Copyright (C) 2021  LekKit <github.com/LekKit>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "ns16550a.h"
#include "chardev.h"
#include "compiler.h"
#include "fdtlib.h"
#include "uart16550_core.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

#define NS16550A_MMIO_SIZE 0x1000

typedef struct {
    uart16550_core_t core;
    rvvm_intc_t*     intc;
    rvvm_irq_t       irq;
} ns16550a_dev_t;

static void ns16550a_irq(void* ctx, bool level)
{
    ns16550a_dev_t* uart = ctx;
    if (level) {
        rvvm_raise_irq(uart->intc, uart->irq);
    } else {
        rvvm_lower_irq(uart->intc, uart->irq);
    }
}

static bool ns16550a_mmio_read(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    ns16550a_dev_t* uart = dev->data;
    UNUSED(size);
    *(uint8_t*)data = uart16550_core_read(&uart->core, offset);
    return true;
}

static bool ns16550a_mmio_write(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    ns16550a_dev_t* uart = dev->data;
    UNUSED(size);
    uart16550_core_write(&uart->core, offset, *(uint8_t*)data);
    return true;
}

static void ns16550a_update(rvvm_mmio_dev_t* dev)
{
    ns16550a_dev_t* uart = dev->data;
    uart16550_core_update(&uart->core);
}

static void ns16550a_remove(rvvm_mmio_dev_t* dev)
{
    ns16550a_dev_t* uart = dev->data;
    uart16550_core_cleanup(&uart->core);
    free(uart);
}

static const rvvm_mmio_type_t ns16550a_dev_type = {
    .name   = "ns16550a",
    .update = ns16550a_update,
    .remove = ns16550a_remove,
};

PUBLIC rvvm_mmio_dev_t* ns16550a_init(rvvm_machine_t* machine, chardev_t* chardev, rvvm_addr_t addr, rvvm_intc_t* intc,
                                      rvvm_irq_t irq)
{
    ns16550a_dev_t* uart = safe_new_obj(ns16550a_dev_t);
    uart->intc           = intc;
    uart->irq            = irq;
    uart16550_core_init(&uart->core, chardev, ns16550a_irq, uart);

    rvvm_mmio_dev_t ns16550a = {
        .addr        = addr,
        .size        = NS16550A_MMIO_SIZE,
        .data        = uart,
        .type        = &ns16550a_dev_type,
        .read        = ns16550a_mmio_read,
        .write       = ns16550a_mmio_write,
        .min_op_size = 1,
        .max_op_size = 1,
    };

    rvvm_mmio_dev_t* mmio = rvvm_attach_mmio(machine, &ns16550a);
    if (mmio == NULL) {
        return mmio;
    }

#ifdef USE_FDT
    struct fdt_node* uart_fdt = fdt_node_create_reg("uart", ns16550a.addr);
    fdt_node_add_prop_reg(uart_fdt, "reg", ns16550a.addr, ns16550a.size);
    fdt_node_add_prop_str(uart_fdt, "compatible", "ns16550a");
    fdt_node_add_prop_u32(uart_fdt, "clock-frequency", 20000000);
    fdt_node_add_prop_u32(uart_fdt, "fifo-size", 16);
    fdt_node_add_prop_str(uart_fdt, "status", "okay");
    rvvm_fdt_describe_irq(uart_fdt, intc, irq);
    fdt_node_add_prop(uart_fdt, "wakeup-source", NULL, 0);
    fdt_node_add_child(rvvm_get_fdt_soc(machine), uart_fdt);
#endif
    return mmio;
}

PUBLIC rvvm_mmio_dev_t* ns16550a_init_auto(rvvm_machine_t* machine, chardev_t* chardev)
{
    rvvm_intc_t*     intc = rvvm_get_intc(machine);
    rvvm_addr_t      addr = rvvm_mmio_zone_auto(machine, NS16550A_ADDR_DEFAULT, NS16550A_MMIO_SIZE);
    rvvm_mmio_dev_t* mmio = ns16550a_init(machine, chardev, addr, intc, rvvm_alloc_irq(intc));
    if (addr == NS16550A_ADDR_DEFAULT && mmio) {
        rvvm_append_cmdline(machine, "console=ttyS");
#ifdef USE_FDT
        struct fdt_node* chosen = fdt_node_find(rvvm_get_fdt_root(machine), "chosen");
        fdt_node_add_prop_str(chosen, "stdout-path", "/soc/uart@10000000");
#endif
    }
    return mmio;
}

PUBLIC rvvm_mmio_dev_t* ns16550a_init_term_auto(rvvm_machine_t* machine)
{
    return ns16550a_init_auto(machine, chardev_term_create());
}

POP_OPTIMIZATION_SIZE
