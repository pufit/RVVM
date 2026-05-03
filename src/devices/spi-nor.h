/*
spi-nor.h - Generic JEDEC SPI NOR flash slave (Winbond W25Q-family compatible)
Copyright (C) 2026  Sol Astrius <sol@astrius.ink>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_SPI_NOR_H
#define RVVM_SPI_NOR_H

#include "rvvmlib.h"
#include "spi-bus.h"

/*
 * Emulates a Winbond W25Q-family SPI NOR flash. Backed by a host file:
 * the file is auto-created (and pre-filled with 0xFF — erased state) if
 * absent or smaller than the chip capacity. Linux's drivers/mtd/spi-nor
 * binds against the Winbond table entry matching the JEDEC ID we emit;
 * mtdblock / sysfs attributes / dd / mkfs.jffs2 all just work.
 *
 * Capacity selects the chip variant (and JEDEC ID byte 3):
 *   2 MB  → W25Q16  (id 0xEF 0x40 0x15)
 *   4 MB  → W25Q32  (id 0xEF 0x40 0x16)
 *   8 MB  → W25Q64  (id 0xEF 0x40 0x17)  ← default
 *   16 MB → W25Q128 (id 0xEF 0x40 0x18)
 *   32 MB → W25Q256 (id 0xEF 0x40 0x19)  (still 24-bit addressing here;
 *                                          kernel falls back to 24-bit)
 *
 * Pass `size_bytes = 0` to default to 8 MB.
 */

//! \brief  Attach a SPI NOR flash to a bus.
//! \param  bus         Controller's spi_bus_t (e.g. from spi_sifive_get_bus)
//! \param  cs_id       Chip-select index, or SPI_AUTO_CS to pick lowest free
//! \param  image_path  Host file backing the flash (created if absent)
//! \param  size_bytes  Capacity. 0 → 8 MB. Anything not in the table above
//!                     rounds up to the next supported size; >32 MB clamps.
//! \return Assigned CS, or SPI_AUTO_CS on failure (file open fail / etc.)
PUBLIC uint16_t spi_nor_attach(spi_bus_t* bus, uint16_t cs_id,
                               const char* image_path, uint64_t size_bytes);

#endif
