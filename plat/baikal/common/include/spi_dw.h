/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SPI_DW_H
#define SPI_DW_H

#include <stddef.h>
#include <stdint.h>

/* flash_info */
#define SECTOR_SIZE		(64 * 1024)
#define SECTOR_CNT		256
#define SPI_NOR_MAX_ID_LEN	3

struct flash_info {
	char *name;
	uint8_t id[SPI_NOR_MAX_ID_LEN];
	uint32_t sector_size;
	uint32_t n_sectors;
};

void dw_spi_init  (int port, int line);
int  dw_spi_read  (int port, int line, uint32_t adr, void *data, size_t size);
int  dw_spi_erase (int port, int line, uint32_t adr, size_t size, size_t sectore_size);
int  dw_spi_write (int port, int line, uint32_t adr, void *data, size_t size);
int  dw_spi_4byte (int port, int line);
int  dw_spi_3byte (int port, int line);
const struct flash_info *dw_spi_get_info(int port, int line);

#endif /* SPI_DW_H */
