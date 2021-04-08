/*
 * Copyright (c) 2020-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <baikal_sip_svc_flash.h>
#include <common/debug.h>
#include <spi_dw.h>

static uint8_t flash_buf[1024] __aligned(8);
static unsigned int flash_buf_idx;
static uint32_t sector_size;
static uint32_t n_sectors;

static int baikal_smc_flash_position(uint64_t x1)
{
	if (x1 > (sizeof(flash_buf) - 4 * sizeof(u_register_t))) {
		return -1;
	}

	flash_buf_idx = x1;
	return 0;
}

static int baikal_smc_flash_push(uint64_t x1,
				 uint64_t x2,
				 uint64_t x3,
				 uint64_t x4)
{
	uint64_t *buf = (void *)&flash_buf[flash_buf_idx];

	buf[0] = x1;
	buf[1] = x2;
	buf[2] = x3;
	buf[3] = x4;
	flash_buf_idx += 4 * sizeof(buf[0]);

	return 0;
}

static int baikal_smc_flash_pull(uint64_t *data)
{
	int i;
	uint64_t *buf = (void *)&flash_buf[flash_buf_idx];

	for (i = 0; i < 4; i++) {
		data[i] = buf[i];
	}

	flash_buf_idx += 4 * sizeof(uint64_t);
	return 0;
}

static int baikal_smc_flash_info(uint64_t *data)
{
	const struct flash_info *info;

	if (!sector_size || !n_sectors) {
		dw_spi_init(0, 0);
		info = dw_spi_get_info(0, 0);

		if (info == NULL) {
			return -1;
		}

		sector_size = info->sector_size;
		n_sectors = info->n_sectors;
	}

	data[0] = sector_size;
	data[1] = n_sectors;
	return 0;
}

int baikal_smc_flash_handler(uint32_t smc_fid,
			     uint64_t x1,
			     uint64_t x2,
			     uint64_t x3,
			     uint64_t x4,
			     uint64_t *data)
{
	switch (smc_fid) {
	case BAIKAL_SMC_FLASH_WRITE:
		return dw_spi_write(0, 0, x1, flash_buf, x2);
	case BAIKAL_SMC_FLASH_READ:
		return dw_spi_read(0, 0, x1, flash_buf, x2);
	case BAIKAL_SMC_FLASH_PUSH:
		return baikal_smc_flash_push(x1, x2, x3, x4);
	case BAIKAL_SMC_FLASH_PULL:
		return baikal_smc_flash_pull(data);
	case BAIKAL_SMC_FLASH_POSITION:
		return baikal_smc_flash_position(x1);
	case BAIKAL_SMC_FLASH_ERASE:
		return dw_spi_erase(0, 0, x1, x2, sector_size);
	case BAIKAL_SMC_FLASH_INFO:
		return baikal_smc_flash_info(data);
	default:
		ERROR("%s: unknown smc_fid 0x%x\n", __func__, smc_fid);
		return -1;
	}
}
