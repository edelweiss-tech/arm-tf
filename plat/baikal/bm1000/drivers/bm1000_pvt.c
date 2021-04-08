/*
 * Copyright (c) 2019-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bm1000_pvt.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>
#include <platform_def.h>

static uintptr_t pvt_get_addr(uint32_t pvt_id, uint32_t offset)
{
	static const uintptr_t pvtcc_bases[] = {
		A57_0_PVTCC_ADDR,
		A57_1_PVTCC_ADDR,
		A57_2_PVTCC_ADDR,
		A57_3_PVTCC_ADDR,
		MALI_PVTCC_ADDR
	};

	/* Ensure that pvt_id is correct and the offset in PVT address range */
	if (pvt_id < ARRAY_SIZE(pvtcc_bases) && !(offset & ~PVT_AREA_MASK)) {
		return pvtcc_bases[pvt_id] + offset;
	}

	return 0;
}

uint32_t pvt_read_reg(uint32_t pvt_id, uint32_t offset)
{
	uintptr_t addr;

	addr = pvt_get_addr(pvt_id, offset);
	if (!addr) {
		return 1;
	}

	return mmio_read_32(addr);
}

uint32_t pvt_write_reg(uint32_t pvt_id, uint32_t offset, uint32_t val)
{
	uintptr_t addr;

	addr = pvt_get_addr(pvt_id, offset);
	if (!addr) {
		return 1;
	}

	mmio_write_32(addr, val);
	return 0;
}
