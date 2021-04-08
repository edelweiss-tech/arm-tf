/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bm1000_cmu.h>
#include <bm1000_private.h>
#include <lib/mmio.h>
#include <platform_def.h>

#define MMMALI_GPR(offset)	(MMMALI_LCRU + LCRU_GPR + offset)

/* Resets */
#define MMMALI_RST			MMMALI_GPR(0)
#define MMMALI_RST_NIC			BIT(0)
#define MMMALI_RST_PVT			BIT(1)

/* AXI */
#define MMMALI_PROT_CTL0_REG		MMMALI_GPR(0x18)
#define MMMALI_PROT_CTL1_REG		MMMALI_GPR(0x20)
#define MMMALI_PROT_CTL_REG_AWPROT(x)	SETMASK(x, 2, 0)
#define MMMALI_PROT_CTL_REG_ARPROT(x)	SETMASK(x, 5, 3)

void mmmali_init(void)
{
	static const cmu_pll_ctl_vals_t mali_lcru_pll_init = {
		0, 0, 0x68, 0, 0, 0x2c, 0, 0x2c
	};

	cmu_pll_on(MMMALI_LCRU, &mali_lcru_pll_init);
	cmu_clkch_enable_by_base(MMMALI_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(0), 2);
	mmio_clrbits_32(MMMALI_RST,
			MMMALI_RST_NIC |
			MMMALI_RST_PVT);
}

void mmmali_ns_access(void)
{
	mmio_write_32(MMMALI_PROT_CTL0_REG,
		      MMMALI_PROT_CTL_REG_AWPROT(2) |
		      MMMALI_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(MMMALI_PROT_CTL1_REG,
		      MMMALI_PROT_CTL_REG_AWPROT(2) |
		      MMMALI_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(BAIKAL_NIC_MALI_TCU, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_MALI,	   SECURE_MODE_OFF);
}
