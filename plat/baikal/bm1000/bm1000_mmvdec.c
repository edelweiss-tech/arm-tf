/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bm1000_cmu.h>
#include <bm1000_private.h>
#include <bm1000_smmu.h>
#include <lib/mmio.h>
#include <platform_def.h>

#define MMVDEC_RST	MMVDEC_GPR(0)
enum {
	MMVDEC_SLAVE_RST   = BIT(0),
	MMVDEC_VDEC_RST	   = BIT(1),
	MMVDEC_MMU_RST	   = BIT(2),
	MMVDEC_MMU_SLV_RST = BIT(3)
};

#define MMVDEC_CFG_REG	MMVDEC_GPR(0x8)
enum {
	MMVDEC_CFG_REG_HEVC_EN	   = BIT(0),
	MMVDEC_CFG_REG_H264_EN	   = BIT(1),
	MMVDEC_CFG_REG_VC1_EN	   = BIT(2),
	MMVDEC_CFG_REG_WMV9_EN	   = BIT(3),
	MMVDEC_CFG_REG_MPEG1_EN	   = BIT(4),
	MMVDEC_CFG_REG_MPEG2_EN	   = BIT(5),
	MMVDEC_CFG_REG_MPEG4_EN	   = BIT(6),
	MMVDEC_CFG_REG_AVS_EN	   = BIT(7),
	MMVDEC_CFG_REG_SORENSON_EN = BIT(8),
	MMVDEC_CFG_REG_RV_EN	   = BIT(9),
	MMVDEC_CFG_REG_VP6_EN	   = BIT(10),
	MMVDEC_CFG_REG_VP8_EN	   = BIT(11),
	MMVDEC_CFG_REG_REG_PROT	   = BIT(16),
	MMVDEC_CFG_REG_BPASIDLE	   = BIT(20)
};

void mmvdec_init(void)
{
	/*
	 * Set PLL clock to 1.42 GHz. It will be further divided by 2
	 * by a clock channel to minimize jitter.
	 */
	static const cmu_pll_ctl_vals_t vdec_lcru_pll_init = {
		0, 0x18, 0xb18, 0, 0, 0x2c, 0, 0x2c
	};

	cmu_pll_on(MMVDEC_LCRU, &vdec_lcru_pll_init);

	/* 1.42 GHz / 2 = 710 MHz */
	cmu_clkch_enable_by_base(MMVDEC_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(0), 2);

	/* Deassert reset signals */
	mmio_clrbits_32(MMVDEC_RST,
			MMVDEC_SLAVE_RST |
			MMVDEC_VDEC_RST	 |
			MMVDEC_MMU_RST	 |
			MMVDEC_MMU_SLV_RST);

	mmvdec_smmu_set_normalize(0);
	mmvdec_smmu_set_domain_cache(0x2, 0x2, 0x7);
	mmvdec_smmu_set_qos(0xf, 0xf);
}

void mmvdec_ns_access(void)
{
	mmio_write_32(BAIKAL_NIC_VDEC_TCU, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_VDEC_CTL, SECURE_MODE_OFF);

	mmio_clrsetbits_32(MMVDEC_SMMU_CFG1_REG,
			   MMVDEC_SMMU_CFG1_REG_AWPROT_MASK |
			   MMVDEC_SMMU_CFG1_REG_ARPROT_MASK,
			   MMVDEC_SMMU_CFG1_REG_AWPROT(0x2) |
			   MMVDEC_SMMU_CFG1_REG_ARPROT(0x2));
}
