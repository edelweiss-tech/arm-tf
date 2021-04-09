/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <baikal_gpio32.h>
#include <bm1000_cmu.h>
#include <bm1000_private.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <platform_def.h>
#include "bm1000_macaddr.h"

#define MMXGBE_LCRU	0x30000000

#define MMXGBE_ASYNCRES_REG			(MMXGBE_LCRU + LCRU_GPR + 0x00)
enum {
	MMXGBE_ASYNCRES_REG_CFG_NICS_RES    = BIT(0),
	MMXGBE_ASYNCRES_REG_CFG_NICM_RES    = BIT(1),
	MMXGBE_ASYNCRES_REG_DMA_NICS_RES    = BIT(2),
	MMXGBE_ASYNCRES_REG_DMA_NICM_RES    = BIT(3),
	MMXGBE_ASYNCRES_REG_XGBE0_PWRON_RES = BIT(4),
	MMXGBE_ASYNCRES_REG_XGBE1_PWRON_RES = BIT(8),
	MMXGBE_ASYNCRES_REG_HDMI_PWRON_RES  = BIT(12)
};

#define MMXGBE_GMAC0_PROT_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x18)
#define MMXGBE_GMAC1_PROT_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x20)
#define MMXGBE_GMAC_PROT_CTL_REG_AWPROT_MASK	GENMASK(   2, 0)
#define MMXGBE_GMAC_PROT_CTL_REG_AWPROT(x)	SETMASK(x, 2, 0)
#define MMXGBE_GMAC_PROT_CTL_REG_ARPROT_MASK	GENMASK(   5, 3)
#define MMXGBE_GMAC_PROT_CTL_REG_ARPROT(x)	SETMASK(x, 5, 3)

#define MMXGBE_GMAC0_CACHE_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x90)
#define MMXGBE_GMAC1_CACHE_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x98)
#define MMXGBE_GMAC_CACHE_CTL_REG_ARCACHE(x)	SETMASK(x,  3,  0)
#define MMXGBE_GMAC_CACHE_CTL_REG_ARDOMAIN(x)	SETMASK(x,  9,  8)
#define MMXGBE_GMAC_CACHE_CTL_REG_AWCACHE(x)	SETMASK(x, 19, 16)
#define MMXGBE_GMAC_CACHE_CTL_REG_AWDOMAIN(x)	SETMASK(x, 25, 24)

#define MMXGBE_XGBE0_PROT_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x08)
#define MMXGBE_XGBE0_PROT_CTL_REG_AWPROT_MASK	GENMASK(   2, 0)
#define MMXGBE_XGBE0_PROT_CTL_REG_AWPROT(x)	SETMASK(x, 2, 0)
#define MMXGBE_XGBE0_PROT_CTL_REG_ARPROT_MASK	GENMASK(   5, 3)
#define MMXGBE_XGBE0_PROT_CTL_REG_ARPROT(x)	SETMASK(x, 5, 3)

#define MMXGBE_XGBE1_PROT_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x10)
#define MMXGBE_XGBE1_PROT_CTL_REG_AWPROT_MASK	GENMASK(   2, 0)
#define MMXGBE_XGBE1_PROT_CTL_REG_AWPROT(x)	SETMASK(x, 2, 0)
#define MMXGBE_XGBE1_PROT_CTL_REG_ARPROT_MASK	GENMASK(   5, 3)
#define MMXGBE_XGBE1_PROT_CTL_REG_ARPROT(x)	SETMASK(x, 5, 3)

#define MMXGBE_HDMI_VIDEO_PROT_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x28)
#define MMXGBE_HDMI_VIDEO_PROT_CTL_REG_ARPROT(x)	SETMASK(x, 5, 3)
#define MMXGBE_HDMI_VIDEO_PROT_CTL_REG_ARPROT_MASK	SETMASK(x, 5, 3)

#define MMXGBE_HDMI_VIDEO_QOS_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x78)
#define MMXGBE_HDMI_VIDEO_QOS_CTL_REG_ARQOS(x)		SETMASK(x, 7, 4)

#define MMXGBE_HDMI_VIDEO_CACHE_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0xa0)
#define MMXGBE_HDMI_VIDEO_CACHE_CTL_REG_ARCACHE(x)	SETMASK(x, 3, 0)
#define MMXGBE_HDMI_VIDEO_CACHE_CTL_REG_ARDOMAIN(x)	SETMASK(x, 9, 8)

#define MMXGBE_HDMI_AUDIO_PROT_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x30)
#define MMXGBE_HDMI_AUDIO_PROT_CTL_REG_ARPROT(x)	SETMASK(x, 5, 3)
#define MMXGBE_HDMI_AUDIO_PROT_CTL_REG_ARPROT_MASK	SETMASK(x, 5, 3)

#define MMXGBE_HDMI_AUDIO_QOS_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0x80)
#define MMXGBE_HDMI_AUDIO_QOS_CTL_REG_ARQOS(x)		SETMASK(x, 7, 4)

#define MMXGBE_HDMI_AUDIO_CACHE_CTL_REG		(MMXGBE_LCRU + LCRU_GPR + 0xa8)
#define MMXGBE_HDMI_AUDIO_CACHE_CTL_REG_ARCACHE(x)	SETMASK(x, 3, 0)
#define MMXGBE_HDMI_AUDIO_CACHE_CTL_REG_ARDOMAIN(x)	SETMASK(x, 9, 8)

enum {
	MMXGBE_CLKCH_CSR0	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(0),
	MMXGBE_CLKCH_CSR1	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(1),
	MMXGBE_CLKCH_XGBE0_REF	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(2),
	MMXGBE_CLKCH_XGBE0_ACLK	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(3),
	MMXGBE_CLKCH_XGBE0_PTP	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(4),
	MMXGBE_CLKCH_XGBE1_REF	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(5),
	MMXGBE_CLKCH_XGBE1_ACLK	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(6),
	MMXGBE_CLKCH_XGBE1_PTP	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(7),
	MMXGBE_CLKCH_GMAC0_ACLK	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(8),
	MMXGBE_CLKCH_GMAC0_PTPCLK    = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(9),
	MMXGBE_CLKCH_GMAC0_TX2CLK    = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(10),
	MMXGBE_CLKCH_GMAC1_ACLK	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(11),
	MMXGBE_CLKCH_GMAC1_PTPCLK    = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(12),
	MMXGBE_CLKCH_GMAC1_TX2CLK    = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(13),
	MMXGBE_CLKCH_MMU	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(14),
	MMXGBE_CLKCH_HDMI_VIDEO_ACLK = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(15),
	MMXGBE_CLKCH_HDMI_AUDIO_ACLK = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(16),
	MMXGBE_CLKCH_HDMI_SFR0	     = MMXGBE_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(17),
	MMXGBE_CLKCH_HDMI_SFR1	     = MMXGBE_LCRU + LCRU_CMU1 + LCRU_CLKCH_OFFSET(0)
};

#define GMAC0_BASE	0x30240000
#define GMAC1_BASE	0x30250000
#define GMAC_MACADDR0HI	0x40
#define GMAC_MACADDR0LO	0x44

#define MMXGBE_HDMI_LCRU_PLL1_RESET_GPIO_PIN	18

static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll_init  = { 0, 0,    0x64, 0,          0, 0x2c, 0, 0x2c };

#if 0
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x1,  0xdddddddd, 0, 0x2c, 0, 0x2c }; /*  25.200 MHz */
#endif
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0x23, 0x2, 0xca, 0,          0, 0x2c, 0, 0x2c }; /*  25.250 MHz */
#if 0
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x2,  0xf684bda1, 0, 0x2c, 0, 0x2c }; /*  40.000 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x4,  0xd097b425, 0, 0x2c, 0, 0x2c }; /*  65.000 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x5,  0x80000000, 0, 0x2c, 0, 0x2c }; /*  74.250 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x8,  0,          0, 0x2c, 0, 0x2c }; /* 108.000 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0xb,  0,          0, 0x2c, 0, 0x2c }; /* 148.500 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0xe,  0x4ec0d0be, 0, 0x2c, 0, 0x2c }; /* 193.153 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x11, 0xe38e38e3, 0, 0x2c, 0, 0x2c }; /* 241.500 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x13, 0xe38e38e3, 0, 0x2c, 0, 0x2c }; /* 268.500 MHz */
static const cmu_pll_ctl_vals_t mmxgbe_lcru_pll1_init = { 0, 0,    0x1a, 0x5a1cac0,  0, 0x2c, 0, 0x2c }; /* 351.297 MHz */
#endif

void mmxgbe_init(void)
{
	uint8_t macaddr[6];

	mmio_clrbits_32(MMXGBE_ASYNCRES_REG,
			MMXGBE_ASYNCRES_REG_CFG_NICS_RES |
			MMXGBE_ASYNCRES_REG_CFG_NICM_RES |
			MMXGBE_ASYNCRES_REG_DMA_NICS_RES |
			MMXGBE_ASYNCRES_REG_DMA_NICM_RES);

#if defined(BE_MBM10) && (BOARD_VER == 0)
	gpio32_dir_set(MMXGBE_HDMI_LCRU_PLL1_RESET_GPIO_PIN);
	gpio32_out_rst(MMXGBE_HDMI_LCRU_PLL1_RESET_GPIO_PIN);
	mdelay(10);
	gpio32_out_set(MMXGBE_HDMI_LCRU_PLL1_RESET_GPIO_PIN);
#endif

	cmu_pll_on(MMXGBE_LCRU, &mmxgbe_lcru_pll_init);
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_CSR0,	       25); /*  50.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_CSR1,	       12); /* 104.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_XGBE0_REF,        8); /* 156.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_XGBE0_ACLK,       5); /* 250.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_XGBE0_PTP,        8); /* 156.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_XGBE1_REF,        8); /* 156.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_XGBE1_ACLK,       5); /* 250.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_XGBE1_PTP,        8); /* 156.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_GMAC0_ACLK,       5); /* 250.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_GMAC0_PTPCLK,    10); /* 125.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_GMAC0_TX2CLK,     5); /* 250.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_GMAC1_ACLK,       5); /* 250.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_GMAC1_PTPCLK,    10); /* 125.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_GMAC1_TX2CLK,     5); /* 250.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_MMU,	        4); /* 312.5 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_HDMI_VIDEO_ACLK,  3); /* 416.7 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_HDMI_AUDIO_ACLK, 10); /* 125.0 MHz */
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_HDMI_SFR0,       50); /*  25.0 MHz */

	cmu_pll_on(MMXGBE_LCRU + LCRU_CMU1, &mmxgbe_lcru_pll1_init);
	cmu_clkch_enable_by_base(MMXGBE_CLKCH_HDMI_SFR1,        1); /* 594.0 MHz */

	/* Deassert XGMAC resets */
	mmio_clrbits_32(MMXGBE_ASYNCRES_REG,
			MMXGBE_ASYNCRES_REG_XGBE0_PWRON_RES |
			MMXGBE_ASYNCRES_REG_XGBE1_PWRON_RES |
			MMXGBE_ASYNCRES_REG_HDMI_PWRON_RES);

	/* GMAC - Domain 2, cached */
	mmio_write_32(MMXGBE_GMAC0_CACHE_CTL_REG,
		      MMXGBE_GMAC_CACHE_CTL_REG_ARDOMAIN(2)  |
		      MMXGBE_GMAC_CACHE_CTL_REG_AWDOMAIN(2)  |
		      MMXGBE_GMAC_CACHE_CTL_REG_ARCACHE(0xb) |
		      MMXGBE_GMAC_CACHE_CTL_REG_AWCACHE(0x7));

	mmio_write_32(MMXGBE_GMAC1_CACHE_CTL_REG,
		      MMXGBE_GMAC_CACHE_CTL_REG_ARDOMAIN(2)  |
		      MMXGBE_GMAC_CACHE_CTL_REG_AWDOMAIN(2)  |
		      MMXGBE_GMAC_CACHE_CTL_REG_ARCACHE(0xb) |
		      MMXGBE_GMAC_CACHE_CTL_REG_AWCACHE(0x7));

	/* VDU, HDMI */
	mmio_write_32(MMXGBE_HDMI_VIDEO_CACHE_CTL_REG,
		      MMXGBE_HDMI_VIDEO_CACHE_CTL_REG_ARDOMAIN(2) |
		      MMXGBE_HDMI_VIDEO_CACHE_CTL_REG_ARCACHE(0xb));

	mmio_write_32(MMXGBE_HDMI_VIDEO_QOS_CTL_REG,
		      MMXGBE_HDMI_VIDEO_QOS_CTL_REG_ARQOS(0xf));

	mmio_write_32(MMXGBE_HDMI_AUDIO_CACHE_CTL_REG,
		      MMXGBE_HDMI_AUDIO_CACHE_CTL_REG_ARDOMAIN(2) |
		      MMXGBE_HDMI_AUDIO_CACHE_CTL_REG_ARCACHE(0xb));

	if (baikal_macaddr_get(BAIKAL_MACADDR_GMAC0, macaddr) == 0) {
		mmio_write_32(GMAC0_BASE + GMAC_MACADDR0HI,
			      macaddr[5] <<  8 |
			      macaddr[4]);

		mmio_write_32(GMAC0_BASE + GMAC_MACADDR0LO,
			      macaddr[3] << 24 |
			      macaddr[2] << 16 |
			      macaddr[1] <<  8 |
			      macaddr[0]);
	}

	if (baikal_macaddr_get(BAIKAL_MACADDR_GMAC1, macaddr) == 0) {
		mmio_write_32(GMAC1_BASE + GMAC_MACADDR0HI,
			      macaddr[5] <<  8 |
			      macaddr[4]);

		mmio_write_32(GMAC1_BASE + GMAC_MACADDR0LO,
			      macaddr[3] << 24 |
			      macaddr[2] << 16 |
			      macaddr[1] <<  8 |
			      macaddr[0]);
	}
#if 0
	/* SMMU */
	mmio_write_32(0x30080000, 0x3a5f0001);
#endif
}

void mmxgbe_ns_access(void)
{
	mmio_write_32(MMXGBE_GMAC0_PROT_CTL_REG,
		      MMXGBE_GMAC_PROT_CTL_REG_AWPROT(2) |
		      MMXGBE_GMAC_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(MMXGBE_GMAC1_PROT_CTL_REG,
		      MMXGBE_GMAC_PROT_CTL_REG_AWPROT(2) |
		      MMXGBE_GMAC_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(MMXGBE_XGBE0_PROT_CTL_REG,
		      MMXGBE_XGBE0_PROT_CTL_REG_AWPROT(2) |
		      MMXGBE_XGBE0_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(MMXGBE_XGBE1_PROT_CTL_REG,
		      MMXGBE_XGBE1_PROT_CTL_REG_AWPROT(2) |
		      MMXGBE_XGBE1_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(MMXGBE_HDMI_VIDEO_PROT_CTL_REG,
		      MMXGBE_HDMI_VIDEO_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(MMXGBE_HDMI_AUDIO_PROT_CTL_REG,
		      MMXGBE_HDMI_AUDIO_PROT_CTL_REG_ARPROT(2));

	mmio_write_32(BAIKAL_NIC_XGB_TCU,  SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_XGB_VDU,  SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_XGB_HDMI, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_XGB0_CNT, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_XGB0_PHY, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_XGB1_CNT, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_XGB1_PHY, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_1GB_0,	   SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_1GB_1,	   SECURE_MODE_OFF);
}
