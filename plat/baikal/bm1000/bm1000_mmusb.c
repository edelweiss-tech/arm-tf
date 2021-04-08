/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bm1000_cmu.h>
#include <bm1000_private.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <platform_def.h>

#define MMUSB_GPR(offset)	(MMUSB_LCRU + LCRU_GPR + offset)

#define MMUSB_RESET				MMUSB_GPR(0)
enum {
	MMUSB_RESET_NIC400_CFG_SLV  = BIT(0),
	MMUSB_RESET_NIC400_CFG_MSTR = BIT(1),
	MMUSB_RESET_NIC400_FNC_SLV  = BIT(2),
	MMUSB_RESET_NIC400_FNC_MSTR = BIT(3),
	MMUSB_RESET_USB2_PHY0	    = BIT(4),
	MMUSB_RESET_USB2_PHY1	    = BIT(5),
	MMUSB_RESET_USB2	    = BIT(6),
	MMUSB_RESET_USB3_PHY0	    = BIT(8),
	MMUSB_RESET_USB3_PHY1	    = BIT(9),
	MMUSB_RESET_USB3_PHY2	    = BIT(10),
	MMUSB_RESET_USB3_PHY3	    = BIT(11),
	MMUSB_RESET_USB3	    = BIT(12),
	MMUSB_RESET_SATA_PHY	    = BIT(16),
	MMUSB_RESET_SATA_CTRL2	    = BIT(17),
	MMUSB_RESET_SATA_CTRL1	    = BIT(18),
	MMUSB_RESET_DMAC	    = BIT(20),
	MMUSB_RESET_GIC		    = BIT(22),
	MMUSB_RESET_MMU		    = BIT(24)
};

/* AXI */
#define MMUSB_SATA0_AXI_SB			MMUSB_GPR(0x10)
#define MMUSB_SATA0_AXI				MMUSB_GPR(0x40)
#define MMUSB_SATA1_AXI_SB			MMUSB_GPR(0x18)
#define MMUSB_SATA1_AXI				MMUSB_GPR(0x48)
#define MMUSB_USB2_AXI_SB			MMUSB_GPR(0x20)
#define MMUSB_USB2_AXI				MMUSB_GPR(0x50)
#define MMUSB_USB3_AXI_SB			MMUSB_GPR(0x28)
#define MMUSB_USB3_AXI				MMUSB_GPR(0x58)
#define MMUSB_DMA330_AXI_SB			MMUSB_GPR(0x30)

/* USB2 */
#define MMUSB_USB2_PHY_CTL			MMUSB_GPR(0xc0)
#define MMUSB_USB2_PHY_CTL_PHS0RCLS_MASK	GENMASK(    1,  0)
#define MMUSB_USB2_PHY_CTL_PHS0RCLS(x)		SETMASK(x,  1,  0)
#define MMUSB_USB2_PHY_CTL_PHS0FSEL_MASK	GENMASK(    4,  2)
#define MMUSB_USB2_PHY_CTL_PHS0FSEL(x)		SETMASK(x,  4,  2)
#define MMUSB_USB2_PHY_CTL_PHS1RCLS_MASK	GENMASK(   17, 16)
#define MMUSB_USB2_PHY_CTL_PHS1RCLS(x)		SETMASK(x, 17, 16)
#define MMUSB_USB2_PHY_CTL_PHS1FSEL_MASK	GENMASK(   20, 18)
#define MMUSB_USB2_PHY_CTL_PHS1FSEL(x)		SETMASK(x, 20, 18)
#define USB2_PHY_REFCLK_CRYSTAL			0x0
#define USB2_PHY_REFCLK_EXTERNAL		0x1
#define USB2_PHY_REFCLK_CLKCORE			0x2
#define USB2_PHY_FSEL_12MHZ			0x2
#define USB2_PHY_FSEL_50MHZ			0x7
#define USB2_PHY_REFCLK_VALUE			USB2_PHY_REFCLK_CLKCORE
#define USB2_PHY_FSEL_VALUE			USB2_PHY_FSEL_50MHZ /* picoPHY uses clock core 50 MHz */

/* USB3 */
#define MMUSB_USB3_PHY0_CTL			MMUSB_GPR(0xe0)
#define MMUSB_USB3_PHY1_CTL			MMUSB_GPR(0x100)
#define MMUSB_USB3_PHY_CTL_REFPAD		BIT(0)
#define MMUSB_USB3_PHY_CTL_MPLLMUL_MASK		GENMASK(    7,  1)
#define MMUSB_USB3_PHY_CTL_MPLLMUL(x)		SETMASK(x,  7,  1)
#define MMUSB_USB3_PHY_CTL_FSEL_MASK		GENMASK(   13,  8)
#define MMUSB_USB3_PHY_CTL_FSEL(x)		SETMASK(x, 13,  8)
#define MMUSB_USB3_PHY_CTL_REFDIV2		BIT(14)
#define MMUSB_USB3_PHY_CTL_REFSSPEN		BIT(15)
#define MMUSB_USB3_PHY_CTL_SSCRCLKS_MASK	GENMASK(   24, 16)
#define MMUSB_USB3_PHY_CTL_SSCRCLKS(x)		SETMASK(x, 24, 16)
#define MMUSB_USB3_PHY_CTL_SSCRANGE_MASK	GENMASK(   27, 25)
#define MMUSB_USB3_PHY_CTL_SSCRANGE(x)		SETMASK(x, 27, 25)
#define MMUSB_USB3_PHY_CTL_CGPEN		BIT(28)
#define MMUSB_USB3_PHY_CTL_CGMEN		BIT(29)

enum {
	MMUSB_CLKCH_SATA_PHY_REF     = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(0),
	MMUSB_CLKCH_SATA_ACLK_CTRL0  = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(1),
	MMUSB_CLKCH_SATA_ACLK_CTRL1  = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(2),
	MMUSB_CLKCH_USB2_PHY0_REF    = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(3),
	MMUSB_CLKCH_USB2_PHY1_REF    = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(4),
	MMUSB_CLKCH_USB2_ACLK	     = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(5),
	MMUSB_CLKCH_USB2_CLK_SOFITP  = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(6),
	MMUSB_CLKCH_USB3_PHY0_REF    = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(7),
	MMUSB_CLKCH_USB3_PHY1_REF    = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(8),
	MMUSB_CLKCH_USB3_PHY2_REF    = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(9),
	MMUSB_CLKCH_USB3_PHY3_REF    = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(10),
	MMUSB_CLKCH_USB3_ACLK	     = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(11),
	MMUSB_CLKCH_USB3_CLK_SOFITP  = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(12),
	MMUSB_CLKCH_USB3_CLK_SUSPEND = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(13),
	MMUSB_CLKCH_MMU_ACLK	     = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(14),
	MMUSB_CLKCH_DMAC_ACLK	     = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(15),
	MMUSB_CLKCH_GIC_ACLK	     = MMUSB_LCRU + LCRU_CMU0 + LCRU_CLKCH_OFFSET(16)
};

void mmusb_init(void)
{
	uint32_t mmusb_gpr0, reg;

	/* PLL 800 MHz */
	static const cmu_pll_ctl_vals_t usb_lcru_pll_init = {
		1, 0, 0x80, 0, 0, 0x2c, 0, 0x2c
	};

	cmu_pll_on(MMUSB_LCRU, &usb_lcru_pll_init);

	cmu_clkch_enable_by_base(MMUSB_CLKCH_SATA_ACLK_CTRL0,  8); /* 100 MHz differential reference clock */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_SATA_ACLK_CTRL1,  8); /* 100 MHz axi clk controller #0 (50 - 100 MHz) */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_SATA_PHY_REF,     8); /* 100 MHz axi clk controller #1 (50 - 100 MHz) */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB2_PHY0_REF,   16); /*  50 MHz max reference PHY #0 clock */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB2_PHY1_REF,   16); /*  50 MHz max reference PHY #1 clock */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB2_ACLK,	      16); /*  50 MHz axi clock */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB2_CLK_SOFITP,  8); /* 100 MHz reference clock for SOF and ITP counter (16.129 - 125 MHz) */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB3_PHY0_REF,    8); /* 100 MHz differential reference clock PHY #0 */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB3_PHY1_REF,    8); /* 100 MHz differential reference clock PHY #1 */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB3_PHY2_REF,   16); /*  50 MHz max reference PHY #2 clock */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB3_PHY3_REF,   16); /*  50 MHz max reference PHY #3 clock */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB3_ACLK,	       6); /* 133 MHz axi clock */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB3_CLK_SOFITP,  8); /* 100 MHz reference clock for SOF and ITP counter (16.129 - 125 MHz) */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_USB3_CLK_SUSPEND, 8); /* 100 MHz suspend clock for low power state (P3) (32 kHz - 125 MHz) */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_MMU_ACLK,	       2); /* 400 MHz axi clk (200 - 400 MHz) */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_DMAC_ACLK,	       4); /* 200 MHz - temp value (250 MHz axi clk (100 - 250 MHz)) */
	cmu_clkch_enable_by_base(MMUSB_CLKCH_GIC_ACLK,	       5); /* 160 MHz */

	/* USB3 PHY clocks */
	reg = mmio_read_32(MMUSB_USB3_PHY0_CTL);
	reg &= ~(MMUSB_USB3_PHY_CTL_CGMEN |
		 MMUSB_USB3_PHY_CTL_CGPEN);

	reg |= MMUSB_USB3_PHY_CTL_REFPAD;
	mmio_write_32(MMUSB_USB3_PHY0_CTL, reg);
	udelay(1);
	reg |= MMUSB_USB3_PHY_CTL_REFSSPEN;
	mmio_write_32(MMUSB_USB3_PHY0_CTL, reg);
	reg = mmio_read_32(MMUSB_USB3_PHY1_CTL);
	reg &= ~(MMUSB_USB3_PHY_CTL_CGMEN |
		 MMUSB_USB3_PHY_CTL_CGPEN);

	reg |= MMUSB_USB3_PHY_CTL_REFPAD;
	mmio_write_32(MMUSB_USB3_PHY1_CTL, reg);
	udelay(1);
	reg |= MMUSB_USB3_PHY_CTL_REFSSPEN;
	mmio_write_32(MMUSB_USB3_PHY1_CTL, reg);

	/* USB2 PHY clocks */
	mmio_clrsetbits_32(MMUSB_USB2_PHY_CTL,
			   MMUSB_USB2_PHY_CTL_PHS0RCLS_MASK |
			   MMUSB_USB2_PHY_CTL_PHS0FSEL_MASK |
			   MMUSB_USB2_PHY_CTL_PHS1RCLS_MASK |
			   MMUSB_USB2_PHY_CTL_PHS1FSEL_MASK,
			   MMUSB_USB2_PHY_CTL_PHS0RCLS(USB2_PHY_REFCLK_VALUE) |
			   MMUSB_USB2_PHY_CTL_PHS0FSEL(USB2_PHY_FSEL_VALUE)   |
			   MMUSB_USB2_PHY_CTL_PHS1RCLS(USB2_PHY_REFCLK_VALUE) |
			   MMUSB_USB2_PHY_CTL_PHS1FSEL(USB2_PHY_FSEL_VALUE));

	/* Deassert reset signals */
	mmusb_gpr0 = mmio_read_32(MMUSB_RESET);
	mmusb_gpr0 &= ~(MMUSB_RESET_MMU		    |
			MMUSB_RESET_GIC		    |
#if 0
			MMUSB_RESET_DMAC	    |
#endif
			MMUSB_RESET_SATA_CTRL1	    |
			MMUSB_RESET_SATA_CTRL2	    |
			MMUSB_RESET_SATA_PHY	    |
			MMUSB_RESET_USB3	    |
			MMUSB_RESET_USB2	    |
			MMUSB_RESET_NIC400_FNC_MSTR |
			MMUSB_RESET_NIC400_FNC_SLV  |
			MMUSB_RESET_NIC400_CFG_MSTR |
			MMUSB_RESET_NIC400_CFG_SLV);

	mmio_write_32(MMUSB_RESET, mmusb_gpr0);

	udelay(50);

	mmusb_gpr0 &= ~(MMUSB_RESET_USB3_PHY3 |
			MMUSB_RESET_USB3_PHY2 |
			MMUSB_RESET_USB3_PHY1 |
			MMUSB_RESET_USB3_PHY0 |
			MMUSB_RESET_USB2_PHY1 |
			MMUSB_RESET_USB2_PHY0);

	mmio_write_32(MMUSB_RESET, mmusb_gpr0);
}

void mmusb_chc(void)
{
	/* SATA0 */
	mmio_write_32(MMUSB_SATA0_AXI_SB, 0xa0000);  /* ARDOMAIN 2 / AWDOMAIN 2 (0xa0000 = Coherent ReadOnce + WriteUnique) */
	mmio_write_32(MMUSB_SATA0_AXI,	  0xb70012); /* ARCACHE 7, AWCACHE 7, AxPROT = 2 */
	/* SATA1 */
	mmio_write_32(MMUSB_SATA1_AXI_SB, 0xa0000);  /* ARDOMAIN 2 / AWDOMAIN 2 (0xa0000 = Coherent ReadOnce + WriteUnique) */
	mmio_write_32(MMUSB_SATA1_AXI,	  0xb70012); /* ARCACHE 7, AWCACHE 7, AxPROT = 2 */
	/* USB2 */
	mmio_write_32(MMUSB_USB2_AXI_SB,  0xa0000);  /* ARDOMAIN 2 / AWDOMAIN 2 (0xa0000 = Coherent ReadOnce + WriteUnique) */
	mmio_write_32(MMUSB_USB2_AXI,	  0x000012); /* AxPROT = 2 */
	/* USB3 */
	mmio_write_32(MMUSB_USB3_AXI_SB,  0xa0000);  /* ARDOMAIN 2 / AWDOMAIN 2 (0xa0000 = Coherent ReadOnce + WriteUnique) */
	mmio_write_32(MMUSB_USB3_AXI,	  0x000012); /* AxPROT = 2 */
}

void mmusb_ns_access(void)
{
	mmio_write_32(BAIKAL_NIC_USB_TCU,      SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_USB_0,	       SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_USB_1,	       SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_SATA_0,       SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_SATA_1,       SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_USB_GIC,      SECURE_MODE_OFF);
#if 0
	mmio_write_32(BAIKAL_NIC_USB_DMA330_0, SECURE_MODE_OFF);
	mmio_write_32(BAIKAL_NIC_USB_DMA330_1, SECURE_MODE_OFF);
#endif
}

#define SATA_0			0x2c600000
#define SATA_1			0x2c610000
#define SATA_GHC		0x4
#define SATA_GHC_RESET		BIT(0)
#define SATA_TIMER1MS		0xe0
#define SATA_CAP		0x0
#define SATA_CAP_SSS		BIT(27)
#define SATA_CAP_SMPS		BIT(28)
#define SATA_PI			0x0c
#define SATA_PORT0_SCTL		0x12c
#define SATA_PORT0_STS		0x128

static void mmusb_init_sata_ch(const uintptr_t base)
{
	uint64_t timeout;

	mmio_setbits_32(base + SATA_GHC, SATA_GHC_RESET);
	timeout = timeout_init_us(100);
	while (mmio_read_32(base + SATA_GHC) & SATA_GHC_RESET) {
		if (timeout_elapsed(timeout)) {
			ERROR("%s(0x%lx): GHC.RESET timeout\n", __func__, base);
			break;
		}
	}

	mmio_clrsetbits_32(base + SATA_CAP,
			   SATA_CAP_SSS,
			   SATA_CAP_SMPS);

	/* SATA timer 150 MHz */
	mmio_write_32(base + SATA_TIMER1MS, 150 * 1000);
	mmio_write_32(base + SATA_PI, 1); /* 1 port */
	mmio_write_32(base + SATA_PORT0_SCTL, 0x10);
	timeout = timeout_init_us(1000);
	while (!(mmio_read_32(base + SATA_PORT0_STS) & 0x11) &&
	       !timeout_elapsed(timeout));

#if 0
	/* SMMU */
	mmio_write_32(0x2c080000, 0xa5f0001);
	mmio_write_32(0x2c080400, 0xa5f0001);
#endif
}

void mmusb_init_sata(void)
{
	mmusb_init_sata_ch(SATA_0);
	mmusb_init_sata_ch(SATA_1);
}
