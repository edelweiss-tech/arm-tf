/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <baikal_scp.h>
#include <baikal_sip_svc_flash.h>
#include <bm1000_cmu.h>
#include <bm1000_pvt.h>
#include <bm1000_smmu.h>
#include <common/debug.h>
#include <common/runtime_svc.h>
#include <libfdt.h>
#include <platform_def.h>

#define BAIKAL_SMC_LCRU_ID		0x82000000
#define BAIKAL_SMC_PVT_ID		0x82000001
#define BAIKAL_SMC_VDU_UPDATE		0x82000100
#define BAIKAL_SMC_SCP_LOG_DISABLE	0x82000200
#define BAIKAL_SMC_SCP_LOG_ENABLE	0x82000201
#define BAIKAL_SMC_VDEC_SMMU_SET_CACHE	0x82000300
#define BAIKAL_SMC_VDEC_SMMU_GET_CACHE	0x82000301

#define BAIKAL_SMC_PLAT_CMU_PLL_SET_RATE	0
#define BAIKAL_SMC_PLAT_CMU_PLL_GET_RATE	1
#define BAIKAL_SMC_PLAT_CMU_PLL_ENABLE		2
#define BAIKAL_SMC_PLAT_CMU_PLL_DISABLE		3
#define BAIKAL_SMC_PLAT_CMU_PLL_ROUND_RATE	4
#define BAIKAL_SMC_PLAT_CMU_PLL_IS_ENABLED	5
#define BAIKAL_SMC_PLAT_CMU_CLKCH_SET_RATE	6
#define BAIKAL_SMC_PLAT_CMU_CLKCH_GET_RATE	7
#define BAIKAL_SMC_PLAT_CMU_CLKCH_ENABLE	8
#define BAIKAL_SMC_PLAT_CMU_CLKCH_DISABLE	9
#define BAIKAL_SMC_PLAT_CMU_CLKCH_ROUND_RATE	10
#define BAIKAL_SMC_PLAT_CMU_CLKCH_IS_ENABLED	11
#define BAIKAL_SMC_PLAT_CMU_CLK_MSHC_SPEED	12

static int baikal_get_cmu_descriptors(void *fdt)
{
	unsigned cmuidx = 0;
	int offs = 0;

	while (1) {
		const fdt32_t *ac;
		int plen;

		/* next */
		offs = fdt_next_node(fdt, offs, NULL);
		if (offs < 0) {
			break;
		}

		/* compatible */
		if (!fdt_node_check_compatible(fdt, offs, "baikal,cmu")) {
			struct cmu_desc *const cmu = cmu_desc_get_by_idx(cmuidx);

			if (cmu == NULL) {
				ERROR("%s: unable to get cmu_desc with idx = %u\n",
				      __func__, cmuidx);
				return 2;
			}

			/* clear */
			cmu->base	     = 0;
			cmu->name	     = NULL;
			cmu->frefclk	     = 0;
			cmu->fpllmin	     = 0;
			cmu->fpllmax	     = 0;
			cmu->fpllreq	     = 0;
			cmu->deny_pll_reconf = false;
			cmu->mshc_clkch	     = -1;

			/* cmu-id */
			ac = fdt_getprop(fdt, offs, "cmu-id", &plen);
			if (ac == NULL) {
				return 2;
			}

			cmu->base = fdt32_to_cpu(*ac);

			/* clocks */
			ac = fdt_getprop(fdt, offs, "clocks", &plen);
			if (ac != NULL) {
				int offs_par;

				offs_par = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*ac));
				ac = fdt_getprop(fdt, offs_par, "clock-frequency", &plen);
				if (ac == NULL) {
					return 2;
				}

				cmu->frefclk = fdt32_to_cpu(*ac);
			} else {
				cmu->frefclk = CMU_REFCLK_DEFAULT_FREQ;
			}

			/* min */
			ac = fdt_getprop(fdt, offs, "min", &plen);
			if (ac == NULL) {
				return 2;
			}

			cmu->fpllmin = fdt32_to_cpu(*ac);

			/* max */
			ac = fdt_getprop(fdt, offs, "max", &plen);
			if (ac == NULL) {
				return 2;
			}

			cmu->fpllmax = fdt32_to_cpu(*ac);
			if (cmu->fpllmax > CMU_PLL_MAX_FREQ) {
				ERROR("%s: CMU 0x%lx: max value (%u Hz) exceeds the allowed PLL frequency (%lu Hz)\n",
				      __func__, cmu->base, cmu->fpllmax, CMU_PLL_MAX_FREQ);

				assert(cmu->fpllmax <= CMU_PLL_MAX_FREQ);

				cmu->fpllmax = CMU_PLL_MAX_FREQ;
				NOTICE("%s: CMU 0x%lx: max value is reduced to %u Hz\n",
				       __func__, cmu->base, cmu->fpllmax);
			}

			/* clock-frequency */
			ac = fdt_getprop(fdt, offs, "clock-frequency", &plen);
			if (ac != NULL) {
				cmu->fpllreq = fdt32_to_cpu(*ac);
			} else {
				cmu->fpllreq = CMU_PLL_DEFAULT_FREQ;
			}

			/* clock-indices */
			ac = fdt_getprop(fdt, offs, "clock-indices", &plen);
			int num_of_chans = plen / 4;
			if (num_of_chans > 1) {
				cmu->deny_pll_reconf = true;
			}

			/* clock-output-names */
			cmu->name = fdt_getprop(fdt, offs, "clock-output-names", &plen);

			/* mshc-channel */
			if (num_of_chans > 1) {
				int idx = fdt_stringlist_search(fdt, offs, "clock-names", "mshc_tx_x2");
				if (idx >= 0) {
					const uint32_t *p = fdt_getprop(fdt, offs, "clock-indices", NULL);
					cmu->mshc_clkch = fdt32_to_cpu(p[idx]);
				}
			}

			/* print */
			INFO("Device Tree node @ 0x%lx: name %s, frefclk %u Hz\n",
			     cmu->base, cmu->name, cmu->frefclk);

			++cmuidx;
		}
	}

	return 0;
}

static int baikal_sip_setup(void)
{
	int ret;
	void *fdt = (void *)(uintptr_t)PLAT_BAIKAL_DT_BASE;

	ret = fdt_open_into(fdt, fdt, PLAT_BAIKAL_DT_MAX_SIZE);
	if (ret < 0) {
		ERROR("Invalid Device Tree at %p: error %d\n", fdt, ret);
		return -1;
	}

	return baikal_get_cmu_descriptors(fdt);
}

#ifndef BE_QEMU
static uint64_t baikal_pvt_handler(uint32_t cmd,
				   uint64_t x1,
				   uint64_t x2,
				   uint64_t x3,
				   void *cookie,
				   void *handle,
				   uint64_t flags)
{
	switch (cmd) {
	case PVT_READ:
		return (uint64_t)pvt_read_reg((uint32_t)x1, (uint32_t)x2);
	case PVT_WRITE:
		return (uint64_t)pvt_write_reg((uint32_t)x1, (uint32_t)x2, (uint32_t)x3);
	default:
		ERROR("%s: unhandled SMC 0x%x\n", __func__, cmd);
		SMC_RET1(handle, SMC_UNK);
	}

	return 0;
}
#endif

static int64_t baikal_plat_sip_handler(uint32_t id,
				       uint64_t cmd,
				       uint64_t x1,
				       uint64_t x2,
				       void *cookie,
				       void *handle,
				       uint64_t flags)
{
	INFO("%s: SMC 0x%llx\n", __func__, cmd);

	switch (cmd) {
	case BAIKAL_SMC_PLAT_CMU_PLL_SET_RATE:
		return cmu_pll_set_rate(id, x2, x1);
	case BAIKAL_SMC_PLAT_CMU_PLL_GET_RATE:
		return cmu_pll_get_rate(id, x2);
	case BAIKAL_SMC_PLAT_CMU_PLL_ENABLE:
		return cmu_pll_enable(id);
	case BAIKAL_SMC_PLAT_CMU_PLL_DISABLE:
		return cmu_pll_disable(id);
	case BAIKAL_SMC_PLAT_CMU_PLL_ROUND_RATE:
		return cmu_pll_round_rate(id, x2, x1);
	case BAIKAL_SMC_PLAT_CMU_PLL_IS_ENABLED:
		return cmu_pll_is_enabled(id);

	case BAIKAL_SMC_PLAT_CMU_CLKCH_SET_RATE:
		return cmu_clkch_set_rate((uint32_t)x2, id, x1);
	case BAIKAL_SMC_PLAT_CMU_CLKCH_GET_RATE:
		return cmu_clkch_get_rate((uint32_t)x2, id);
	case BAIKAL_SMC_PLAT_CMU_CLKCH_ENABLE:
		return cmu_clkch_enable((uint32_t)x2, id);
	case BAIKAL_SMC_PLAT_CMU_CLKCH_DISABLE:
		return cmu_clkch_disable((uint32_t)x2, id);
	case BAIKAL_SMC_PLAT_CMU_CLKCH_ROUND_RATE:
		return cmu_clkch_round_rate((uint32_t)x2, id, x1);
	case BAIKAL_SMC_PLAT_CMU_CLKCH_IS_ENABLED:
		return cmu_clkch_is_enabled((uint32_t)x2, id);
	}

	ERROR("%s: unhandled SMC: cmd:0x%llx id:0x%x\n", __func__, cmd, id);
	return SMC_UNK;
}

/* This function is responsible for handling all SiP calls from the NS world */
uint64_t sip_smc_handler(uint32_t smc_fid,
			 uint64_t x1,
			 uint64_t x2,
			 uint64_t x3,
			 uint64_t x4,
			 void *cookie,
			 void *handle,
			 uint64_t flags)
{
	uint64_t ret = 0;
	uint64_t data[4];

	/* Determine which security state this SMC originated from */
	if (is_caller_secure(flags)) {
		ERROR("%s: SMC secure world's call (0x%x)\n", __func__, smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}

	switch (smc_fid) {
#ifndef BE_QEMU
	case BAIKAL_SMC_PVT_ID:
		ret = baikal_pvt_handler(x1, x2, x3, x4, cookie, handle, flags);
		break;
#endif
	case BAIKAL_SMC_VDEC_SMMU_SET_CACHE:
		ret = mmvdec_smmu_set_domain_cache(x1, x2, x3);
		break;
	case BAIKAL_SMC_VDEC_SMMU_GET_CACHE:
		ret = mmvdec_smmu_get_domain_cache();
		break;
	case BAIKAL_SMC_VDU_UPDATE:
		ret = scp_cmd('V', x1, x2);
		break;
	case BAIKAL_SMC_SCP_LOG_DISABLE:
		ret = scp_cmd('T', 0, 0);
		break;
	case BAIKAL_SMC_SCP_LOG_ENABLE:
		ret = scp_cmd('t', 0, 0);
		break;
	case BAIKAL_SMC_FLASH_WRITE:
	case BAIKAL_SMC_FLASH_READ:
	case BAIKAL_SMC_FLASH_PULL:
	case BAIKAL_SMC_FLASH_PUSH:
	case BAIKAL_SMC_FLASH_POSITION:
	case BAIKAL_SMC_FLASH_ERASE:
	case BAIKAL_SMC_FLASH_INFO:
		ret = baikal_smc_flash_handler(smc_fid, x1, x2, x3, x4, data);
		if (ret) {
			break;
		}

		if (smc_fid == BAIKAL_SMC_FLASH_PULL) {
			SMC_RET4(handle, data[0], data[1], data[2], data[3]);
		} else if (smc_fid == BAIKAL_SMC_FLASH_INFO) {
			SMC_RET3(handle, ret, data[0], data[1]);
		}

		break;
	default:
		ret = baikal_plat_sip_handler(x1, x2, x3, x4, cookie, handle, flags);
		break;
	}

	SMC_RET1(handle, ret);
}

/* Define a runtime service descriptor for fast SMC calls */
DECLARE_RT_SVC(
	baikal_sip_svc,
	OEN_SIP_START,
	OEN_SIP_END,
	SMC_TYPE_FAST,
	baikal_sip_setup,
	(rt_svc_handle_t)sip_smc_handler
);
