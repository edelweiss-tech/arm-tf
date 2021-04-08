/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <baikal_console.h>
#include <baikal_gicv3.h>
#include <bm1000_cmu.h>
#include <bm1000_private.h>
#include <common/bl_common.h>
#include <drivers/generic_delay_timer.h>
#include <plat/arm/common/plat_arm.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include "bm1000_mmca57.h"
#include "bm1000_splash.h"
#include "bm1000_vdu.h"

#define MALI_PVT_CCH_ON(addr, div)   cmu_clkch_enable_by_base(addr + PVT_CCH_OFFSET, div)
#define CORTEX_PVT_CCH_ON(addr, div) cmu_clkch_enable_by_base(addr + PVT_CCH_OFFSET, div)

/*
 * The next 3 constants identify the extents of the code, RO data region and the
 * limit of the BL3-1 image.  These addresses are used by the MMU setup code and
 * therefore they must be page-aligned.  It is the responsibility of the linker
 * script to ensure that __RO_START__, __RO_END__ & __BL31_END__ linker symbols
 * refer to page-aligned addresses.
 */
#define BL31_RO_BASE (unsigned long)(&__RO_START__)
#define BL31_RO_LIMIT (unsigned long)(&__RO_END__)
#define BL31_END (unsigned long)(&__BL31_END__)

extern uint8_t bl31_logo[];
extern uint8_t bl31_sdk_version_logo[];

modeline_t fdt_lvds_video_mode;

/*
 * Placeholder variables for copying the arguments that have been passed to
 * BL3-1 from BL2.
 */
static entry_point_info_t bl32_image_ep_info;
static entry_point_info_t bl33_image_ep_info;

/*
 * Perform any BL3-1 early platform setup.  Here is an opportunity to copy
 * parameters passed by the calling EL (S-EL1 in BL2 & S-EL3 in BL1) before
 * they are lost (potentially). This needs to be done before the MMU is
 * initialized so that the memory layout can be used while creating page
 * tables. BL2 has flushed this information to memory, so we are guaranteed
 * to pick up good data.
 */
void bl31_early_platform_setup2(u_register_t arg0,
				u_register_t arg1,
				u_register_t arg2,
				u_register_t arg3)
{
	baikal_console_boot_init();

	/* Check params passed from BL2 */
	bl_params_t *params_from_bl2 = (bl_params_t *)arg0;

	assert(params_from_bl2);
	assert(params_from_bl2->h.type == PARAM_BL_PARAMS);
	assert(params_from_bl2->h.version >= VERSION_2);

	bl_params_node_t *bl_params = params_from_bl2->head;

	/*
	 * Copy BL33 and BL32 (if present), entry point information.
	 * They are stored in Secure RAM, in BL2's address space.
	 */
	while (bl_params) {
		if (bl_params->image_id == BL32_IMAGE_ID) {
			bl32_image_ep_info = *bl_params->ep_info;
		}

		if (bl_params->image_id == BL33_IMAGE_ID) {
			bl33_image_ep_info = *bl_params->ep_info;
		}

		bl_params = bl_params->next_params_info;
	}

	if (!bl33_image_ep_info.pc) {
		panic();
	}
}

void bl31_plat_arch_setup(void)
{
	plat_arm_interconnect_init();
	plat_arm_interconnect_enter_coherency();

	baikal_configure_mmu_el3(BL31_RO_BASE, (BL31_END - BL31_RO_BASE),
				 BL31_RO_BASE, BL31_RO_LIMIT,
				 BL_COHERENT_RAM_BASE, BL_COHERENT_RAM_END);
}

void bl31_platform_setup(void)
{
	int fb_cpp;

	generic_delay_timer_init();

	INFO("Init LSP...\n");
	mmlsp_init();
	mmlsp_ns_access();

#ifndef BE_QEMU
	INFO("Init Cortex PVT...\n");
	CORTEX_PVT_CCH_ON(A57_0_PVTCC_ADDR, PVT_DIV);
	CORTEX_PVT_CCH_ON(A57_1_PVTCC_ADDR, PVT_DIV);
	CORTEX_PVT_CCH_ON(A57_2_PVTCC_ADDR, PVT_DIV);
	CORTEX_PVT_CCH_ON(A57_3_PVTCC_ADDR, PVT_DIV);
#endif

	INFO("Init XGbE...\n");
	fb_cpp = bmp_to_fb((uintptr_t)FB2_BASE, &hdmi_video_mode, bl31_logo, 0, 0, 1);
	if (fb_cpp) {
		if (fb_cpp > 2) {
			int h1, h2, w1, w2;

			/*
			 * Put SDK version just behind the logo,
			 * aligned to its right edge
			 */
			bmp_get_dimensions(bl31_logo, &w1, &h1);
			bmp_get_dimensions(bl31_sdk_version_logo, &w2, &h2);
			bmp_to_fb((uintptr_t)FB2_BASE,
				  &hdmi_video_mode,
				  bl31_sdk_version_logo,
				  (w1 - w2) / 2,
				  (h1 + h2) / 2,
				  0);
		}

#ifndef BE_QEMU
		wait_for_vblank(MMXGBE_VDU_BASE);
#endif
		vdu_set_fb(MMXGBE_VDU_BASE,
			   (uintptr_t)FB2_BASE,
			   &hdmi_video_mode,
			   fb_cpp);
	}

	mmxgbe_ns_access();

	INFO("Init USB...\n");
	mmusb_init();
	mmusb_ns_access();
	mmusb_chc();
	mmusb_init_sata();

	INFO("Init Mali...\n");
	mmmali_init();
	mmmali_ns_access();

#ifndef BE_QEMU
	INFO("Init Mali PVT...\n");
	MALI_PVT_CCH_ON(MALI_PVTCC_ADDR, PVT_DIV);
#endif

	INFO("Init PCIe...\n");
	mmpcie_init();
	mmpcie_ns_access();

#ifndef BE_QEMU
	INFO("Init VDec...\n");
	mmvdec_init();
	mmvdec_ns_access();
#endif

	baikal_gic_driver_init();
	baikal_gic_init();
}

void bl31_plat_enable_mmu(uint32_t flags)
{
	plat_arm_interconnect_enter_coherency();
	enable_mmu_el3(flags);
}

void bl31_plat_runtime_setup(void)
{
	int fb_cpp;
	unsigned idx;
	struct cmu_desc *lvds_vdu_cmu = NULL;

	/* Set frequencies from device tree */
	for (idx = 0; ; ++idx) {
		struct cmu_desc *const cmu = cmu_desc_get_by_idx(idx);
		if (cmu == NULL || !cmu->base) {
			break;
		}

		if (cmu->base == LVDS_VDU_CMU_BASE) {
			lvds_vdu_cmu = cmu;
		} else if (!cmu_is_mmca57(cmu->base) && !cmu->deny_pll_reconf) {
			cmu_pll_reconf_nr(cmu);
		} else {
			if (cmu_is_mmca57(cmu->base)) {
				mmca57_reconf_sclken(cmu->base, 2);
			}

			cmu_pll_set_rate(cmu->base,
					 cmu->frefclk,
					 cmu->fpllreq);
		}
	}

	fb_cpp = bmp_to_fb((uintptr_t)FB0_BASE, lvds_video_mode, bl31_logo, 0, 0, 1);
	if (fb_cpp) {
		vdu_set_fb(MMAVLSP_VDU_BASE,
			   (uintptr_t)FB0_BASE,
			   lvds_video_mode,
			   fb_cpp);

		if (fb_cpp > 2) {
			int h1, h2, w1, w2;

			/*
			 * Put SDK version just behind the logo,
			 * aligned to its right edge
			 */
			bmp_get_dimensions(bl31_logo, &w1, &h1);
			bmp_get_dimensions(bl31_sdk_version_logo, &w2, &h2);
			bmp_to_fb((uintptr_t)FB0_BASE,
				  lvds_video_mode,
				  bl31_sdk_version_logo,
				  (w1 - w2) / 2,
				  (h1 + h2) / 2,
				  0);
		}

		if (!fdt_get_panel(&fdt_lvds_video_mode)) {
			lvds_video_mode = &fdt_lvds_video_mode;
		}

		if (lvds_vdu_cmu != NULL) {
			cmu_pll_set_rate(LVDS_VDU_CMU_BASE,
					 lvds_vdu_cmu->frefclk,
					 lvds_video_mode->clock * 7);
		}

		vdu_init(MMAVLSP_VDU_BASE, (uintptr_t)FB0_BASE, lvds_video_mode);
	}
}

/*
 * Return a pointer to the 'entry_point_info' structure of the next image
 * for the security state specified. BL3-3 corresponds to the non-secure
 * image type while BL3-2 corresponds to the secure image type. A NULL
 * pointer is returned if the image does not exist.
 */
entry_point_info_t *bl31_plat_get_next_image_ep_info(uint32_t type)
{
	entry_point_info_t *next_image_info;

	assert(sec_state_is_valid(type));
	next_image_info = (type == NON_SECURE) ?
			  &bl33_image_ep_info : &bl32_image_ep_info;
	/*
	 * None of the images on the ARM development platforms can have 0x0
	 * as the entrypoint
	 */
	if (next_image_info->pc) {
		return next_image_info;
	} else {
		return NULL;
	}
}
