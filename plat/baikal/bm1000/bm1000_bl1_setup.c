/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <arch_helpers.h>
#include <assert.h>
#include <baikal_console.h>
#include <baikal_io_storage.h>
#include <bm1000_cmu.h>
#include <bm1000_private.h>
#include <common/bl_common.h>
#include <drivers/generic_delay_timer.h>
#include <lib/mmio.h>
#include <plat/arm/common/plat_arm.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include <spi_dw.h>
#include "bm1000_splash.h"

static uint64_t trusted_mailbox[1 + PLATFORM_CORE_COUNT]
	__attribute__ ((used, section(".trusted_mailbox")));
CASSERT(sizeof(trusted_mailbox) == PLAT_BAIKAL_TRUSTED_MAILBOX_SIZE,
	assert_trusted_mailbox_size);

/* Data structure which holds the extents of the trusted SRAM for BL1 */
static meminfo_t bl2_tzram_layout;

/*
 * Cannot use default weak implementation in bl1_main.c because BL1 RW and BL2
 * data size exceeds Mailbox SRAM.
 */
int bl1_plat_handle_post_image_load(unsigned image_id)
{
	image_desc_t *image_desc;
	entry_point_info_t *ep_info;

	if (image_id != BL2_IMAGE_ID) {
		return 0;
	}

	/* Get the image descriptor */
	image_desc = bl1_plat_get_image_desc(BL2_IMAGE_ID);
	assert(image_desc != NULL);

	/* Get the entry point info */
	ep_info = &image_desc->ep_info;

	bl2_tzram_layout.total_base = BL2_BASE;
	bl2_tzram_layout.total_size = BL2_SIZE;

	flush_dcache_range((uintptr_t)&bl2_tzram_layout, sizeof(meminfo_t));
	ep_info->args.arg1 = (uintptr_t)&bl2_tzram_layout;

	VERBOSE("BL1: BL2 memory layout address = %p\n",
		(void *)&bl2_tzram_layout);

	return 0;
}

static void ccn_hnf_sam_setup(void)
{
	unsigned i;
	uintptr_t pccn = PLAT_ARM_CCN_BASE + CCN_HNF_OFFSET + 8;
	uint64_t snf0 = 0, snf1 = 0;

	INFO("%s...\n", __func__);

	if (cmu_pll_is_enabled(MMDDR0_LCRU)) {
		snf0 = 4;
	}

	if (cmu_pll_is_enabled(MMDDR1_LCRU)) {
		snf1 = 14;
	}

	if (!snf0 && !snf1) {
		ERROR("%s: no DDRs\n", __func__);
		return;
	} else if (!snf0) {
		snf0 = snf1;
	} else if (!snf1) {
		snf1 = snf0;
	}

	INFO("%s(%llu, %llu)\n", __func__, snf0, snf1);
	for (i = 0; i < 4; ++i) {
		mmio_write_64(pccn, snf0);
		pccn += 0x10000;
	}

	for (i = 0; i < 4; ++i) {
		mmio_write_64(pccn, snf1);
		pccn += 0x10000;
	}

	dsbish();
	isb();
}

static void xlat_region_memtest(void)
{
	uint64_t data;
	uint64_t *ptr;
	uint64_t seed;

	INFO("%s: 0x%x-0x%x\n", __func__,
	     BL1_XLAT_BASE, BL1_XLAT_BASE + BL1_XLAT_SIZE - 1);

	seed = read_cntpct_el0();
	data = seed;
	ptr = (uint64_t *)BL1_XLAT_BASE;
	while (ptr < (uint64_t *)(BL1_XLAT_BASE + BL1_XLAT_SIZE)) {
		data ^= data << 13;
		data ^= data >> 7;
		data ^= data << 17;
		*ptr++ = data;
	}

	data = seed;
	ptr = (uint64_t *)BL1_XLAT_BASE;
	while (ptr < (uint64_t *)(BL1_XLAT_BASE + BL1_XLAT_SIZE)) {
		data ^= data << 13;
		data ^= data >> 7;
		data ^= data << 17;
		if (*ptr != data) {
			ERROR("%s @ %p: expected 0x%llx, actual 0x%llx\n",
			      __func__, ptr, data, *ptr);
		}
		++ptr;
	}
}

/* Perform any BL1 specific platform actions */
void bl1_early_platform_setup(void)
{
	/* Initialize the console to provide early debug support */
	baikal_console_boot_init();

	assert(trusted_mailbox == (void *)PLAT_BAIKAL_TRUSTED_MAILBOX_BASE);

	ccn_hnf_sam_setup();
#ifndef BE_VCS
	xlat_region_memtest();
#endif
}

void baikal_configure_mmu_bl1(unsigned long total_base,
			      unsigned long total_size,
			      unsigned long ro_start,
			      unsigned long ro_limit,
			      unsigned long coh_start,
			      unsigned long coh_limit);

void bl1_plat_arch_setup(void)
{
	baikal_configure_mmu_bl1(BL1_RW_BASE, BL1_RW_LIMIT - BL1_RW_BASE,
				 BL1_RO_BASE, BL1_RO_LIMIT,
				 BL_COHERENT_RAM_BASE, BL_COHERENT_RAM_END);
}

void bl1_platform_setup(void)
{
	extern uint8_t bl1_logo[];

	/*
	 * Initialize Interconnect for this cluster during cold boot.
	 * No need for locks as no other CPU is active.
	 */
	plat_arm_interconnect_init();

	/* Enable Interconnect coherency for the primary CPU's cluster */
	plat_arm_interconnect_enter_coherency();

	write_cntfrq_el0(plat_get_syscnt_freq2());
	generic_delay_timer_init();

	plat_baikal_io_setup();
	dw_spi_init(0, 0);
	mmxgbe_init();
	hdmi_early_splash(bl1_logo);
}
