/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BM1000_PRIVATE_H
#define BM1000_PRIVATE_H

#include <stdint.h>

/* Bit handling */
#define SETMASK(x, u, l)	((x << l) & GENMASK(u, l))

/* LCRU register blocks */
#define LCRU_CMU0	0
#define LCRU_CMU1	0x10000
#define LCRU_CMU2	0x20000
#define LCRU_GPI	0x30000
#define LCRU_SREG	0x40000
#define LCRU_GPR	0x50000
#define LCRU_PVT	0x60000

#define LCRU_CLKCH_BASE		0x20
#define LCRU_CLKCH_STEP		0x10
#define LCRU_CLKCH_OFFSET(x)	(LCRU_CLKCH_BASE + (LCRU_CLKCH_STEP * x))

/* NIC options to move peripheral device to non-secure world */
#define SECURE_MODE_ON		0
#define SECURE_MODE_OFF		1

void baikal_configure_mmu_el1(unsigned long total_base, unsigned long total_size,
			      unsigned long ro_start, unsigned long ro_limit,
			      unsigned long coh_start, unsigned long coh_limit);

void baikal_configure_mmu_el3(unsigned long total_base, unsigned long total_size,
			      unsigned long ro_start, unsigned long ro_limit,
			      unsigned long coh_start, unsigned long coh_limit);

unsigned int plat_baikal_calc_core_pos(u_register_t mpidr);

int dt_add_psci_node(void *fdt);
int dt_add_psci_cpu_enable_methods(void *fdt);
int fdt_update_memory(void *fdt, const uint64_t region_descs[][2], const unsigned region_num);
int fdt_memory_node_read(uint64_t region_descs[3][2]);

void mmlsp_init(void);
void mmlsp_ns_access(void);

void mmmali_init(void);
void mmmali_ns_access(void);

void mmpcie_init(void);
void mmpcie_ns_access(void);

void mmusb_chc(void);
void mmusb_init(void);
void mmusb_init_sata(void);
void mmusb_ns_access(void);

void mmvdec_init(void);
void mmvdec_ns_access(void);

void mmxgbe_init(void);
void mmxgbe_ns_access(void);

#endif /* BM1000_PRIVATE_H */
