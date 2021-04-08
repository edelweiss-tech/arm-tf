/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bm1000_private.h>
#include <common/debug.h>
#include <drivers/console.h>
#include <lib/psci/psci.h>
#include <lib/utils_def.h>
#include <libfdt.h>
#include <string.h>

static int append_psci_compatible(void *fdt, int offs, const char *str)
{
	return fdt_appendprop(fdt, offs, "compatible", str, strlen(str) + 1);
}

int dt_add_psci_node(void *fdt)
{
	int offs;

	if (fdt_path_offset(fdt, "/psci") >= 0) {
		WARN("PSCI Device Tree node already exists!\n");
		return 0;
	}

	offs = fdt_path_offset(fdt, "/");
	if (offs < 0)
		return -1;
	offs = fdt_add_subnode(fdt, offs, "psci");
	if (offs < 0)
		return -1;
	if (append_psci_compatible(fdt, offs, "arm,psci-1.0"))
		return -1;
	if (append_psci_compatible(fdt, offs, "arm,psci-0.2"))
		return -1;
	if (append_psci_compatible(fdt, offs, "arm,psci"))
		return -1;
	if (fdt_setprop_string(fdt, offs, "method", "smc"))
		return -1;
	if (fdt_setprop_u32(fdt, offs, "cpu_suspend", PSCI_CPU_SUSPEND_AARCH64))
		return -1;
	if (fdt_setprop_u32(fdt, offs, "cpu_off", PSCI_CPU_OFF))
		return -1;
	if (fdt_setprop_u32(fdt, offs, "cpu_on", PSCI_CPU_ON_AARCH64))
		return -1;
	if (fdt_setprop_u32(fdt, offs, "sys_poweroff", PSCI_SYSTEM_OFF))
		return -1;
	if (fdt_setprop_u32(fdt, offs, "sys_reset", PSCI_SYSTEM_RESET))
		return -1;
	return 0;
}

static int check_node_compat_prefix(void *fdt, int offs, const char *prefix)
{
	const size_t prefix_len = strlen(prefix);
	size_t l;
	int plen;
	const char *prop;

	prop = fdt_getprop(fdt, offs, "compatible", &plen);
	if (!prop)
		return -1;

	while (plen > 0) {
		if (memcmp(prop, prefix, prefix_len) == 0)
			return 0; /* match */

		l = strlen(prop) + 1;
		prop += l;
		plen -= l;
	}

	return -1;
}

int dt_add_psci_cpu_enable_methods(void *fdt)
{
	int offs = 0;

	while (1) {
		offs = fdt_next_node(fdt, offs, NULL);
		if (offs < 0)
			break;
		if (fdt_getprop(fdt, offs, "enable-method", NULL))
			continue; /* already set */
		if (check_node_compat_prefix(fdt, offs, "arm,cortex-a"))
			continue; /* no compatible */
		if (fdt_setprop_string(fdt, offs, "enable-method", "psci"))
			return -1;
		/* Need to restart scanning as offsets may have changed */
		offs = 0;
	}
	return 0;
}

int fdt_update_memory(void *fdt,
		      const uint64_t region_descs[][2],
		      const unsigned region_num)
{
	int err;
	int memoff;
	uint64_t memregs[3][2];
	unsigned region;

	if (region_num < 1 || region_num > ARRAY_SIZE(memregs)) {
		ERROR("%s: invalid number of memory regions\n", __func__);
		return -1;
	}

	memoff = fdt_path_offset(fdt, "/memory@80000000");
	if (memoff == -FDT_ERR_NOTFOUND) {
		memoff = fdt_add_subnode(fdt, 0, "memory@80000000");
		if (memoff < 0) {
			ERROR("%s: unable to add a memory subnode\n", __func__);
			return memoff;
		}

		err = fdt_setprop_string(fdt, memoff, "device_type", "memory");
		if (err) {
			ERROR("%s: unable to set device_type of a memory subnode\n",
			      __func__);
			return err;
		}
	}

	for (region = 0; region < region_num; ++region) {
		memregs[region][0] = cpu_to_fdt64(region_descs[region][0]);
		memregs[region][1] = cpu_to_fdt64(region_descs[region][1]);
	}

	err = fdt_setprop(fdt, memoff, "reg", memregs,
			  sizeof(memregs[0]) * region_num);
	if (err) {
		ERROR("FDT: unable to set reg property of a memory subnode\n");
		return err;
	}

	return 0;
}

int fdt_memory_node_read(uint64_t region_descs[3][2])
{
	void *fdt = (void *)(uintptr_t)PLAT_BAIKAL_DT_BASE;
	int memoff;
	unsigned region;
	const uint64_t *prop;
	int proplen;
	int ret;

	for (region = 0; region < 3; ++region) {
		region_descs[region][0] = 0;
		region_descs[region][1] = 0;
	}

	ret = fdt_open_into(fdt, fdt, PLAT_BAIKAL_DT_MAX_SIZE);
	if (ret < 0) {
		ERROR("%s: unable to open FDT\n", __func__);
		return ret;
	}

	memoff = fdt_path_offset(fdt, "/memory@80000000");
	if (memoff == -FDT_ERR_NOTFOUND) {
		ERROR("%s: node is not found\n", __func__);
		return memoff;
	}

	prop = fdt_getprop(fdt, memoff, "reg", &proplen);
	if (prop == NULL) {
		ERROR("%s: reg is not found\n", __func__);
		return -1;
	} else if (!proplen || (proplen % 16) || proplen > 48) {
		ERROR("%s: incorrect 'reg' property length\n", __func__);
		return -1;
	}

	for (region = 0; region < proplen / 16; ++region) {
		region_descs[region][0] = fdt64_to_cpu(prop[2 * region + 0]);
		region_descs[region][1] = fdt64_to_cpu(prop[2 * region + 1]);
	}

	return 0;
}
