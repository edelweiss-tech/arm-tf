/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <baikal_scp.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <errno.h>
#include <lib/mmio.h>
#include <platform_def.h>

#pragma pack(1)
struct scp_service {
	uint32_t	req_id;
	uint32_t	exec_id;
	uint8_t		op;
	uint8_t		st;
	uint16_t	size;
	uint32_t	offset;
	uint32_t	buf[0];
};
#pragma pack()

void *scp_buf(void)
{
	volatile struct scp_service *const scp =
		(volatile struct scp_service *const)SCP_SERVICE_BASE;

	return (void *)scp->buf;
}

static bool scp_busy(void)
{
	volatile struct scp_service *const scp =
		(volatile struct scp_service *const)SCP_SERVICE_BASE;

	return (mmio_read_32(SCP2AP_STATUS_R(0)) ||
		scp->req_id != scp->exec_id ||
		(scp->st != 'E' && scp->st != 'G'));
}

int scp_cmd(uint8_t op, uint32_t arg0, uint32_t arg1)
{
	volatile struct scp_service *const scp =
		(volatile struct scp_service *const)SCP_SERVICE_BASE;

	static uint32_t id = 1;
	char s_op[2] = {op, 0};
	uint64_t timeout;

	if (scp_busy()) {
		WARN("%s %s busy\n", __func__, s_op);
		return -EBUSY;
	}

	switch (op) {
	case 'V':
		scp->op = op;
		scp->buf[0] = arg0;
		scp->buf[1] = arg1;
		break;
	case 'R':
	case 'W':
	case 'E':
		scp->op = op;
		scp->offset = arg0 + FLASH_MAP_SCP;
		scp->size   = (uint16_t)arg1;
		break;
	case 'T':
	case 't':
		scp->op = op;
		break;
	default:
		WARN("%s %s unsupported\n", __func__, s_op);
		return -EFAULT;
	}

	scp->req_id = id++;

	dmbst();

	mmio_write_32(AP2SCP_SET_R(0), 1); /* send signal */

	timeout = timeout_init_us(1000000);
	while (scp_busy()) {
		if (timeout_elapsed(timeout)) {
			WARN("%s %s timeout\n", __func__, s_op);
			return -ETIMEDOUT;
		}
	}

	dmbsy();

	if (scp->st == 'G') {
		return 0;
	}

	WARN("%s %s failed\n", __func__, s_op);
	return -EFAULT;
}
