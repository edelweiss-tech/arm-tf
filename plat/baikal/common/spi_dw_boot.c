/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <baikal_gpio32.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/utils_def.h>
#include <platform_def.h>
#include <spi_dw.h>
#include <stdint.h>
#include "spi_dw_list.h"

/* Registers */
#define SPI_PORT(p)	(SPI_BASE + (p) * SPI_OFFSET)
#define SPI_CTRLR0(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x00))
#define SPI_CTRLR1(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x04))
#define SPI_SPIENR(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x08))
#define SPI_SER(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x10))
#define SPI_BAUDR(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x14))
#define SPI_TXFTLR(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x18))
#define SPI_RXFTLR(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x1c))
#define SPI_SR(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x28))
#define SPI_IMR(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x2c))
#define SPI_DR(p)	*(volatile uint32_t *)((uintptr_t)(SPI_PORT(p) + 0x60))

/* CTRLR0 */
#define SPI_TMOD_OFFSET		8
#define SPI_TMOD_MASK		(3 << SPI_TMOD_OFFSET)
#define SPI_TMOD_TXRX		0
#define SPI_TMOD_TX		1
#define SPI_TMOD_RX		2
#define SPI_TMOD_EEPROMREAD	3

#define SPI_DFS_OFFSET		0
#define SPI_DFS_MASK		(3 << SPI_DFS_OFFSET)
#define SPI_DFS(x)		(x - 1)

/* SPIENR */
#define SPI_SPIENR_SPI_DE	0
#define SPI_SPIENR_SPI_EN	1

/* SR */
#define SPI_SR_BUSY		BIT(0)
#define SPI_SR_TFNF		BIT(1)
#define SPI_SR_TFE		BIT(2)
#define SPI_SR_RFNE		BIT(3)

/* SPI Flash status register */
#define SPI_FLASH_SR_WIP	BIT(0) /* Write In Progress */
#define SPI_FLASH_SR_WEL	BIT(1) /* Write Enable Latch */

/* SPI Flash flag status register */
#define SPI_FLAG_4BYTE		BIT(0) /* 4-byte Address Enabling */
#define SPI_FLAG_PROTECTION	BIT(1) /* Protection */
#define SPI_FLAG_PR_SUSPEND	BIT(2) /* Program Suspend */
#define SPI_FLAG_VPP		BIT(3) /* VPP */
#define SPI_FLAG_PROGRAM	BIT(4) /* Program */
#define SPI_FLAG_ERASE		BIT(5) /* Erase */
#define SPI_FLAG_ER_SUSPEND	BIT(6) /* Erase Suspend */
#define SPI_FLAG_PE		BIT(7) /* P/E Controller (not WIP) */

/* SPI Flash commands */
#define CMD_FLASH_RDID		0x9f /* (0, 1-20) Read identification */
#define CMD_FLASH_READ		0x03 /* (3, 1-∞ ) Read Data Bytes */
#define CMD_FLASH_WREN		0x06 /* (0, 0   ) Write Enable */
#define CMD_FLASH_WRDI		0x04 /* (0, 0   ) Write Disable */
#define CMD_FLASH_PP		0x02 /* (3, 256 ) Page Program */
#define CMD_FLASH_SSE		0x20 /* (3, 0   ) SubSector Erase */
#define CMD_FLASH_SE		0xd8 /* (3, 0   ) Sector Erase */
#define CMD_FLASH_RDSR		0x05 /* (0, 1   ) Read Status Register */
#define CMD_FLASH_WRSR		0x01 /* (0, 1-∞ ) Write Status Register */
#define CMD_FLASH_RDLR		0xe8 /* (3, 1-∞ ) Read Lock Register */
#define CMD_FLASH_WRLR		0xe5 /* (3, 1   ) Write Lock Register */
#define CMD_FLASH_RFSR		0x70 /* (1 to ∞)  Read Flag Status Register */
#define CMD_FLASH_CLFSR		0x50 /* (0) Clear Flag Status Register */
#define CMD_FLASH_BE		0xc7 /* (0) Bulk Erase */
#define CMD_FLASH_RSTEN		0x66 /* Reset Enable */
#define CMD_FLASH_RST		0x99 /* Reset Memory */
#define CMD_FLASH_EN4BYTEADDR	0xb7 /* Enter 4-byte address mode */
#define CMD_FLASH_EX4BYTEADDR	0xe9 /* Exit 4-byte address mode */
#define CMD_FLASH_WREAR		0xc5 /* Write Extended Address Register */
#define CMD_FLASH_RDEAR		0xc8 /* Read Extended Address Register */

/* Put address */
#define ADR_MODE_3BYTE		3
#define SPI_ADR_LEN_3BYTE	3
#define SPI_SET_ADDRESS_3BYTE(a, b)		\
({	uint8_t *_b = (void *)(b);		\
	_b[1] = (((a) >> 8 * 2) & 0xff);	\
	_b[2] = (((a) >> 8 * 1) & 0xff);	\
	_b[3] = (((a) >> 8 * 0) & 0xff);	\
})

#define ADR_MODE_4BYTE		4
#define SPI_ADR_LEN_4BYTE	4
#define SPI_SET_ADDRESS_4BYTE(a, b)		\
({	uint8_t *_b = (void *)(b);		\
	_b[1] = (((a) >> 8 * 3) & 0xff);	\
	_b[2] = (((a) >> 8 * 2) & 0xff);	\
	_b[3] = (((a) >> 8 * 1) & 0xff);	\
	_b[4] = (((a) >> 8 * 0) & 0xff);	\
})

#define SPI_AUTO_READ_SIZE	UL(0x10000)
#define SPI_BAUDR_SCKDV		32
#define SPI_CMD_LEN		(1 + SPI_ADR_LEN_4BYTE)
#define SPI_PAGE_SIZE		UL(256) /* (3, 256) Page Program */

/* Chip select GPIO */
#if defined(BE_DBM) || defined(BE_QEMU)
#define CS_GPIO	24
#elif defined(BE_MBM10_2FLASH)
#define CS_GPIO	15
#endif

static int adr_mode;

static int transfer(int port, int line,
		    void *cmd_,	uint32_t cmd_len,
		    void *tx_,	uint32_t tx_len,
		    void *rx_,	uint32_t rx_len)
{
	int err = -1;
	uint8_t *cmd = cmd_;
	uint8_t *tx = tx_;
	uint8_t *rx = rx_;
	uint8_t *cmdend	= (void *)((intptr_t)cmd + (intptr_t)cmd_len);
	uint8_t *txend	= (void *)((intptr_t)tx	 + (intptr_t)tx_len);
	uint8_t *rxend	= (void *)((intptr_t)rx	 + (intptr_t)rx_len);
	uint64_t timeout;

	if (cmd == NULL || !cmd_len) {
		ERROR("SPI: %s: incorrect args\n", __func__);
		return -1;
	}

	gpio32_dir_set(CS_GPIO);
	gpio32_out_set(CS_GPIO);

	line = 1 << line;

	SPI_SPIENR(port) = SPI_SPIENR_SPI_DE;
	SPI_SER(port) = 0; /* disable all lines */
	SPI_CTRLR0(port) &= ~SPI_TMOD_MASK;

	if (rx_len) {
		/* mode: read */
		SPI_CTRLR0(port) |= SPI_TMOD_EEPROMREAD << SPI_TMOD_OFFSET;
	} else {
		/* mode: write */
		SPI_CTRLR0(port) |= SPI_TMOD_TX << SPI_TMOD_OFFSET;
	}

	switch ((SPI_CTRLR0(port) & SPI_TMOD_MASK) >> SPI_TMOD_OFFSET) {
	case SPI_TMOD_TX:
		SPI_SPIENR(port) = SPI_SPIENR_SPI_EN; /* enable FIFO */

		while (cmd != cmdend) {
			if (SPI_SR(port) & SPI_SR_TFNF) {
				SPI_DR(port) = *cmd; /* push cmd */
				cmd++;
			} else {
				timeout = timeout_init_us(100000);

				while (!(SPI_SR(port) & SPI_SR_TFNF)) {
					if (timeout_elapsed(timeout)) {
						ERROR("SPI: timeout, %s:%u\n",
						      __func__, __LINE__);
						goto exit;
					}
				}
			}
		}

		while (tx != txend && (SPI_SR(port) & SPI_SR_TFNF)) {
			SPI_DR(port) = *tx; /* push tx */
			tx++;
		}

		SPI_SER(port) = line; /* start sending */

		while (tx != txend) {
			if (SPI_SR(port) & SPI_SR_TFNF) {
				SPI_DR(port) = *tx;
				tx++;
				SPI_SER(port) = line; /* restart if dropped */
			} else {
				timeout = timeout_init_us(100000);

				while (!(SPI_SR(port) & SPI_SR_TFNF)) {
					if (timeout_elapsed(timeout)) {
						ERROR("SPI: timeout, %s:%u\n",
						      __func__, __LINE__);
						goto exit;
					}
				}
			}
		}

		for (timeout = timeout_init_us(100000);;) {
			unsigned status = SPI_SR(port);
			if ((status & SPI_SR_TFE) && !(status & SPI_SR_BUSY)) {
				break;
			} else if (timeout_elapsed(timeout)) {
				ERROR("SPI: timeout, %s:%u\n",
				      __func__, __LINE__);
				goto exit;
			}
		}

		break;

	case SPI_TMOD_EEPROMREAD:
		if (rx == NULL || !rx_len || rx_len > SPI_AUTO_READ_SIZE) {
			ERROR("SPI: %s: eeprom\n", __func__);
			goto exit;
		}

		SPI_CTRLR1(port) = rx_len - 1; /* set read size */
		SPI_SPIENR(port) = SPI_SPIENR_SPI_EN; /* enable FIFO */

		while (cmd != cmdend) {
			if (SPI_SR(port) & SPI_SR_TFNF) {
				SPI_DR(port) = *cmd; /* push cmd */
				cmd++;
			} else {
				timeout = timeout_init_us(100000);

				while (!(SPI_SR(port) & SPI_SR_TFNF)) {
					if (timeout_elapsed(timeout)) {
						ERROR("SPI: timeout, %s:%u\n",
						      __func__, __LINE__);
						goto exit;
					}
				}
			}
		}

		SPI_SER(port) = line; /* start sending */

		while (rx != rxend) { /* read incoming data */
			if (SPI_SR(port) & SPI_SR_RFNE) {
				*rx = SPI_DR(port);
				rx++;
			} else {
				timeout = timeout_init_us(100000);

				while (!(SPI_SR(port) & SPI_SR_RFNE)) {
					if (timeout_elapsed(timeout)) {
						ERROR("SPI: timeout, %s:%u\n",
						      __func__, __LINE__);
						goto exit;
					}
				}
			}
		}

		break;

	case SPI_TMOD_TXRX:
	case SPI_TMOD_RX:
	default:
		ERROR("SPI: %s: mode\n", __func__);
		goto exit;
	}

	err = 0;

exit:
	if (err)
		ERROR("Transfer error: cmd %x, status %x\n", *(uint8_t *)cmd_, SPI_SR(port));
	gpio32_out_rst(CS_GPIO);
	return err;
}

static int exec(int port, int line, uint8_t cmd_op, uint32_t address,
		void *buf, uint32_t lenbuf)
{
	uint8_t cmd[SPI_CMD_LEN];
	uint8_t *in = 0, *out = 0;
	uint32_t lencmd = 0, lenin = 0, lenout = 0;

	/* Save the SPI flash instruction */
	cmd[0] = cmd_op;
	lencmd += 1;

	/* Prepare arguments for the SPI transaction */
	switch (cmd_op) {
	case CMD_FLASH_RDID:  /* Read identification */
	case CMD_FLASH_RDSR:  /* Read Status Register */
	case CMD_FLASH_RDEAR: /* Read Extended Address Register */
	case CMD_FLASH_RFSR:  /* Read Flag Status Register */
		out = buf;
		lenout = lenbuf;
		break;

	case CMD_FLASH_READ: /* Read Data Bytes */
		out = buf;
		lenout = lenbuf;
	case CMD_FLASH_SSE:  /* SubSector Erase */
	case CMD_FLASH_SE:   /* Sector Erase */
		if (adr_mode == ADR_MODE_4BYTE) {
			SPI_SET_ADDRESS_4BYTE(address, cmd);
			lencmd += SPI_ADR_LEN_4BYTE;
		} else if (adr_mode == ADR_MODE_3BYTE) {
			SPI_SET_ADDRESS_3BYTE(address, cmd);
			lencmd += SPI_ADR_LEN_3BYTE;
		} else {
			ERROR("SPI: %s: incorrect address mode\n", __func__);
			return -1;
		}

		break;

	case CMD_FLASH_WREAR:
	case CMD_FLASH_WRSR: /* Write Status Register */
		in = buf;
		lenin = 1;
		break;

	case CMD_FLASH_EN4BYTEADDR:
	case CMD_FLASH_EX4BYTEADDR:
	case CMD_FLASH_WRDI: /* Write Disable */
	case CMD_FLASH_WREN: /* Write Enable */
	case CMD_FLASH_BE:   /* Bulk Erase */
		break;

	case CMD_FLASH_PP:   /* Page Program */
		if (lenbuf > SPI_PAGE_SIZE) {
			ERROR("SPI: %s: incorrect lenbuf: %u\n",
			      __func__, lenbuf);
			return -1;
		}

		in = buf;
		lenin = lenbuf;
		if (adr_mode == ADR_MODE_4BYTE) {
			SPI_SET_ADDRESS_4BYTE(address, cmd);
			lencmd += SPI_ADR_LEN_4BYTE;
		} else if (adr_mode == ADR_MODE_3BYTE) {
			SPI_SET_ADDRESS_3BYTE(address, cmd);
			lencmd += SPI_ADR_LEN_3BYTE;
		} else {
			ERROR("SPI: %s: incorrect address mode\n", __func__);
			return -1;
		}
		break;

	default:
		ERROR("SPI: %s: unknown cmd_op: 0x%x\n", __func__, cmd_op);
		return -1;
	}

	/* Execute the SPI transaction */
	return transfer(port, line,
			cmd, lencmd,
			in,  lenin,
			out, lenout);
}

static int status(int port, int line, void *status_)
{
	return exec(port, line, CMD_FLASH_RDSR, 0, status_, 1);
}

static int wren(int port, int line)
{
	int err;
	uint8_t st;

	err = exec(port, line, CMD_FLASH_WREN, 0, 0, 0);
	if (err) {
		return err;
	}

	err = status(port, line, &st);
	if (err) {
		return err;
	}

	return st & SPI_FLASH_SR_WEL ? 0 : 1;
}

static int wait(int port, int line)
{
	int err = -1;
	uint8_t st;
	uint64_t timeout;

	timeout = timeout_init_us(1000000);
	do {
		if (status(port, line, &st)) {
			goto exit;
		} else if (timeout_elapsed(timeout)) {
			ERROR("SPI: timeout, %s:%u\n", __func__, __LINE__);
			goto exit;
		}
	} while (st & SPI_FLASH_SR_WIP);

	err = 0;
exit:
	return err;
}

int dw_spi_erase(int port,
		 int line,
		 uint32_t adr,
		 size_t size,
		 size_t sector_size)
{
	int err;

	VERBOSE("SPI: %s(0x%x, 0x%lx, 0x%lx)\n",
		__func__, adr, size, sector_size);

	if (!sector_size) {
		ERROR("SPI: %s: incorrect sector_size: %lu\n",
		      __func__, sector_size);
		return -1;
	}

	while (size) {
		err = wren(port, line);
		if (err) {
			ERROR("SPI: %s: wren\n", __func__);
			return err;
		}

		err = exec(port, line, CMD_FLASH_SE, adr, 0, 0);
		if (err) {
			ERROR("SPI: %s: exec\n", __func__);
			return err;
		}

		err = wait(port, line);
		if (err) {
			ERROR("SPI: %s: wait\n", __func__);
			return err;
		}

		adr  += sector_size;
		size -= MIN(size, sector_size);
	}

	return 0;
}

int dw_spi_write(int port, int line, uint32_t adr, void *data, size_t size)
{
	int err;
	int part;
	char *pdata = data;

	VERBOSE("SPI: %s(0x%x, 0x%lx)\n", __func__, adr, size);

	while (size) {
		part = MIN(size, SPI_PAGE_SIZE);

		/* fix size */
		int p1 = adr / SPI_PAGE_SIZE; /* page number */
		int p2 = (adr + part) / SPI_PAGE_SIZE;
		if (p1 != p2) { /* page overflow ? */
			p2 *= SPI_PAGE_SIZE; /* page base address */
			part = p2 - adr;
		}

		err = wren(port, line);
		if (err) {
			ERROR("SPI: %s: wren\n", __func__);
			return err;
		}

		err = exec(port, line, CMD_FLASH_PP, adr, pdata, part);
		if (err) {
			ERROR("SPI: %s: exec\n", __func__);
			return err;
		}

		err = wait(port, line);
		if (err) {
			ERROR("SPI: %s: wait\n", __func__);
			return err;
		}

		adr   += part;
		pdata += part;
		size  -= part;
	}

	return 0;
}

int dw_spi_read(int port, int line, uint32_t adr, void *data, size_t size)
{
	int err;
	int part;
	char *pdata = data;

	VERBOSE("SPI: %s(0x%x, 0x%lx)\n", __func__, adr, size);

	while (size) {
		part = MIN(size, SPI_AUTO_READ_SIZE);
		err = exec(port, line, CMD_FLASH_READ, adr, pdata, part);
		if (err) {
			ERROR("SPI: %s: exec\n", __func__);
			return err;
		}

		adr   += part;
		pdata += part;
		size  -= part;
	}

	return 0;
}

int dw_spi_3byte(int port, int line)
{
	int err;

	VERBOSE("SPI: %s\n", __func__);

	err = wren(port, line);
	if (err) {
		ERROR("SPI: %s: wren\n", __func__);
		return err;
	}

	err = exec(port, line, CMD_FLASH_EX4BYTEADDR, 0, 0, 0);
	if (err) {
		ERROR("SPI: %s: exec\n", __func__);
		return err;
	}

	adr_mode = ADR_MODE_3BYTE;

	return 0;
}

int dw_spi_4byte(int port, int line)
{
	int err;

	VERBOSE("SPI: %s\n", __func__);

	err = wren(port, line);
	if (err) {
		ERROR("SPI: %s: wren\n", __func__);
		return err;
	}

	err = exec(port, line, CMD_FLASH_EN4BYTEADDR, 0, 0, 0);
	if (err) {
		ERROR("SPI: %s: exec\n", __func__);
		return err;
	}

	adr_mode = ADR_MODE_4BYTE;

	return 0;
}

static void dw_spi_init_regs(int port)
{
	VERBOSE("SPI: %s\n", __func__);

	SPI_SPIENR(port) = SPI_SPIENR_SPI_DE; /* disable device */

	SPI_CTRLR0(port) = 0;
	SPI_CTRLR0(port) |= SPI_DFS(8) << SPI_DFS_OFFSET;

	SPI_CTRLR1(port) = 0;
	SPI_BAUDR(port) = SPI_BAUDR_SCKDV;
	SPI_TXFTLR(port) = 0;
	SPI_RXFTLR(port) = 0;
	SPI_IMR(port) = 0;
	SPI_SER(port) = 0;
}

const struct flash_info *dw_spi_get_info(int port, int line)
{
	unsigned i;
	uint8_t id[SPI_NOR_MAX_ID_LEN];

	/* read */
	if (exec(port, line, CMD_FLASH_RDID, 0, id, sizeof(id))) {
		ERROR("SPI: error while reading JEDEC ID\n");
		return NULL;
	}

	/* find */
	for (i = 0; i < ARRAY_SIZE(spi_nor_ids); ++i) {
		const struct flash_info *info;
		info = &spi_nor_ids[i];
		if (!memcmp(info->id, id, sizeof(id))) {
			INFO("SPI Flash: %s, %u MiB, %u sectors x %u KiB\n",
			  info->name,
			  ((info->sector_size / 1024) * info->n_sectors) / 1024,
			  info->n_sectors, info->sector_size / 1024);

			return info;
		}
	}

	ERROR("unrecognized JEDEC ID bytes: %02x%02x%02x\n",
	      id[0], id[1], id[2]);

	return NULL;
}

void dw_spi_init(int port, int line)
{
	const struct flash_info *info;
	uint64_t total;

	VERBOSE("SPI: %s\n", __func__);
	dw_spi_init_regs(port);

	info = dw_spi_get_info(port, line);
	if (info == NULL) {
		ERROR("SPI: %s: info is NULL\n", __func__);
		return;
	}

	total = info->sector_size * info->n_sectors;
	if (total > 16 * 1024 * 1024) {
		dw_spi_4byte(port, line);
	} else {
		dw_spi_3byte(port, line);
	}
}
