/*
 * Copyright (c) 2018-2021, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SPI_DW_LIST_H
#define SPI_DW_LIST_H

#define FLASH_INFO(_jedec_id, _sector_size, _n_sectors)	\
	.id = { ((_jedec_id) >> 16) & 0xff,		\
		((_jedec_id) >>  8) & 0xff,		\
		((_jedec_id) >>  0) & 0xff },		\
	.sector_size = (_sector_size) * 1024,		\
	.n_sectors = (_n_sectors),

const struct flash_info spi_nor_ids[] = {
    /* Atmel */
    { "at25fs010",   FLASH_INFO(0x1f6601,  32,    4) },
    { "at25fs040",   FLASH_INFO(0x1f6604,  64,    8) },
    { "at25df041a",  FLASH_INFO(0x1f4401,  64,    8) },
    { "at25df321",   FLASH_INFO(0x1f4700,  64,   64) },
    { "at25df321a",  FLASH_INFO(0x1f4701,  64,   64) },
    { "at25df641",   FLASH_INFO(0x1f4800,  64,  128) },
    { "at25sl321",   FLASH_INFO(0x1f4216,  64,   64) },
    { "at26f004",    FLASH_INFO(0x1f0400,  64,    8) },
    { "at26df081a",  FLASH_INFO(0x1f4501,  64,   16) },
    { "at26df161a",  FLASH_INFO(0x1f4601,  64,   32) },
    { "at26df321",   FLASH_INFO(0x1f4700,  64,   64) },
    { "at45db081d",  FLASH_INFO(0x1f2500,  64,   16) },

    /* EON */
    { "en25f32",     FLASH_INFO(0x1c3116,  64,   64) },
    { "en25p32",     FLASH_INFO(0x1c2016,  64,   64) },
    { "en25q32b",    FLASH_INFO(0x1c3016,  64,   64) },
    { "en25p64",     FLASH_INFO(0x1c2017,  64,  128) },
    { "en25q64",     FLASH_INFO(0x1c3017,  64,  128) },
    { "en25q80a",    FLASH_INFO(0x1c3014,  64,   16) },
    { "en25qh16",    FLASH_INFO(0x1c7015,  64,   32) },
    { "en25qh32",    FLASH_INFO(0x1c7016,  64,   64) },
    { "en25qh64",    FLASH_INFO(0x1c7017,  64,  128) },
    { "en25qh128",   FLASH_INFO(0x1c7018,  64,  256) },
    { "en25qh256",   FLASH_INFO(0x1c7019,  64,  512) },
    { "en25s64",     FLASH_INFO(0x1c3817,  64,  128) },

    /* ESMT */
    { "f25l32pa",    FLASH_INFO(0x8c2016,  64,   64) },
    { "f25l32qa",    FLASH_INFO(0x8c4116,  64,   64) },
    { "f25l64qa",    FLASH_INFO(0x8c4117,  64,  128) },

    /* GigaDevice */
    { "gd25q16",     FLASH_INFO(0xc84015,  64,   32) },
    { "gd25q32",     FLASH_INFO(0xc84016,  64,   64) },
    { "gd25lq32",    FLASH_INFO(0xc86016,  64,   64) },
    { "gd25q64",     FLASH_INFO(0xc84017,  64,  128) },
    { "gd25lq64c",   FLASH_INFO(0xc86017,  64,  128) },
    { "gd25lq128d",  FLASH_INFO(0xc86018,  64,  256) },
    { "gd25q128",    FLASH_INFO(0xc84018,  64,  256) },
    { "gd25q256",    FLASH_INFO(0xc84019,  64,  512) },

    /* Fujitsu */
    { "mb85rs1mt",   FLASH_INFO(0x047f27, 128,    1) },

    /* Intel/Numonyx */
    { "160s33b",     FLASH_INFO(0x898911,  64,   32) },
    { "320s33b",     FLASH_INFO(0x898912,  64,   64) },
    { "640s33b",     FLASH_INFO(0x898913,  64,  128) },

    /* ISSI */
    { "is25cd512",   FLASH_INFO(0x7f9d20,  32,    2) },
    { "is25lq040b",  FLASH_INFO(0x9d4013,  64,    8) },
    { "is25lp016d",  FLASH_INFO(0x9d6015,  64,   32) },
    { "is25lp080d",  FLASH_INFO(0x9d6014,  64,   16) },
    { "is25lp032",   FLASH_INFO(0x9d6016,  64,   64) },
    { "is25lp064",   FLASH_INFO(0x9d6017,  64,  128) },
    { "is25lp128",   FLASH_INFO(0x9d6018,  64,  256) },
    { "is25lp256",   FLASH_INFO(0x9d6019,  64,  512) },
    { "is25wp032",   FLASH_INFO(0x9d7016,  64,   64) },
    { "is25wp064",   FLASH_INFO(0x9d7017,  64,  128) },
    { "is25wp128",   FLASH_INFO(0x9d7018,  64,  256) },
    { "is25wp256",   FLASH_INFO(0x9d7019,  64,  512) },

    /* Macronix */
    { "mx25l512e",   FLASH_INFO(0xc22010,  64,    1) },
    { "mx25l2005a",  FLASH_INFO(0xc22012,  64,    4) },
    { "mx25l4005a",  FLASH_INFO(0xc22013,  64,    8) },
    { "mx25l8005",   FLASH_INFO(0xc22014,  64,   16) },
    { "mx25l1606e",  FLASH_INFO(0xc22015,  64,   32) },
    { "mx25l3205d",  FLASH_INFO(0xc22016,  64,   64) },
    { "mx25l3255e",  FLASH_INFO(0xc29e16,  64,   64) },
    { "mx25l6405d",  FLASH_INFO(0xc22017,  64,  128) },
    { "mx25u2033e",  FLASH_INFO(0xc22532,  64,    4) },
    { "mx25u3235f",  FLASH_INFO(0xc22536,  64,   64) },
    { "mx25u4035",   FLASH_INFO(0xc22533,  64,    8) },
    { "mx25u8035",   FLASH_INFO(0xc22534,  64,   16) },
    { "mx25u6435f",  FLASH_INFO(0xc22537,  64,  128) },
    { "mx25l12805d", FLASH_INFO(0xc22018,  64,  256) },
    { "mx25l12855e", FLASH_INFO(0xc22618,  64,  256) },
    { "mx25r1635f",  FLASH_INFO(0xc22815,  64,   32) },
    { "mx25r3235f",  FLASH_INFO(0xc22816,  64,   64) },
    { "mx25u12835f", FLASH_INFO(0xc22538,  64,  256) },
    { "mx25l25635e", FLASH_INFO(0xc22019,  64,  512) },
    { "mx25u25635f", FLASH_INFO(0xc22539,  64,  512) },
    { "mx25u51245g", FLASH_INFO(0xc2253a,  64,  102) },
    { "mx25v8035f",  FLASH_INFO(0xc22314,  64,   16) },
    { "mx25l25655e", FLASH_INFO(0xc22619,  64,  512) },
    { "mx25l51245g", FLASH_INFO(0xc2201a,  64, 1024) },
    { "mx66l51235l", FLASH_INFO(0xc2201a,  64, 1024) },
    { "mx66u51235f", FLASH_INFO(0xc2253a,  64, 1024) },
    { "mx66l1g45g",  FLASH_INFO(0xc2201b,  64, 2048) },
    { "mx66l1g55g",  FLASH_INFO(0xc2261b,  64, 2048) },
    { "mx66u2g45g",  FLASH_INFO(0xc2253c,  64, 4096) },

    /* Micron */
    { "n25q016a",    FLASH_INFO(0x20bb15,  64,   32) },
    { "n25q032",     FLASH_INFO(0x20ba16,  64,   64) },
    { "n25q032a",    FLASH_INFO(0x20bb16,  64,   64) },
    { "n25q064",     FLASH_INFO(0x20ba17,  64,  128) },
    { "n25q064a",    FLASH_INFO(0x20bb17,  64,  128) },
    { "n25q128a11",  FLASH_INFO(0x20bb18,  64,  256) },
    { "n25q128a13",  FLASH_INFO(0x20ba18,  64,  256) },
    { "n25q256a",    FLASH_INFO(0x20ba19,  64,  512) },
    { "n25q512a",    FLASH_INFO(0x20bb20,  64, 1024) },
    { "n25q256ax1",  FLASH_INFO(0x20bb19,  64,  512) },
    { "n25q512ax3",  FLASH_INFO(0x20ba20,  64, 1024) },
    { "n25q00",      FLASH_INFO(0x20ba21,  64, 2048) },
    { "n25q00a",     FLASH_INFO(0x20bb21,  64, 2048) },
    { "mt25ql02g",   FLASH_INFO(0x20ba22,  64, 4096) },
    { "mt25qu02g",   FLASH_INFO(0x20bb22,  64, 4096) },
    { "mt35xu512aba",FLASH_INFO(0x2c5b1a,  128, 512) },
    { "mt35xu02g",   FLASH_INFO(0x2c5b1c,  128*1024, 2048) },

    /* PMC */
    { "pm25lq032",   FLASH_INFO(0x7f9d46,  64,   64) },

    /* SST */
    { "sst25vf040b", FLASH_INFO(0xbf258d,  64,    8) },
    { "sst25vf080b", FLASH_INFO(0xbf258e,  64,   16) },
    { "sst25vf016b", FLASH_INFO(0xbf2541,  64,   32) },
    { "sst25vf032b", FLASH_INFO(0xbf254a,  64,   64) },
    { "sst25vf064c", FLASH_INFO(0xbf254b,  64,  128) },
    { "sst25wf512",  FLASH_INFO(0xbf2501,  64,    1) },
    { "sst25wf010",  FLASH_INFO(0xbf2502,  64,    2) },
    { "sst25wf020",  FLASH_INFO(0xbf2503,  64,    4) },
    { "sst25wf020a", FLASH_INFO(0x621612,  64,    4) },
    { "sst25wf040b", FLASH_INFO(0x621613,  64,    8) },
    { "sst25wf040",  FLASH_INFO(0xbf2504,  64,    8) },
    { "sst25wf080",  FLASH_INFO(0xbf2505,  64,   16) },
    { "sst26wf016b", FLASH_INFO(0xbf2651,  64,   32) },
    { "sst26vf016b", FLASH_INFO(0xbf2641,  64,   32) },
    { "sst26vf064b", FLASH_INFO(0xbf2643,  64,  128) },

    /* ST Microelectronics */
    { "m25p05",      FLASH_INFO(0x202010,  32,    2) },
    { "m25p10",      FLASH_INFO(0x202011,  32,    4) },
    { "m25p20",      FLASH_INFO(0x202012,  64,    4) },
    { "m25p40",      FLASH_INFO(0x202013,  64,    8) },
    { "m25p80",      FLASH_INFO(0x202014,  64,   16) },
    { "m25p16",      FLASH_INFO(0x202015,  64,   32) },
    { "m25p32",      FLASH_INFO(0x202016,  64,   64) },
    { "m25p64",      FLASH_INFO(0x202017,  64,  128) },
    { "m25p128",     FLASH_INFO(0x202018, 256,   64) },
    { "m45pe10",     FLASH_INFO(0x204011,  64,    2) },
    { "m45pe80",     FLASH_INFO(0x204014,  64,   16) },
    { "m45pe16",     FLASH_INFO(0x204015,  64,   32) },
    { "m25pe20",     FLASH_INFO(0x208012,  64,    4) },
    { "m25pe80",     FLASH_INFO(0x208014,  64,   16) },
    { "m25pe16",     FLASH_INFO(0x208015,  64,   32) },
    { "m25px16",     FLASH_INFO(0x207115,  64,   32) },
    { "m25px32",     FLASH_INFO(0x207116,  64,   64) },
    { "m25px32-s0",  FLASH_INFO(0x207316,  64,   64) },
    { "m25px32-s1",  FLASH_INFO(0x206316,  64,   64) },
    { "m25px64",     FLASH_INFO(0x207117,  64,  128) },
    { "m25px80",     FLASH_INFO(0x207114,  64,   16) },

    /* Winbond */
    { "w25x05",      FLASH_INFO(0xef3010,  64,    1) },
    { "w25x10",      FLASH_INFO(0xef3011,  64,    2) },
    { "w25x20",      FLASH_INFO(0xef3012,  64,    4) },
    { "w25x40",      FLASH_INFO(0xef3013,  64,    8) },
    { "w25x80",      FLASH_INFO(0xef3014,  64,   16) },
    { "w25x16",      FLASH_INFO(0xef3015,  64,   32) },
    { "w25q16dw",    FLASH_INFO(0xef6015,  64,   32) },
    { "w25x32",      FLASH_INFO(0xef3016,  64,   64) },
    { "w25q20cl",    FLASH_INFO(0xef4012,  64,    4) },
    { "w25q20bw",    FLASH_INFO(0xef5012,  64,    4) },
    { "w25q20ew",    FLASH_INFO(0xef6012,  64,    4) },
    { "w25q32",      FLASH_INFO(0xef4016,  64,   64) },
    { "w25q32dw",    FLASH_INFO(0xef6016,  64,   64) },
    { "w25q32jv",    FLASH_INFO(0xef7016,  64,   64) },
    { "w25q32jwm",   FLASH_INFO(0xef8016,  64,   64) },
    { "w25x64",      FLASH_INFO(0xef3017,  64,  128) },
    { "w25q64",      FLASH_INFO(0xef4017,  64,  128) },
    { "w25q64dw",    FLASH_INFO(0xef6017,  64,  128) },
    { "w25q64jvm",   FLASH_INFO(0xef7017,  64,  128) },
    { "w25q128fw",   FLASH_INFO(0xef6018,  64,  256) },
    { "w25q128jv",   FLASH_INFO(0xef7018,  64,  256) },
    { "w25q80",      FLASH_INFO(0xef5014,  64,   16) },
    { "w25q80bl",    FLASH_INFO(0xef4014,  64,   16) },
    { "w25q128",     FLASH_INFO(0xef4018,  64,  256) },
    { "w25q256",     FLASH_INFO(0xef4019,  64,  512) },
    { "w25q256jvm",  FLASH_INFO(0xef7019,  64,  512) },
    { "w25q256jw",   FLASH_INFO(0xef6019,  64,  512) },
    { "w25m512jv",   FLASH_INFO(0xef7119,  64, 1024) },
    { "w25q16jv-im/jm", FLASH_INFO(0xef7015, 64,  32) },
    { "w25q256fw",   FLASH_INFO(0xef6019,  64,  512) },

    /* Spansion */
    { "s25sl004a",   FLASH_INFO(0x010212,  64,    8) },
    { "s25sl008a",   FLASH_INFO(0x010213,  64,   16) },
    { "s25sl016a",   FLASH_INFO(0x010214,  64,   32) },
    { "s25sl032a",   FLASH_INFO(0x010215,  64,   64) },
    { "s25sl064a",   FLASH_INFO(0x010216,  64,  128) },
    { "s25fl004k",   FLASH_INFO(0xef4013,  64,    8) },
    { "s25fl008k",   FLASH_INFO(0xef4014,  64,   16) },
    { "s25fl016k",   FLASH_INFO(0xef4015,  64,   32) },
    { "s25fl064k",   FLASH_INFO(0xef4017,  64,  128) },
    { "s25fl116k",   FLASH_INFO(0x014015,  64,   32) },
    { "s25fl132k",   FLASH_INFO(0x014016,  64,   64) },
    { "s25fl164k",   FLASH_INFO(0x014017,  64,  128) },
    { "s25fl204k",   FLASH_INFO(0x014013,  64,    8) },
    { "s25fl208k",   FLASH_INFO(0x014014,  64,   16) },
    { "s25fl064l",   FLASH_INFO(0x016017,  64,  128) },
    { "s25fl128l",   FLASH_INFO(0x016018,  64,  256) },
    { "s25fl256l",   FLASH_INFO(0x016019,  64,  512) }
};

#endif /* SPI_DW_LIST_H */
