/*
 * Copyright (c) 2020, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>

uint32_t crc32(const void *data, unsigned int size);

#endif /* CRC32_H */
