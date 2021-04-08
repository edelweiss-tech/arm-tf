/*
 * Copyright (c) 2020, Baikal Electronics, JSC. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BM1000_SMBUS_H
#define BM1000_SMBUS_H

unsigned smbus_txrx(const unsigned bus,
		    const unsigned addr,
		    const void *const txbuf,
		    const unsigned txbufsize,
		    void *const rxbuf,
		    const unsigned rxbufsize);

#endif /* BM1000_SMBUS_H */
