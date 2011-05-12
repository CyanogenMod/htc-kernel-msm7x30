/* include/linux/isl29028.h
 *
 * Copyright (C) 2010 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ISL29028_H
#define __ISL29028_H

#define ISL29028_I2C_NAME "isl29028"

#define ISL29028_CHIPID		0x00
#define ISL29028_CONFIGURE	0x01
#define ISL29028_INTERRUPT	0x02
#define ISL29028_PROX_LT	0x03
#define ISL29828_PROX_HT	0x04
#define ISL29028_LS_TH1		0x05
#define ISL29028_LS_TH2		0x06
#define ISL29028_LS_TH3		0x07
#define ISL29028_PROX_DATA	0x08
#define ISL29028_LS_DATA1	0x09
#define ISL29028_LS_DATA2	0x0A
#define ISL29028_TEST1		0x0E
#define ISL29028_TEST2		0x0F

#define ISL29028_PROX_EN	(1 << 7)
#define ISL29028_PROX_DR	(1 << 3)
#define ISL29028_ALS_EN		(1 << 2)
#define ISL29028_INT_PROX_FLAG	(1 << 7)
#define ISL29028_INT_ALS_FLAG	(1 << 3)
#define ISL29028_INT_ALS_PRST	(2 << 1)

#define ALS_CALIBRATED		0x6DA5
#define PS_CALIBRATED		0x5053

extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;

struct isl29028_platform_data {
	int intr;
	uint16_t levels[10];
	uint16_t golden_adc;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t lt;
	uint8_t ht;
	uint8_t debounce;
	uint8_t *mapping_table;
	uint8_t mapping_size;
	uint8_t enable_polling_ignore;
};

#endif
