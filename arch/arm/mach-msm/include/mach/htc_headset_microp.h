/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_microp.h
 *
 * HTC Micro-P headset driver.
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

#ifndef HTC_HEADSET_MICROP_H
#define HTC_HEADSET_MICROP_H

struct htc_headset_microp_platform_data {
	/* Headset detection */
	int hpin_int;
	unsigned int hpin_irq;
	uint8_t hpin_mask[3];

	/* Remote key detection */
	int remote_int;
	unsigned int remote_irq;

	/* Remote key interrupt enable */
	unsigned int remote_enable_pin;

	/* ADC tables */
	uint8_t adc_channel;
	uint16_t adc_remote[6];
	uint16_t adc_metrico[2];
};

struct htc_headset_microp_info {
	struct htc_headset_microp_platform_data pdata;
	int hpin_gpio_mask;
	unsigned int hpin_debounce;
	struct wake_lock hs_wake_lock;
};

#endif
