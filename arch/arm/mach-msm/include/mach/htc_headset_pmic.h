/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_pmic.h
 *
 * HTC PMIC headset driver.
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

#ifndef HTC_HEADSET_PMIC_H
#define HTC_HEADSET_PMIC_H

#define DRIVER_HS_PMIC_RPC_KEY	(1 << 0)

struct htc_headset_pmic_platform_data {
	unsigned int driver_flag;
	unsigned int hpin_gpio;
	unsigned int hpin_irq;
	unsigned int key_enable_gpio;

	/* ADC tables */
	uint32_t adc_mic;
	uint32_t adc_remote[6];
	uint32_t adc_metrico[2];
};

struct htc_35mm_pmic_info {
	struct htc_headset_pmic_platform_data pdata;
	unsigned int hpin_irq_type;
	unsigned int hpin_debounce;
	struct wake_lock hs_wake_lock;
};

#endif
