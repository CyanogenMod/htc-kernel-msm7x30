/* include/asm/mach-msm/htc_acoustic_7x30.h
 *
 * Copyright (C) 2010 HTC Corporation.
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
#ifndef _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_7X30_H_
#define _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_7X30_H_
#define PROPERTY_VALUE_MAX  92

struct acoustic_tables {
	char aic3254[PROPERTY_VALUE_MAX];
	char adie[PROPERTY_VALUE_MAX];
	char spkamp[PROPERTY_VALUE_MAX];
	char acdb[PROPERTY_VALUE_MAX];
	char aic3254_dsp[PROPERTY_VALUE_MAX];
};

struct acoustic_ops {
	void (*enable_mic_bias)(int en, int shift);
	int (*support_audience)(void);
	int (*support_aic3254) (void);
	int (*support_back_mic) (void);
	void (*mic_disable) (int mic);
	void (*mute_headset_amp) (int en);
	void (*get_acoustic_tables)(struct acoustic_tables *tb);
	void (*enable_back_mic) (int en);
	void (*enable_usb_headset)(int en);
};

void acoustic_register_ops(struct acoustic_ops *ops);
int htc_acdb_transfer(void);

#endif

