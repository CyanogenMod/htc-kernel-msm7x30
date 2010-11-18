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

struct acoustic_ops {
	void (*enable_mic_bias)(int en, int shift);
	int (*support_audience)(void);
	int (*support_aic3254) (void);
	int (*support_back_mic) (void);
	void (*mic_disable) (int mic);
};

void acoustic_register_ops(struct acoustic_ops *ops);
int htc_acdb_transfer(void);

#endif

