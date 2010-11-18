/* arch/arm/mach-msm/pm.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
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

#ifndef __ARCH_ARM_MACH_MSM_PM_H
#define __ARCH_ARM_MACH_MSM_PM_H

#include <mach/msm_iomap.h>

#define A11S_CLK_SLEEP_EN_ADDR MSM_CSR_BASE + 0x11c

#define CLK_SLEEP_EN_ARM11_CORE	0x01
#define CLK_SLEEP_EN_ARM11_AHB	0x02
#define CLK_SLEEP_EN_ID_BRIDGE	0x04
#define CLK_SLEEP_EN_DMA_BRIDGE	0x08
#define CLK_SLEEP_EN_PBUS	0x10
#define CLK_SLEEP_EN_DEBUG_TIME	0x20
#define CLK_SLEEP_EN_GP_TIMER	0x40

enum {
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
	MSM_PM_SLEEP_MODE_APPS_SLEEP,
	MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT,
	MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE,
	//MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN,
	MSM_PM_SLEEP_MODE_NR
};

struct msm_pm_platform_data {
	u8 supported;
	u8 suspend_enabled;  /* enabled for suspend */
	u8 idle_enabled;     /* enabled for idle low power */
	u32 latency;         /* interrupt latency in microseconds when entering
				and exiting the low power mode */
	u32 residency;       /* time threshold in microseconds beyond which
				staying in the low power mode saves power */
};

extern void msm_pm_set_platform_data(struct msm_pm_platform_data *data);

#endif
