/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _ARCH_ARM_MACH_MSM_PM_BOOT_H
#define _ARCH_ARM_MACH_MSM_PM_BOOT_H

enum {
	MSM_PM_BOOT_CONFIG_TZ		= 0,
	MSM_PM_BOOT_CONFIG_RESET_VECTOR	= 1,
};

#ifdef CONFIG_PM
int __init msm_pm_boot_init(int boot_config, uint32_t *address);
#else
static inline int __init msm_pm_boot_init(int boot_config, uint32_t* address)
{
	return 0;
}
#endif
void msm_pm_boot_config_before_pc(unsigned int cpu, unsigned long entry);
void msm_pm_boot_config_after_pc(unsigned int cpu);

#endif
