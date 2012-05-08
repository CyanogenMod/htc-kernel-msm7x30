/* arch/arm/mach-msm/pm.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/types.h>
#include <linux/cpuidle.h>

/* This constant is used in bootloader to decide actions. */
#define RESTART_REASON_BOOT_BASE	0x77665500
#define RESTART_REASON_BOOTLOADER	(RESTART_REASON_BOOT_BASE | 0x00)
#define RESTART_REASON_REBOOT		(RESTART_REASON_BOOT_BASE | 0x01)
#define RESTART_REASON_RECOVERY		(RESTART_REASON_BOOT_BASE | 0x02)
#define RESTART_REASON_ERASE_EFS		(RESTART_REASON_BOOT_BASE | 0x03)
#define RESTART_REASON_RAMDUMP		(RESTART_REASON_BOOT_BASE | 0xAA)
#define RESTART_REASON_POWEROFF		(RESTART_REASON_BOOT_BASE | 0xBB)
#define RESTART_REASON_ERASE_FLASH	(RESTART_REASON_BOOT_BASE | 0xEF)

/*
   This restart constant is used for oem commands.
   The actual value is parsed from reboot commands.
   RIL FATAL will use oem-99 to restart a device.
*/
#define RESTART_REASON_OEM_BASE		0x6f656d00
#define RESTART_REASON_RIL_FATAL	(RESTART_REASON_OEM_BASE | 0x99)

#ifdef CONFIG_SMP
extern int pen_release;
extern void msm_secondary_startup(void);
#else
#define msm_secondary_startup NULL
#endif

enum msm_pm_sleep_mode {
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
	MSM_PM_SLEEP_MODE_APPS_SLEEP,
	MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT,
	MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE,
	MSM_PM_SLEEP_MODE_NR
};

#define MSM_PM_MODE(cpu, mode_nr)  ((cpu) * MSM_PM_SLEEP_MODE_NR + (mode_nr))

struct msm_pm_platform_data {
	u8 idle_supported;   /* Allow device to enter mode during idle */
	u8 suspend_supported; /* Allow device to enter mode during suspend */
	u8 suspend_enabled;  /* enabled for suspend */
	u8 idle_enabled;     /* enabled for idle low power */
	u32 latency;         /* interrupt latency in microseconds when entering
				and exiting the low power mode */
	u32 residency;       /* time threshold in microseconds beyond which
				staying in the low power mode saves power */
};


void msm_pm_set_platform_data(struct msm_pm_platform_data *data, int count);
int msm_pm_idle_prepare(struct cpuidle_device *dev);
int msm_pm_idle_enter(enum msm_pm_sleep_mode sleep_mode);

#ifdef CONFIG_PM
void msm_pm_set_rpm_wakeup_irq(unsigned int irq);
int msm_pm_platform_secondary_init(unsigned int cpu);
#else
static inline void msm_pm_set_rpm_wakeup_irq(unsigned int irq) {}
static inline int msm_pm_platform_secondary_init(unsigned int cpu)
{ return -ENOSYS; }
#endif
int print_gpio_buffer(struct seq_file *m);
int free_gpio_buffer(void);

extern int board_mfg_mode(void);
extern char *board_get_mfg_sleep_gpio_table(void);
extern void gpio_set_diag_gpio_table(unsigned long *dwMFG_gpio_table);

#endif  /* __ARCH_ARM_MACH_MSM_PM_H */
