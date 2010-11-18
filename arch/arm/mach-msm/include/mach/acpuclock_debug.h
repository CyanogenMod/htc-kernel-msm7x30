/* arch/arm/mach-msm/acpuclock_debug.h
 *
 * Copyright (C) 2010 HTC Corporation
 * Author: Eiven Peng <eiven_peng@htc.com>
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

#ifndef __ARCH_ARM_MACH_ACPUCLOCK_DEBUG_H
#define __ARCH_ARM_MACH_ACPUCLOCK_DEBUG_H

struct acpuclock_debug_dev {
	const char *name;
	void (*set_wfi_ramp_down) (int enable);
	void (*set_pwrc_ramp_down) (int enable);
	int (*get_wfi_ramp_down) (void);
	int (*get_pwrc_ramp_down) (void);
	unsigned int (*get_current_vdd) (void);
	int (*update_freq_tbl) (unsigned int acpu_khz, unsigned int acpu_vdd);
};

#ifndef CONFIG_ACPUCLOCK_DEBUG
static inline int register_acpuclock_debug_dev(
	struct acpuclock_debug_dev *acpu_dev)
{
	return -1;
}
#else
extern int register_acpuclock_debug_dev(
	struct acpuclock_debug_dev *acpu_dev);
#endif


#endif /* __ARCH_ARM_MACH_ACPUCLOCK_DEBUG_H */

