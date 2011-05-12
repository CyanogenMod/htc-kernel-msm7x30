/* include/asm/mach-msm/htc_acdb_7x30.h
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
#ifndef _ARCH_ARM_MACH_MSM_HTC_ACDB_7X30_H_
#define _ARCH_ARM_MACH_MSM_HTC_ACDB_7X30_H_

struct acdb_ops {
	uint32_t (*get_smem_size)(void);
	uint32_t (*get_acdb_radio_buffer_size)(void);
};

void acdb_register_ops(struct acdb_ops *ops);
#endif

