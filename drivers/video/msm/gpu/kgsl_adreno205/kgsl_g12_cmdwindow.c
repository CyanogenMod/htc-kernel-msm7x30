/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_log.h"
#include "kgsl_g12.h"

#include "g12_reg.h"
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/msm_kgsl.h>

#define KGSL_G12_CMDWINDOW_TARGET_MASK		0x000000FF
#define KGSL_G12_CMDWINDOW_ADDR_MASK		0x00FFFF00
#define KGSL_G12_CMDWINDOW_TARGET_SHIFT		0
#define KGSL_G12_CMDWINDOW_ADDR_SHIFT		8

int kgsl_g12_cmdwindow_write(struct kgsl_device *device,
		enum kgsl_cmdwindow_type target, unsigned int addr,
		unsigned int data)
{
	unsigned int cmdwinaddr;
	unsigned int cmdstream;

	KGSL_DRV_INFO("enter (device=%p,addr=%08x,data=0x%x)\n", device, addr,
			data);

	if (target < KGSL_CMDWINDOW_MIN ||
		target > KGSL_CMDWINDOW_MAX) {
		KGSL_DRV_ERR("dev %p invalid target\n", device);
		return -EINVAL;
	}

	if (!(device->flags & KGSL_FLAGS_INITIALIZED)) {
		KGSL_DRV_ERR("Trying to write uninitialized device.\n");
		return -EINVAL;
	}

	if (target == KGSL_CMDWINDOW_MMU)
		cmdstream = ADDR_VGC_MMUCOMMANDSTREAM;
	else
		cmdstream = ADDR_VGC_COMMANDSTREAM;

	cmdwinaddr = ((target << KGSL_G12_CMDWINDOW_TARGET_SHIFT) &
			KGSL_G12_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((addr << KGSL_G12_CMDWINDOW_ADDR_SHIFT) &
			KGSL_G12_CMDWINDOW_ADDR_MASK);

	kgsl_g12_regwrite(device, cmdstream >> 2, cmdwinaddr);
	kgsl_g12_regwrite(device, cmdstream >> 2, data);

	return 0;
}
