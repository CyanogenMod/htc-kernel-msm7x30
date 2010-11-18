/*
 * arch/arm/mach-msm/include/mach/msm_flashlight.h - The flashlight header
 * Copyright (C) 2009  HTC Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __ASM_ARCH_MSM8X50_FLASHLIGHT_H
#define __ASM_ARCH_MSM8X50_FLASHLIGHT_H
#include <linux/earlysuspend.h>

#define FLASHLIGHT_NAME "flashlight"

#define FLASHLIGHT_OFF   0
#define FLASHLIGHT_TORCH 1
#define FLASHLIGHT_FLASH 2
#define FLASHLIGHT_NUM   3

enum flashlight_mode_flags {
	FL_MODE_OFF = 0,
	FL_MODE_TORCH,
	FL_MODE_FLASH,
	FL_MODE_PRE_FLASH,
	FL_MODE_TORCH_LED_A,
	FL_MODE_TORCH_LED_B,
	FL_MODE_TORCH_LEVEL_1,
	FL_MODE_TORCH_LEVEL_2,
};

struct flashlight_platform_data {
	void (*gpio_init) (void);
	uint32_t torch;
	uint32_t flash;
	uint32_t flash_adj;
	uint32_t flash_duration_ms;
	uint8_t led_count; /* 0: 1 LED, 1: 2 LED */
};

int aat1271_flashlight_control(int mode);
int adp1650_flashlight_control(int mode);

#undef __ASM_ARCH_MSM8X50_FLASHLIGHT_H
#endif
