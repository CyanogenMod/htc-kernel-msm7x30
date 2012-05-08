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

#ifndef __HTC_FLASHLIGHT_H
#define __HTC_FLASHLIGHT_H
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
	FL_MODE_CAMERA_EFFECT_FLASH,
	FL_MODE_CAMERA_EFFECT_PRE_FLASH,
	FL_MODE_FLASH_LEVEL1,
	FL_MODE_FLASH_LEVEL2,
//HTC_START_Simon.Ti_Liu_20120209 linear led
	FL_MODE_FLASH_LEVEL3,
	FL_MODE_FLASH_LEVEL4,
	FL_MODE_FLASH_LEVEL5,
	FL_MODE_FLASH_LEVEL6,
	FL_MODE_FLASH_LEVEL7,
//HTC_END

};
#if defined(CONFIG_FLASHLIGHT_AAT1271) || defined(CONFIG_LEDS_MAX8957_FLASH)
struct flashlight_platform_data {
	void (*gpio_init) (void);
	uint32_t torch;
	uint32_t flash;
	uint32_t flash_duration_ms;
	uint8_t led_count; /* 0: 1 LED, 1: 2 LED */

};
int aat1271_flashlight_control(int mode);
#endif


#ifdef CONFIG_FLASHLIGHT_AAT1277
struct flashlight_platform_data {
	void (*gpio_init) (void);
	uint32_t torch;
	uint32_t flash;
	uint32_t torch_set1;
	uint32_t torch_set2;
	uint32_t flash_duration_ms;
};
int aat1277_flashlight_control(int mode);
#endif


#ifdef CONFIG_FLASHLIGHT_AAT3177
struct flashlight_platform_data {
	void (*gpio_init) (void);
	uint32_t flash;
	uint32_t flash_duration_ms;

};
int aat3177_flashlight_control(int mode);
#endif


#ifdef CONFIG_FLASHLIGHT_TPS61310
struct TPS61310_flashlight_platform_data {
	void (*gpio_init) (void);
	uint32_t flash_duration_ms;
	uint8_t led_count; /* 0: 1 LED, 1: 2 LED */
	uint32_t tps61310_strb0;
	uint32_t tps61310_strb1;
};
int tps61310_flashlight_control(int mode);
#endif

#undef __HTC_FLASHLIGHT_H
#endif
