/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef CAMSENSOR_S5K4E1GX
#define CAMSENSOR_S5K4E1GX

#include <linux/types.h>
#include <mach/camera.h>

struct s5k4e1gx_i2c_reg_conf {
	unsigned short waddr;
	unsigned char  bdata;
};

struct s5k4e1gx_reg_t {
	struct s5k4e1gx_i2c_reg_conf *lc_preview;
	uint16_t lc_preview_size;
	struct s5k4e1gx_i2c_reg_conf *lc_capture;
	uint16_t lc_capture_size;
	struct s5k4e1gx_i2c_reg_conf *lc_common;
	uint16_t lc_common_size;
	struct s5k4e1gx_i2c_reg_conf *analog_settings_evt2;
	uint16_t analog_settings_evt2_size;
	struct s5k4e1gx_i2c_reg_conf *analog_settings_evt3;
	uint16_t analog_settings_evt3_size;
	struct s5k4e1gx_i2c_reg_conf *analog_settings_saga;
	uint16_t analog_settings_saga_size;
	/* 1126 for improve shutter of MIPI */
	struct s5k4e1gx_i2c_reg_conf *analog_settings_saga_zero_shutter;
	uint16_t analog_settings_saga_zero_shutter_size;
};

extern struct s5k4e1gx_reg_t s5k4e1gx_regs;

#endif /* CAMSENSOR_S5K4E1GX */
