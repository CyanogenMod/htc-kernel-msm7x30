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

#ifndef S5K6AAFX_H
#define S5K6AAFX_H

#include <linux/types.h>
#include <mach/camera.h>

/* S5K6AAFX Registers and their values */
/* Sensor Core Registers */
#define S5K6AAFX_REG_I2C_MODE 0xFCFC
#define  S5K6AAFX_I2C_MODE_SENSOR 0x0000
#define  S5K6AAFX_I2C_MODE_GENERAL 0xD000

#define  S5K6AAFX_REG_MODEL_ID 0x0152
#define  S5K6AAFX_MODEL_ID 0x06AA

/* Mode select register */
#define S5K6AAFX_REG_MODE_SELECT 0x107E
#define S5K6AAFX_MODE_SELECT_STREAM 0x0000
#define S5K6AAFX_MODE_SELECT_SW_STANDBY 0x0001

#define S5K6AAFX_ADDH_SW_REG_INT 0x7000
#define S5K6AAFX_REG_W_ADDH 0x0028
#define S5K6AAFX_REG_W_ADDL 0x002A
#define S5K6AAFX_REG_R_ADDH 0x002C
#define S5K6AAFX_REG_R_ADDL 0x002E
#define S5K6AAFX_REG_WR 0x0F12

#define S5K6AAFX_QTR_SIZE_WIDTH 0x0280
#define S5K6AAFX_QTR_SIZE_HEIGHT 0x01E0
#define S5K6AAFX_FULL_SIZE_WIDTH 0x0500
#define S5K6AAFX_FULL_SIZE_HEIGHT 0x0400

extern struct s5k6aafx_reg s5k6aafx_regs;

struct s5k6aafx_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

struct s5k6aafx_reg {
	const struct s5k6aafx_i2c_reg_conf *reset_init;
	uint16_t reset_init_size;
        const struct s5k6aafx_i2c_reg_conf *TP_init;
        uint16_t TP_init_size;
        const struct s5k6aafx_i2c_reg_conf *analog_setting_init;
        uint16_t analog_setting_init_size;
	const struct s5k6aafx_i2c_reg_conf *register_init;
	uint16_t register_init_size;
	const struct s5k6aafx_i2c_reg_conf *clk_init;
	uint16_t clk_init_size;
	const struct s5k6aafx_i2c_reg_conf *prev_snap_conf_init;
	uint16_t prev_snap_conf_init_size;
};

#endif /* S5K6AAFX_H */
