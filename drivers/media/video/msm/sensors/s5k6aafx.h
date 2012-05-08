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

#define S5K6AAFX_REG_3TC_PCFG_usFrTimeType 0x02C2
#define S5K6AAFX_REG_3TC_PCFG_usMaxFrTimeMsecMult10 0x02C6
#define S5K6AAFX_REG_3TC_PCFG_usMinFrTimeMsecMult10 0x02C8
#define S5K6AAFX_REG_TC_GP_PrevConfigChanged 0x021E

#define S5K6AAFX_QTR_SIZE_WIDTH 0x0280
#define S5K6AAFX_QTR_SIZE_HEIGHT 0x01E0
#ifdef CONFIG_MACH_VERDI_LTE    /* 1280 x 752 for Puccini */
#define S5K6AAFX_720P_SIZE_WIDTH 0x0500     /* 1280 */
#define S5K6AAFX_720P_SIZE_HEIGHT 0x02F0    /* 752 */
#else
#define S5K6AAFX_720P_SIZE_WIDTH 0x0500     /* 1280 */
#define S5K6AAFX_720P_SIZE_HEIGHT 0x02D0    /* 720 */
#endif
#define S5K6AAFX_FULL_SIZE_WIDTH 0x0500
#define S5K6AAFX_FULL_SIZE_HEIGHT 0x0400
#define S5K6AAFX_ADJ_FULL_SIZE_WIDTH S5K6AAFX_QTR_SIZE_WIDTH*2
#define S5K6AAFX_ADJ_FULL_SIZE_HEIGHT S5K6AAFX_QTR_SIZE_HEIGHT*2

extern struct s5k6aafx_reg s5k6aafx_regs;

#if defined(CONFIG_MACH_SHOOTER) || defined(CONFIG_MACH_SHOOTER_K) || defined(CONFIG_MACH_SHOOTER_U) || defined(CONFIG_MACH_RIDER) || defined(CONFIG_MACH_HOLIDAY) || defined(CONFIG_MACH_VERDI_LTE) || defined(CONFIG_MACH_KINGDOM) || defined(CONFIG_MACH_SHOOTER_CT)
extern struct s5k6aafx_reg s5k6aafx_regs_cob;
#else
#define s5k6aafx_regs_cob s5k6aafx_regs
#endif

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
	const struct s5k6aafx_i2c_reg_conf *mipi_clk_init;
	uint16_t mipi_clk_init_size;
	const struct s5k6aafx_i2c_reg_conf *clk_init;
	uint16_t clk_init_size;
	const struct s5k6aafx_i2c_reg_conf *prev_snap_conf_init;
	uint16_t prev_snap_conf_init_size;
	/* for full-size preview */
	const struct s5k6aafx_i2c_reg_conf *clk_init_tb2;
	uint16_t clk_init_tb2_size;
	const struct s5k6aafx_i2c_reg_conf *prev_snap_conf_init_tb2;
	uint16_t prev_snap_conf_init_tb2_size;
	const struct s5k6aafx_i2c_reg_conf *prev_HD;
	uint16_t prev_HD_size;
	const struct s5k6aafx_i2c_reg_conf *prev_VGA;
	uint16_t prev_VGA_size;
	const struct s5k6aafx_i2c_reg_conf *prev_mode_switch_VGA;
	uint16_t prev_mode_switch_VGA_size;
};


#endif /* S5K6AAFX_H */
