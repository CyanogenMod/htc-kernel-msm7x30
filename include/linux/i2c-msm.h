/*
 * include/linux/i2c-msm.h - platform data structure for i2c controller
 *
 * Copyright (C) 2009 HTC.
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

#ifndef _I2C_MSM_H
#define _I2C_MSM_H

struct msm_i2c_device_platform_data {
	int i2c_clock;
	int clock_strength;
	int data_strength;
};

#endif /* _I2C_MSM_H */
