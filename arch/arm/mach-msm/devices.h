/* linux/arch/arm/mach-msm/devices.h
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef __ARCH_ARM_MACH_MSM_DEVICES_H
#define __ARCH_ARM_MACH_MSM_DEVICES_H

extern struct platform_device msm_device_uart1;
extern struct platform_device msm_device_uart2;
extern struct platform_device msm_device_uart3;

extern struct platform_device msm_device_uart_dm1;
extern struct platform_device msm_device_uart_dm2;

extern struct platform_device msm_device_sdc1;
extern struct platform_device msm_device_sdc2;
extern struct platform_device msm_device_sdc3;
extern struct platform_device msm_device_sdc4;

extern struct platform_device msm_device_hsusb;

extern struct platform_device msm_device_i2c;

extern struct platform_device msm_device_smd;

extern struct platform_device msm_device_nand;
#ifdef CONFIG_MSM_ROTATOR
extern struct platform_device msm_rotator_device;
#endif

#ifdef CONFIG_I2C_SSBI
extern struct platform_device msm_device_ssbi6;
extern struct platform_device msm_device_ssbi7;
#endif
extern struct platform_device msm_device_mddi0;
extern struct platform_device msm_device_mddi1;
extern struct platform_device msm_device_mdp;
#if defined(CONFIG_ARCH_MSM7X30)
extern struct platform_device msm_device_i2c_2;
extern struct platform_device qup_device_i2c;
extern struct platform_device msm_device_vidc_720p;
#endif
#ifdef CONFIG_SPI_QSD_NEW
extern struct platform_device qsdnew_device_spi;
#endif
extern struct platform_device msm_device_touchscreen;
extern struct platform_device msm_device_spi;
extern unsigned engineer_id;
#ifdef CONFIG_MSM_SSBI
extern struct platform_device msm_device_ssbi_pmic;
#endif

#endif
