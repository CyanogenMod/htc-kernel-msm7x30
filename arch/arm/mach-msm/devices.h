/* linux/arch/arm/mach-msm/devices.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include "clock.h"

extern struct platform_device msm_ebi0_thermal;
extern struct platform_device msm_ebi1_thermal;

extern struct platform_device msm_device_uart1;
extern struct platform_device msm_device_uart2;
extern struct platform_device msm_device_uart3;

extern struct platform_device msm_device_uart_dm1;
extern struct platform_device msm_device_uart_dm2;
extern struct platform_device msm_device_uart_dm11;
extern struct platform_device msm_device_uart_dm12;
#ifdef CONFIG_MSM_GSBI9_UART
extern struct platform_device msm_device_uart_gsbi9;
#endif

extern struct platform_device msm_device_sdc1;
extern struct platform_device msm_device_sdc2;
extern struct platform_device msm_device_sdc3;
extern struct platform_device msm_device_sdc4;

extern struct platform_device msm_device_hsusb_otg;
extern struct platform_device msm_device_hsusb_peripheral;
extern struct platform_device msm_device_hsusb_host;
extern struct platform_device msm_device_hsusb_host2;
extern struct platform_device msm_device_hsusb;

extern struct platform_device msm_device_otg;
extern struct platform_device usb_diag_device;

extern struct platform_device msm_device_i2c;

extern struct platform_device msm_device_i2c_2;

extern struct platform_device qup_device_i2c;

extern struct platform_device msm_gsbi3_qup_i2c_device;
extern struct platform_device msm_gsbi4_qup_i2c_device;
extern struct platform_device msm_gsbi5_qup_i2c_device;
extern struct platform_device msm_gsbi7_qup_i2c_device;
extern struct platform_device msm_gsbi8_qup_i2c_device;
extern struct platform_device msm_gsbi9_qup_i2c_device;
extern struct platform_device msm_gsbi10_qup_i2c_device;
extern struct platform_device msm_gsbi12_qup_i2c_device;

extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_dmov;

extern struct platform_device msm_device_nand;
extern struct platform_device msm_device_tssc;
#ifdef CONFIG_MSM_ROTATOR
extern struct platform_device msm_rotator_device;
#endif

extern struct platform_device msm_device_tsif;
#ifdef CONFIG_I2C_SSBI
extern struct platform_device msm_device_ssbi1;
extern struct platform_device msm_device_ssbi2;
extern struct platform_device msm_device_ssbi3;
extern struct platform_device msm_device_ssbi6;
extern struct platform_device msm_device_ssbi7;
#endif
extern struct platform_device msm_gsbi1_qup_spi_device;
extern struct platform_device msm_gsbi8_qup_spi_device;
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
extern struct platform_device *msm_footswitch_devices[];
extern unsigned msm_num_footswitch_devices;

extern struct clk msm_clocks_7x01a[];
extern unsigned msm_num_clocks_7x01a;

extern struct clk msm_clocks_7x25[];
extern unsigned msm_num_clocks_7x25;

extern struct clk msm_clocks_7x27[];
extern unsigned msm_num_clocks_7x27;

extern struct clk msm_clocks_7x30[];
extern unsigned msm_num_clocks_7x30;

extern struct clk msm_clocks_8x50[];
extern unsigned msm_num_clocks_8x50;

extern struct clk msm_clocks_8x60[];
extern unsigned msm_num_clocks_8x60;

void __init msm_fb_register_device(char *name, void *data);
void __init msm_camera_register_device(void *, uint32_t, void *);
extern struct platform_device msm_device_touchscreen;
extern struct platform_device msm_device_spi;
extern unsigned engineer_id;
extern int ps_type;
#ifdef CONFIG_MSM_SSBI
extern struct platform_device msm_device_ssbi_pmic;
#endif

#endif
