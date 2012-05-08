/* linux/arch/arm/mach-msm/devices.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/clkdev.h>
#include <linux/platform_device.h>
#include "clock.h"

void __init msm9615_device_init(void);
void __init msm9615_map_io(void);
void __init msm_map_msm9615_io(void);
void __init msm9615_init_irq(void);

extern struct platform_device asoc_msm_pcm;
extern struct platform_device asoc_msm_dai0;
extern struct platform_device asoc_msm_dai1;
#if defined (CONFIG_SND_MSM_MVS_DAI_SOC)
extern struct platform_device asoc_msm_mvs;
extern struct platform_device asoc_mvs_dai0;
extern struct platform_device asoc_mvs_dai1;
#endif

extern struct platform_device msm_ebi0_thermal;
extern struct platform_device msm_ebi1_thermal;

extern struct platform_device msm_device_uart1;
extern struct platform_device msm_device_uart2;
extern struct platform_device msm_device_uart3;

extern struct platform_device msm_device_uart_dm1;
extern struct platform_device msm_device_uart_dm2;
extern struct platform_device msm_device_uart_dm3;
extern struct platform_device msm_device_uart_dm11;
extern struct platform_device msm_device_uart_dm12;
extern struct platform_device *msm_device_uart_gsbi9;
extern struct platform_device msm_device_uart_dm6;

extern struct platform_device msm8960_device_uart_gsbi2;
extern struct platform_device msm8960_device_uart_gsbi5;
extern struct platform_device msm8960_device_uart_gsbi8;
extern struct platform_device msm8960_device_ssbi_pm8921;
extern struct platform_device msm8960_device_qup_i2c_gsbi2;
extern struct platform_device msm8960_device_qup_i2c_gsbi3;
extern struct platform_device msm8960_device_qup_i2c_gsbi4;
extern struct platform_device msm8960_device_qup_i2c_gsbi5;
extern struct platform_device msm8960_device_qup_i2c_gsbi8;
extern struct platform_device msm8960_device_qup_i2c_gsbi12;
extern struct platform_device msm8960_device_qup_spi_gsbi1;
extern struct platform_device msm8960_device_qup_spi_gsbi10;
extern struct platform_device msm8960_gemini_device;

extern struct platform_device apq8064_device_uart_gsbi1;
extern struct platform_device apq8064_device_uart_gsbi3;
extern struct platform_device apq8064_device_qup_i2c_gsbi4;
extern struct platform_device apq8064_device_qup_spi_gsbi5;
extern struct platform_device apq8064_slim_ctrl;
extern struct platform_device apq8064_device_ssbi_pmic1;
extern struct platform_device apq8064_device_ssbi_pmic2;

extern struct platform_device msm9615_device_uart_gsbi4;
extern struct platform_device msm9615_device_qup_i2c_gsbi5;
extern struct platform_device msm9615_device_qup_spi_gsbi3;
extern struct platform_device msm9615_device_ssbi_pmic1;
extern struct platform_device msm9615_device_tsens;

extern struct platform_device msm_device_sdc1;
extern struct platform_device msm_device_sdc2;
extern struct platform_device msm_device_sdc3;
extern struct platform_device msm_device_sdc4;

extern struct platform_device msm_device_gadget_peripheral;
extern struct platform_device msm_device_hsusb_host;
extern struct platform_device msm_device_hsusb_host2;
extern struct platform_device msm_device_hsic_host;
extern struct platform_device msm_device_hsusb;

extern struct platform_device msm_device_otg;

extern struct platform_device msm8960_device_otg;
extern struct platform_device msm8960_device_gadget_peripheral;

extern struct platform_device apq8064_device_otg;
extern struct platform_device apq8064_usb_diag_device;
extern struct platform_device apq8064_device_gadget_peripheral;

extern struct platform_device msm_device_i2c;

extern struct platform_device msm_device_i2c_2;

extern struct platform_device qup_device_i2c;

extern struct platform_device msm_gsbi0_qup_i2c_device;
extern struct platform_device msm_gsbi1_qup_i2c_device;
extern struct platform_device msm_gsbi2_qup_i2c_device;
extern struct platform_device msm_gsbi3_qup_i2c_device;
extern struct platform_device msm_gsbi4_qup_i2c_device;
extern struct platform_device msm_gsbi5_qup_i2c_device;
extern struct platform_device msm_gsbi7_qup_i2c_device;
extern struct platform_device msm_gsbi8_qup_i2c_device;
extern struct platform_device msm_gsbi9_qup_i2c_device;
extern struct platform_device msm_gsbi10_qup_i2c_device;
extern struct platform_device msm_gsbi12_qup_i2c_device;

extern struct platform_device msm_slim_ctrl;
extern struct platform_device msm_device_sps;
extern struct platform_device msm_device_sps_apq8064;
extern struct platform_device msm_device_bam_dmux;
extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_smd_apq8064;
extern struct platform_device msm_device_dmov;
extern struct platform_device msm8960_device_dmov;
extern struct platform_device apq8064_device_dmov;
extern struct platform_device msm9615_device_dmov;
extern struct platform_device msm_device_dmov_adm0;
extern struct platform_device msm_device_dmov_adm1;

extern struct platform_device msm_device_nand;

extern struct platform_device msm_device_tssc;

extern struct platform_device msm_rotator_device;

extern struct platform_device msm_device_tsif[2];

extern struct platform_device msm_device_ssbi_pmic1;
extern struct platform_device msm_device_ssbi_pmic2;
extern struct platform_device msm_device_ssbi1;
extern struct platform_device msm_device_ssbi2;
extern struct platform_device msm_device_ssbi3;
extern struct platform_device msm_device_ssbi7;

#ifdef CONFIG_MSM_SSBI
extern struct platform_device msm_device_pm8058;
extern struct platform_device msm_device_pm8901;
#endif

extern struct platform_device msm_gsbi1_qup_spi_device;

extern struct platform_device msm_device_vidc_720p;

extern struct platform_device msm_pcm;
extern struct platform_device msm_pcm_routing;
extern struct platform_device msm_cpudai0;
extern struct platform_device msm_cpudai1;
extern struct platform_device msm_cpudai_hdmi_rx;
extern struct platform_device msm_cpudai_bt_rx;
extern struct platform_device msm_cpudai_bt_tx;
extern struct platform_device msm_cpudai_fm_rx;
extern struct platform_device msm_cpudai_fm_tx;
extern struct platform_device msm_cpudai_auxpcm_rx;
extern struct platform_device msm_cpudai_auxpcm_tx;
extern struct platform_device msm_cpu_fe;
extern struct platform_device msm_stub_codec;
extern struct platform_device msm_voice;
extern struct platform_device msm_voip;
extern struct platform_device msm_lpa_pcm;
extern struct platform_device msm_pcm_hostless;
extern struct platform_device msm_cpudai_afe_01_rx;
extern struct platform_device msm_cpudai_afe_01_tx;
extern struct platform_device msm_cpudai_afe_02_rx;
extern struct platform_device msm_cpudai_afe_02_tx;
extern struct platform_device msm_pcm_afe;

extern struct platform_device *msm_footswitch_devices[];
extern unsigned msm_num_footswitch_devices;

extern struct platform_device fsm_qfp_fuse_device;

extern struct platform_device fsm_xo_device;

extern struct platform_device qfec_device;

extern struct platform_device msm_kgsl_3d0;
extern struct platform_device msm_kgsl_2d0;
extern struct platform_device msm_kgsl_2d1;

extern struct platform_device msm_mipi_dsi1_device;

extern struct clk_lookup msm_clocks_fsm9xxx[];
extern unsigned msm_num_clocks_fsm9xxx;

extern struct platform_device msm_footswitch;

void __init msm_fb_register_device(char *name, void *data);
void __init msm_camera_register_device(void *, uint32_t, void *);
struct platform_device *msm_add_gsbi9_uart(void);
extern struct platform_device msm_device_touchscreen;
extern unsigned engineer_id;
extern int ps_type;

extern struct pil_device peripheral_dsps;
extern struct platform_device led_pdev;

extern struct platform_device msm_rpm_device;
extern struct platform_device msm_device_rng;

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
extern struct platform_device msm9615_qcrypto_device;
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
extern struct platform_device msm9615_qcedev_device;
#endif
extern struct platform_device msm8960_device_watchdog;
extern struct platform_device msm8660_device_watchdog;
extern struct platform_device msm8064_device_watchdog;
extern struct platform_device msm9615_device_watchdog;

extern struct platform_device msm_etb_device;
extern struct platform_device msm_tpiu_device;
extern struct platform_device msm_funnel_device;
extern struct platform_device msm_debug_device;
extern struct platform_device msm_ptm_device;
#endif
