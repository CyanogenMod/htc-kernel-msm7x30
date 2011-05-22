/* linux/arch/arm/mach-msm/board-speedy-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Author: Jay Tu <jay_tu@htc.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>

#include "pmic.h"
#include "board-speedy.h"
#include "devices.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif
#define DEFAULT_BRIGHTNESS 100
extern int panel_type;

#define DRIVER_IC_CUT2			4
#define	PANEL_WHITESTONE	0
#define	PANEL_SPEEDY_SHARP	3
#define PANEL_SPEEDY_SHARP_CUT2        (PANEL_SPEEDY_SHARP | DRIVER_IC_CUT2)

static struct clk *axi_clk;

static struct vreg *V_LCMIO_1V8, *V_LCM_2V85;
static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
};

enum led_brightness brightness_value = DEFAULT_BRIGHTNESS;

/* use one flag to have better backlight on/off performance */
static int speedy_set_dim = 1;

static void speedy_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	if (val < 30)
		shrink_br = 8;
	else if ((val >= 30) && (val <= 143))
		shrink_br = 147 * (val - 30) / 113 + 8;
	else
		shrink_br = 100 * (val - 143) / 112 + 155;
	mutex_lock(&cabc.lock);
	if (speedy_set_dim == 1) {
		client->remote_write(client, 0x2C, 0x5300);
		/* we dont need set dim again */
		speedy_set_dim = 0;
	}
	client->remote_write(client, 0x00, 0x5500);
	client->remote_write(client, shrink_br, 0x5100);
	brightness_value = val;
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
speedy_get_brightness(struct led_classdev *led_cdev)
{
	/*FIXME:workaround for NOVATEK driver IC*/
#if 0
	struct msm_mddi_client_data *client = cabc.client_data;
	return client->remote_read(client, 0x5100);
#else
	return brightness_value;
#endif
}

static void speedy_backlight_switch(int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;

		/* LED core uses get_brightness for default value
		 * If the physical layer is not ready, we should
		 * not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		speedy_set_brightness(&cabc.lcd_backlight, val);
		/* set next backlight value with dim */
		speedy_set_dim = 1;
	} else {
		clear_bit(GATE_ON, &cabc.status);
		speedy_set_brightness(&cabc.lcd_backlight, 0);
	}
}

static int speedy_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = speedy_set_brightness;
	cabc.lcd_backlight.brightness_get = speedy_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

/* ------------------------------------------------------------------- */

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

#define REG_WAIT (0xffff)
struct nov_regs {
	unsigned reg;
	unsigned val;
};

static struct nov_regs sharp_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 120},
	{0x17C0, 0x0F},
	{0x53C0, 0x00},
	{0x54C0, 0x50},
	{0x89C3, 0x80},
	{0x92C2, 0x08},
	{0x0180, 0x14},
	{0x0280, 0x11},
	{0x0380, 0x33},
	{0x0480, 0x60},
	{0x0580, 0x60},
	{0x0680, 0x60},
	{0x0780, 0x00},
	{0x0880, 0x44},
	{0x0980, 0x54},
	{0x0A80, 0x10},
	{0x0B80, 0x55},
	{0x0C80, 0x55},
	{0x0D80, 0x30},
	{0x0E80, 0x44},
	{0x0F80, 0x54},
	{0x1080, 0x30},
	{0x1280, 0x77},
	{0x1380, 0x10},
	{0x1480, 0x0F},
	{0x1580, 0x9B},
	{0x1680, 0xD4},
	{0x1780, 0x00},
	{0x1880, 0x00},
	{0x1980, 0x06},
	{0x1C80, 0x01},
	{0x1F80, 0x00},
	{0x2080, 0x01},
	{0x2180, 0x35},
	{0x2480, 0x3B},
	{0x2580, 0x43},
	{0x2680, 0x52},
	{0x2780, 0x60},
	{0x2880, 0x1A},
	{0x2980, 0x2F},
	{0x2A80, 0x5D},
	{0x2B80, 0x59},
	{0x2D80, 0x1F},
	{0x2F80, 0x23},
	{0x3080, 0x95},
	{0x3180, 0x1C},
	{0x3280, 0x3F},
	{0x3380, 0x5B},
	{0x3480, 0x8E},
	{0x3580, 0xB1},
	{0x3680, 0xC8},
	{0x3780, 0x5E},
	{0x3880, 0x13},
	{0x3980, 0x29},
	{0x3A80, 0x40},
	{0x3B80, 0x5A},
	{0x3D80, 0x29},
	{0x3F80, 0x44},
	{0x4080, 0x65},
	{0x4180, 0x5F},
	{0x4280, 0x1C},
	{0x4380, 0x21},
	{0x4480, 0x9D},
	{0x4580, 0x23},
	{0x4680, 0x50},
	{0x4780, 0x65},
	{0x4880, 0x98},
	{0x4980, 0xA6},
	{0x4A80, 0xB6},
	{0x4B80, 0x3D},
	{0x4C80, 0x3B},
	{0x4D80, 0x43},
	{0x4E80, 0x52},
	{0x4F80, 0x60},
	{0x5080, 0x1A},
	{0x5180, 0x2F},
	{0x5280, 0x5D},
	{0x5380, 0x59},
	{0x5480, 0x1F},
	{0x5580, 0x23},
	{0x5680, 0x95},
	{0x5780, 0x1C},
	{0x5880, 0x3F},
	{0x5980, 0x5B},
	{0x5A80, 0x8E},
	{0x5B80, 0xB1},
	{0x5C80, 0xC8},
	{0x5D80, 0x5E},
	{0x5E80, 0x13},
	{0x5F80, 0x29},
	{0x6080, 0x40},
	{0x6180, 0x5A},
	{0x6280, 0x29},
	{0x6380, 0x44},
	{0x6480, 0x65},
	{0x6580, 0x5F},
	{0x6680, 0x1C},
	{0x6780, 0x21},
	{0x6880, 0x9D},
	{0x6980, 0x23},
	{0x6A80, 0x50},
	{0x6B80, 0x65},
	{0x6C80, 0x98},
	{0x6D80, 0xA6},
	{0x6E80, 0xB6},
	{0x6F80, 0x3D},
	{0x7080, 0x3B},
	{0x7180, 0x43},
	{0x7280, 0x52},
	{0x7380, 0x60},
	{0x7480, 0x1A},
	{0x7580, 0x2F},
	{0x7680, 0x5D},
	{0x7780, 0x59},
	{0x7880, 0x1F},
	{0x7980, 0x23},
	{0x7A80, 0x95},
	{0x7B80, 0x1C},
	{0x7C80, 0x3F},
	{0x7D80, 0x5B},
	{0x7E80, 0x8E},
	{0x7F80, 0xB1},
	{0x8080, 0xC8},
	{0x8180, 0x5E},
	{0x8280, 0x13},
	{0x8380, 0x29},
	{0x8480, 0x40},
	{0x8580, 0x5A},
	{0x8680, 0x29},
	{0x8780, 0x44},
	{0x8880, 0x65},
	{0x8980, 0x5F},
	{0x8A80, 0x1C},
	{0x8B80, 0x21},
	{0x8C80, 0x9D},
	{0x8D80, 0x23},
	{0x8E80, 0x50},
	{0x8F80, 0x65},
	{0x9080, 0x98},
	{0x9180, 0xA6},
	{0x9280, 0xB6},
	{0x9380, 0x3D},
	{0x9480, 0xA6},
	{0x9580, 0x01},
	{0x9680, 0x26},
	{0x9780, 0xA6},
	{0x9880, 0x30},
	{0x9980, 0x20},
	{0x9A80, 0x0A},
	{0x9B80, 0x0A},
	{0x9C80, 0x12},
	{0x9D80, 0x00},
	{0x9E80, 0x00},
	{0x9F80, 0x12},
	{0xA080, 0x00},
	{0xA280, 0x00},
	{0xA380, 0x3C},
	{0xA480, 0x05},
	{0xA580, 0xC0},
	{0xA680, 0x01},
	{0xA780, 0x00},
	{0xA980, 0x00},
	{0xAA80, 0x00},
	{0xAB80, 0x70},
	{0xE780, 0x00},
	{0xE880, 0x00},
	{0xED80, 0x0A},
	{0xEE80, 0x80},
	{0xF380, 0xCC},
	{0x2900, 0x00},
	{0x3500, 0x00},
	{0x4400, 0x02},
	{0x4401, 0x58},
};


static int
speedy_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size;
	unsigned reg, val;
	struct nov_regs *init_seq;

	init_seq = sharp_init_seq;
	array_size = ARRAY_SIZE(sharp_init_seq);

	for (i = 0; i < array_size; i++) {
		reg = cpu_to_le32(init_seq[i].reg);
		val = cpu_to_le32(init_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else {
			client_data->remote_write(client_data, val, reg);
			if (reg == 0x1100)
				client_data->send_powerdown(client_data);
		}
	}
	if(axi_clk)
		clk_set_rate(axi_clk, 0);

	return 0;
}

static int
speedy_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	return 0;
}

static int
speedy_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	client_data->auto_hibernate(client_data, 0);
	/* set dim off for performance */
	client_data->remote_write(client_data, 0x0, 0x5300);
	speedy_backlight_switch(LED_OFF);
	client_data->remote_write(client_data, 0, 0x2800);
	client_data->remote_write(client_data, 0, 0x1000);
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
speedy_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	client_data->auto_hibernate(client_data, 0);
	client_data->remote_write(client_data, 0x24, 0x5300);
	client_data->remote_write(client_data, 0x0A, 0x22C0);
	client_data->auto_hibernate(client_data, 1);
	hr_msleep(30);
	speedy_backlight_switch(LED_FULL);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = speedy_mddi_init,
	.uninit = speedy_mddi_uninit,
	.blank = speedy_panel_blank,
	.unblank = speedy_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 48,
		.height = 80,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
	},
};

static void
mddi_novatec_power(struct msm_mddi_client_data *client_data, int on)
{
	if (panel_type == 0) {
		if (on) {
			if(axi_clk)
				clk_set_rate(axi_clk, 192000000);

			vreg_enable(V_LCM_2V85);
			vreg_enable(V_LCMIO_1V8);
			hr_msleep(20);
			gpio_set_value(SPEEDY_LCD_RSTz, 1);
			hr_msleep(25);
			gpio_set_value(SPEEDY_LCD_RSTz, 0);
			hr_msleep(10);
			gpio_set_value(SPEEDY_LCD_RSTz, 1);
			hr_msleep(20);
		} else {
			vreg_disable(V_LCMIO_1V8);
			vreg_disable(V_LCM_2V85);
			gpio_set_value(SPEEDY_LCD_RSTz, 0);
		}
	} else {
		if (on) {
			if(axi_clk)
				clk_set_rate(axi_clk, 192000000);

			vreg_enable(V_LCM_2V85);
			hr_msleep(3);
			vreg_disable(V_LCM_2V85);
			hr_msleep(50);
			vreg_enable(V_LCM_2V85);
			vreg_enable(V_LCMIO_1V8);
			hr_msleep(2);
			gpio_set_value(SPEEDY_LCD_RSTz, 1);
			hr_msleep(1);
			gpio_set_value(SPEEDY_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(SPEEDY_LCD_RSTz, 1);
			hr_msleep(60);
		} else {
			gpio_set_value(SPEEDY_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(V_LCMIO_1V8);
			vreg_disable(V_LCM_2V85);
		}
	}
}

static void panel_nov_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	printk(KERN_INFO "mddi fixup\n");
	*mfr_name = 0xb9f6;
	*product_code = 0x5560;
}

static struct msm_mddi_platform_data mddi_pdata = {
	.fixup = panel_nov_fixup,
	.power_client = mddi_novatec_power,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5560),
			.name = "mddi_c_b9f6_5560",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			.product_id = (0xb9f6 << 16 | 0x5580),
			.name = "mddi_c_b9f6_5580",
			.id = 1,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_driver speedy_backlight_driver = {
	.probe = speedy_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
	.overrides = MSM_MDP4_MDDI_DMA_SWITCH
#ifdef CONFIG_OVERLAY_FORCE_UPDATE
	 | MSM_MDP_FORCE_UPDATE
#endif
	,
};

int __init speedy_init_panel(unsigned int sys_rev)
{
	int rc;

	B(KERN_INFO "%s: enter.\n", __func__);

	/* lcd panel power */
	/* 2.85V -- LDO20 */
	V_LCM_2V85 = vreg_get(NULL, "gp13");

	if (IS_ERR(V_LCM_2V85)) {
		pr_err("%s: LCM_2V85 get failed (%ld)\n",
			__func__, PTR_ERR(V_LCM_2V85));
		return -1;
	}
	if (sys_rev > 0)
		V_LCMIO_1V8 = vreg_get(NULL, "lvsw0");
	else
		V_LCMIO_1V8 = vreg_get(NULL, "wlan2");

	if (IS_ERR(V_LCMIO_1V8)) {
		pr_err("%s: LCMIO_1V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_1V8));
		return -1;
	}

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	if (panel_type == 0) {
		mddi_pdata.clk_rate = 384000000;
		mddi_pdata.type = MSM_MDP_MDDI_TYPE_I;
	} else {
		if (panel_type & DRIVER_IC_CUT2)
			mddi_pdata.clk_rate = 384000000;
		else
			mddi_pdata.clk_rate = 256000000;
		mddi_pdata.type = MSM_MDP_MDDI_TYPE_II;
	}

	axi_clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(axi_clk)) {
		pr_err("%s: failed to get axi clock\n", __func__);
		return PTR_ERR(axi_clk);
	}

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	rc = platform_driver_register(&speedy_backlight_driver);
	if (rc)
		return rc;

	return 0;
}
