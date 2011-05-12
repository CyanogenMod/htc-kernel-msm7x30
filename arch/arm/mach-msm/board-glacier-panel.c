/* linux/arch/arm/mach-msm/board-glacier-panel.c
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


#include "board-glacier.h"
#include "devices.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif

#define DEFAULT_BRIGHTNESS 100

extern int panel_type;

#define PANEL_OBSOLATE_0		0
#define PANEL_SHARP			1
#define PANEL_SONY			2

#define PWM_USER_DEF	 		143
#define PWM_USER_MIN			30
#define PWM_USER_DIM			20
#define PWM_USER_MAX			255

#define PWM_SHARP_DEF			103
#define PWM_SHARP_MIN			11
#define PWM_SHARP_MAX			218

#define PWM_SONY_DEF			120
#define PWM_SONY_MIN			13
#define PWM_SONY_MAX			255

static struct clk *axi_clk;

struct vreg {
        const char *name;
        unsigned id;
};

struct vreg *vreg_ldo19, *vreg_ldo20;
struct vreg *vreg_ldo12;

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
static int glacier_set_dim = 1;


static int glacier_shrink_pwm(int brightness, int user_def,
		int user_min, int user_max, int panel_def,
		int panel_min, int panel_max)
{
	if (brightness < PWM_USER_DIM) {
		return 0;
	}

	if (brightness < user_min) {
		return panel_min;
	}

	if (brightness > user_def) {
		brightness = (panel_max - panel_def) *
			(brightness - user_def) /
			(user_max - user_def) +
			panel_def;
	} else {
			brightness = (panel_def - panel_min) *
			(brightness - user_min) /
			(user_def - user_min) +
			panel_min;
	}

        return brightness;
}

static void glacier_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	printk(KERN_DEBUG "set brightness = %d\n", val);

	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	if(panel_type == PANEL_SHARP)
		shrink_br = glacier_shrink_pwm(val, PWM_USER_DEF,
				PWM_USER_MIN, PWM_USER_MAX, PWM_SHARP_DEF,
				PWM_SHARP_MIN, PWM_SHARP_MAX);
	else
		shrink_br = glacier_shrink_pwm(val, PWM_USER_DEF,
				PWM_USER_MIN, PWM_USER_MAX, PWM_SONY_DEF,
				PWM_SONY_MIN, PWM_SONY_MAX);

	mutex_lock(&cabc.lock);
	if (glacier_set_dim == 1) {
		client->remote_write(client, 0x2C, 0x5300);
		/* we dont need set dim again */
		glacier_set_dim = 0;
	}
	client->remote_write(client, 0x00, 0x5500);
	client->remote_write(client, shrink_br, 0x5100);
	brightness_value = val;
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
glacier_get_brightness(struct led_classdev *led_cdev)
{
	/*FIXME:workaround for NOVATEK driver IC*/
#if 0
	struct msm_mddi_client_data *client = cabc.client_data;
	return client->remote_read(client, 0x5100);
#else
	return brightness_value;
#endif
}

static void glacier_backlight_switch(int on)
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
		glacier_set_brightness(&cabc.lcd_backlight, val);
		/* set next backlight value with dim */
		glacier_set_dim = 1;
	} else {
		glacier_set_brightness(&cabc.lcd_backlight, 0);
		clear_bit(GATE_ON, &cabc.status);
	}
}

static int glacier_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = glacier_set_brightness;
	cabc.lcd_backlight.brightness_get = glacier_get_brightness;
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
/*
static struct vreg *vreg_lcd_2v8;
static struct vreg *vreg_lcd_1v8;
*/
#define REG_WAIT (0xffff)

struct nov_regs {
	unsigned reg;
	unsigned val;
};

static struct nov_regs sharp_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 120},
	{0x3500, 0x00},
	{0x5100, 0x00},

	{0x89C3, 0x0080},
	{0x92C2, 0x0008},
	{0x0180, 0x0014},
	{0x0280, 0x0011},
	{0x0380, 0x0033},
	{0x0480, 0x0054},
	{0x0580, 0x0054},
	{0x0680, 0x0054},
	{0x0780, 0x0000},
	{0x0880, 0x0044},
	{0x0980, 0x0034},
	{0x0A80, 0x0010},
	{0x0B80, 0x0055},
	{0x0C80, 0x0055},
	{0x0D80, 0x0030},
	{0x0E80, 0x0044},
	{0x0F80, 0x0054},
	{0x1080, 0x0030},
	{0x1280, 0x0077},
	{0x1380, 0x0021},
	{0x1480, 0x000E},
	{0x1580, 0x0098},
	{0x1680, 0x00CC},
	{0x1780, 0x0000},
	{0x1880, 0x0000},
	{0x1980, 0x0000},
	{0x1C80, 0x0000},
	{0x1F80, 0x0005},
	{0x2480, 0x001A},
	{0x2580, 0x001F},
	{0x2680, 0x002D},
	{0x2780, 0x003E},
	{0x2880, 0x000D},
	{0x2980, 0x0021},
	{0x2A80, 0x0058},
	{0x2B80, 0x002A},
	{0x2D80, 0x0020},
	{0x2F80, 0x0027},
	{0x3080, 0x0061},
	{0x3180, 0x0017},
	{0x3280, 0x0037},
	{0x3380, 0x0053},
	{0x3480, 0x005A},
	{0x3580, 0x008E},
	{0x3680, 0x00A7},
	{0x3780, 0x003E},
	{0x3880, 0x002B},
	{0x3980, 0x002E},
	{0x3A80, 0x0036},
	{0x3B80, 0x0041},
	{0x3D80, 0x001A},
	{0x3F80, 0x002D},
	{0x4080, 0x005D},
	{0x4180, 0x003D},
	{0x4280, 0x0020},
	{0x4380, 0x0027},
	{0x4480, 0x0076},
	{0x4580, 0x0017},
	{0x4680, 0x0039},
	{0x4780, 0x0055},
	{0x4880, 0x0071},
	{0x4980, 0x00A6},
	{0x4A80, 0x00BF},
	{0x4B80, 0x0055},
	{0x4C80, 0x0055},
	{0x4D80, 0x0058},
	{0x4E80, 0x005F},
	{0x4F80, 0x0066},
	{0x5080, 0x0018},
	{0x5180, 0x0026},
	{0x5280, 0x0057},
	{0x5380, 0x003D},
	{0x5480, 0x001E},
	{0x5580, 0x0026},
	{0x5680, 0x006B},
	{0x5780, 0x0017},
	{0x5880, 0x003B},
	{0x5980, 0x004F},
	{0x5A80, 0x005A},
	{0x5B80, 0x008E},
	{0x5C80, 0x00A7},
	{0x5D80, 0x003E},
	{0x5E80, 0x0066},
	{0x5F80, 0x0068},
	{0x6080, 0x006C},
	{0x6180, 0x006E},
	{0x6280, 0x0016},
	{0x6380, 0x002A},
	{0x6480, 0x0059},
	{0x6580, 0x004C},
	{0x6680, 0x001E},
	{0x6780, 0x0025},
	{0x6880, 0x007B},
	{0x6980, 0x0017},
	{0x6A80, 0x003A},
	{0x6B80, 0x0053},
	{0x6C80, 0x0071},
	{0x6D80, 0x00A6},
	{0x6E80, 0x00BF},
	{0x6F80, 0x0055},
	{0x7080, 0x0063},
	{0x7180, 0x0066},
	{0x7280, 0x0070},
	{0x7380, 0x0076},
	{0x7480, 0x0018},
	{0x7580, 0x0027},
	{0x7680, 0x0058},
	{0x7780, 0x0047},
	{0x7880, 0x001E},
	{0x7980, 0x0025},
	{0x7A80, 0x0072},
	{0x7B80, 0x0018},
	{0x7C80, 0x003B},
	{0x7D80, 0x004C},
	{0x7E80, 0x005A},
	{0x7F80, 0x008E},
	{0x8080, 0x00A7},
	{0x8180, 0x003E},
	{0x8280, 0x0075},
	{0x8380, 0x0077},
	{0x8480, 0x007C},
	{0x8580, 0x007E},
	{0x8680, 0x0016},
	{0x8780, 0x002C},
	{0x8880, 0x005C},
	{0x8980, 0x0055},
	{0x8A80, 0x001F},
	{0x8B80, 0x0024},
	{0x8C80, 0x0082},
	{0x8D80, 0x0015},
	{0x8E80, 0x0038},
	{0x8F80, 0x0050},
	{0x9080, 0x0071},
	{0x9180, 0x00A6},
	{0x9280, 0x00BF},
	{0x9380, 0x0055},
	{0x9480, 0x00B5},
	{0x9580, 0x0004},
	{0x9680, 0x0018},
	{0x9780, 0x00B0},
	{0x9880, 0x0020},
	{0x9980, 0x0028},
	{0x9A80, 0x0008},
	{0x9B80, 0x0004},
	{0x9C80, 0x0012},
	{0x9D80, 0x0000},
	{0x9E80, 0x0000},
	{0x9F80, 0x0012},
	{0xA080, 0x0000},
	{0xA280, 0x0000},
	{0xA380, 0x003C},
	{0xA480, 0x0001},
	{0xA580, 0x00C0},
	{0xA680, 0x0001},
	{0xA780, 0x0000},
	{0xA980, 0x0000},
	{0xAA80, 0x0000},
	{0xAB80, 0x0070},
	{0xE780, 0x0011},
	{0xE880, 0x0011},
	{0xED80, 0x000A},
	{0xEE80, 0x0080},
	{0xF780, 0x000D},
	{0x2900, 0x0000},
	{0x4400, 0x0001},
	{0x4401, 0x002b},
	{0x0480, 0x0063},
	{0x22c0, 0x0006},
};

static struct nov_regs sony_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 120},
	{0x3600, 0xD0},
	{0x3500, 0x0000},
	{0x5100, 0x00},

	{0x2480, 0x0069},
	{0x2580, 0x006C},
	{0x2680, 0x0074},
	{0x2780, 0x007C},
	{0x2880, 0x0016},
	{0x2980, 0x0029},
	{0x2A80, 0x005A},
	{0x2B80, 0x0072},
	{0x2D80, 0x001D},
	{0x2F80, 0x0024},
	{0x3080, 0x00B6},
	{0x3180, 0x001A},
	{0x3280, 0x0044},
	{0x3380, 0x005B},
	{0x3480, 0x00C9},
	{0x3580, 0x00F1},
	{0x3680, 0x00FD},
	{0x3780, 0x007F},
	{0x3880, 0x0069},
	{0x3980, 0x006C},
	{0x3A80, 0x0074},
	{0x3B80, 0x007C},
	{0x3D80, 0x0016},
	{0x3F80, 0x0029},
	{0x4080, 0x005A},
	{0x4180, 0x0072},
	{0x4280, 0x001D},
	{0x4380, 0x0024},
	{0x4480, 0x00B6},
	{0x4580, 0x001A},
	{0x4680, 0x0044},
	{0x4780, 0x005B},
	{0x4880, 0x00C9},
	{0x4980, 0x00F1},
	{0x4A80, 0x00FD},
	{0x4B80, 0x007F},
	{0x4C80, 0x004F},
	{0x4D80, 0x0053},
	{0x4E80, 0x0060},
	{0x4F80, 0x006A},
	{0x5080, 0x0016},
	{0x5180, 0x0029},
	{0x5280, 0x005B},
	{0x5380, 0x006C},
	{0x5480, 0x001E},
	{0x5580, 0x0025},
	{0x5680, 0x00B3},
	{0x5780, 0x001C},
	{0x5880, 0x004A},
	{0x5980, 0x0061},
	{0x5A80, 0x00B6},
	{0x5B80, 0x00C6},
	{0x5C80, 0x00F6},
	{0x5D80, 0x007F},
	{0x5E80, 0x004F},
	{0x5F80, 0x0053},
	{0x6080, 0x0060},
	{0x6180, 0x006A},
	{0x6280, 0x0016},
	{0x6380, 0x0029},
	{0x6480, 0x005B},
	{0x6580, 0x006C},
	{0x6680, 0x001E},
	{0x6780, 0x0025},
	{0x6880, 0x00B3},
	{0x6980, 0x001C},
	{0x6A80, 0x004A},
	{0x6B80, 0x0061},
	{0x6C80, 0x00B6},
	{0x6D80, 0x00C6},
	{0x6E80, 0x00F6},
	{0x6F80, 0x007F},
	{0x7080, 0x0000},
	{0x7180, 0x000A},
	{0x7280, 0x0027},
	{0x7380, 0x003C},
	{0x7480, 0x001D},
	{0x7580, 0x0030},
	{0x7680, 0x0060},
	{0x7780, 0x0063},
	{0x7880, 0x0020},
	{0x7980, 0x0026},
	{0x7A80, 0x00B2},
	{0x7B80, 0x001C},
	{0x7C80, 0x0049},
	{0x7D80, 0x0060},
	{0x7E80, 0x00B9},
	{0x7F80, 0x00D1},
	{0x8080, 0x00FB},
	{0x8180, 0x007F},
	{0x8280, 0x0000},
	{0x8380, 0x000A},
	{0x8480, 0x0027},
	{0x8580, 0x003C},
	{0x8680, 0x001D},
	{0x8780, 0x0030},
	{0x8880, 0x0060},
	{0x8980, 0x0063},
	{0x8A80, 0x0020},
	{0x8B80, 0x0026},
	{0x8C80, 0x00B2},
	{0x8D80, 0x001C},
	{0x8E80, 0x0049},
	{0x8F80, 0x0060},
	{0x9080, 0x00B9},
	{0x9180, 0x00D1},
	{0x9280, 0x00FB},
	{0x9380, 0x007F},

	{0x2900, 0x0000},
	{0x22c0, 0x0006},
	{0x4400, 0x0002},
	{0x4401, 0x0058},
	{0x0480, 0x0063},
};


static int
glacier_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size;
	unsigned reg, val;
	struct nov_regs *init_seq;

	if (panel_type == PANEL_SONY) {
		init_seq = sony_init_seq;
		array_size = ARRAY_SIZE(sony_init_seq);
	} else {
		init_seq = sharp_init_seq;
		array_size = ARRAY_SIZE(sharp_init_seq);
	}

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
glacier_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	client_data->auto_hibernate(client_data, 0);
	client_data->remote_write(client_data, 0, 0x2800);
	client_data->remote_write(client_data, 0, 0x1000);
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
glacier_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	/* set dim off for performance */
	client_data->remote_write(client_data, 0x24, 0x5300);
	glacier_backlight_switch(LED_OFF);
	return 0;
}

static int
glacier_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s +\n", __func__);

	client_data->auto_hibernate(client_data, 0);

	if (panel_type == PANEL_SHARP) {
		/* disable driver ic flip since sharp used mdp flip */
		client_data->remote_write(client_data, 0x00, 0x3600);
	}

	client_data->remote_write(client_data, 0x24, 0x5300);
	hr_msleep(30);
	glacier_backlight_switch(LED_FULL);
	client_data->auto_hibernate(client_data, 1);

	B(KERN_DEBUG "%s -\n", __func__);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = glacier_mddi_init,
	.uninit = glacier_mddi_uninit,
	.blank = glacier_panel_blank,
	.unblank = glacier_panel_unblank,
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
	unsigned pulldown = 1;

	if (panel_type == 0) {
		if (on) {
			if(axi_clk)
				clk_set_rate(axi_clk, 192000000);

			vreg_enable(vreg_ldo20);
			hr_msleep(5);
			vreg_disable(vreg_ldo20);
			hr_msleep(55);
			gpio_set_value(GLACIER_LCD_2V85_EN, 1);
			/* OJ_2V85*/
			vreg_enable(vreg_ldo12);
			hr_msleep(1);
			vreg_enable(vreg_ldo20);
			hr_msleep(2);
			vreg_enable(vreg_ldo19);
			hr_msleep(2);
			gpio_set_value(GLACIER_MDDI_RSTz, 1);
			hr_msleep(2);
			gpio_set_value(GLACIER_MDDI_RSTz, 0);
			hr_msleep(2);
			gpio_set_value(GLACIER_MDDI_RSTz, 1);
			hr_msleep(65);
		} else {
			hr_msleep(130);
			gpio_set_value(GLACIER_MDDI_RSTz, 0);
			hr_msleep(15);
			vreg_disable(vreg_ldo20);
			hr_msleep(15);
			vreg_disable(vreg_ldo19);
			/* OJ_2V85*/
			vreg_disable(vreg_ldo12);
			gpio_set_value(GLACIER_LCD_2V85_EN, 0);
			msm_proc_comm(PCOM_VREG_PULLDOWN, &pulldown, &vreg_ldo20->id);
			msm_proc_comm(PCOM_VREG_PULLDOWN, &pulldown, &vreg_ldo19->id);
			msm_proc_comm(PCOM_VREG_PULLDOWN, &pulldown, &vreg_ldo12->id);
		}
	} else {
		if (on) {
			if(axi_clk)
				clk_set_rate(axi_clk, 192000000);

			vreg_enable(vreg_ldo20);
			hr_msleep(5);
			vreg_disable(vreg_ldo20);
			hr_msleep(55);
			gpio_set_value(GLACIER_LCD_2V85_EN, 1);
			/* OJ_2V85*/
			vreg_enable(vreg_ldo12);
			hr_msleep(1);
			vreg_enable(vreg_ldo20);
			hr_msleep(2);
			vreg_enable(vreg_ldo19);
			hr_msleep(2);
			gpio_set_value(GLACIER_MDDI_RSTz, 1);
			hr_msleep(2);
			gpio_set_value(GLACIER_MDDI_RSTz, 0);
			hr_msleep(2);
			gpio_set_value(GLACIER_MDDI_RSTz, 1);
			hr_msleep(65);
		} else {
			hr_msleep(130);
			gpio_set_value(GLACIER_MDDI_RSTz, 0);
			hr_msleep(15);
			vreg_disable(vreg_ldo20);
			hr_msleep(15);
			vreg_disable(vreg_ldo19);
			/* OJ_2V85*/
			vreg_disable(vreg_ldo12);
			gpio_set_value(GLACIER_LCD_2V85_EN, 0);
			msm_proc_comm(PCOM_VREG_PULLDOWN, &pulldown, &vreg_ldo20->id);
			msm_proc_comm(PCOM_VREG_PULLDOWN, &pulldown, &vreg_ldo19->id);
			msm_proc_comm(PCOM_VREG_PULLDOWN, &pulldown, &vreg_ldo12->id);
		}
	}
}

static void panel_nov_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	printk("mddi fixup\n");
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

static struct platform_driver glacier_backlight_driver = {
	.probe = glacier_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata_sharp = {
	.overrides = MSM_MDP_PANEL_FLIP_UD | MSM_MDP_PANEL_FLIP_LR,
};

static struct msm_mdp_platform_data mdp_pdata_common = {
	.overrides = MSM_MDP4_MDDI_DMA_SWITCH,
};

int __init glacier_init_panel(void)
{
	int rc;

	B(KERN_INFO "%s: enter. panel type %d\n", __func__, panel_type);

	/* turn on L12 for OJ. Note: must before L19 */
	vreg_ldo12 = vreg_get(NULL, "gp9");
	vreg_set_level(vreg_ldo12, 2850);

	/* lcd panel power */
	/* 2.85V -- LDO20 */
	vreg_ldo20 = vreg_get(NULL, "gp13");

	if (IS_ERR(vreg_ldo20)) {
		pr_err("%s: gp13 vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_ldo20));
		return -1;
	}

	vreg_ldo19 = vreg_get(NULL, "wlan2");

	if (IS_ERR(vreg_ldo19)) {
		pr_err("%s: wlan2 vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_ldo19));
		return -1;
	}

	rc = vreg_set_level(vreg_ldo20, 3000);
	if (rc) {
		pr_err("%s: vreg LDO20 set level failed (%d)\n",
			__func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo19, 1800);
	if (rc) {
		pr_err("%s: vreg LDO19 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (panel_type == PANEL_SHARP)
		msm_device_mdp.dev.platform_data = &mdp_pdata_sharp;
	else
		msm_device_mdp.dev.platform_data = &mdp_pdata_common;

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	mddi_pdata.clk_rate = 384000000;

	if (panel_type == 0) {
		mddi_pdata.type = MSM_MDP_MDDI_TYPE_I;
	} else {
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

	rc = platform_driver_register(&glacier_backlight_driver);
	if (rc)
		return rc;

	return 0;
}
