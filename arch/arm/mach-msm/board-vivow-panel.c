/* linux/arch/arm/mach-msm/board-vivow-panel.c
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
#include <mach/panel_id.h>

#include "pmic.h"
#include "board-vivow.h"
#include "devices.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif
#define DEFAULT_BRIGHTNESS 100
extern int panel_type;

#define DRIVER_IC_CUT2			4
#define PANEL_VIVOW_SHARP		1
#define PANEL_VIVOW_SONY		2
#define PANEL_VIVOW_SHARP_CUT2		(PANEL_VIVOW_SHARP | DRIVER_IC_CUT2)
#define PANEL_VIVOW_SONY_CUT2		(PANEL_VIVOW_SONY | DRIVER_IC_CUT2)
#define	PANEL_VIVOW_HITACHI		PANEL_ID_VIVOW_HITACHI

#define VIVOW_BR_DEF_USER_PWM		143
#define VIVOW_BR_MIN_USER_PWM		30
#define VIVOW_BR_MAX_USER_PWM		255
#define VIVOW_BR_DEF_SONY_PANEL_PWM		135
#define VIVOW_BR_MIN_SONY_PANEL_PWM		8
#define VIVOW_BR_MAX_SONY_PANEL_PWM		255
#define VIVOW_BR_DEF_HITACHI_PANEL_PWM		146
#define VIVOW_BR_MIN_HITACHI_PANEL_PWM		8
#define VIVOW_BR_MAX_HITACHI_PANEL_PWM		255

#define LCM_CMD(_cmd, _delay, ...)                              \
{                                                               \
        .cmd = _cmd,                                            \
        .delay = _delay,                                        \
        .vals = (u8 []){__VA_ARGS__},                           \
        .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}
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
	CABC_STATE,
};

struct mddi_cmd {
        unsigned char cmd;
        unsigned delay;
        unsigned char *vals;
        unsigned len;
};

static struct mddi_cmd hitachi_renesas_cmd[] = {
	LCM_CMD(0x2A, 0, 0x00, 0x00, 0x01, 0xDF),
	LCM_CMD(0x2B, 0, 0x00, 0x00, 0x03, 0x1F),
	LCM_CMD(0x36, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x3A, 0, 0x55, 0x55, 0x55, 0x55),//set_pixel_format 0x66 for 18bit/pixel, 0x77 for 24bit/pixel
	LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
	LCM_CMD(0x35, 0, 0x00, 0x00, 0x00, 0x00),//TE enable
	LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
//	LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
//	LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
//	LCM_CMD(0x29, 100,0x00,0x00,0x00,0x00),
};

static struct mddi_cmd hitachi_renesas_driving_cmd[] = {
	LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
	LCM_CMD(0xC1, 0, 0x43, 0x31, 0x00, 0x00),
	LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
};


static struct mddi_cmd hitachi_renesas_backlight_cmd[] = {
	LCM_CMD(0xB9, 0, 0x00, 0x00, 0x00, 0x00,
			 0xff, 0x00, 0x00, 0x00,
			 0x03, 0x00, 0x00, 0x00,
			 0x08, 0x00, 0x00, 0x00,),
};

enum led_brightness vivow_brightness_value = DEFAULT_BRIGHTNESS;// multiple definition of `brightness_value' in board-glacier-panel.c

static void
do_renesas_cmd(struct msm_mddi_client_data *client_data, struct mddi_cmd *cmd_table, ssize_t size)
{
	struct mddi_cmd *pcmd = NULL;
	for (pcmd = cmd_table; pcmd < cmd_table + size; pcmd++) {
		client_data->remote_write_vals(client_data, pcmd->vals,
			pcmd->cmd, pcmd->len);
		if (pcmd->delay)
			hr_msleep(pcmd->delay);
	}
}

static int vivow_shrink_pwm(int brightness)
{
	int level;
	unsigned int min_pwm, def_pwm, max_pwm;

	if(panel_type == PANEL_VIVOW_HITACHI) {
		min_pwm = VIVOW_BR_MIN_HITACHI_PANEL_PWM;
		def_pwm = VIVOW_BR_DEF_HITACHI_PANEL_PWM;
		max_pwm = VIVOW_BR_MAX_HITACHI_PANEL_PWM;
		if (brightness <= VIVOW_BR_DEF_USER_PWM) {
			if (brightness <= VIVOW_BR_MIN_USER_PWM)
				level = min_pwm;
			else
				level = (def_pwm - min_pwm) *
					(brightness - VIVOW_BR_MIN_USER_PWM) /
					(VIVOW_BR_DEF_USER_PWM - VIVOW_BR_MIN_USER_PWM) +
					min_pwm;
		} else
			level = (max_pwm - def_pwm) *
			(brightness - VIVOW_BR_DEF_USER_PWM) /
			(VIVOW_BR_MAX_USER_PWM - VIVOW_BR_DEF_USER_PWM) +
			def_pwm;
	} else {
		min_pwm = VIVOW_BR_MIN_SONY_PANEL_PWM;
		def_pwm = VIVOW_BR_DEF_SONY_PANEL_PWM;
		max_pwm = VIVOW_BR_MAX_SONY_PANEL_PWM;
		if (brightness <= VIVOW_BR_DEF_USER_PWM) {
			if (brightness <= VIVOW_BR_MIN_USER_PWM)
				level = min_pwm;
			else
				level = (def_pwm - min_pwm) *
					(brightness - VIVOW_BR_MIN_USER_PWM) /
					(VIVOW_BR_DEF_USER_PWM - VIVOW_BR_MIN_USER_PWM) +
					min_pwm;
		} else
			level = (max_pwm - def_pwm) *
			(brightness - VIVOW_BR_DEF_USER_PWM) /
			(VIVOW_BR_MAX_USER_PWM - VIVOW_BR_DEF_USER_PWM) +
			def_pwm;
	}

	return level;
}

/* use one flag to have better backlight on/off performance */
static int vivow_set_dim = 1;

static void vivow_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;
	struct mddi_cmd *pcmd = hitachi_renesas_backlight_cmd;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;
	shrink_br = vivow_shrink_pwm(val);

	if(!client) {
		pr_info("null mddi client");
		return;
	}

	mutex_lock(&cabc.lock);
	if(panel_type == PANEL_VIVOW_HITACHI) {
		pcmd->vals[4] = shrink_br;
		client->remote_write(client, 0x04, 0xB0);
	        client->remote_write_vals(client, pcmd->vals, pcmd->cmd, pcmd->len);
		client->remote_write(client, 0x03, 0xB0);
	} else {
		if (vivow_set_dim == 1) {
			client->remote_write(client, 0x2C, 0x5300);
			/* we dont need set dim again */
			vivow_set_dim = 0;
		}
		client->remote_write(client, 0x00, 0x5500);
		client->remote_write(client, shrink_br, 0x5100);
	}

	vivow_brightness_value = val;
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
vivow_get_brightness(struct led_classdev *led_cdev)
{
	/*FIXME:workaround for NOVATEK driver IC*/
#if 0
	struct msm_mddi_client_data *client = cabc.client_data;
	return client->remote_read(client, 0x5100);
#else
	return vivow_brightness_value;
#endif
}

static void vivow_backlight_switch(int on)
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
		vivow_set_brightness(&cabc.lcd_backlight, val);
		/* set next backlight value with dim */
		vivow_set_dim = 1;
	} else {
		clear_bit(GATE_ON, &cabc.status);
		vivow_set_brightness(&cabc.lcd_backlight, 0);
	}
}

static int vivow_cabc_switch(int on)
{
	struct msm_mddi_client_data *client = cabc.client_data;

	if (test_bit(CABC_STATE, &cabc.status) == on)
               return 1;

	if (on) {
		printk(KERN_DEBUG "turn on CABC\n");
		set_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x01, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	} else {
		printk(KERN_DEBUG "turn off CABC\n");
		clear_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x00, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	}
	return 1;
}

static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count);
#define CABC_ATTR(name) __ATTR(name, 0644, auto_backlight_show, auto_backlight_store)

static struct device_attribute auto_attr = CABC_ATTR(auto);
static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - 1, "%d\n",
				test_bit(CABC_STATE, &cabc.status));
	return i;
}

static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned long res;

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		printk(KERN_ERR "invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	if (vivow_cabc_switch(!!res))
		count = -EIO;

err_out:
	return count;
}

static int vivow_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = vivow_set_brightness;
	cabc.lcd_backlight.brightness_get = vivow_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;

	err = device_create_file(cabc.lcd_backlight.dev, &auto_attr);
	if (err)
		goto err_out;

	return 0;

err_out:
	device_remove_file(&pdev->dev, &auto_attr);
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
	{0x89C3, 0x80},
	{0x92C2, 0x08},
	{0x0180, 0x14},
	{0x0280, 0x11},
	{0x0380, 0x33},
	{0x0480, 0x63},
	{0x0580, 0x63},
	{0x0680, 0x63},
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
	{0x1380, 0x21},
	{0x1480, 0x0E},
	{0x1580, 0xB7},
	{0x1680, 0xEC},
	{0x1780, 0x00},
	{0x1880, 0x00},
	{0x1980, 0x00},
	{0x1C80, 0x00},
	{0x1F80, 0x05},
	{0x2480, 0x2B},
	{0x2580, 0x2F},
	{0x2680, 0x3D},
	{0x2780, 0x48},
	{0x2880, 0x1A},
	{0x2980, 0x2E},
	{0x2A80, 0x5F},
	{0x2B80, 0x42},
	{0x2D80, 0x20},
	{0x2F80, 0x27},
	{0x3080, 0x78},
	{0x3180, 0x1A},
	{0x3280, 0x40},
	{0x3380, 0x55},
	{0x3480, 0x5F},
	{0x3580, 0x7B},
	{0x3680, 0x92},
	{0x3780, 0x42},
	{0x3880, 0x3D},
	{0x3980, 0x40},
	{0x3A80, 0x4E},
	{0x3B80, 0x5A},
	{0x3D80, 0x19},
	{0x3F80, 0x2E},
	{0x4080, 0x5F},
	{0x4180, 0x55},
	{0x4280, 0x1F},
	{0x4380, 0x26},
	{0x4480, 0x8E},
	{0x4580, 0x19},
	{0x4680, 0x41},
	{0x4780, 0x55},
	{0x4880, 0x77},
	{0x4980, 0x92},
	{0x4A80, 0xA9},
	{0x4B80, 0x59},
	{0x4C80, 0x2B},
	{0x4D80, 0x2F},
	{0x4E80, 0x3D},
	{0x4F80, 0x48},
	{0x5080, 0x1A},
	{0x5180, 0x2E},
	{0x5280, 0x5F},
	{0x5380, 0x42},
	{0x5480, 0x20},
	{0x5580, 0x27},
	{0x5680, 0x78},
	{0x5780, 0x1A},
	{0x5880, 0x40},
	{0x5980, 0x55},
	{0x5A80, 0x5F},
	{0x5B80, 0x7B},
	{0x5C80, 0x92},
	{0x5D80, 0x42},
	{0x5E80, 0x3D},
	{0x5F80, 0x40},
	{0x6080, 0x4E},
	{0x6180, 0x5A},
	{0x6280, 0x19},
	{0x6380, 0x2E},
	{0x6480, 0x5F},
	{0x6580, 0x55},
	{0x6680, 0x1F},
	{0x6780, 0x26},
	{0x6880, 0x8E},
	{0x6980, 0x19},
	{0x6A80, 0x41},
	{0x6B80, 0x55},
	{0x6C80, 0x77},
	{0x6D80, 0x92},
	{0x6E80, 0xA9},
	{0x6F80, 0x59},
	{0x7080, 0x2B},
	{0x7180, 0x2F},
	{0x7280, 0x3D},
	{0x7380, 0x48},
	{0x7480, 0x1A},
	{0x7580, 0x2E},
	{0x7680, 0x5F},
	{0x7780, 0x42},
	{0x7880, 0x20},
	{0x7980, 0x27},
	{0x7A80, 0x78},
	{0x7B80, 0x1A},
	{0x7C80, 0x40},
	{0x7D80, 0x55},
	{0x7E80, 0x5F},
	{0x7F80, 0x7B},
	{0x8080, 0x92},
	{0x8180, 0x42},
	{0x8280, 0x3D},
	{0x8380, 0x40},
	{0x8480, 0x4E},
	{0x8580, 0x5A},
	{0x8680, 0x19},
	{0x8780, 0x2E},
	{0x8880, 0x5F},
	{0x8980, 0x55},
	{0x8A80, 0x1F},
	{0x8B80, 0x26},
	{0x8C80, 0x8E},
	{0x8D80, 0x19},
	{0x8E80, 0x41},
	{0x8F80, 0x55},
	{0x9080, 0x77},
	{0x9180, 0x92},
	{0x9280, 0xA9},
	{0x9380, 0x59},
	{0x9480, 0xB5},
	{0x9580, 0x04},
	{0x9680, 0x18},
	{0x9780, 0xB0},
	{0x9880, 0x20},
	{0x9980, 0x28},
	{0x9A80, 0x08},
	{0x9B80, 0x04},
	{0x9C80, 0x12},
	{0x9D80, 0x00},
	{0x9E80, 0x00},
	{0x9F80, 0x12},
	{0xA080, 0x00},
	{0xA280, 0x00},
	{0xA380, 0x3C},
	{0xA480, 0x01},
	{0xA580, 0xC0},
	{0xA680, 0x01},
	{0xA780, 0x00},
	{0xA980, 0x00},
	{0xAA80, 0x00},
	{0xAB80, 0x70},
	{0xE780, 0x11},
	{0xE880, 0x11},
	{0xED80, 0x0A},
	{0xEE80, 0x80},
	{0xF780, 0x0D},
	{0x2900, 0x00},
	{0x3500, 0x00},
	{0x4400, 0x02},
	{0x4401, 0x58},
};
static struct nov_regs sony_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 120},
	{0x0480, 0x63},
	{0x0580, 0x63},
	{0x0680, 0x63},
	{0x5E00, 0x06},
	{0x1DC0, 0x3F},
	{0x1EC0, 0x40},
	{0x3BC0, 0xF3},
	{0x3DC0, 0xEF},
	{0x3FC0, 0xEB},
	{0x40C0, 0xE7},
	{0x41C0, 0xE3},
	{0x42C0, 0xDF},
	{0x43C0, 0xDB},
	{0x44C0, 0xD7},
	{0x45C0, 0xD3},
	{0x46C0, 0xCF},
	{0x3500, 0x00},
	{0x4400, 0x02},
	{0x4401, 0x58},
	{0x2900, 0x00},
};

static int
vivow_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size = 0;
	unsigned reg, val;
	struct nov_regs *init_seq= NULL;
	B(KERN_DEBUG "%s\n", __func__);
	client_data->auto_hibernate(client_data, 0);

	if(panel_type == PANEL_VIVOW_HITACHI) {
		do_renesas_cmd(client_data, hitachi_renesas_cmd, ARRAY_SIZE(hitachi_renesas_cmd));
	}
	else {
		if (panel_type == PANEL_VIVOW_SONY
			|| panel_type == PANEL_VIVOW_SONY_CUT2) {
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
				hr_msleep(val);
			else {
				client_data->remote_write(client_data, val, reg);
				if (reg == 0x1100)
					client_data->send_powerdown(client_data);
			}
		}
	}
	client_data->auto_hibernate(client_data, 1);

	if(axi_clk)
		clk_set_rate(axi_clk, 0);

	return 0;
}

static int
vivow_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int
vivow_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	client_data->auto_hibernate(client_data, 0);
	if(panel_type == PANEL_VIVOW_HITACHI) {
		client_data->remote_write(client_data, 0x0, 0x28);
		vivow_backlight_switch(LED_OFF);
		client_data->remote_write(client_data, 0x0, 0xB8);
		client_data->remote_write(client_data, 0x0, 0x10);
		hr_msleep(72);
	}else {

		client_data->remote_write(client_data, 0x0, 0x5300);
		vivow_backlight_switch(LED_OFF);
		client_data->remote_write(client_data, 0, 0x2800);
		client_data->remote_write(client_data, 0, 0x1000);
	}
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
vivow_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	client_data->auto_hibernate(client_data, 0);
	if(panel_type == PANEL_VIVOW_HITACHI) {
		client_data->remote_write(client_data, 0x0, 0x11);
		hr_msleep(125);
		vivow_backlight_switch(LED_FULL);
		do_renesas_cmd(client_data, hitachi_renesas_driving_cmd, ARRAY_SIZE(hitachi_renesas_driving_cmd));
		client_data->remote_write(client_data, 0x0, 0x29);

	}else {
		client_data->remote_write(client_data, 0x24, 0x5300);
		client_data->remote_write(client_data, 0x0A, 0x22C0);
		hr_msleep(30);
		vivow_backlight_switch(LED_FULL);
	}
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = vivow_mddi_init,
	.uninit = vivow_mddi_uninit,
	.blank = vivow_panel_blank,
	.unblank = vivow_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 36,
		.height = 108,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
		.vsync_gpio = 30,
	},
};

static struct msm_mddi_bridge_platform_data renesas_client_data = {
	.init = vivow_mddi_init,
	.uninit = vivow_mddi_uninit,
	.blank = vivow_panel_blank,
	.unblank = vivow_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 52,
		.height = 86,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
		.vsync_gpio = 30,
	},
};

static void
mddi_power(struct msm_mddi_client_data *client_data, int on)
{
	int rc;
	unsigned config;
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);

	if (on) {
		if(axi_clk)
			clk_set_rate(axi_clk, 192000000);

		config = PCOM_GPIO_CFG(VIVOW_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

		if(panel_type == PANEL_VIVOW_HITACHI) {
			vreg_enable(V_LCMIO_1V8);
			vreg_enable(V_LCM_2V85);
			hr_msleep(1);

			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(5);
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(5);
		} else {
			vreg_enable(V_LCM_2V85);
			hr_msleep(3);
			vreg_enable(V_LCMIO_1V8);
			hr_msleep(5);

			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(1);
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(15);
		}

	} else {

		if(panel_type == PANEL_VIVOW_HITACHI) {
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(V_LCM_2V85);
			vreg_disable(V_LCMIO_1V8);
		} else {
			hr_msleep(80);
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(V_LCMIO_1V8);
			vreg_disable(V_LCM_2V85);
		}

		config = PCOM_GPIO_CFG(VIVOW_MDDI_TE, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	}
}

static void mddi_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	printk(KERN_INFO "mddi fixup\n");
	if (panel_type == PANEL_VIVOW_SONY
		|| panel_type == PANEL_VIVOW_SONY_CUT2) {
		*mfr_name = 0xb9f6;
		*product_code = 0x5560;
	}else {
		*mfr_name = 0xb9f6;
		*product_code = 0x1408;
	}
}

static struct msm_mddi_platform_data mddi_pdata = {
	.fixup = mddi_fixup,
	.power_client = mddi_power,
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
			.product_id = (0xb9f6 << 16 | 0x1408),
			.name = "mddi_renesas_b9f6_61408",
			.id = 1,
			.client_data = &renesas_client_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_driver vivow_backlight_driver = {
	.probe = vivow_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
	.overrides = MSM_MDP4_MDDI_DMA_SWITCH,
};

int __init vivow_init_panel(unsigned int sys_rev)
{
	int rc;

	B(KERN_INFO "%s(%d): enter. panel_type 0x%08x\n", __func__, __LINE__, panel_type);

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	if (panel_type & DRIVER_IC_CUT2 || panel_type == PANEL_VIVOW_HITACHI)
		mddi_pdata.clk_rate = 384000000;
	else
		mddi_pdata.clk_rate = 256000000;

	mddi_pdata.type = MSM_MDP_MDDI_TYPE_II;

	axi_clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(axi_clk)) {
		pr_err("%s: failed to get axi clock\n", __func__);
		return PTR_ERR(axi_clk);
	}

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	if(panel_type == PANEL_VIVOW_HITACHI)
		vivow_backlight_driver.driver.name = "renesas_backlight";

	rc = platform_driver_register(&vivow_backlight_driver);
	if (rc)
		return rc;

	/* lcd panel power */
	/* 2.85V -- LDO20 */
	V_LCM_2V85 = vreg_get(NULL, "gp13");

	if (IS_ERR(V_LCM_2V85)) {
		pr_err("%s: LCM_2V85 get failed (%ld)\n",
			__func__, PTR_ERR(V_LCM_2V85));
		return -1;
	}
	V_LCMIO_1V8 = vreg_get(NULL, "lvsw0");

	if (IS_ERR(V_LCMIO_1V8)) {
		pr_err("%s: LCMIO_1V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_1V8));
		return -1;
	}

	return 0;
}
