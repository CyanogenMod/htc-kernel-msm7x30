/* linux/arch/arm/mach-msm/board-runnymede-panel.c
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
#include <mach/msm_fb-7x30.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/panel_id.h>

#include "../board-runnymede.h"
#include "../devices.h"
#include "../proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

static struct vreg *V_LCMIO_1V8;
static struct vreg *V_LCMIO_2V8;

static struct clk *axi_clk;

#define PWM_USER_DEF	 	143
#define PWM_USER_MIN		30
#define PWM_USER_DIM		9
#define PWM_USER_MAX		255

#define PWM_NOVATEK_DEF		102
#define PWM_NOVATEK_MIN		6
#define PWM_NOVATEK_MAX		255

#define DEFAULT_BRIGHTNESS      PWM_USER_DEF

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
	int last_shrink_br;
} cabc;

enum {
	GATE_ON = 1 << 0,
};

enum led_brightness brightness_value = DEFAULT_BRIGHTNESS;

/* use one flag to have better backlight on/off performance */
static int runnymede_set_dim = 1;

static int
runnymede_shrink_pwm(int brightness, int user_def,
		int user_min, int user_max, int panel_def,
		int panel_min, int panel_max)
{
	if (brightness < PWM_USER_DIM)
		return 0;

	if (brightness < user_min)
		return panel_min;

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

static void
runnymede_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	shrink_br = runnymede_shrink_pwm(val, PWM_USER_DEF,
				PWM_USER_MIN, PWM_USER_MAX, PWM_NOVATEK_DEF,
				PWM_NOVATEK_MIN, PWM_NOVATEK_MAX);

	if (!client) {
		pr_info("null mddi client");
		return;
	}

	if (cabc.last_shrink_br == shrink_br) {
		pr_info("[BKL] identical shrink_br");
		return;
	}

	mutex_lock(&cabc.lock);

	if (runnymede_set_dim == 1) {
		client->remote_write(client, 0x2C, 0x5300);
		/* we dont need set dim again */
		runnymede_set_dim = 0;
	}
	client->remote_write(client, 0x00, 0x5500);
	client->remote_write(client, shrink_br, 0x5100);

	/* Update the last brightness */
	cabc.last_shrink_br = shrink_br;
	brightness_value = val;
	mutex_unlock(&cabc.lock);

	printk(KERN_INFO "set brightness to %d\n", shrink_br);
}

static enum led_brightness
runnymede_get_brightness(struct led_classdev *led_cdev)
{

	return brightness_value;
}

static void
runnymede_backlight_switch(int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;
		/* LED core uses get_brightness for default value
		  If the physical layer is not ready, we should
		not count on it */
		if (val == 0)
			val = brightness_value;
		runnymede_set_brightness(&cabc.lcd_backlight, val);
		 /*set next backlight value with dim */
		runnymede_set_dim = 1;
	} else {
		clear_bit(GATE_ON, &cabc.status);
		cabc.last_shrink_br = 0;
	}
}

static int
runnymede_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);

	mutex_init(&cabc.lock);
	cabc.last_shrink_br = 0;
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = runnymede_set_brightness;
	cabc.lcd_backlight.brightness_get = runnymede_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;

	return 0;
#if 0
	err = device_create_file(cabc.lcd_backlight.dev, &auto_attr);
	if (err)
		goto err_out;

	return 0;

err_out:
		device_remove_file(&pdev->dev, &auto_attr);
#endif

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

/* ------------------------------------------------------------------- */

static struct resource resources_msm_fb[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

#define REG_WAIT (0xffff)
struct nov_regs {
	unsigned reg;
	unsigned val;
};

struct nov_regs rue_sony_init_seq[] = {
	{ 0x1100 , 0x0000 },
	{REG_WAIT, 120},

	{ 0xF000 , 0x0055 },
	{ 0xF001 , 0x00AA },
	{ 0xF002 , 0x0052 },
	{ 0xF003 , 0x0008 },
	{ 0xF004 , 0x0000 },

	{ 0xB100 , 0x00EC },
	{ 0xB101 , 0x0000 },

	{ 0xE000 , 0x0000 }, /*PWM frequency = 9.77KHz */
	{ 0xE001 , 0x0002 },

	{ 0xF000 , 0x0055 },
	{ 0xF001 , 0x00AA },
	{ 0xF002 , 0x0052 },
	{ 0xF003 , 0x0008 },
	{ 0xF004 , 0x0001 },

	{ 0xCE00 , 0x0000 },
	{ 0xCE01 , 0x0000 },
	{ 0xCE02 , 0x0000 },
	{ 0xCE03 , 0x0000 },
	{ 0xCE04 , 0x0000 },
	{ 0xCE05 , 0x0000 },
	{ 0xCE06 , 0x0000 },

	{ 0xFF00 , 0x00AA },
	{ 0xFF01 , 0x0055 },
	{ 0xFF02 , 0x0025 },
	{ 0xFF03 , 0x0001 },

	{ 0xF813 , 0x0023 },

	{ 0x2900 , 0x0000 },
	{REG_WAIT, 40},

	{ 0x4400 , 0x0002 },
	{ 0x4401 , 0x0058 },
	{ 0x5E00 , 0x0006 },
	{ 0x5300 , 0x0020 },
	{ 0x3500 , 0x0000 }, /* TE Enable */
	{ 0x5500 , 0x0000 },
	{ 0x5100 , 0x0000 }, /* Brightness value */
};

static int
runnymede_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size = 0;
	unsigned reg, val;
	struct nov_regs *init_seq = NULL;

	B(KERN_INFO "%s\n", __func__);
	client_data->auto_hibernate(client_data, 0);

	switch (panel_type) {
	case PANEL_ID_RUE_SONY_NT:
	default:
		init_seq = rue_sony_init_seq;
		array_size = ARRAY_SIZE(rue_sony_init_seq);
	break;
	}

	for (i = 0; i < array_size; i++) {
		reg = cpu_to_le32(init_seq[i].reg);
		val = cpu_to_le32(init_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
	}

	client_data->auto_hibernate(client_data, 1);

	if (axi_clk)
		clk_set_rate(axi_clk, 0);

	return 0;
}

static int
runnymede_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int
runnymede_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);

	client_data->auto_hibernate(client_data, 0);

	client_data->remote_write(client_data, 0, 0x2800);
	client_data->remote_write(client_data, 0x0, 0x5300);
	runnymede_backlight_switch(LED_OFF);
	client_data->remote_write(client_data, 0, 0x1000);

	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
runnymede_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	client_data->auto_hibernate(client_data, 0);
	/* HTC, Add 50 ms delay for stability of driver IC at high temperature */
	hr_msleep(50);
	client_data->remote_write(client_data, 0x24, 0x5300);
	runnymede_backlight_switch(LED_FULL);
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = runnymede_mddi_init,
	.uninit = runnymede_mddi_uninit,
	.blank = runnymede_panel_blank,
	.unblank = runnymede_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 61,
		.height = 101,
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
		if (axi_clk)
			clk_set_rate(axi_clk, 192000000);

		config = PCOM_GPIO_CFG(RUNNYMEDE_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(RUNNYMEDE_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(RUNNYMEDE_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

		vreg_enable(V_LCMIO_1V8);
		vreg_enable(V_LCMIO_2V8);
		hr_msleep(2);

		gpio_set_value(RUNNYMEDE_LCD_RSTz, 1);
		hr_msleep(13);
		gpio_set_value(RUNNYMEDE_LCD_RSTz, 0);
		hr_msleep(13);
		gpio_set_value(RUNNYMEDE_LCD_RSTz, 1);
		hr_msleep(13);
	} else {
		gpio_set_value(RUNNYMEDE_LCD_RSTz, 0);
		vreg_disable(V_LCMIO_2V8);
		vreg_disable(V_LCMIO_1V8);

		config = PCOM_GPIO_CFG(RUNNYMEDE_MDDI_TE, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(RUNNYMEDE_LCD_ID1, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(RUNNYMEDE_LCD_ID0, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	}
}

static void
panel_mddi_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	printk(KERN_DEBUG"mddi fixup\n");
	*mfr_name = 0xb9f6;
	*product_code = 0x5560;
}

static struct msm_mddi_platform_data mddi_pdata = {
	.fixup = panel_mddi_fixup,
	.power_client = mddi_power,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5560),
			.name = "mddi_c_b9f6_5560",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_driver runnymede_backlight_driver = {
	.probe = runnymede_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
	.overrides = 0,
	.color_format = MSM_MDP_OUT_IF_FMT_RGB888,
#ifdef CONFIG_MDP4_HW_VSYNC
	.xres = 480,
	.yres = 800,
	.back_porch = 4,
	.front_porch = 2,
	.pulse_width = 4,
#endif
};

int __init runnymede_init_panel(void)
{
	int rc;

	B(KERN_INFO "%s: enter.\n", __func__);

	/* lcd panel power */
	V_LCMIO_1V8 = vreg_get(NULL, "wlan2");

	if (IS_ERR(V_LCMIO_1V8)) {
		pr_err("%s: LCMIO_1V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_1V8));
		return -1;
	}

	V_LCMIO_2V8 = vreg_get(NULL, "gp10");

	if (IS_ERR(V_LCMIO_2V8)) {
		pr_err("%s: LCMIO_2V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_2V8));
		return -1;
	}

	resources_msm_fb[0].start = msm_fb_base;
	resources_msm_fb[0].end = msm_fb_base + MSM_FB_SIZE - 1;

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	mddi_pdata.clk_rate = 384000000;

	mddi_pdata.type = MSM_MDP_MDDI_TYPE_II;

	axi_clk = clk_get(NULL, "ebi1_mddi_clk");
	if (IS_ERR(axi_clk)) {
		pr_err("%s: failed to get axi clock\n", __func__);
		return PTR_ERR(axi_clk);
	}

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	rc = platform_driver_register(&runnymede_backlight_driver);
	if (rc)
		return rc;

	return 0;
}
