/* linux/arch/arm/mach-msm/board-saga-panel.c
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
#include <mach/msm_panel.h>
#include <mach/panel_id.h>


#include "board-saga.h"
#include "devices.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif
#define LCM_GPIO_CFG(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
extern int panel_type;
struct vreg *vreg_ldo19, *vreg_ldo20;
struct mddi_cmd {
        unsigned char cmd;
        unsigned delay;
        unsigned char *vals;
        unsigned len;
};
#define prm_size 20
#define LCM_CMD(_cmd, _delay, ...)                              \
{                                                               \
        .cmd = _cmd,                                            \
        .delay = _delay,                                        \
        .vals = (u8 []){__VA_ARGS__},                           \
        .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}
#define DEFAULT_BRIGHTNESS 255
#define PWM_USER_DEF	 		143
#define PWM_USER_MIN			30
#define PWM_USER_DIM			 9
#define PWM_USER_MAX			255

#define PWM_HITACHI_DEF			147
#define PWM_HITACHI_MIN			 9
#define PWM_HITACHI_MAX			217
enum {
	GATE_ON = 1 << 0,
};
static struct renesas_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} renesas;
static struct mddi_cmd tear[] = {
	LCM_CMD(0x44, 0, 0x01, 0x00, 0x00, 0x00,
                         0x90, 0x00, 0x00, 0x00)
		};

static struct mddi_cmd saga_renesas_cmd[] = {
	LCM_CMD(0x2A, 0, 0x00, 0x00, 0x01, 0xDF),
	LCM_CMD(0x2B, 0, 0x00, 0x00, 0x03, 0x1F),
	LCM_CMD(0x36, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x3A, 0, 0x55, 0x00, 0x00, 0x00),//set_pixel_format 0x66 for 18bit/pixel, 0x77 for 24bit/pixel
};
static struct mddi_cmd saga_renesas_backlight_blank_cmd[] = {
	LCM_CMD(0xB9, 0, 0x00, 0x00, 0x00, 0x00,
			 0xff, 0x00, 0x00, 0x00,
			 0x04, 0x00, 0x00, 0x00,//adjust PWM frequency to 10.91k .
			 0x08, 0x00, 0x00, 0x00,),
};
static struct mddi_cmd gama[] = {
           LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
           LCM_CMD(0xC1, 0, 0x43, 0x00, 0x00, 0x00,
                                 0x31, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00,
                                 0x21, 0x00, 0x00, 0x00,
                                 0x21, 0x00, 0x00, 0x00,
                                 0x32, 0x00, 0x00, 0x00,
                                 0x12, 0x00, 0x00, 0x00,
                                 0x28, 0x00, 0x00, 0x00,
                                 0x4A, 0x00, 0x00, 0x00,
                                 0x1E, 0x00, 0x00, 0x00,
                                 0xA5, 0x00, 0x00, 0x00,
                                 0x0F, 0x00, 0x00, 0x00,
                                 0x58, 0x00, 0x00, 0x00,
                                 0x21, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xC8, 0, 0x2D, 0x00, 0x00, 0x00,
                                 0x2F, 0x00, 0x00, 0x00,
                                 0x31, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x3E, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x23, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0B, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00,
                                 0x2D, 0x00, 0x00, 0x00,
                                 0x2F, 0x00, 0x00, 0x00,
                                 0x31, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x3E, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x23, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0B, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xC9, 0, 0x00, 0x00, 0x00, 0x00,
                                 0x0F, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x33, 0x00, 0x00, 0x00,
                                 0x4D, 0x00, 0x00, 0x00,
                                 0x38, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x11, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00,
                                 0x0F, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x33, 0x00, 0x00, 0x00,
                                 0x4D, 0x00, 0x00, 0x00,
                                 0x38, 0x00, 0x00, 0x00,
                                 0x25, 0x00, 0x00, 0x00,
                                 0x18, 0x00, 0x00, 0x00,
                                 0x11, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xCA, 0, 0x27, 0x00, 0x00, 0x00,
                                 0x2A, 0x00, 0x00, 0x00,
                                 0x2E, 0x00, 0x00, 0x00,
                                 0x34, 0x00, 0x00, 0x00,
                                 0x3C, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x24, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0C, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00,
                                 0x27, 0x00, 0x00, 0x00,
                                 0x2A, 0x00, 0x00, 0x00,
                                 0x2E, 0x00, 0x00, 0x00,
                                 0x34, 0x00, 0x00, 0x00,
                                 0x3C, 0x00, 0x00, 0x00,
                                 0x51, 0x00, 0x00, 0x00,
                                 0x36, 0x00, 0x00, 0x00,
                                 0x24, 0x00, 0x00, 0x00,
                                 0x16, 0x00, 0x00, 0x00,
                                 0x0C, 0x00, 0x00, 0x00,
                                 0x02, 0x00, 0x00, 0x00,
                                 0x01, 0x00, 0x00, 0x00 ),
           LCM_CMD(0xD5, 0, 0x14, 0x00, 0x00, 0x00,
                                 0x14, 0x00, 0x00, 0x00
           ),
           LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
};
static uint32_t display_on_gpio_table[] = {
	LCM_GPIO_CFG(SAGA_LCD_PCLK_1, 1),
	LCM_GPIO_CFG(SAGA_LCD_DE, 1),
	LCM_GPIO_CFG(SAGA_LCD_VSYNC, 1),
	LCM_GPIO_CFG(SAGA_LCD_HSYNC, 1),
	LCM_GPIO_CFG(SAGA_LCD_G0, 1),
	LCM_GPIO_CFG(SAGA_LCD_G1, 1),
	LCM_GPIO_CFG(SAGA_LCD_G2, 1),
	LCM_GPIO_CFG(SAGA_LCD_G3, 1),
	LCM_GPIO_CFG(SAGA_LCD_G4, 1),
	LCM_GPIO_CFG(SAGA_LCD_G5, 1),
	LCM_GPIO_CFG(SAGA_LCD_B0, 1),
	LCM_GPIO_CFG(SAGA_LCD_B1, 1),
	LCM_GPIO_CFG(SAGA_LCD_B2, 1),
	LCM_GPIO_CFG(SAGA_LCD_B3, 1),
	LCM_GPIO_CFG(SAGA_LCD_B4, 1),
	LCM_GPIO_CFG(SAGA_LCD_B5, 1),
	LCM_GPIO_CFG(SAGA_LCD_R0, 1),
	LCM_GPIO_CFG(SAGA_LCD_R1, 1),
	LCM_GPIO_CFG(SAGA_LCD_R2, 1),
	LCM_GPIO_CFG(SAGA_LCD_R3, 1),
	LCM_GPIO_CFG(SAGA_LCD_R4, 1),
	LCM_GPIO_CFG(SAGA_LCD_R5, 1),
};

static uint32_t display_off_gpio_table[] = {
	LCM_GPIO_CFG(SAGA_LCD_PCLK_1, 0),
	LCM_GPIO_CFG(SAGA_LCD_DE, 0),
	LCM_GPIO_CFG(SAGA_LCD_VSYNC, 0),
	LCM_GPIO_CFG(SAGA_LCD_HSYNC, 0),
	LCM_GPIO_CFG(SAGA_LCD_G0, 0),
	LCM_GPIO_CFG(SAGA_LCD_G1, 0),
	LCM_GPIO_CFG(SAGA_LCD_G2, 0),
	LCM_GPIO_CFG(SAGA_LCD_G3, 0),
	LCM_GPIO_CFG(SAGA_LCD_G4, 0),
	LCM_GPIO_CFG(SAGA_LCD_G5, 0),
	LCM_GPIO_CFG(SAGA_LCD_B0, 0),
	LCM_GPIO_CFG(SAGA_LCD_B1, 0),
	LCM_GPIO_CFG(SAGA_LCD_B2, 0),
	LCM_GPIO_CFG(SAGA_LCD_B3, 0),
	LCM_GPIO_CFG(SAGA_LCD_B4, 0),
	LCM_GPIO_CFG(SAGA_LCD_B5, 0),
	LCM_GPIO_CFG(SAGA_LCD_R0, 0),
	LCM_GPIO_CFG(SAGA_LCD_R1, 0),
	LCM_GPIO_CFG(SAGA_LCD_R2, 0),
	LCM_GPIO_CFG(SAGA_LCD_R3, 0),
	LCM_GPIO_CFG(SAGA_LCD_R4, 0),
	LCM_GPIO_CFG(SAGA_LCD_R5, 0),
};

static int panel_gpio_switch(int on)
{
	config_gpio_table(
		!!on ? display_on_gpio_table : display_off_gpio_table,
		!!on ? ARRAY_SIZE(display_on_gpio_table) : ARRAY_SIZE(display_off_gpio_table));

	return 0;
}

static inline int is_sony_panel(void){
	return (panel_type == PANEL_ID_SAG_SONY)? 1 : 0;
}
static inline int is_hitachi_panel(void){
	return (panel_type == PANEL_ID_SAG_HITACHI)? 1 : 0;
}
static int panel_sony_power(int on)
{
	int rc = 0;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		rc = vreg_enable(vreg_ldo19);
	}
	if (rc) {
		pr_err("%s: LDO19 vreg enable failed (%d)\n",
		__func__, rc);
		return -1;
	}

	if (on)
		rc = vreg_enable(vreg_ldo20);
	if (rc) {
		pr_err("%s: LDO20 vreg enable failed (%d)\n",
			__func__, rc);
		return -1;
	}

	if (on) {
		hr_msleep(10);
		gpio_set_value(SAGA_LCD_RSTz_ID1, 1);
		hr_msleep(10);
		gpio_set_value(SAGA_LCD_RSTz_ID1, 0);
		udelay(500);
		gpio_set_value(SAGA_LCD_RSTz_ID1, 1);
		hr_msleep(10);
	} else if (!on) {
		hr_msleep(10);
		gpio_set_value(SAGA_LCD_RSTz_ID1, 0);
		hr_msleep(120);
	}

	if(!on) {
		rc = vreg_disable(vreg_ldo19);
		rc = vreg_disable(vreg_ldo20);
	}

	if (rc) {
		pr_err("%s: LDO19, 20 vreg disable failed (%d)\n",
		__func__, rc);
		return -1;
	}

	return 0;
}


static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct panel_platform_data sonywvga_data = {
        .fb_res = &resources_msm_fb[0],
        .power = panel_sony_power,
        .gpio_switch = panel_gpio_switch,
};

static struct platform_device sonywvga_panel = {
        .name = "panel-sonywvga-s6d16a0x21-7x30",
        .id = -1,
        .dev = {
                .platform_data = &sonywvga_data,
        },
};
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
enum led_brightness brightness_value = DEFAULT_BRIGHTNESS;

/* use one flag to have better backlight on/off performance */
static int saga_set_dim = 1;


static int
saga_shrink_pwm(int brightness, int user_def,
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

static void
saga_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = renesas.client_data;
	unsigned int shrink_br = val;
	struct mddi_cmd *pcmd = saga_renesas_backlight_blank_cmd;

	//printk(KERN_DEBUG "set brightness = %d\n", val);

	if (test_bit(GATE_ON, &renesas.status) == 0)
		return;

	shrink_br = saga_shrink_pwm(val, PWM_USER_DEF,
				PWM_USER_MIN, PWM_USER_MAX, PWM_HITACHI_DEF,
				PWM_HITACHI_MIN, PWM_HITACHI_MAX);
	pcmd->vals[4] = shrink_br;

	mutex_lock(&renesas.lock);
	if (saga_set_dim == 1) {
		//client->remote_write(client, 0x2C, 0x5300);
		/* we dont need set dim again */
		saga_set_dim = 0;
	}
	client->remote_write(client, 0x04, 0xB0);
        client->remote_write_vals(client, pcmd->vals, pcmd->cmd, pcmd->len);
	client->remote_write(client, 0x03, 0xB0);
	brightness_value = val;
	mutex_unlock(&renesas.lock);
}

static enum led_brightness
saga_get_brightness(struct led_classdev *led_cdev)
{

	return brightness_value;
}


static void
saga_backlight_switch(struct msm_mddi_client_data *client_data, int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &renesas.status);
		val = renesas.lcd_backlight.brightness;
		/* LED core uses get_brightness for default value
		  If the physical layer is not ready, we should*/
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		saga_set_brightness(&renesas.lcd_backlight, val);
		 /*set next backlight value with dim */
		//glacier_set_dim = 1;
	} else {
		do_renesas_cmd(client_data, saga_renesas_backlight_blank_cmd, ARRAY_SIZE(saga_renesas_backlight_blank_cmd));
		saga_set_brightness(&renesas.lcd_backlight, 0);
		clear_bit(GATE_ON, &renesas.status);
	}
}

static int
saga_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	mutex_init(&renesas.lock);
	renesas.client_data = pdev->dev.platform_data;
	renesas.lcd_backlight.name = "lcd-backlight";
	renesas.lcd_backlight.brightness_set = saga_set_brightness;
	renesas.lcd_backlight.brightness_get = saga_get_brightness;
	err = led_classdev_register(&pdev->dev, &renesas.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&renesas.lcd_backlight);
	return err;
}

static int
saga_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	do_renesas_cmd(client_data, saga_renesas_cmd, ARRAY_SIZE(saga_renesas_cmd));
	return 0;
}

static int
saga_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	client_data->auto_hibernate(client_data, 0);
	client_data->remote_write(client_data, 0x0, 0x10);
	hr_msleep(72);
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
saga_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	client_data->remote_write(client_data, 0x04, 0xB0);
	client_data->remote_write(client_data, 0x0, 0x28);
	saga_backlight_switch(client_data,LED_OFF);
	client_data->remote_write(client_data, 0x0, 0xB8);
	client_data->remote_write(client_data, 0x03, 0xB0);
	hr_msleep(72);
	return 0;
}

static int
saga_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s +\n", __func__);

	client_data->auto_hibernate(client_data, 0);

	client_data->remote_write(client_data, 0x04, 0xB0);

	client_data->remote_write(client_data, 0x0, 0x11);
	hr_msleep(125);
	do_renesas_cmd(client_data, gama, ARRAY_SIZE(gama));
	saga_backlight_switch(client_data, LED_FULL);
	client_data->remote_write(client_data, 0x0, 0x29);
	do_renesas_cmd(client_data, tear, ARRAY_SIZE(tear));
	client_data->remote_write(client_data, 0x0, 0x35);
	client_data->remote_write(client_data, 0x03, 0xB0);

	client_data->auto_hibernate(client_data, 1);

	B(KERN_DEBUG "%s -\n", __func__);
	return 0;
}

static struct msm_mddi_bridge_platform_data renesas_client_data = {
	.init = saga_mddi_init,
	.uninit = saga_mddi_uninit,
	.blank = saga_panel_blank,
	.unblank = saga_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 48,
		.height = 80,
		.output_format = 0,
	},
};

static void
mddi_hitachi_power(struct msm_mddi_client_data *client_data, int on)
{
	if (panel_type == PANEL_ID_SAG_HITACHI) {
		if (on) {
			vreg_enable(vreg_ldo19);
			gpio_set_value(SAGA_MDDI_RSTz,0);
			vreg_enable(vreg_ldo20);
			hr_msleep(1);
			gpio_set_value(SAGA_MDDI_RSTz,1);
			hr_msleep(5);
		}
		else {
			vreg_disable(vreg_ldo19);
			vreg_disable(vreg_ldo20);
		}
	}
}

static void
panel_renesas_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	printk("mddi fixup\n");
	*mfr_name = 0xb9f6;
	*product_code = 0x1408;
}


static struct msm_mddi_platform_data mddi_pdata = {
	.fixup = panel_renesas_fixup,
	.power_client = mddi_hitachi_power,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x1408),
			.name = "mddi_renesas_b9f6_61408",
			.id = 0,
			.client_data = &renesas_client_data,
			.clk_rate = 0,
		},
	},
};
static struct platform_driver saga_backlight_driver = {
	.probe = saga_backlight_probe,
	.driver = {
		.name = "renesas_backlight",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata_hitachi = {
	//.overrides = MSM_MDP_PANEL_FLIP_UD | MSM_MDP_PANEL_FLIP_LR,
	.overrides = MSM_MDP4_MDDI_DMA_SWITCH,
};

int __init saga_init_panel(void)
{
        int ret = 0;
	int rc = 0;

	vreg_ldo19 = vreg_get(NULL, "wlan2");

	if (IS_ERR(vreg_ldo19)) {
		pr_err("%s: wlan2 vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_ldo19));
		return -1;
	}

	/* lcd panel power */
	/* 2.85V -- LDO20 */
	vreg_ldo20 = vreg_get(NULL, "gp13");

	if (IS_ERR(vreg_ldo20)) {
		pr_err("%s: gp13 vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_ldo20));
		return -1;
	}

	rc = vreg_set_level(vreg_ldo19, 1800);
	if (rc) {
		pr_err("%s: vreg LDO19 set level failed (%d)\n",
		       __func__, rc);
		return -1;
	}


	if (is_sony_panel()){
		rc = vreg_set_level(vreg_ldo20, 2600);
		if (rc) {
			pr_err("%s: vreg LDO20 set level failed (%d)\n",
				__func__, rc);
			return -1;
		}
		ret = platform_device_register(&sonywvga_panel);
	}
	else if (is_hitachi_panel()){
		rc = vreg_set_level(vreg_ldo20, 2850);
		if (rc) {
			pr_err("%s: vreg LDO20 set level failed (%d)\n",
				__func__, rc);
			return -1;
		}
		msm_device_mdp.dev.platform_data = &mdp_pdata_hitachi;
		rc = platform_device_register(&msm_device_mdp);
		if (rc)
			return rc;

		mddi_pdata.clk_rate = 384000000;
		mddi_pdata.type = MSM_MDP_MDDI_TYPE_II;
		msm_device_mddi0.dev.platform_data = &mddi_pdata;
		rc = platform_device_register(&msm_device_mddi0);
		if (rc)
			return rc;
		rc = platform_driver_register(&saga_backlight_driver);
		if (rc)
			return rc;
	}

	return ret;
}

