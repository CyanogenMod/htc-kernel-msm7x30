/* linux/arch/arm/mach-msm/panel-sonywvga-s6d16a0x21-7x30.c
 *
 * Copyright (c) 2009 Google Inc.
 * Copyright (c) 2009 HTC.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <linux/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_panel.h>
#include <mach/panel_id.h>
#include <mach/vreg.h>
#include <linux/spi/spi.h>
#include <mach/atmega_microp.h>
#include "devices.h"

//#define DEBUG_LCM
#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)	printk(fmt, ## arg)
#else
#define LCMDBG(fmt, arg...)	{}
#endif

#define SAMSUNG_PANEL		0
/*Bitwise mask for SONY PANEL ONLY*/
#define SONY_PANEL		0x1		/*Set bit 0 as 1 when it is SONY PANEL*/
#define SONY_PWM_SPI		0x2		/*Set bit 1 as 1 as PWM_SPI mode, otherwise it is PWM_MICROP mode*/
#define SONY_GAMMA		0x4		/*Set bit 2 as 1 when panel contains GAMMA table in its NVM*/
#define SONY_RGB666		0x8		/*Set bit 3 as 1 when panel is 18 bit, otherwise it is 16 bit*/

extern int panel_type;
unsigned int g_unblank_stage = 0;

#define SONYWVGA_MIN_VAL		10
#define SONYWVGA_MAX_VAL		250
#define SONYWVGA_DEFAULT_VAL	(SONYWVGA_MIN_VAL +		\
					 (SONYWVGA_MAX_VAL -	\
					  SONYWVGA_MIN_VAL) / 2)

#define SONYWVGA_BR_DEF_USER_PWM         143
#define SONYWVGA_BR_MIN_USER_PWM         30
#define SONYWVGA_BR_MAX_USER_PWM         255
#define SONYWVGA_BR_DEF_PANEL_PWM        128
#define SONYWVGA_BR_MIN_PANEL_PWM        8
#define SONYWVGA_BR_MAX_PANEL_PWM        255
#define SONYWVGA_BR_DEF_PANEL_UP_PWM    132
#define SONYWVGA_BR_MIN_PANEL_UP_PWM    9
#define SONYWVGA_BR_MAX_PANEL_UP_PWM    255

static DEFINE_MUTEX(panel_lock);
static uint8_t last_val_pwm = SONYWVGA_BR_DEF_PANEL_PWM;
static int (*sonywvga_power)(int on);
static int (*gpio_switch)(int on);
static struct wake_lock panel_idle_lock;

inline int is_sony_spi(void)
{
	if( panel_type == PANEL_ID_SAG_SONY )
		return ( (panel_type & BL_MASK) == BL_SPI ? 1 : 0 );
	else
		return ( panel_type & SONY_PWM_SPI ? 1 : 0 );
}

inline int is_sony_with_gamma(void)
{
	if(panel_type == PANEL_ID_SAG_SONY)
		return 1;
	else
		return (panel_type & SONY_GAMMA ? 1 : 0);
}

inline int is_sony_RGB666(void)
{
	if(panel_type == PANEL_ID_SAG_SONY)
		return ((panel_type & DEPTH_MASK) == DEPTH_RGB666 ? 1 : 0);
	else
		return (panel_type & SONY_RGB666 ? 1 : 0);
}


static const char *PanelVendor = "sony";
static const char *PanelNAME = "s6d16a0x21";
static const char *PanelSize = "wvga";

static ssize_t panel_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", PanelVendor, PanelNAME, PanelSize);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(panel, 0444, panel_vendor_show, NULL);
static struct kobject *android_display;

static int display_sysfs_init(void)
{
	int ret ;
	printk(KERN_INFO "display_sysfs_init : kobject_create_and_add\n");
	android_display = kobject_create_and_add("android_display", NULL);
	if (android_display == NULL) {
		printk(KERN_INFO "display_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	printk(KERN_INFO "display_sysfs_init : sysfs_create_file\n");
	ret = sysfs_create_file(android_display, &dev_attr_panel.attr);
	if (ret) {
		printk(KERN_INFO "display_sysfs_init : sysfs_create_file " \
		"failed\n");
		kobject_del(android_display);
	}

	return 0 ;
}

static int
sonywvga_panel_shrink_pwm(int brightness)
{
	int level;
	unsigned int min_pwm, def_pwm, max_pwm;

	if(!is_sony_spi()) {
		min_pwm = SONYWVGA_BR_MIN_PANEL_UP_PWM;
		def_pwm = SONYWVGA_BR_DEF_PANEL_UP_PWM;
		max_pwm = SONYWVGA_BR_MAX_PANEL_UP_PWM;
	} else {
		min_pwm = SONYWVGA_BR_MIN_PANEL_PWM;
		def_pwm = SONYWVGA_BR_DEF_PANEL_PWM;
		max_pwm = SONYWVGA_BR_MAX_PANEL_PWM;
	}

	if (brightness <= SONYWVGA_BR_DEF_USER_PWM) {
		if (brightness <= SONYWVGA_BR_MIN_USER_PWM)
			level = min_pwm;
		else
			level = (def_pwm - min_pwm) *
				(brightness - SONYWVGA_BR_MIN_USER_PWM) /
				(SONYWVGA_BR_DEF_USER_PWM - SONYWVGA_BR_MIN_USER_PWM) +
				min_pwm;
	} else
		level = (max_pwm - def_pwm) *
		(brightness - SONYWVGA_BR_DEF_USER_PWM) /
		(SONYWVGA_BR_MAX_USER_PWM - SONYWVGA_BR_DEF_USER_PWM) +
		def_pwm;

	return level;
}

static int sonywvga_panel_power(int on)
{
	int ret = -EIO;

	if (sonywvga_power) {
		ret = (*sonywvga_power)(on);
		if (ret)
			goto power_fail;
	}

	if (gpio_switch) {
		ret = (*gpio_switch)(on);
		if (ret)
			goto power_fail;
	}
	return 0;

power_fail:
	return ret;
}

extern int qspi_send_9bit(struct spi_msg *msg);

#define LCM_CMD(_cmd, ...)					\
{                                                               \
        .cmd = _cmd,                                            \
        .data = (u8 []){__VA_ARGS__},                           \
        .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}
//2010-5-21 Rev May21-2(Wx, Wy)=(0.306, 0.315) Gamma = 2.2
static struct spi_msg SONY_TFT_INIT_TABLE[] = {
        //Change to level 2
	LCM_CMD(0xF1, 0x5A, 0x5A),
	LCM_CMD(0xFA, 0x32, 0x3F, 0x3F, 0x29, 0x3E, 0x3C, 0x3D, 0x2C,
		0x27, 0x3D, 0x2E, 0x31, 0x3A, 0x34, 0x36, 0x1A, 0x3F, 0x3F, 0x2E,
		0x40, 0x3C, 0x3C, 0x2B, 0x25, 0x39, 0x25, 0x23, 0x2A, 0x20, 0x22,
		0x00, 0x3F, 0x3F, 0x2F, 0x3E, 0x3C, 0x3C, 0x2A, 0x23, 0x35, 0x1E,
		0x18, 0x1C, 0x0C, 0x0E),
	LCM_CMD(0xFB, 0x00, 0x0D, 0x09, 0x0C, 0x26, 0x2E, 0x31, 0x22,
		0x19, 0x33, 0x22, 0x23, 0x21, 0x17, 0x00, 0x00, 0x25, 0x1D, 0x1F,
		0x35, 0x3C, 0x3A, 0x26, 0x1B, 0x34, 0x23, 0x23, 0x1F, 0x12, 0x00,
		0x00, 0x3F, 0x31, 0x33, 0x43, 0x48, 0x41, 0x2A, 0x1D, 0x35, 0x23,
		0x23, 0x21, 0x10, 0x00),
        // F3h Power control
	LCM_CMD(0xF3, 0x00, 0x10, 0x25, 0x01, 0x2D, 0x2D, 0x24, 0x2D, 0x10,
		0x10, 0x0A, 0x37),
        // F4h VCOM Control
	LCM_CMD(0xF4, 0x88, 0x20, 0x00, 0xAF, 0x64, 0x00, 0xAA, 0x64, 0x00, 0x00),
        //Change to level 1
	LCM_CMD(0xF0, 0x5A, 0x5A),
};


static struct spi_msg SONY_GAMMA_UPDATE_TABLE[] = {
	LCM_CMD(0x53, 0x24),
	LCM_CMD(0xF0, 0x5A, 0x5A),
	LCM_CMD(0xF1, 0x5A, 0x5A),
	LCM_CMD(0xD0, 0x5A, 0x5A),
	LCM_CMD(0xC2, 0x53, 0x12),
};
static struct spi_msg SAG_SONY_GAMMA_UPDATE_TABLE[] = {
	LCM_CMD(0x53, 0x24),
	LCM_CMD(0xF0, 0x5A, 0x5A),
	LCM_CMD(0xF1, 0x5A, 0x5A),
	LCM_CMD(0xD0, 0x5A, 0x5A),
	LCM_CMD(0xC2, 0x36, 0x12),//Change PWM to 13k for HW's request
};



static int lcm_write_tb(struct spi_msg cmd_table[], unsigned size)
{
        int i;

        for (i = 0; i < size; i++) {
		qspi_send_9bit(&cmd_table[i]);
        }
        return 0;
}

static char shrink_pwm = 0x00;
static struct spi_msg gamma_update = {
		.cmd = 0x51,
		.len = 1,
		.data = &shrink_pwm,
};

static struct spi_msg unblank_msg = {
		.cmd = 0x29,
		.len = 0,
};

static struct spi_msg blank_msg= {
		.cmd = 0x28,
		.len = 0,
};

static struct spi_msg init_cmd = {
		.cmd = 0x11,
		.len = 0,
};

static char init_data = 0x05;
static struct spi_msg init_cmd2 = {
		.cmd = 0x3A,
		.len = 1,
		.data = &init_data,
};

/*
 * Caller must make sure the spi is ready
 * */
static void sonywvga_set_gamma_val(int val)
{
	uint8_t data[4] = {0, 0, 0, 0};

	if (!is_sony_spi()) {
		//turn on backlight
		data[0] = 5;
		data[1] = sonywvga_panel_shrink_pwm(val);
		data[3] = 1;
		microp_i2c_write(0x25, data, 4);
	} else {
		shrink_pwm = sonywvga_panel_shrink_pwm(val);
	        qspi_send_9bit(&gamma_update);
		if( panel_type == PANEL_ID_SAG_SONY )
			lcm_write_tb(SAG_SONY_GAMMA_UPDATE_TABLE,  ARRAY_SIZE(SAG_SONY_GAMMA_UPDATE_TABLE));
		else
			lcm_write_tb(SONY_GAMMA_UPDATE_TABLE,  ARRAY_SIZE(SONY_GAMMA_UPDATE_TABLE));
	}
	last_val_pwm = val;
}

static int sonywvga_panel_unblank(struct msm_lcdc_panel_ops *panel_data)
{
	LCMDBG("%s\n", __func__);

        wake_lock(&panel_idle_lock);
        mutex_lock(&panel_lock);

	hr_msleep(100);
	qspi_send_9bit(&unblank_msg);
	hr_msleep(20);
	//init gamma setting
	if(!is_sony_with_gamma())
		lcm_write_tb(SONY_TFT_INIT_TABLE,
			ARRAY_SIZE(SONY_TFT_INIT_TABLE));

	sonywvga_set_gamma_val(last_val_pwm);

	g_unblank_stage = 1;

        mutex_unlock(&panel_lock);
        wake_unlock(&panel_idle_lock);

	return 0;
}

static int sonywvga_panel_blank(struct msm_lcdc_panel_ops *panel_data)
{
	uint8_t data[4] = {0, 0, 0, 0};
	LCMDBG("%s\n", __func__);

	mutex_lock(&panel_lock);

	blank_msg.cmd = 0x28;
	qspi_send_9bit(&blank_msg);
	blank_msg.cmd = 0x10;
	qspi_send_9bit(&blank_msg);
	hr_msleep(40);
	g_unblank_stage = 0;
	mutex_unlock(&panel_lock);
	sonywvga_panel_power(0);

	if (!is_sony_spi()) {
		data[0] = 5;
		data[1] = 0;
		data[3] = 1;
		microp_i2c_write(0x25, data, 4);
	}

	return 0;
}

static int sonywvga_panel_init(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("%s()\n", __func__);

	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	sonywvga_panel_power(1);
	hr_msleep(45);
	qspi_send_9bit(&init_cmd);
	hr_msleep(5);
	if (is_sony_RGB666()) {
		init_data = 0x06;
		qspi_send_9bit(&init_cmd2);
	} else {
		init_data = 0x05;
		qspi_send_9bit(&init_cmd2);
	}
	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);

	return 0;
}

static struct msm_lcdc_panel_ops sonywvga_lcdc_panel_ops = {
	.init = sonywvga_panel_init,
	.blank = sonywvga_panel_blank,
	.unblank = sonywvga_panel_unblank,
};

static struct msm_lcdc_timing sonywvga_lcdc_timing = {
	.clk_rate		= 24576000,	//In Bravoc, Radio will adjust clk rate from 24.57Mhz to 23.59Mhz to pass EMC testing
	.hsync_pulse_width	= 2,
	.hsync_back_porch	= 18,
	.hsync_front_porch	= 20,
	.hsync_skew		= 0,
	.vsync_pulse_width	= 2,
	.vsync_back_porch	= 5,
	.vsync_front_porch	= 4,
	.vsync_act_low		= 1,
	.hsync_act_low		= 1,
	.den_act_low		= 0,
};

static struct msm_fb_data sonywvga_lcdc_fb_data = {
	.xres		= 480,
	.yres		= 800,
	.width		= 48,
	.height		= 80,
	.output_format	= 0,
};

static struct msm_lcdc_platform_data sonywvga_lcdc_platform_data = {
	.panel_ops	= &sonywvga_lcdc_panel_ops,
	.timing		= &sonywvga_lcdc_timing,
	.fb_id		= 0,
	.fb_data	= &sonywvga_lcdc_fb_data,
};

static struct platform_device sonywvga_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &sonywvga_lcdc_platform_data,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
        .color_format = MSM_MDP_OUT_IF_FMT_RGB666,
};

void sonywvga_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness val)
{
	led_cdev->brightness = val;

	mutex_lock(&panel_lock);
	if(g_unblank_stage)
		sonywvga_set_gamma_val(val);
	else
		last_val_pwm =val;
	mutex_unlock(&panel_lock);
}

static struct led_classdev sonywvga_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = sonywvga_brightness_set,
};

static int sonywvga_backlight_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &sonywvga_backlight_led);
	if (rc)
		LCMDBG("backlight: failure on register led_classdev\n");
	return 0;
}

static struct platform_device sonywvga_backlight = {
	.name = "sonywvga-backlight",
};

static struct platform_driver sonywvga_backlight_driver = {
	.probe		= sonywvga_backlight_probe,
	.driver		= {
		.name	= "sonywvga-backlight",
		.owner	= THIS_MODULE,
	},
};

static int __init sonywvga_init_panel(void)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);

	if(is_sony_RGB666())
		msm_device_mdp.dev.platform_data = &mdp_pdata;

	ret = platform_device_register(&msm_device_mdp);
	if (ret)
		return ret;

	/* set gpio to proper state in the beginning */
	if (gpio_switch)
		(*gpio_switch)(1);

	ret = platform_device_register(&sonywvga_lcdc_device);
	if (ret)
		return ret;

	wake_lock_init(&panel_idle_lock, WAKE_LOCK_SUSPEND,
			"backlight_present");

	ret = platform_device_register(&sonywvga_backlight);
	if (ret)
		return ret;

	return 0;
}

static int sonywvga_probe(struct platform_device *pdev)
{
	int rc = -EIO;
	struct panel_platform_data *pdata;
	pdata = pdev->dev.platform_data;

	sonywvga_power = pdata->power;
	gpio_switch = pdata->gpio_switch;
	sonywvga_lcdc_platform_data.fb_resource = pdata->fb_res;

	if(!is_sony_spi())
		last_val_pwm = SONYWVGA_DEFAULT_VAL;
	else
		last_val_pwm = SONYWVGA_BR_DEF_PANEL_PWM;

	rc = sonywvga_init_panel();
	if (rc)
		printk(KERN_ERR "%s fail %d\n", __func__, rc);

	display_sysfs_init();
	return rc;
}

static struct platform_driver sonywvga_driver = {
	.probe = sonywvga_probe,
	.driver = { .name = "panel-sonywvga-s6d16a0x21-7x30" },
};

static int __init sonywvga_7x30_init(void)
{
	return platform_driver_register(&sonywvga_driver);
}

static int __init sonywvga_backlight_7x30_init(void)
{
	return platform_driver_register(&sonywvga_backlight_driver);
}

device_initcall(sonywvga_7x30_init);
module_init(sonywvga_backlight_7x30_init);
