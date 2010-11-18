/* linux/arch/arm/mach-msm/panel-samsung-s6e63m0.c
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
#include <linux/spi/spi.h>

#include "devices.h"

#define DEBUG_LCM
#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)	printk(fmt, ## arg)
#else
#define LCMDBG(fmt, arg...)	{}
#endif


#define LCM_CMD_SEQ	(struct lcm_cmd[])
#define LCM_REG_END	(-1)
#define LCM_CMD_END	{-1, 0, 0}
struct lcm_cmdII {
        u8 cmd;
        u8 *vals;
        unsigned len;
};

#define LCM_CMD(_cmd, ...)					\
{                                                               \
        .cmd = _cmd,                                            \
        .data = (u8 []){__VA_ARGS__},                           \
        .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}

static struct spi_msg lcm_init_seq[] = {
	LCM_CMD(0xF8, 0x01, 0x27, 0x27, 0x07, 0x07, 0x54, 0x9F, 0x63, 0x86,
                0x1A, 0x33, 0x0D, 0x00, 0x00),
        LCM_CMD(0xF2, 0x02, 0x03, 0x1C, 0x10, 0x10),
        LCM_CMD(0xF7, 0x00, 0x03, 0x01),
        LCM_CMD(0xF6, 0x00, 0x8C, 0x07),
        LCM_CMD(0xB3, 0x0C),
        LCM_CMD(0xB5, 0x2C, 0x12, 0x0C, 0x0A, 0x10, 0x0E, 0x17, 0x13, 0x1F,
                0x1A, 0x2A, 0x24, 0x1F, 0x1B, 0x1A, 0x17, 0x2B, 0x26, 0x22,
                0x20, 0x3A, 0x34, 0x30, 0x2C, 0x29, 0x26, 0x25, 0x23, 0x21,
                0x20, 0x1E, 0x1E),
        LCM_CMD(0xB7, 0x2C, 0x12, 0x0C, 0x0A, 0x10, 0x0E, 0x17, 0x13, 0x1F,
                0x1A, 0x2A, 0x24, 0x1F, 0x1B, 0x1A, 0x17, 0x2B, 0x26, 0x22,
                0x20, 0x3A, 0x34, 0x30, 0x2C, 0x29, 0x26, 0x25, 0x23, 0x21,
                0x20, 0x1E, 0x1E),
        LCM_CMD(0xB6, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44, 0x44, 0x44, 0x55,
                0x55, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66),
        LCM_CMD(0xB8, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44, 0x44, 0x44, 0x55,
                0x55, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66),
        LCM_CMD(0xB9, 0x2C, 0x12, 0x0C, 0x0A, 0x10, 0x0E, 0x17, 0x13, 0x1F,
                0x1A, 0x2A, 0x24, 0x1F, 0x1B, 0x1A, 0x17, 0x2B, 0x26, 0x22,
                0x20, 0x3A, 0x34, 0x30, 0x2C, 0x29, 0x26, 0x25, 0x23, 0x21,
                0x20, 0x1E, 0x1E),
        LCM_CMD(0xBA, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44, 0x44, 0x44, 0x55,
                0x55, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66),

};

static struct spi_msg samsung_oled_gamma_table[] = {
        /*level 20*/
        LCM_CMD(0xFA, 0x02, 0x18, 0x08, 0x24, 0xB4, 0x66, 0x95, 0xD3, 0xCC, 0xC6,
        0xDE, 0xD1, 0xC4, 0xDD, 0xDE, 0xD5, 0x00, 0x30, 0x00, 0x2F, 0x00, 0x41),
        /*level 100*/
        LCM_CMD(0xFA, 0x02, 0x18, 0x08, 0x24, 0x6D, 0x4A, 0x3C, 0xBF, 0xBF, 0xB0,
        0xB7, 0xBB, 0xAB, 0xCC, 0xCE, 0xC3, 0x00, 0x69, 0x00, 0x68, 0x00, 0x89),
        /*level 130*/
        LCM_CMD(0xFA, 0x02, 0x18, 0x08, 0x24, 0x6A, 0x51, 0x3C, 0xBB, 0xBC, 0xAC,
        0xB6, 0xBA, 0xAA, 0xCA, 0xCC, 0xC1, 0x00, 0x74, 0x00, 0x73, 0x00, 0x97),
        /*level 160*/
        LCM_CMD(0xFA, 0x02, 0x18, 0x08, 0x24, 0x6E, 0x53, 0x3E, 0xB9, 0xBC, 0xAB,
        0xB5, 0xB8, 0xA8, 0xC7, 0xCA, 0xBF, 0x00, 0x7F, 0x00, 0x7E, 0x00, 0xA5),
        /*level 190*/
        LCM_CMD(0xFA, 0x02, 0x18, 0x08, 0x24, 0x6A, 0x58, 0x3D, 0xB8, 0xBA, 0xAA,
        0xB0, 0xB5, 0xA4, 0xC6, 0xC8, 0xBD, 0x00, 0x89, 0x00, 0x88, 0x00, 0xB2),
        /*level 220*/
        LCM_CMD(0xFA, 0x02, 0x18, 0x08, 0x24, 0x67, 0x59, 0x3B, 0xB7, 0xB9, 0xA8,
        0xB0, 0xB4, 0xA4, 0xC3, 0xC6, 0xBA, 0x00, 0x92, 0x00, 0x91, 0x00, 0xBE),
       /*level 250*/
        LCM_CMD(0xFA, 0x02, 0x18, 0x08, 0x24, 0x64, 0x56, 0x37, 0xB5, 0xB8, 0xA7,
        0xAE, 0xB2, 0xA1, 0xC2, 0xC4, 0xBA, 0x00, 0x9A, 0x00, 0x9A, 0x00, 0xC8),
};


#define SAMSUNG_OLED_NUM_LEVELS 7

#define SAMSUNG_OLED_MIN_VAL		10
#define SAMSUNG_OLED_MAX_VAL		250
#define SAMSUNG_OLED_DEFAULT_LEVEL	3

#define SAMSUNG_OLED_LEVEL_STEP		((SAMSUNG_OLED_MAX_VAL -	\
					  SAMSUNG_OLED_MIN_VAL) /	\
					 (SAMSUNG_OLED_NUM_LEVELS - 1))

#define SPI_CONFIG              (0x00000000)
#define SPI_IO_CONTROL          (0x00000004)
#define SPI_OPERATIONAL         (0x00000030)
#define SPI_ERROR_FLAGS_EN      (0x00000038)
#define SPI_ERROR_FLAGS         (0x00000034)
#define SPI_OUTPUT_FIFO         (0x00000100)
extern int panel_type;
extern int qspi_send(uint32_t id, uint8_t data);
extern int qspi_send_9bit(struct spi_msg *msg);


static int lcm_write_cmd(struct spi_msg cmd)
{
        int ret = -1;
	ret = qspi_send_9bit(&cmd);
        return 0;
}

static int lcm_write_seq(struct spi_msg cmd_table[], unsigned size)
{
        int i;

        for (i = 0; i < size; i++) {
                lcm_write_cmd(cmd_table[i]);
        }
        return 0;
}


/* ---------------------------------------------------- */

static DEFINE_MUTEX(panel_lock);
static uint8_t last_level = SAMSUNG_OLED_DEFAULT_LEVEL;
static int (*amoled_power)(int on);
static int (*gpio_switch)(int on);
static struct wake_lock panel_idle_lock;
static struct led_trigger *amoled_lcd_backlight;
static int panel_id = 0;
static char update = 0x03;
static struct spi_msg gamma_update = {
		.cmd = 0xFA,
		.len = 1,
		.data = &update,
};

static struct spi_msg unblank_msg = {
		.cmd = 0x29,
		.len = 0,
};

static struct spi_msg blank_cmd = {
		.cmd = 0x10,
		.len = 0,
};

static struct spi_msg init_cmd = {
		.cmd = 0x11,
		.len = 0,
};


/*
 * Caller must make sure the spi is ready
 * */
static void amoled_set_gamma_val(int level)
{
	lcm_write_cmd(samsung_oled_gamma_table[level]);
	qspi_send_9bit(&gamma_update);

	last_level = level;
}

static int amoled_panel_unblank(struct msm_lcdc_panel_ops *panel_data)
{
	LCMDBG("%s\n", __func__);

	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	amoled_set_gamma_val(last_level);
	/* Display on */
	qspi_send_9bit(&unblank_msg);
	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);

	LCMDBG("%s: last_level = %d\n", __func__, last_level);
	led_trigger_event(amoled_lcd_backlight, LED_FULL);
	return 0;
}

static int amoled_panel_power(int on)
{
	int ret = -EIO;

	if (amoled_power) {
		ret = (*amoled_power)(on);
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

static int amoled_panel_blank(struct msm_lcdc_panel_ops *panel_data)
{
	LCMDBG("%s\n", __func__);
	mutex_lock(&panel_lock);
	qspi_send_9bit(&blank_cmd);
	hr_msleep(120);
	mutex_unlock(&panel_lock);
	amoled_panel_power(0);
	led_trigger_event(amoled_lcd_backlight, LED_OFF);
	return 0;
}

static int samsung_oled_panel_init(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("%s()\n", __func__);

	amoled_panel_power(1);

	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	lcm_write_seq(lcm_init_seq, ARRAY_SIZE(lcm_init_seq));
	/* standby off */
	qspi_send_9bit(&init_cmd);
	hr_msleep(120);
	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);
	return 0;
}

static struct msm_lcdc_panel_ops amoled_lcdc_panel_ops = {
	.init = samsung_oled_panel_init,
	.blank = amoled_panel_blank,
	.unblank = amoled_panel_unblank,
};

static struct msm_lcdc_timing amoled_lcdc_timing = {
	.clk_rate		= 24576000,
	.hsync_skew		= 0,
	.vsync_act_low		= 1,
	.hsync_act_low		= 1,
	.den_act_low		= 1,
};

static struct msm_fb_data amoled_lcdc_fb_data = {
	.xres		= 480,
	.yres		= 800,
	.width		= 48,
	.height		= 80,
	.output_format	= 0,
};

static struct msm_lcdc_platform_data amoled_lcdc_platform_data = {
	.panel_ops	= &amoled_lcdc_panel_ops,
	.timing		= &amoled_lcdc_timing,
	.fb_id		= 0,
	.fb_data	= &amoled_lcdc_fb_data,
};

static struct platform_device amoled_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &amoled_lcdc_platform_data,
	},
};

void samsung_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness val)
{
	uint8_t new_level;
	if(val < 30)
		new_level = 0;
	else if (val <= LED_FULL / (SAMSUNG_OLED_NUM_LEVELS - 1))
		new_level = 1;
	else
		new_level = (val * (SAMSUNG_OLED_NUM_LEVELS - 1)) / LED_FULL;

	if(last_level == new_level)
		return;
	else {
		LCMDBG("set brightness = (%d, %d)\n", new_level, val);
		mutex_lock(&panel_lock);
		amoled_set_gamma_val(new_level);
		mutex_unlock(&panel_lock);
	}
}

static int amoled_panel_detect(void)
{
	return panel_type;
}

static struct led_classdev amoled_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
};

static int amoled_backlight_probe(struct platform_device *pdev)
{
	int rc;
	amoled_backlight_led.brightness_set = samsung_brightness_set;
	rc = led_classdev_register(&pdev->dev, &amoled_backlight_led);
	if (rc)
		LCMDBG("backlight: failure on register led_classdev\n");
	return 0;
}

static struct platform_device amoled_backlight = {
	.name = "panel-s6e63m0-backlight",
};

static struct platform_driver amoled_backlight_driver = {
	.probe		= amoled_backlight_probe,
	.driver		= {
		.name	= "panel-s6e63m0-backlight",
		.owner	= THIS_MODULE,
	},
};

static const char *PanelVendor = "samsung";
static const char *PanelNAME = "s6e63m0";
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

static int __init amoled_init_panel(void)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);

	ret = platform_device_register(&msm_device_mdp);
	if (ret)
		return ret;

#if 0   /* uses spi driver temporarily */
	ret = init_spi_hack();
	if (ret != 0)
		return ret;
#endif
	/* set gpio to proper state in the beginning */
	if (gpio_switch)
		(*gpio_switch)(1);

	ret = platform_device_register(&amoled_lcdc_device);
	if (ret)
		return ret;

	wake_lock_init(&panel_idle_lock, WAKE_LOCK_SUSPEND,
			"backlight_present");

	ret = platform_device_register(&amoled_backlight);
	if (ret)
		return ret;

	led_trigger_register_simple("lcd-backlight-gate",
			&amoled_lcd_backlight);
	return 0;
}

static int __init amoled_probe(struct platform_device *pdev)
{
	int rc = -EIO;
	struct panel_platform_data *pdata;
	pdata = pdev->dev.platform_data;

	amoled_power = pdata->power;
	gpio_switch = pdata->gpio_switch;
	amoled_lcdc_platform_data.fb_resource = pdata->fb_res;
	panel_id = amoled_panel_detect();

	amoled_lcdc_timing.hsync_pulse_width      = 2;
	amoled_lcdc_timing.hsync_back_porch       = 14;
	amoled_lcdc_timing.hsync_front_porch     = 16;
	amoled_lcdc_timing.vsync_pulse_width      = 2;
	amoled_lcdc_timing.vsync_back_porch       = 1;
	amoled_lcdc_timing.vsync_front_porch      = 28;
	printk("Panel type = %d\n", amoled_panel_detect());

	rc = amoled_init_panel();
	if (rc)
		printk(KERN_ERR "%s fail %d\n", __func__, rc);

	display_sysfs_init();
	return rc;
}

static struct platform_driver amoled_driver = {
	.probe = amoled_probe,
	.driver = { .name = "panel-s6e63m0" },
};

static int __init amoled_init(void)
{
	return platform_driver_register(&amoled_driver);
}

static int __init amoled_backlight_init(void)
{
	return platform_driver_register(&amoled_backlight_driver);
}

device_initcall(amoled_init);
module_init(amoled_backlight_init);
