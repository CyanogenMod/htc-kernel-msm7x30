/* linux/arch/arm/mach-msm/board-spade-panel.c
 *
 * Copyright (c) 2010 HTC.
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

#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <mach/vreg.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/atmega_microp.h>

#include "board-spade.h"
#include "devices.h"
#include "proc_comm.h"

#define DEBUG_LCM

#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)     printk("[lcm]%s"fmt, __func__, ##arg)
#else
#define LCMDBG(fmt, arg...)     {}
#endif

#define BRIGHTNESS_DEFAULT_LEVEL        102

enum {
	PANEL_AUO,
	PANEL_SHARP,
	PANEL_UNKNOW
};

int qspi_send_16bit(unsigned char id, unsigned data);
int qspi_send_9bit(struct spi_msg *msg);

static int spade_adjust_backlight(enum led_brightness val);

extern int panel_type;
static DEFINE_MUTEX(panel_lock);
static struct vreg *vreg_lcm_1v8, *vreg_lcm_2v8;

static atomic_t lcm_init_done = ATOMIC_INIT(1);
static uint8_t last_val = BRIGHTNESS_DEFAULT_LEVEL;
static bool screen_on = true;

#define LCM_GPIO_CFG(gpio, func) \
	PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)

static uint32_t display_on_gpio_table[] = {
	LCM_GPIO_CFG(SPADE_LCD_PCLK, 1),
	LCM_GPIO_CFG(SPADE_LCD_DE, 1),
	LCM_GPIO_CFG(SPADE_LCD_VSYNC, 1),
	LCM_GPIO_CFG(SPADE_LCD_HSYNC, 1),

	LCM_GPIO_CFG(SPADE_LCD_R0, 1),
	LCM_GPIO_CFG(SPADE_LCD_R1, 1),
	LCM_GPIO_CFG(SPADE_LCD_R2, 1),
	LCM_GPIO_CFG(SPADE_LCD_R3, 1),
	LCM_GPIO_CFG(SPADE_LCD_R4, 1),
	LCM_GPIO_CFG(SPADE_LCD_R5, 1),
	LCM_GPIO_CFG(SPADE_LCD_R6, 1),
	LCM_GPIO_CFG(SPADE_LCD_R7, 1),

	LCM_GPIO_CFG(SPADE_LCD_G0, 1),
	LCM_GPIO_CFG(SPADE_LCD_G1, 1),
	LCM_GPIO_CFG(SPADE_LCD_G2, 1),
	LCM_GPIO_CFG(SPADE_LCD_G3, 1),
	LCM_GPIO_CFG(SPADE_LCD_G4, 1),
	LCM_GPIO_CFG(SPADE_LCD_G5, 1),
	LCM_GPIO_CFG(SPADE_LCD_G6, 1),
	LCM_GPIO_CFG(SPADE_LCD_G7, 1),

	LCM_GPIO_CFG(SPADE_LCD_B0, 1),
	LCM_GPIO_CFG(SPADE_LCD_B1, 1),
	LCM_GPIO_CFG(SPADE_LCD_B2, 1),
	LCM_GPIO_CFG(SPADE_LCD_B3, 1),
	LCM_GPIO_CFG(SPADE_LCD_B4, 1),
	LCM_GPIO_CFG(SPADE_LCD_B5, 1),
	LCM_GPIO_CFG(SPADE_LCD_B6, 1),
	LCM_GPIO_CFG(SPADE_LCD_B7, 1),
	//LCM_GPIO_CFG(SPADE_LCD_RSTz, 1),
};

static uint32_t display_off_gpio_table[] = {
	LCM_GPIO_CFG(SPADE_LCD_PCLK, 0),
	LCM_GPIO_CFG(SPADE_LCD_DE, 0),
	LCM_GPIO_CFG(SPADE_LCD_VSYNC, 0),
	LCM_GPIO_CFG(SPADE_LCD_HSYNC, 0),

	LCM_GPIO_CFG(SPADE_LCD_R0, 0),
	LCM_GPIO_CFG(SPADE_LCD_R1, 0),
	LCM_GPIO_CFG(SPADE_LCD_R2, 0),
	LCM_GPIO_CFG(SPADE_LCD_R3, 0),
	LCM_GPIO_CFG(SPADE_LCD_R4, 0),
	LCM_GPIO_CFG(SPADE_LCD_R5, 0),
	LCM_GPIO_CFG(SPADE_LCD_R6, 0),
	LCM_GPIO_CFG(SPADE_LCD_R7, 0),

	LCM_GPIO_CFG(SPADE_LCD_G0, 0),
	LCM_GPIO_CFG(SPADE_LCD_G1, 0),
	LCM_GPIO_CFG(SPADE_LCD_G2, 0),
	LCM_GPIO_CFG(SPADE_LCD_G3, 0),
	LCM_GPIO_CFG(SPADE_LCD_G4, 0),
	LCM_GPIO_CFG(SPADE_LCD_G5, 0),
	LCM_GPIO_CFG(SPADE_LCD_G6, 0),
	LCM_GPIO_CFG(SPADE_LCD_G7, 0),

	LCM_GPIO_CFG(SPADE_LCD_B0, 0),
	LCM_GPIO_CFG(SPADE_LCD_B1, 0),
	LCM_GPIO_CFG(SPADE_LCD_B2, 0),
	LCM_GPIO_CFG(SPADE_LCD_B3, 0),
	LCM_GPIO_CFG(SPADE_LCD_B4, 0),
	LCM_GPIO_CFG(SPADE_LCD_B5, 0),
	LCM_GPIO_CFG(SPADE_LCD_B6, 0),
	LCM_GPIO_CFG(SPADE_LCD_B7, 0),
};

static void spade_sharp_panel_power(bool on_off)
{
	if (!!on_off) {
		LCMDBG("(%d):\n", on_off);
		screen_on = true;
		config_gpio_table( display_on_gpio_table,
			ARRAY_SIZE(display_on_gpio_table));
		gpio_set_value(SPADE_LCD_RSTz, 0);
		vreg_enable(vreg_lcm_2v8);
		vreg_enable(vreg_lcm_1v8);
		udelay(10);
		gpio_set_value(SPADE_LCD_RSTz, 1);
		hr_msleep(20);
	} else {
		LCMDBG("(%d):\n", on_off);
		gpio_set_value(SPADE_LCD_RSTz, 0);
		hr_msleep(70);
		vreg_disable(vreg_lcm_2v8);
		vreg_disable(vreg_lcm_1v8);
		config_gpio_table(display_off_gpio_table,
			ARRAY_SIZE(display_off_gpio_table));
	}
}

static void spade_auo_panel_power(bool on_off)
{
	if (!!on_off) {
		LCMDBG("%s(%d):\n", __func__, on_off);
		gpio_set_value(SPADE_LCD_RSTz, 1);
		udelay(500);
		gpio_set_value(SPADE_LCD_RSTz, 0);
		udelay(500);
		gpio_set_value(SPADE_LCD_RSTz, 1);
		hr_msleep(20);
		config_gpio_table( display_on_gpio_table,
				ARRAY_SIZE(display_on_gpio_table));
	} else {
		LCMDBG("%s(%d):\n", __func__, on_off);
		gpio_set_value(SPADE_LCD_RSTz, 1);
		hr_msleep(70);
		config_gpio_table( display_off_gpio_table,
				ARRAY_SIZE(display_off_gpio_table));
	}
}

struct lcm_cmd {
        uint8_t		cmd;
        uint8_t		data;
};

#define LCM_MDELAY	0x03

/*----------------------------------------------------------------------------*/

static struct lcm_cmd auo_init_seq[] = {
        {0x1, 0xc0}, {0x0, 0x00}, {0x2, 0x86},
        {0x1, 0xc0}, {0x0, 0x01}, {0x2, 0x00},
        {0x1, 0xc0}, {0x0, 0x02}, {0x2, 0x86},
        {0x1, 0xc0}, {0x0, 0x03}, {0x2, 0x00},
        {0x1, 0xc1}, {0x0, 0x00}, {0x2, 0x40},
        {0x1, 0xc2}, {0x0, 0x00}, {0x2, 0x02},
	{LCM_MDELAY, 1},
        {0x1, 0xc2}, {0x0, 0x02}, {0x2, 0x32},

        /* Gamma setting: start */
        {0x1, 0xe0}, {0x0, 0x00}, {0x2, 0x0e },
        {0x1, 0xe0}, {0x0, 0x01}, {0x2, 0x34},
        {0x1, 0xe0}, {0x0, 0x02}, {0x2, 0x3f},
        {0x1, 0xe0}, {0x0, 0x03}, {0x2, 0x49},
        {0x1, 0xe0}, {0x0, 0x04}, {0x2, 0x1d},
        {0x1, 0xe0}, {0x0, 0x05}, {0x2, 0x2c},
        {0x1, 0xe0}, {0x0, 0x06}, {0x2, 0x5f},
        {0x1, 0xe0}, {0x0, 0x07}, {0x2, 0x3a},
        {0x1, 0xe0}, {0x0, 0x08}, {0x2, 0x20},
        {0x1, 0xe0}, {0x0, 0x09}, {0x2, 0x28},
        {0x1, 0xe0}, {0x0, 0x0a}, {0x2, 0x80},
        {0x1, 0xe0}, {0x0, 0x0b}, {0x2, 0x13},
        {0x1, 0xe0}, {0x0, 0x0c}, {0x2, 0x32},
        {0x1, 0xe0}, {0x0, 0x0d}, {0x2, 0x56},
        {0x1, 0xe0}, {0x0, 0x0e}, {0x2, 0x79},
        {0x1, 0xe0}, {0x0, 0x0f}, {0x2, 0xb8},
        {0x1, 0xe0}, {0x0, 0x10}, {0x2, 0x55},
        {0x1, 0xe0}, {0x0, 0x11}, {0x2, 0x57},

        {0x1, 0xe1}, {0x0, 0x00}, {0x2, 0x0e},
        {0x1, 0xe1}, {0x0, 0x01}, {0x2, 0x34},
        {0x1, 0xe1}, {0x0, 0x02}, {0x2, 0x3f},
        {0x1, 0xe1}, {0x0, 0x03}, {0x2, 0x49},
        {0x1, 0xe1}, {0x0, 0x04}, {0x2, 0x1d},
        {0x1, 0xe1}, {0x0, 0x05}, {0x2, 0x2c},
        {0x1, 0xe1}, {0x0, 0x06}, {0x2, 0x5f},
        {0x1, 0xe1}, {0x0, 0x07}, {0x2, 0x3a},
        {0x1, 0xe1}, {0x0, 0x08}, {0x2, 0x20},
        {0x1, 0xe1}, {0x0, 0x09}, {0x2, 0x28},
        {0x1, 0xe1}, {0x0, 0x0a}, {0x2, 0x80},
        {0x1, 0xe1}, {0x0, 0x0b}, {0x2, 0x13},
        {0x1, 0xe1}, {0x0, 0x0c}, {0x2, 0x32},
        {0x1, 0xe1}, {0x0, 0x0d}, {0x2, 0x56},
        {0x1, 0xe1}, {0x0, 0x0e}, {0x2, 0x79},
        {0x1, 0xe1}, {0x0, 0x0f}, {0x2, 0xb8},
        {0x1, 0xe1}, {0x0, 0x10}, {0x2, 0x55},
        {0x1, 0xe1}, {0x0, 0x11}, {0x2, 0x57},

        {0x1, 0xe2}, {0x0, 0x00}, {0x2, 0x0e},
        {0x1, 0xe2}, {0x0, 0x01}, {0x2, 0x34},
        {0x1, 0xe2}, {0x0, 0x02}, {0x2, 0x3f},
        {0x1, 0xe2}, {0x0, 0x03}, {0x2, 0x49},
        {0x1, 0xe2}, {0x0, 0x04}, {0x2, 0x1d},
        {0x1, 0xe2}, {0x0, 0x05}, {0x2, 0x2c},
        {0x1, 0xe2}, {0x0, 0x06}, {0x2, 0x5d},
        {0x1, 0xe2}, {0x0, 0x07}, {0x2, 0x3a},
        {0x1, 0xe2}, {0x0, 0x08}, {0x2, 0x20},
        {0x1, 0xe2}, {0x0, 0x09}, {0x2, 0x28},
        {0x1, 0xe2}, {0x0, 0x0a}, {0x2, 0x80},
        {0x1, 0xe2}, {0x0, 0x0b}, {0x2, 0x13},
        {0x1, 0xe2}, {0x0, 0x0c}, {0x2, 0x32},
        {0x1, 0xe2}, {0x0, 0x0d}, {0x2, 0x56},
        {0x1, 0xe2}, {0x0, 0x0e}, {0x2, 0x79},
        {0x1, 0xe2}, {0x0, 0x0f}, {0x2, 0xb8},
        {0x1, 0xe2}, {0x0, 0x10}, {0x2, 0x55},
        {0x1, 0xe2}, {0x0, 0x11}, {0x2, 0x57},

        {0x1, 0xe3}, {0x0, 0x00}, {0x2, 0x0e},
        {0x1, 0xe3}, {0x0, 0x01}, {0x2, 0x34},
        {0x1, 0xe3}, {0x0, 0x02}, {0x2, 0x3f},
        {0x1, 0xe3}, {0x0, 0x03}, {0x2, 0x49},
        {0x1, 0xe3}, {0x0, 0x04}, {0x2, 0x1d},
        {0x1, 0xe3}, {0x0, 0x05}, {0x2, 0x2c},
        {0x1, 0xe3}, {0x0, 0x06}, {0x2, 0x5f},
        {0x1, 0xe3}, {0x0, 0x07}, {0x2, 0x3a},
        {0x1, 0xe3}, {0x0, 0x08}, {0x2, 0x20},
        {0x1, 0xe3}, {0x0, 0x09}, {0x2, 0x28},
        {0x1, 0xe3}, {0x0, 0x0a}, {0x2, 0x80},
        {0x1, 0xe3}, {0x0, 0x0b}, {0x2, 0x13},
        {0x1, 0xe3}, {0x0, 0x0c}, {0x2, 0x32},
        {0x1, 0xe3}, {0x0, 0x0d}, {0x2, 0x56},
        {0x1, 0xe3}, {0x0, 0x0e}, {0x2, 0x79},
        {0x1, 0xe3}, {0x0, 0x0f}, {0x2, 0xb8},
        {0x1, 0xe3}, {0x0, 0x10}, {0x2, 0x55},
        {0x1, 0xe3}, {0x0, 0x11}, {0x2, 0x57},

        {0x1, 0xe4}, {0x0, 0x00}, {0x2, 0x0e},
        {0x1, 0xe4}, {0x0, 0x01}, {0x2, 0x34},
        {0x1, 0xe4}, {0x0, 0x02}, {0x2, 0x3f},
        {0x1, 0xe4}, {0x0, 0x03}, {0x2, 0x49},
        {0x1, 0xe4}, {0x0, 0x04}, {0x2, 0x1d},
        {0x1, 0xe4}, {0x0, 0x05}, {0x2, 0x2c},
        {0x1, 0xe4}, {0x0, 0x06}, {0x2, 0x5f},
        {0x1, 0xe4}, {0x0, 0x07}, {0x2, 0x3a},
        {0x1, 0xe4}, {0x0, 0x08}, {0x2, 0x20},
        {0x1, 0xe4}, {0x0, 0x09}, {0x2, 0x28},
        {0x1, 0xe4}, {0x0, 0x0a}, {0x2, 0x80},
        {0x1, 0xe4}, {0x0, 0x0b}, {0x2, 0x13},
        {0x1, 0xe4}, {0x0, 0x0c}, {0x2, 0x32},
        {0x1, 0xe4}, {0x0, 0x0d}, {0x2, 0x56},
        {0x1, 0xe4}, {0x0, 0x0e}, {0x2, 0x79},
        {0x1, 0xe4}, {0x0, 0x0f}, {0x2, 0xb8},
        {0x1, 0xe4}, {0x0, 0x10}, {0x2, 0x55},
        {0x1, 0xe4}, {0x0, 0x11}, {0x2, 0x57},

        {0x1, 0xe5}, {0x0, 0x00}, {0x2, 0x0e},
        {0x1, 0xe5}, {0x0, 0x01}, {0x2, 0x34},
        {0x1, 0xe5}, {0x0, 0x02}, {0x2, 0x3f},
        {0x1, 0xe5}, {0x0, 0x03}, {0x2, 0x49},
        {0x1, 0xe5}, {0x0, 0x04}, {0x2, 0x1d},
        {0x1, 0xe5}, {0x0, 0x05}, {0x2, 0x2c},
        {0x1, 0xe5}, {0x0, 0x06}, {0x2, 0x5f},
        {0x1, 0xe5}, {0x0, 0x07}, {0x2, 0x3a},
        {0x1, 0xe5}, {0x0, 0x08}, {0x2, 0x20},
        {0x1, 0xe5}, {0x0, 0x09}, {0x2, 0x28},
        {0x1, 0xe5}, {0x0, 0x0a}, {0x2, 0x80},
        {0x1, 0xe5}, {0x0, 0x0b}, {0x2, 0x13},
        {0x1, 0xe5}, {0x0, 0x0c}, {0x2, 0x32},
        {0x1, 0xe5}, {0x0, 0x0d}, {0x2, 0x56},
        {0x1, 0xe5}, {0x0, 0x0e}, {0x2, 0x79},
        {0x1, 0xe5}, {0x0, 0x0f}, {0x2, 0xb8},
        {0x1, 0xe5}, {0x0, 0x10}, {0x2, 0x55},
        {0x1, 0xe5}, {0x0, 0x11}, {0x2, 0x57},
        /* Gamma setting: done */

	/* Set RGB-888 */
        {0x1, 0x3a}, {0x0, 0x00}, {0x2, 0x70},
        {0x1, 0x11}, {0x0, 0x00}, {0x2, 0x00},
        {LCM_MDELAY, 120},
        {0x1, 0x29}, {0x0, 0x0 }, {0x2, 0x0 },
};

static struct lcm_cmd auo_uninit_seq[] = {
	{0x1, 0x28}, {0x0, 0x00}, {0x2, 0x00},
	{LCM_MDELAY, 34},
	{0x1, 0x10}, {0x0, 0x00}, {0x2, 0x00},
	{LCM_MDELAY, 100},
};

static struct lcm_cmd auo_sleep_in_seq[] = {
	{0x1, 0x28}, {0x0, 0x00}, {0x2, 0x00},
	{0x1, 0x11}, {0x0, 0x00}, {0x2, 0x00},
	{LCM_MDELAY, 5},
	{0x1, 0x4f}, {0x0, 0x00}, {0x2, 0x01},
};

#ifdef CONFIG_PANEL_SELF_REFRESH
static struct lcm_cmd auo_refresh_in_seq[] = {
        {0x1, 0x3B}, {0x0, 0x00}, {0x2, 0x13},
};

static struct lcm_cmd auo_refresh_out_seq[] = {
        {0x1, 0x3B}, {0x0, 0x00}, {0x2, 0x03},
};
#endif


static int lcm_auo_write_seq(struct lcm_cmd *cmd_table, unsigned size)
{
        int i;

        for (i = 0; i < size; i++) {
		if (cmd_table[i].cmd == LCM_MDELAY) {
			hr_msleep(cmd_table[i].data);
			continue;
		}
		qspi_send_16bit(cmd_table[i].cmd, cmd_table[i].data);
	}
        return 0;
}

static int spade_auo_panel_init(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("\n");

	mutex_lock(&panel_lock);
	spade_auo_panel_power(1);
	lcm_auo_write_seq(auo_init_seq, ARRAY_SIZE(auo_init_seq));
	mutex_unlock(&panel_lock);
	return 0;
}

static int spade_auo_panel_uninit(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("\n");
	mutex_lock(&panel_lock);
	spade_auo_panel_power(0);
	mutex_unlock(&panel_lock);
	return 0;
}
static int spade_auo_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("\n");
	atomic_set(&lcm_init_done, 1);
	spade_adjust_backlight(last_val);

	return 0;
}

static int spade_auo_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("\n");
	spade_adjust_backlight(0);
	atomic_set(&lcm_init_done, 0);
	mutex_lock(&panel_lock);
	lcm_auo_write_seq(auo_uninit_seq, ARRAY_SIZE(auo_uninit_seq));
	mutex_unlock(&panel_lock);

	return 0;
}

static int spade_auo_panel_shutdown(struct msm_lcdc_panel_ops *ops)
{
        lcm_auo_write_seq(auo_uninit_seq, ARRAY_SIZE(auo_uninit_seq));
        spade_auo_panel_power(0);

	return 0;
}

#ifdef CONFIG_PANEL_SELF_REFRESH
static int spade_auo_panel_refresh_enable(struct msm_lcdc_panel_ops *ops)
{
	mutex_lock(&panel_lock);
        lcm_auo_write_seq(auo_refresh_in_seq, ARRAY_SIZE(auo_refresh_in_seq));
	mutex_unlock(&panel_lock);

        return 0;
}


static int spade_auo_panel_refresh_disable(struct msm_lcdc_panel_ops *ops)
{
	mutex_lock(&panel_lock);
        lcm_auo_write_seq(auo_refresh_out_seq, ARRAY_SIZE(auo_refresh_out_seq));
	mutex_unlock(&panel_lock);

        return 0;
}
#endif


static struct msm_lcdc_panel_ops spade_auo_panel_ops = {
	.init		= spade_auo_panel_init,
	.uninit		= spade_auo_panel_uninit,
	.blank		= spade_auo_panel_blank,
	.unblank	= spade_auo_panel_unblank,
	.shutdown	= spade_auo_panel_shutdown,
};

#ifdef CONFIG_PANEL_SELF_REFRESH
static struct msm_lcdc_timing spade_auo_timing = {
	.clk_rate		= 25000000,
	.hsync_pulse_width	= 2,
	.hsync_back_porch	= 10,
	.hsync_front_porch	= 10,
	.hsync_skew		= 0,
	.vsync_pulse_width	= 2,
	.vsync_back_porch	= 3,
	.vsync_front_porch	= 25,
	.vsync_act_low		= 1,
	.hsync_act_low		= 1,
	.den_act_low		= 0,
};
#else
static struct msm_lcdc_timing spade_auo_timing = {
	.clk_rate		= 24576000,
	.hsync_pulse_width	= 2,
	.hsync_back_porch	= 30,
	.hsync_front_porch	= 2,
	.hsync_skew		= 0,
	.vsync_pulse_width	= 2,
	.vsync_back_porch	= 5,
	.vsync_front_porch	= 2,
	.vsync_act_low		= 1,
	.hsync_act_low		= 1,
	.den_act_low		= 0,
};
#endif
/*----------------------------------------------------------------------------*/
#define LCM_CMD(_cmd, ...)                                      \
{                                                               \
	.cmd = _cmd,                                            \
	.data = (u8 []){__VA_ARGS__},                           \
	.len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}

static struct spi_msg sharp_init_seq[] = {
        LCM_CMD(0x11), LCM_CMD(LCM_MDELAY, 110), LCM_CMD(0x29),
};

static struct spi_msg sharp_uninit_seq[] = {
        LCM_CMD(0x28), LCM_CMD(0x10),
};

static int lcm_sharp_write_seq(struct spi_msg *cmd_table, unsigned size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (cmd_table[i].cmd == LCM_MDELAY) {
			hr_msleep(cmd_table[i].data[0]);
			continue;
		}
		qspi_send_9bit(cmd_table + i);
	}
	return 0;
}

static int spade_sharp_panel_init(struct msm_lcdc_panel_ops *ops)
{
        LCMDBG("\n");

        mutex_lock(&panel_lock);
        spade_sharp_panel_power(1);
        lcm_sharp_write_seq(sharp_init_seq, ARRAY_SIZE(sharp_init_seq));
        mutex_unlock(&panel_lock);
        return 0;
}

static int spade_sharp_panel_uninit(struct msm_lcdc_panel_ops *ops)
{
        LCMDBG("\n");

        mutex_lock(&panel_lock);
        lcm_sharp_write_seq(sharp_uninit_seq, ARRAY_SIZE(sharp_uninit_seq));
        spade_sharp_panel_power(0);
	screen_on = false;
        mutex_unlock(&panel_lock);
        return 0;
}

static int spade_sharp_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("\n");
	atomic_set(&lcm_init_done, 1);
	spade_adjust_backlight(last_val);

        return 0;
}

static int spade_sharp_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("\n");
	spade_adjust_backlight(0);
	atomic_set(&lcm_init_done, 0);
        return 0;
}

static int spade_sharp_panel_shutdown(struct msm_lcdc_panel_ops *ops)
{
        lcm_sharp_write_seq(sharp_uninit_seq, ARRAY_SIZE(sharp_uninit_seq));
        spade_sharp_panel_power(0);

        return 0;
}

static struct msm_lcdc_panel_ops spade_sharp_panel_ops = {
	.init		= spade_sharp_panel_init,
	.uninit		= spade_sharp_panel_uninit,
	.blank		= spade_sharp_panel_blank,
	.unblank	= spade_sharp_panel_unblank,
	.shutdown	= spade_sharp_panel_shutdown,
};

static struct msm_lcdc_timing spade_sharp_timing = {
        .clk_rate               = 24576000,
        .hsync_pulse_width      = 6,
        .hsync_back_porch       = 21,
        .hsync_front_porch      = 6,
        .hsync_skew             = 0,
        .vsync_pulse_width      = 3,
        .vsync_back_porch       = 6,
        .vsync_front_porch      = 3,
        .vsync_act_low          = 1,
        .hsync_act_low          = 1,
        .den_act_low            = 0,
};

/*----------------------------------------------------------------------------*/

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_fb_data spade_lcdc_fb_data = {
	.xres		= 480,
	.yres		= 800,
	.width		= 62,
	.height		= 106,
	.output_format	= 0,
};

static struct msm_lcdc_platform_data spade_lcdc_platform_data = {
	.fb_id		= 0,
	.fb_data	= &spade_lcdc_fb_data,
	.fb_resource	= &resources_msm_fb[0],
};

static struct platform_device spade_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &spade_lcdc_platform_data,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
        .color_format = MSM_MDP_OUT_IF_FMT_RGB888,
};

/*----------------------------------------------------------------------------*/

static int spade_adjust_backlight(enum led_brightness val)
{
        uint8_t data[4] = {     /* PWM setting of microp, see p.8 */
                0x05,           /* Fading time; suggested: 5/10/15/20/25 */
                val,            /* Duty Cycle */
                0x00,           /* Channel H byte */
                0x20,           /* Channel L byte */
                };
	uint8_t shrink_br;

        mutex_lock(&panel_lock);
        if (val == 0)
                shrink_br = 0;
        else if (val <= 30)
                shrink_br = 7;
        else if ((val > 30) && (val <= 143))
                shrink_br = (91 - 7) * (val - 30) / (143 - 30) + 7;
        else
                shrink_br = (217 - 91) * (val - 143) / (255 - 143) + 91;
        data[1] = shrink_br;

        LCMDBG("(%d), shrink_br=%d\n", val, shrink_br);
        microp_i2c_write(0x25, data, sizeof(data));
        last_val = shrink_br ? shrink_br: last_val;
        mutex_unlock(&panel_lock);

	return shrink_br;
}

static void spade_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness val)
{
	if (atomic_read(&lcm_init_done) == 0) {
		last_val = val ? val : last_val;
		LCMDBG(":lcm not ready, val=%d\n", val);
		return;
	}
	led_cdev->brightness = spade_adjust_backlight(val);
}

static struct led_classdev spade_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = spade_brightness_set,
};

static int spade_backlight_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &spade_backlight_led);
	if (rc)
		LCMDBG("backlight: failure on register led_classdev\n");
	return 0;
}

static struct platform_device spade_backlight_pdev = {
	.name = "spade-backlight",
};

static struct platform_driver spade_backlight_pdrv = {
	.probe          = spade_backlight_probe,
	.driver         = {
		.name   = "spade-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init spade_backlight_init(void)
{
	return platform_driver_register(&spade_backlight_pdrv);
}

/*----------------------------------------------------------------------------*/
int spade_panel_sleep_in(void)
{
	int ret;

	LCMDBG(", screen=%s\n", screen_on ? "on" : "off");
	if (screen_on)
		return 0;

	mutex_lock(&panel_lock);
	switch (panel_type) {
		case PANEL_AUO:
			spade_auo_panel_power(1);
			lcm_auo_write_seq(auo_sleep_in_seq,
				ARRAY_SIZE(auo_sleep_in_seq));
			ret = 0;
			break;
		case PANEL_SHARP:
			spade_sharp_panel_power(1);
			lcm_sharp_write_seq(sharp_uninit_seq,
				ARRAY_SIZE(sharp_uninit_seq));
			ret = 0;
			break;
		default:
			ret = -EINVAL;
			break;
	}
	mutex_unlock(&panel_lock);
	return ret;
}

int __init spade_init_panel(void)
{
	int ret;

        vreg_lcm_1v8 = vreg_get(0, "gp13");
        if (IS_ERR(vreg_lcm_1v8))
                return PTR_ERR(vreg_lcm_1v8);
        vreg_lcm_2v8 = vreg_get(0, "wlan2");
        if (IS_ERR(vreg_lcm_2v8))
                return PTR_ERR(vreg_lcm_2v8);

	LCMDBG("panel_type=%d\n", panel_type);
	switch (panel_type) {
	case PANEL_AUO:
#ifdef CONFIG_PANEL_SELF_REFRESH
		mdp_pdata.overrides = MSM_MDP_RGB_PANEL_SELE_REFRESH,
		LCMDBG("ACE AUO Panel:RGB_PANEL_SELE_REFRESH \n");
		spade_auo_panel_ops.refresh_enable = spade_auo_panel_refresh_enable;
		spade_auo_panel_ops.refresh_disable = spade_auo_panel_refresh_disable;
#endif
		spade_lcdc_platform_data.timing = &spade_auo_timing;
		spade_lcdc_platform_data.panel_ops = &spade_auo_panel_ops;

		break;
	case PANEL_SHARP:
		spade_lcdc_platform_data.timing = &spade_sharp_timing;
		spade_lcdc_platform_data.panel_ops = &spade_sharp_panel_ops;
		break;
	default:
		return -EINVAL;
	}
	msm_device_mdp.dev.platform_data = &mdp_pdata;
	ret = platform_device_register(&msm_device_mdp);
	if (ret != 0)
		return ret;

	ret = platform_device_register(&spade_lcdc_device);
	if (ret != 0)
		return ret;

	ret = platform_device_register(&spade_backlight_pdev);
        if (ret)
                return ret;

	return 0;
}

module_init(spade_backlight_init);
