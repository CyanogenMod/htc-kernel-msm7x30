/* arch/arm/mach-msm/board-glacier-keypad.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include "board-glacier.h"
#include "proc_comm.h"

#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_glacier."
module_param_named(keycaps, keycaps, charp, 0);

static struct gpio_event_direct_entry glacier_keypad_input_map[] = {
	{
		.gpio = GLACIER_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_OJ_ACTION),
		.code = BTN_MOUSE,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_HOME_KEY),
		.code = KEY_HOME,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_MENU_KEY),
		.code = KEY_MENU,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_BACK_KEY),
		.code = KEY_BACK,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_SEND_KEY),
		.code = KEY_F13,/*Genius/voice command key*/
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_CAM_STEP2),
		.code = KEY_CAMERA,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(GLACIER_CAM_STEP1),
		.code = KEY_HP,
	},
};

static void glacier_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(GLACIER_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));

}

static struct gpio_event_input_info glacier_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.info.oj_btn = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = glacier_keypad_input_map,
	.keymap_size = ARRAY_SIZE(glacier_keypad_input_map),
	.setup_input_gpio = glacier_setup_input_gpio,
};

static struct gpio_event_info *glacier_keypad_info[] = {
	&glacier_keypad_input_info.info,
};

static struct gpio_event_platform_data glacier_keypad_data = {
	.names = {
		"glacier-keypad",
		NULL,
	},
	.info = glacier_keypad_info,
	.info_count = ARRAY_SIZE(glacier_keypad_info),
};

static struct platform_device glacier_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &glacier_keypad_data,
	},
};

static int glacier_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data glacier_reset_keys_pdata = {
	.keys_up = glacier_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device glacier_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &glacier_reset_keys_pdata,
};

int __init glacier_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n",	__func__);

	if (platform_device_register(&glacier_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&glacier_keypad_input_device);
}

