/* linux/arch/arm/mach-msm/board-mecha-keypad.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include "board-mecha.h"
#include "proc_comm.h"
#include <linux/mfd/pmic8058.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_mecha."
module_param_named(keycaps, keycaps, charp, 0);

static struct gpio_event_direct_entry mecha_keypad_input_map[] = {
	{
		.gpio = MECHA_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(MECHA_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(MECHA_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
};

static void mecha_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(MECHA_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info mecha_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = mecha_keypad_input_map,
	.keymap_size = ARRAY_SIZE(mecha_keypad_input_map),
	.setup_input_gpio = mecha_setup_input_gpio,
};

static struct gpio_event_info *mecha_keypad_info[] = {
	&mecha_keypad_input_info.info,
};

static struct gpio_event_platform_data mecha_keypad_data = {
	.names = {
		"mecha-keypad",
		NULL,
	},
	.info = mecha_keypad_info,
	.info_count = ARRAY_SIZE(mecha_keypad_info),
};

static struct platform_device mecha_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &mecha_keypad_data,
	},
};
/*
static int mecha_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/
static struct keyreset_platform_data mecha_reset_keys_pdata = {
	/*.keys_up = mecha_reset_keys_up,*/
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

struct platform_device mecha_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &mecha_reset_keys_pdata,
};

int __init mecha_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	if (platform_device_register(&mecha_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&mecha_keypad_input_device);
}
