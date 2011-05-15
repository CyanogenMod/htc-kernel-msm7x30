/* arch/arm/mach-msm/board-speedy-keypad.c
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

#include "board-speedy.h"
#include "proc_comm.h"
#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_speedy."
module_param_named(keycaps, keycaps, charp, 0);
#if 0
/* fix up last row issue, + 1 */
#define SPEEDY_KEY_MATRIX_ROW_NUM (7 + 1)
#define SPEEDY_KEY_MATRIX_COL_NUM 8

#define KEYMAP_INDEX(col, row) ((col) * SPEEDY_KEY_MATRIX_ROW_NUM + (row))

static const unsigned short speedy_keymap_x0[SPEEDY_KEY_MATRIX_COL_NUM *
						SPEEDY_KEY_MATRIX_ROW_NUM] = {
};

static struct resource speedy_resources_keypad[] = {
	{
		.start	= PM8058_IRQ_KEYPAD,
		.end	= PM8058_IRQ_KEYPAD,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_IRQ_KEYSTUCK,
		.end	= PM8058_IRQ_KEYSTUCK,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device speedy_keypad_resource = {
	.name		= PMIC8058_KEYPAD_NAME,
	.id		= -1,
	.num_resources  = ARRAY_SIZE(speedy_resources_keypad),
	.resource       = speedy_resources_keypad,
};

static struct pmic8058_keypad_data speedy_keypad_matrix_info = {
	/*.info.func = pmic8058_event_matrix_func,*/
	.input_name = PMIC8058_KEYPAD_NAME,
	.num_rows = SPEEDY_KEY_MATRIX_ROW_NUM,
	.num_cols = SPEEDY_KEY_MATRIX_COL_NUM,
	.rows_gpio_start = 8,
	.cols_gpio_start = 0,
	.keymap_size = ARRAY_SIZE(speedy_keymap_x0),
	.keymap = (unsigned int *)speedy_keymap_x0,
	.debounce_ms = {8},
	.scan_delay_ms = 32,
	.row_hold_us = 125,
	.wakeup = 1,
	.p_device = &speedy_keypad_resource,
	.flags = (/*PMIC8058_PRINT_UNMAPPED_KEYS |
		PMIC8058_PRINT_MAPPED_KEYS |*/
		PMIC8058_PRINT_SEND_KEYS /*|
		PMIC8058_PRINT_PHANTOM_KEYS*/),
};
#endif

#ifdef CONFIG_MSM_SSBI
static unsigned int speedy_pmic_col_gpios[] = {
	PM8058_GPIO_PM_TO_SYS(PMGPIO(1)), PM8058_GPIO_PM_TO_SYS(PMGPIO(2)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(3)), PM8058_GPIO_PM_TO_SYS(PMGPIO(4)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(5)), PM8058_GPIO_PM_TO_SYS(PMGPIO(6)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(7)),
};
static unsigned int speedy_pmic_row_gpios[] = {
	PM8058_GPIO_PM_TO_SYS(PMGPIO(9)), PM8058_GPIO_PM_TO_SYS(PMGPIO(10)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(11)), PM8058_GPIO_PM_TO_SYS(PMGPIO(12)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(13)), PM8058_GPIO_PM_TO_SYS(PMGPIO(14)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(15)),

};

#define KEYMAP_NUM_ROWS		ARRAY_SIZE(speedy_pmic_row_gpios)
#define KEYMAP_NUM_COLS		ARRAY_SIZE(speedy_pmic_col_gpios)
#define KEYMAP_INDEX(row, col)	(((row) * KEYMAP_NUM_COLS) + (col))
#define KEYMAP_SIZE		(KEYMAP_NUM_ROWS * KEYMAP_NUM_COLS)

static unsigned short speedy_pmic_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] =  KEY_EMAIL,
	[KEYMAP_INDEX(0, 1)] =  KEY_RESERVED,
	[KEYMAP_INDEX(0, 2)] =  KEY_LEFTSHIFT,
	[KEYMAP_INDEX(0, 3)] =  KEY_LEFTALT,
	[KEYMAP_INDEX(0, 4)] =  KEY_LEFT,
	[KEYMAP_INDEX(0, 5)] =  KEY_UP,
	[KEYMAP_INDEX(0, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(1, 0)] =  KEY_Q,
	[KEYMAP_INDEX(1, 1)] =  KEY_A,
	[KEYMAP_INDEX(1, 2)] =  KEY_Z,
	[KEYMAP_INDEX(1, 3)] =  KEY_MENU,
	[KEYMAP_INDEX(1, 4)] =  KEY_DOWN,
	[KEYMAP_INDEX(1, 5)] =  KEY_SEARCH,
	[KEYMAP_INDEX(1, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(2, 0)] =  KEY_W,
	[KEYMAP_INDEX(2, 1)] =  KEY_S,
	[KEYMAP_INDEX(2, 2)] =  KEY_X,
	[KEYMAP_INDEX(2, 3)] =  KEY_RESERVED,
	[KEYMAP_INDEX(2, 4)] =  KEY_M,
	[KEYMAP_INDEX(2, 5)] =  KEY_J,
	[KEYMAP_INDEX(2, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(3, 0)] =  KEY_E,
	[KEYMAP_INDEX(3, 1)] =  KEY_D,
	[KEYMAP_INDEX(3, 2)] =  KEY_C,
	[KEYMAP_INDEX(3, 3)] =  KEY_COMMA,
	[KEYMAP_INDEX(3, 4)] =  KEY_U,
	[KEYMAP_INDEX(3, 5)] =  KEY_RESERVED,
	[KEYMAP_INDEX(3, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(4, 0)] =   KEY_R,
	[KEYMAP_INDEX(4, 1)] =   KEY_F,
	[KEYMAP_INDEX(4, 2)] =   KEY_V,
	[KEYMAP_INDEX(4, 3)] =   KEY_RESERVED,
	[KEYMAP_INDEX(4, 4)] =   KEY_I,
	[KEYMAP_INDEX(4, 5)] =   KEY_O,
	[KEYMAP_INDEX(4, 6)] =   KEY_P,

	[KEYMAP_INDEX(5, 0)] =  KEY_T,
	[KEYMAP_INDEX(5, 1)] =  KEY_G,
	[KEYMAP_INDEX(5, 2)] =  KEY_B,
	[KEYMAP_INDEX(5, 3)] =  KEY_SPACE,
	[KEYMAP_INDEX(5, 4)] =  KEY_K,
	[KEYMAP_INDEX(5, 5)] =  KEY_L,
	[KEYMAP_INDEX(5, 6)] =  KEY_BACKSPACE,

	[KEYMAP_INDEX(6, 0)] =  KEY_Y,
	[KEYMAP_INDEX(6, 1)] =  KEY_H,
	[KEYMAP_INDEX(6, 2)] =  KEY_N,
	[KEYMAP_INDEX(6, 3)] =  KEY_DOT,
	[KEYMAP_INDEX(6, 4)] =  KEY_QUESTION,
	[KEYMAP_INDEX(6, 5)] =  KEY_RIGHT,
	[KEYMAP_INDEX(6, 6)] =  KEY_ENTER,
};

static struct gpio_event_matrix_info speedy_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
//	.info.no_suspend = true,
	.keymap = speedy_pmic_keymap,
	.output_gpios = speedy_pmic_row_gpios,
	.input_gpios = speedy_pmic_col_gpios,
	.noutputs = KEYMAP_NUM_ROWS,
	.ninputs = KEYMAP_NUM_COLS,
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS | GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};
#endif

static struct gpio_event_direct_entry speedy_keypad_input_map[] = {
	{
		.gpio = SPEEDY_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
//		.wakeup = 1,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(SPEEDY_VOL_UP),
		.code = KEY_VOLUMEUP,
//		.wakeup = 1,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(SPEEDY_VOL_DN),
		.code = KEY_VOLUMEDOWN,
//		.wakeup = 1,
	},
};

static void speedy_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(SPEEDY_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info speedy_keypad_input_info = {
	.info.func = gpio_event_input_func,
//	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = speedy_keypad_input_map,
	.keymap_size = ARRAY_SIZE(speedy_keypad_input_map),
	.setup_input_gpio = speedy_setup_input_gpio,
};

static void speedy_set_qty_irq(uint8_t disable)
{
	uint32_t i;
	static uint8_t already_disabled;
	pr_info("%s disable=%d, already_disabled=%d\n",
			__func__, disable, already_disabled);

	if (!(disable ^ already_disabled))
		return;

	already_disabled = disable;
	for (i = 0; i < KEYMAP_NUM_COLS; i++) {
		if (disable)
			disable_irq(gpio_to_irq(speedy_pmic_col_gpios[i]));
		else
			enable_irq(gpio_to_irq(speedy_pmic_col_gpios[i]));
	}
}

static struct gpio_event_direct_entry speedy_sliding_switch[] = {
	{
		.gpio = SPEEDY_SLIDING_INTZ,
		.code = SW_LID,
//		.wakeup = 1,
	},
};

static struct gpio_event_switch_info speedy_keypad_switch_info = {
	.info.func = gpio_event_switch_func,
//	.info.no_suspend = true,
#if 1
	.flags = GPIOEDF_PRINT_KEYS,
#else
	.flags = GPIOKPF_ACTIVE_HIGH,
	/* FIXME: since sliding is opened on Latte barebone system,
	 * set the active direction inverse on purpose
	 * just for our convenience in current stage.
	 */
#endif
	.type = EV_SW,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = speedy_sliding_switch,
	.keymap_size = ARRAY_SIZE(speedy_sliding_switch),
//	.setup_input_gpio = speedy_sliding_input_gpio,
	.set_qty_irq = speedy_set_qty_irq,
};

static struct gpio_event_info *speedy_keypad_info[] = {
	&speedy_keypad_input_info.info,
	&speedy_keypad_switch_info.info,
#ifdef CONFIG_MSM_SSBI
	&speedy_keypad_matrix_info.info,
#endif
};

static int speedy_pmic8058_keypad_power(
			const struct gpio_event_platform_data *pdata, bool on)
{
	/* if we need it... */
	return 0;
}

static struct gpio_event_platform_data speedy_keypad_data = {
	.names = {
		"speedy-keypad",
		NULL,
	},
	.info = speedy_keypad_info,
	.info_count = ARRAY_SIZE(speedy_keypad_info),
	.power = speedy_pmic8058_keypad_power,
};

static struct platform_device speedy_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &speedy_keypad_data,
	},
};

#ifdef CONFIG_INPUT_KEYRESET
/*
static int speedy_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/
static struct keyreset_platform_data speedy_reset_keys_pdata = {
	/*.keys_up = speedy_reset_keys_up,*/
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

struct platform_device speedy_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &speedy_reset_keys_pdata,
};
#endif

int __init speedy_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n",	__func__);
#ifdef CONFIG_INPUT_KEYRESET
	if (platform_device_register(&speedy_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);
#endif

	if (system_rev == 0) {
		speedy_keypad_data.name = "speedy-keypad-v0";
		speedy_pmic_keymap[KEYMAP_INDEX(0, 1)] =  KEY_HOME;
		speedy_pmic_keymap[KEYMAP_INDEX(2, 3)] =  KEY_BACK;
	}

	return platform_device_register(&speedy_keypad_input_device);
}

