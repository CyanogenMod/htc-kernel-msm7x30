/* arch/arm/mach-msm/board-vision-keypad.c
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
#include <mach/board_htc.h>

#include "board-vision.h"
#include "proc_comm.h"

#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_vision."
module_param_named(keycaps, keycaps, charp, 0);
#if 0
/* fix up last row issue, + 1 */
#define VISION_KEY_MATRIX_ROW_NUM (7 + 1)
#define VISION_KEY_MATRIX_COL_NUM 8

#define KEYMAP_INDEX(col, row) ((col) * VISION_KEY_MATRIX_ROW_NUM + (row))

static const unsigned short vision_keymap_x0[VISION_KEY_MATRIX_COL_NUM *
						VISION_KEY_MATRIX_ROW_NUM] = {

};

static struct resource vision_resources_keypad[] = {
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

struct platform_device vision_keypad_resource = {
	.name		= PMIC8058_KEYPAD_NAME,
	.id		= -1,
	.num_resources  = ARRAY_SIZE(vision_resources_keypad),
	.resource       = vision_resources_keypad,
};

static struct pmic8058_keypad_data vision_keypad_matrix_info = {
	/*.info.func = pmic8058_event_matrix_func,*/
	.input_name = PMIC8058_KEYPAD_NAME,
	.num_rows = VISION_KEY_MATRIX_ROW_NUM,
	.num_cols = VISION_KEY_MATRIX_COL_NUM,
	.rows_gpio_start = 8,
	.cols_gpio_start = 0,
	.keymap_size = ARRAY_SIZE(vision_keymap_x0),
	.keymap = vision_keymap_x0,
	.debounce_ms = 8,
	.scan_delay_ms = 32,
	.row_hold_us = 125,
	.wakeup = 1,
	.p_device = &vision_keypad_resource,
	.flags = (/*PMIC8058_PRINT_UNMAPPED_KEYS |
		PMIC8058_PRINT_MAPPED_KEYS |*/
		PMIC8058_PRINT_SEND_KEYS /*|
		PMIC8058_PRINT_PHANTOM_KEYS*/),
};
#endif
#ifdef CONFIG_MSM_SSBI
static unsigned int vision_pmic_col_gpios[] = {
	PM8058_GPIO_PM_TO_SYS(PMGPIO(1)), PM8058_GPIO_PM_TO_SYS(PMGPIO(2)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(3)), PM8058_GPIO_PM_TO_SYS(PMGPIO(4)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(5)), PM8058_GPIO_PM_TO_SYS(PMGPIO(6)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(7)),
};
static unsigned int vision_pmic_row_gpios[] = {
	PM8058_GPIO_PM_TO_SYS(PMGPIO(9)), PM8058_GPIO_PM_TO_SYS(PMGPIO(10)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(11)), PM8058_GPIO_PM_TO_SYS(PMGPIO(12)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(13)), PM8058_GPIO_PM_TO_SYS(PMGPIO(14)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(15)),

};

#define KEYMAP_NUM_ROWS		ARRAY_SIZE(vision_pmic_row_gpios)
#define KEYMAP_NUM_COLS		ARRAY_SIZE(vision_pmic_col_gpios)
#define KEYMAP_INDEX(row, col)	(((row) * KEYMAP_NUM_COLS) + (col))
#define KEYMAP_SIZE		(KEYMAP_NUM_ROWS * KEYMAP_NUM_COLS)

static unsigned short vision_pmic_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_F15,
	[KEYMAP_INDEX(0, 1)] = KEY_RIGHTALT,
	[KEYMAP_INDEX(0, 2)] =  KEY_SEARCH,
	[KEYMAP_INDEX(0, 3)] =  KEY_ENTER,
	[KEYMAP_INDEX(0, 4)] =  KEY_L,
	[KEYMAP_INDEX(0, 5)] =  KEY_BACKSPACE,
	[KEYMAP_INDEX(0, 6)] =  KEY_RIGHTSHIFT,

	[KEYMAP_INDEX(1, 0)] =  KEY_F14,
	[KEYMAP_INDEX(1, 1)] =  KEY_M,
	[KEYMAP_INDEX(1, 2)] =  KEY_QUESTION,
	[KEYMAP_INDEX(1, 3)] =  KEY_K,
	[KEYMAP_INDEX(1, 4)] =  KEY_O,
	[KEYMAP_INDEX(1, 5)] =  KEY_P,
	[KEYMAP_INDEX(1, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(2, 0)] =  KEY_DOT,
	[KEYMAP_INDEX(2, 1)] =  KEY_N,
	[KEYMAP_INDEX(2, 2)] =  KEY_H,
	[KEYMAP_INDEX(2, 3)] =  KEY_J,
	[KEYMAP_INDEX(2, 4)] =  KEY_I,
	[KEYMAP_INDEX(2, 5)] =  KEY_U,
	[KEYMAP_INDEX(2, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(3, 0)] =  KEY_SPACE,
	[KEYMAP_INDEX(3, 1)] =  KEY_V,
	[KEYMAP_INDEX(3, 2)] =  KEY_B,
	[KEYMAP_INDEX(3, 3)] =  KEY_G,
	[KEYMAP_INDEX(3, 4)] =  KEY_T,
	[KEYMAP_INDEX(3, 5)] =  KEY_Y,
	[KEYMAP_INDEX(3, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(4, 0)] =   KEY_COMMA,
	[KEYMAP_INDEX(4, 1)] =   KEY_C,
	[KEYMAP_INDEX(4, 2)] =   KEY_D,
	[KEYMAP_INDEX(4, 3)] =   KEY_F,
	[KEYMAP_INDEX(4, 4)] =   KEY_R,
	[KEYMAP_INDEX(4, 5)] =   KEY_E,
	[KEYMAP_INDEX(4, 6)] =   KEY_RESERVED,

	[KEYMAP_INDEX(5, 0)] =  KEY_F13, /*also as sym/search/language*/
	[KEYMAP_INDEX(5, 1)] =  KEY_Z,
	[KEYMAP_INDEX(5, 2)] =  KEY_X,
	[KEYMAP_INDEX(5, 3)] =  KEY_S,
	[KEYMAP_INDEX(5, 4)] =  KEY_A,
	[KEYMAP_INDEX(5, 5)] =  KEY_W,
	[KEYMAP_INDEX(5, 6)] =  KEY_RESERVED,

	[KEYMAP_INDEX(6, 0)] =  KEY_LEFTSHIFT,
	[KEYMAP_INDEX(6, 1)] =  KEY_LEFTALT,
	[KEYMAP_INDEX(6, 2)] =  KEY_MENU,
	[KEYMAP_INDEX(6, 3)] =  KEY_WWW, /*also as tab*/
	[KEYMAP_INDEX(6, 4)] =  KEY_EMAIL,
	[KEYMAP_INDEX(6, 5)] =  KEY_Q,
	[KEYMAP_INDEX(6, 6)] =  KEY_RESERVED,
};

static struct gpio_event_matrix_info vision_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = vision_pmic_keymap,
	.output_gpios = vision_pmic_row_gpios,
	.input_gpios = vision_pmic_col_gpios,
	.noutputs = KEYMAP_NUM_ROWS,
	.ninputs = KEYMAP_NUM_COLS,
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS | GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};
#endif
static struct gpio_event_direct_entry vision_keypad_input_map[] = {
	{
		.gpio = VISION_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(VISION_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(VISION_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(VISION_OJ_ACTION),
		.code = BTN_MOUSE,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(VISION_CAM_STEP2),
		.code = KEY_CAMERA,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(VISION_CAM_STEP1),
		.code = KEY_HP,
	},
};

static void vision_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(VISION_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT,	GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info vision_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.info.oj_btn = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = vision_keypad_input_map,
	.keymap_size = ARRAY_SIZE(vision_keypad_input_map),
	.setup_input_gpio = vision_setup_input_gpio,
};

static void vision_set_qty_irq(uint8_t disable)
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
			disable_irq(gpio_to_irq(vision_pmic_col_gpios[i]));
		else
			enable_irq(gpio_to_irq(vision_pmic_col_gpios[i]));
	}
}

static struct gpio_event_direct_entry vision_sliding_switch[] = {
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(VISION_SLIDING_INTz),
		.code = SW_LID,
	},
};

static struct gpio_event_switch_info vision_keypad_switch_info = {
	.info.func = gpio_event_switch_func,
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
	.keymap = vision_sliding_switch,
	.keymap_size = ARRAY_SIZE(vision_sliding_switch),
	.set_qty_irq = vision_set_qty_irq,
};

static struct gpio_event_info *vision_keypad_info[] = {
	&vision_keypad_input_info.info,
	&vision_keypad_switch_info.info,
#ifdef CONFIG_MSM_SSBI
	&vision_keypad_matrix_info.info,
#endif
};

static struct gpio_event_platform_data vision_keypad_data = {
	.names = {
		"vision-keypad",
		NULL,
	},
	.info = vision_keypad_info,
	.info_count = ARRAY_SIZE(vision_keypad_info),
};

static struct platform_device vision_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &vision_keypad_data,
	},
};

static int vision_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data vision_reset_keys_pdata = {
	.keys_up = vision_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device vision_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &vision_reset_keys_pdata,
};

char *SKU_WWE = "(HTC__001),(HTC__016),(HTC__032),(HTC__038),(HTC__E11),"\
		"(VODAP001),(VODAPE17),(BM___001),(ORANGB10),(ORANG006)";
char *SKU_WWE_BOPOMO = "(HTC__621)";
char *SKU_HK = "(HTC__622)";
char *SKU_SEA = "(HTC__044)";
char *SKU_FRA = "(HTC__203),(HTC__E41),(ORANG202)";
char *SKU_ITA = "(HTC__405)";
char *SKU_TUR = "(HTC__M27)";
char *SKU_ELL = "(HTC__N34)";
char *SKU_GER = "(VODAP102),(VODAP110),(VODAP120),(HTC__102),(O2___102)";
char *SKU_ARA = "(HTC__J15)";
char *SKU_ESN = "(HTC__304),(VODAP304)";
char *SKU_NOR = "(HTC__Y13)";
char *SKU_RUS = "(HTC__A07)";
char *SKU_TMUS = "(T-MOB010)";

int __init vision_init_keypad(void)
{
	char *get_cid;
	char *get_carrier;
	char *get_keycaps;
	uint8_t cid_len;

	printk(KERN_DEBUG "%s\n",	__func__);
	board_get_cid_tag(&get_cid);
	board_get_carrier_tag(&get_carrier);
	board_get_keycaps_tag(&get_keycaps);
	printk(KERN_DEBUG "%s: get CID: %s\n\tCarrier: %s, Keycaps: %s\n",
		__func__, get_cid, get_carrier, get_keycaps);
	cid_len = strlen(get_cid);

	if (cid_len) {
		if (strstr(SKU_TMUS, get_cid) != NULL) {
			pr_info("%s: SKU is TMUS\n", __func__);
		} else if (strstr(SKU_WWE_BOPOMO, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-wwe-bopomo";

		} else if (strstr(SKU_FRA, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(0, 2)] = KEY_APOSTROPHE;
			vision_pmic_keymap[KEYMAP_INDEX(5, 0)] = KEY_SEARCH;
			vision_pmic_keymap[KEYMAP_INDEX(5, 1)] = KEY_W;
			vision_pmic_keymap[KEYMAP_INDEX(5, 4)] = KEY_Q;
			vision_pmic_keymap[KEYMAP_INDEX(5, 5)] = KEY_Z;
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_pmic_keymap[KEYMAP_INDEX(6, 5)] = KEY_A;
			vision_keypad_data.name = "vision-keypad-fra";

		} else if (strstr(SKU_ITA, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-ita";

		} else if (strstr(SKU_TUR, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(0, 2)] = KEY_APOSTROPHE;
			vision_pmic_keymap[KEYMAP_INDEX(5, 0)] = KEY_SEARCH;
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-tur";

		} else if (strstr(SKU_ELL, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-ell";

		} else if (strstr(SKU_GER, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(0, 2)] = KEY_SLASH;
			vision_pmic_keymap[KEYMAP_INDEX(3, 5)] = KEY_Z;
			vision_pmic_keymap[KEYMAP_INDEX(5, 0)] = KEY_SEARCH;
			vision_pmic_keymap[KEYMAP_INDEX(5, 1)] = KEY_Y;
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-ger";

		} else if (strstr(SKU_ARA, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(0, 2)] = KEY_F16;
			vision_pmic_keymap[KEYMAP_INDEX(5, 0)] = KEY_SEARCH;
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-ara";

		} else if (strstr(SKU_ESN, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(0, 2)] = KEY_APOSTROPHE;
			vision_pmic_keymap[KEYMAP_INDEX(5, 0)] = KEY_SEARCH;
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-esn";

		} else if (strstr(SKU_NOR, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(0, 1)] = KEY_F16;
			vision_pmic_keymap[KEYMAP_INDEX(0, 2)] = KEY_F17;
			vision_pmic_keymap[KEYMAP_INDEX(1, 2)] = KEY_F18;
			vision_pmic_keymap[KEYMAP_INDEX(5, 0)] = KEY_SEARCH;
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-nor";

		} else if (strstr(SKU_RUS, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(0, 1)] = KEY_F16;
			vision_pmic_keymap[KEYMAP_INDEX(0, 2)] = KEY_SEMICOLON;
			vision_pmic_keymap[KEYMAP_INDEX(0, 6)] = KEY_F17;
			vision_pmic_keymap[KEYMAP_INDEX(2, 0)] = KEY_F18;
			vision_pmic_keymap[KEYMAP_INDEX(4, 0)] = KEY_DOT;
			vision_pmic_keymap[KEYMAP_INDEX(5, 0)] = KEY_SEARCH;
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-rus";

		} else if (strstr(SKU_HK, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-hk";

		} else if (strstr(SKU_SEA, get_cid) != NULL) {

			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-sea";

		} else {

			if (strstr(SKU_WWE, get_cid) == NULL)
				pr_warning("%s: CID not matched\n", __func__);
			vision_pmic_keymap[KEYMAP_INDEX(6, 3)] = KEY_TAB;
			vision_keypad_data.name = "vision-keypad-wwe";

		}
	}

	if (platform_device_register(&vision_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&vision_keypad_input_device);
}

