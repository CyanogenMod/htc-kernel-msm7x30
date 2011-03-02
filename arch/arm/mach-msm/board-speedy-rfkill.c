/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
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

/* Control bluetooth power for speedy platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include "gpio_chip.h"
#include "proc_comm.h"
#include "board-speedy.h"

#define ID_BT	1
#define CLK_OFF	0
#define CLK_ON	1

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

extern int speedy_wifi_bt_sleep_clk_ctl(int on, int id);

/* bt initial configuration */
static uint32_t speedy_bt_init_table[] = {

	/* BT_RTS */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_RTS,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_CTS */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_CTS,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_RX */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_RX,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_TX */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_TX,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),

	/* BT_RESET_N */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_RESET_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_SHUTDOWN_N */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),

	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_HOST_WAKE,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_2MA),
	/* BT_CHIP_WAKE */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
};

/* bt on configuration */
static uint32_t speedy_bt_on_table[] = {

	/* BT_RTS */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_RTS,
				1,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_CTS */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_CTS,
				1,
				GPIO_INPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_RX */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_RX,
				1,
				GPIO_INPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_TX */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_TX,
				1,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),

	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_HOST_WAKE,
				0,
				GPIO_INPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_CHIP_WAKE */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),

	/* BT_RESET_N */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_RESET_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_SHUTDOWN_N */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
};

/* bt off configuration */
static uint32_t speedy_bt_off_table[] = {

	/* BT_RTS */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_RTS,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_CTS */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_CTS,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_2MA),
	/* BT_RX */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_RX,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_2MA),
	/* BT_TX */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_UART1_TX,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),

	/* BT_RESET_N */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_RESET_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
	/* BT_SHUTDOWN_N */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),

	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_HOST_WAKE,
				0,
				GPIO_INPUT,
				GPIO_PULL_UP,
				GPIO_2MA),
	/* BT_CHIP_WAKE */
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_OUTPUT,
				GPIO_NO_PULL,
				GPIO_2MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for (n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void speedy_config_bt_init(void)
{
	speedy_wifi_bt_sleep_clk_ctl(CLK_ON, ID_BT);
	mdelay(2);

	/* set bt initial configuration*/
	config_bt_table(speedy_bt_init_table,
				ARRAY_SIZE(speedy_bt_init_table));
	mdelay(2);

	/* BT_RESET_N */
	gpio_configure(SPEEDY_GPIO_BT_RESET_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(1);
	/* BT_SHUTDOWN_N */
	gpio_configure(SPEEDY_GPIO_BT_SHUTDOWN_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(1);

	/* BT_CHIP_WAKE */
	gpio_configure(SPEEDY_GPIO_BT_CHIP_WAKE,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);

	/* BT_RTS */
	gpio_configure(SPEEDY_GPIO_BT_UART1_RTS,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	/* BT_CTS */
	gpio_configure(SPEEDY_GPIO_BT_UART1_CTS,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	/* BT_RX */
	gpio_configure(SPEEDY_GPIO_BT_UART1_RX,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	/* BT_TX */
	gpio_configure(SPEEDY_GPIO_BT_UART1_TX,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);

}

static void speedy_config_bt_on(void)
{
	speedy_wifi_bt_sleep_clk_ctl(CLK_ON, ID_BT);
	mdelay(2);

	/* set bt on configuration*/
	config_bt_table(speedy_bt_on_table,
				ARRAY_SIZE(speedy_bt_on_table));
	mdelay(2);

	/* BT_SHUTDOWN_N */
	gpio_configure(SPEEDY_GPIO_BT_SHUTDOWN_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(1);

	/* BT_RESET_N */
	gpio_configure(SPEEDY_GPIO_BT_RESET_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(1);

}

static void speedy_config_bt_off(void)
{

	/* BT_RESET_N */
	gpio_configure(SPEEDY_GPIO_BT_RESET_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(1);

	/* BT_SHUTDOWN_N */
	gpio_configure(SPEEDY_GPIO_BT_SHUTDOWN_N,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(1);

	/* set bt off configuration*/
	config_bt_table(speedy_bt_off_table,
				ARRAY_SIZE(speedy_bt_off_table));
	mdelay(2);

	/* BT_RTS */
	gpio_configure(SPEEDY_GPIO_BT_UART1_RTS,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	/* BT_CTS */

	/* BT_RX */

	/* BT_TX */
	gpio_configure(SPEEDY_GPIO_BT_UART1_TX,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);

	/* BT_HOST_WAKE */

	/* BT_CHIP_WAKE */
	gpio_configure(SPEEDY_GPIO_BT_CHIP_WAKE,
				GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);

	speedy_wifi_bt_sleep_clk_ctl(CLK_OFF, ID_BT);
	mdelay(2);
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
			speedy_config_bt_on();
	else
			speedy_config_bt_off();

	return 0;
}

static struct rfkill_ops speedy_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int speedy_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true; /* off */

	speedy_config_bt_init();	/* bt gpio initial config */

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
						 &speedy_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_reset;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_reset:
	return rc;
}

static int speedy_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static struct platform_driver speedy_rfkill_driver = {
	.probe = speedy_rfkill_probe,
	.remove = speedy_rfkill_remove,
	.driver = {
		.name = "speedy_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init speedy_rfkill_init(void)
{
	if (!machine_is_speedy())
		return 0;

	return platform_driver_register(&speedy_rfkill_driver);
}

static void __exit speedy_rfkill_exit(void)
{
	platform_driver_unregister(&speedy_rfkill_driver);
}

module_init(speedy_rfkill_init);
module_exit(speedy_rfkill_exit);
MODULE_DESCRIPTION("speedy rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
