/* arch/arm/mach-msm/include/mach/htc_fmtx_rfkill.c
 *
 * Copyright (C) 2010 HTC, Inc.
 * Author: assd bt <assd_bt@htc.com>
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

/* Control bluetooth power for glacier platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <asm/mach-types.h>

#include <mach/htc_fmtx_rfkill.h>
#include <linux/mfd/pmic8058.h>

#define FMTX_DBG
#define FMTX_ON 1
#define FMTX_OFF 0

static struct rfkill *htc_fmtx_rfk;
static const char htc_fmtx_name[] = "fmtx";
static int htc_fmtx_switch_pin;

/* pm8058 config */
static struct pm8058_gpio pmic_gpio_htc_fmtx_output = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,	/* L2 2.6 V */
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_NORMAL,
};

static int htc_fmtx_switch(int enable)
{
	int err = 0;

	#ifdef FMTX_DBG
	pr_err("--- %s(enable=%d, pin=%d) ---\n", __func__,
			enable, htc_fmtx_switch_pin);
	#endif

	if (htc_fmtx_switch_pin < 0) {
		pr_err("err: fmtx switch pin config error\n");
		return -EINVAL;
	}

	if (enable)
		pmic_gpio_htc_fmtx_output.output_value = 1;
	else
		pmic_gpio_htc_fmtx_output.output_value = 0;

	err = pm8058_gpio_config(htc_fmtx_switch_pin,
						&pmic_gpio_htc_fmtx_output);
	if (err)
		pr_err("err: fmtx switch fail, err=%d\n", err);

	return err;
}

static int htc_fmtx_set_power(void *data, bool blocked)
{
	if (!blocked)
			htc_fmtx_switch(FMTX_ON);
	else
			htc_fmtx_switch(FMTX_OFF);

	return 0;
}

static struct rfkill_ops htc_fmtx_rfkill_ops = {
	.set_block = htc_fmtx_set_power,
};

static int htc_fmtx_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true; /* off */
	struct htc_fmtx_platform_data *pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		htc_fmtx_switch_pin = -1;
		pr_err("err: null fmtx pdata\n");
		return -EINVAL;
	} else {
		htc_fmtx_switch_pin = pdata->switch_pin;
	}

	htc_fmtx_set_power(NULL, default_state);

	htc_fmtx_rfk = rfkill_alloc(htc_fmtx_name, &pdev->dev, RFKILL_TYPE_FMTX,
						 &htc_fmtx_rfkill_ops, NULL);
	if (!htc_fmtx_rfk) {
		rc = -ENOMEM;
		goto err_htc_fmtx_rfkill_reset;
	}

	rfkill_set_states(htc_fmtx_rfk, default_state, false);

	/* userspace cannot take exclusive control */
	rc = rfkill_register(htc_fmtx_rfk);
	if (rc)
		goto err_htc_fmtx_rfkill_reg;

	return 0;

err_htc_fmtx_rfkill_reg:
	rfkill_destroy(htc_fmtx_rfk);
err_htc_fmtx_rfkill_reset:
	return rc;
}

static int htc_fmtx_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(htc_fmtx_rfk);
	rfkill_destroy(htc_fmtx_rfk);

	return 0;
}

static struct platform_driver htc_fmtx_rfkill_driver = {
	.probe = htc_fmtx_rfkill_probe,
	.remove = htc_fmtx_rfkill_remove,
	.driver = {
		.name = "htc_fmtx_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init htc_fmtx_rfkill_init(void)
{
	return platform_driver_register(&htc_fmtx_rfkill_driver);
}

static void __exit htc_fmtx_rfkill_exit(void)
{
	platform_driver_unregister(&htc_fmtx_rfkill_driver);
}

module_init(htc_fmtx_rfkill_init);
module_exit(htc_fmtx_rfkill_exit);
MODULE_DESCRIPTION("htc fmtx rfkill");
MODULE_AUTHOR("assd bt <assd_bt@htc.com>");
MODULE_LICENSE("GPL");
