/*
 *
 * /arch/arm/mach-msm/htc_headset_gpio.c
 *
 * HTC GPIO headset driver.
 *
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>

#define DRIVER_NAME "HS_GPIO"

static struct workqueue_struct *detect_wq;
static void detect_gpio_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(detect_gpio_work, detect_gpio_work_func);

static struct workqueue_struct *button_wq;
static void button_gpio_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_gpio_work, button_gpio_work_func);

static struct htc_headset_gpio_info *hi;

static int hs_gpio_hpin_state(void)
{
	HS_DBG();

	return gpio_get_value(hi->pdata.hpin_gpio);
}

void hs_gpio_key_enable(int enable)
{
	HS_DBG();

	if (hi->pdata.key_enable_gpio)
		gpio_set_value(hi->pdata.key_enable_gpio, enable);
}

void hs_gpio_mic_select(int enable)
{
	HS_DBG();

	if (hi->pdata.mic_select_gpio)
		gpio_set_value(hi->pdata.mic_select_gpio, enable);
}

static void detect_gpio_work_func(struct work_struct *work)
{
	int insert = 0;

	HS_DBG();

	insert = gpio_get_value(hi->pdata.hpin_gpio) ? 0 : 1;

	if (hi->headset_state == insert)
		return;

	hi->headset_state = insert;
	hs_notify_plug_event(insert);
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int gpio1 = 0;
	int gpio2 = 0;
	int retry_limit = 10;
	unsigned int irq_type = IRQF_TRIGGER_NONE;

	hs_notify_hpin_irq();

	HS_DBG();

	do {
		gpio1 = gpio_get_value(hi->pdata.hpin_gpio);
		irq_type = gpio1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
		set_irq_type(hi->hpin_irq, irq_type);
		gpio2 = gpio_get_value(hi->pdata.hpin_gpio);
	} while (gpio1 != gpio2 && retry_limit-- > 0);

	HS_DBG("gpio2 = %d (%d retries)", gpio2, (10-retry_limit));

	if ((hi->headset_state == 0) ^ gpio2) {
		wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
		queue_delayed_work(detect_wq, &detect_gpio_work,
				   hi->hpin_debounce);
	}

	return IRQ_HANDLED;
}

static void button_gpio_work_func(struct work_struct *work)
{
	HS_DBG();
	hs_notify_key_irq();
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	HS_DBG();

	hi->key_irq_type ^= irq_mask;
	set_irq_type(hi->key_irq, hi->key_irq_type);

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(button_wq, &button_gpio_work, HS_JIFFIES_ZERO);

	return IRQ_HANDLED;
}

static void hs_gpio_register(void)
{
	struct headset_notifier notifier;

	if (hi->pdata.hpin_gpio) {
		notifier.id = HEADSET_REG_HPIN_GPIO;
		notifier.func = hs_gpio_hpin_state;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.mic_select_gpio) {
		notifier.id = HEADSET_REG_MIC_SELECT;
		notifier.func = hs_gpio_mic_select;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_gpio_key_enable;
		headset_notifier_register(&notifier);
	}
}

static int htc_headset_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct htc_headset_gpio_platform_data *pdata = pdev->dev.platform_data;

	HS_LOG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_headset_gpio_info), GFP_KERNEL);
	if (!hi) {
		HS_ERR("Failed to allocate memory for headset info");
		return -ENOMEM;
	}

	hi->pdata.hpin_gpio = pdata->hpin_gpio;
	hi->pdata.key_gpio = pdata->key_gpio;
	hi->pdata.key_enable_gpio = pdata->key_enable_gpio;
	hi->pdata.mic_select_gpio = pdata->mic_select_gpio;

	hi->hpin_debounce = HS_JIFFIES_ZERO;
	hi->key_irq_type = IRQF_TRIGGER_LOW;
	hi->headset_state = 0;

	detect_wq = create_workqueue("HS_GPIO_DETECT");
	if (detect_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	button_wq = create_workqueue("HS_GPIO_BUTTON");
	if (button_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create button workqueue");
		goto err_create_button_work_queue;
	}

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	if (hi->pdata.hpin_gpio) {
		ret = gpio_request(hi->pdata.hpin_gpio, "HS_GPIO_DETECT");
		if (ret < 0)
			goto err_hpin_gpio_request;

		ret = gpio_direction_input(hi->pdata.hpin_gpio);
		if (ret < 0)
			goto err_hpin_gpio_direction_input;

		ret = gpio_to_irq(hi->pdata.hpin_gpio);
		if (ret < 0)
			goto err_hpin_gpio_to_irq;
		else
			hi->hpin_irq = (unsigned int) ret;

		ret = request_irq(hi->hpin_irq, detect_irq_handler,
				  IRQF_TRIGGER_LOW, "HS_GPIO_DETECT", NULL);
		if (ret < 0)
			goto err_hpin_request_irq;

		ret = set_irq_wake(hi->hpin_irq, 1);
		if (ret < 0)
			goto err_hpin_set_irq_wake;
	}

	if (hi->pdata.key_gpio) {
		ret = gpio_request(hi->pdata.key_gpio, "HS_GPIO_BUTTON");
		if (ret < 0)
			goto err_key_gpio_request;

		ret = gpio_direction_input(hi->pdata.key_gpio);
		if (ret < 0)
			goto err_key_gpio_direction_input;

		ret = gpio_to_irq(hi->pdata.key_gpio);
		if (ret < 0)
			goto err_key_gpio_to_irq;
		else
			hi->key_irq = (unsigned int) ret;

		ret = request_irq(hi->key_irq, button_irq_handler,
				  hi->key_irq_type, "HS_GPIO_BUTTON", NULL);
		if (ret < 0)
			goto err_key_request_irq;

		ret = set_irq_wake(hi->key_irq, 1);
		if (ret < 0)
			goto err_key_set_irq_wake;
	}

	hs_gpio_register();
	hs_notify_driver_ready(DRIVER_NAME);

	HS_LOG("--------------------");

	return 0;

err_key_set_irq_wake:
	if (hi->pdata.key_gpio)
		free_irq(hi->key_irq, 0);

err_key_request_irq:
err_key_gpio_to_irq:
err_key_gpio_direction_input:
	if (hi->pdata.key_gpio)
		gpio_free(hi->pdata.key_gpio);

err_key_gpio_request:

err_hpin_set_irq_wake:
	if (hi->pdata.hpin_gpio)
		free_irq(hi->hpin_irq, 0);

err_hpin_request_irq:
err_hpin_gpio_to_irq:
err_hpin_gpio_direction_input:
	if (hi->pdata.hpin_gpio)
		gpio_free(hi->pdata.hpin_gpio);

err_hpin_gpio_request:
	wake_lock_destroy(&hi->hs_wake_lock);
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_gpio_remove(struct platform_device *pdev)
{
	if (hi->pdata.key_gpio) {
		free_irq(hi->key_irq, 0);
		gpio_free(hi->pdata.key_gpio);
	}

	if (hi->pdata.hpin_gpio) {
		free_irq(hi->hpin_irq, 0);
		gpio_free(hi->pdata.hpin_gpio);
	}

	wake_lock_destroy(&hi->hs_wake_lock);
	destroy_workqueue(button_wq);
	destroy_workqueue(detect_wq);

	kfree(hi);

	return 0;
}

static struct platform_driver htc_headset_gpio_driver = {
	.probe		= htc_headset_gpio_probe,
	.remove		= htc_headset_gpio_remove,
	.driver		= {
		.name		= "HTC_HEADSET_GPIO",
		.owner		= THIS_MODULE,
	},
};

static int __init htc_headset_gpio_init(void)
{
	return platform_driver_register(&htc_headset_gpio_driver);
}

static void __exit htc_headset_gpio_exit(void)
{
	platform_driver_unregister(&htc_headset_gpio_driver);
}

module_init(htc_headset_gpio_init);
module_exit(htc_headset_gpio_exit);

MODULE_DESCRIPTION("HTC GPIO headset driver");
MODULE_LICENSE("GPL");
