/*
 *
 * /arch/arm/mach-msm/htc_headset_microp.c
 *
 * HTC Micro-P headset driver.
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
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#include <mach/atmega_microp.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_microp.h>

#define DRIVER_NAME "HS_MICROP"

static struct htc_headset_microp_info *hi;

static struct workqueue_struct *detect_wq;
static void detect_microp_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(detect_microp_work, detect_microp_work_func);

static struct workqueue_struct *button_wq;
static void button_microp_work_func(struct work_struct *work);
static DECLARE_WORK(button_microp_work, button_microp_work_func);

static int hs_microp_hpin_state(void)
{
	int ret = 0;
	int gpio_status = 0;
	uint8_t data[3];

	HS_DBG();

	microp_read_gpio_status(data);
	gpio_status = data[0] << 16 | data[1] << 8 | data[2];
	ret = (gpio_status & hi->hpin_gpio_mask) ? 1 : 0;

	return ret;
}

static void hs_microp_key_enable(int enable)
{
	uint8_t addr;
	uint8_t data[3];

	HS_DBG();

	if (hi->pdata.remote_enable_pin) {
		addr = (enable) ? MICROP_I2C_WCMD_GPO_LED_STATUS_EN :
				  MICROP_I2C_WCMD_GPO_LED_STATUS_DIS;
		data[0] = (hi->pdata.remote_enable_pin >> 16) & 0xFF;
		data[1] = (hi->pdata.remote_enable_pin >> 8) & 0xFF;
		data[2] = (hi->pdata.remote_enable_pin >> 0) & 0xFF;
		microp_i2c_write(addr, data, 3);
	}
}

static int headset_microp_enable_interrupt(int interrupt, int enable)
{
	uint8_t addr = 0x00;
	uint8_t data[2];

	HS_DBG();

	addr = (enable) ? MICROP_I2C_WCMD_GPI_INT_CTL_EN :
			  MICROP_I2C_WCMD_GPI_INT_CTL_DIS;

	memset(data, 0x00, sizeof(data));
	data[0] = (uint8_t) (interrupt >> 8);
	data[1] = (uint8_t) interrupt;

	return microp_i2c_write(addr, data, 2);
}

static int headset_microp_enable_button_int(int enable)
{
	int ret = 0;

	HS_DBG();

	if (hi->pdata.remote_int)
		ret = headset_microp_enable_interrupt(hi->pdata.remote_int,
						      enable);

	return ret;
}

static int headset_microp_remote_adc(int *adc)
{
	int ret = 0;
	uint8_t data[2];

	HS_DBG();

	data[0] = 0x00;
	data[1] = hi->pdata.adc_channel;
	ret = microp_read_adc(data);
	if (ret != 0) {
		HS_ERR("Failed to read Micro-P ADC");
		return 0;
	}

	*adc = data[0] << 8 | data[1];
	HS_LOG("Remote ADC %d (0x%X)", *adc, *adc);

	return 1;
}

static int headset_microp_mic_status(void)
{
	int ret = HEADSET_NO_MIC;
	int adc = 0;

	HS_DBG();

	ret = headset_microp_remote_adc(&adc);
	if (!ret) {
		HS_ERR("Failed to read Micro-P remote ADC");
		return HEADSET_NO_MIC;
	}

	if (hi->pdata.adc_metrico[0] && hi->pdata.adc_metrico[1] &&
	    adc >= hi->pdata.adc_metrico[0] && adc <= hi->pdata.adc_metrico[1])
		ret = HEADSET_METRICO; /* For Metrico lab test */
	else if (adc >= HS_DEF_MIC_ADC_10_BIT)
		ret = HEADSET_MIC;
	else
		ret = HEADSET_NO_MIC;

	return ret;
}

static void detect_microp_work_func(struct work_struct *work)
{
	int insert = 0;

	HS_DBG();

	insert = hs_microp_hpin_state() ? 0 : 1;
	hs_notify_plug_event(insert);
}

static void button_microp_work_func(struct work_struct *work)
{
	int ret = 0;
	int keycode = -1;
	uint8_t data[2];

	HS_DBG();

	memset(data, 0x00, sizeof(data));
	ret = microp_i2c_read(MICROP_I2C_RCMD_REMOTE_KEYCODE, data, 2);
	if (ret != 0) {
		HS_ERR("Failed to read Micro-P remote key code");
		return;
	}

	if (!data[1])
		keycode = -1; /* no key code */
	else if (data[1] & 0x80)
		keycode = 0; /* release key code */
	else
		keycode = (int) data[1];

	HS_LOG("Key code %d", keycode);

	hs_notify_key_event(keycode);
}

static irqreturn_t htc_headset_microp_detect_irq(int irq, void *data)
{
	hs_notify_hpin_irq();

	HS_DBG();

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(detect_wq, &detect_microp_work, hi->hpin_debounce);

	return IRQ_HANDLED;
}

static irqreturn_t htc_headset_microp_button_irq(int irq, void *data)
{
	HS_DBG();

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_work(button_wq, &button_microp_work);

	return IRQ_HANDLED;
}

static void hs_microp_register(void)
{
	struct headset_notifier notifier;

	if (hi->pdata.hpin_int) {
		notifier.id = HEADSET_REG_HPIN_GPIO;
		notifier.func = hs_microp_hpin_state;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.adc_channel) {
		notifier.id = HEADSET_REG_REMOTE_ADC;
		notifier.func = headset_microp_remote_adc;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_MIC_STATUS;
		notifier.func = headset_microp_mic_status;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.remote_int) {
		notifier.id = HEADSET_REG_KEY_INT_ENABLE;
		notifier.func = headset_microp_enable_button_int;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.remote_enable_pin) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_microp_key_enable;
		headset_notifier_register(&notifier);
	}
}

static int htc_headset_microp_probe(struct platform_device *pdev)
{
	int i = 0;
	int ret = 0;
	uint8_t data[12];

	struct htc_headset_microp_platform_data *pdata = NULL;

	HS_LOG("++++++++++++++++++++");

	pdata = pdev->dev.platform_data;

	hi = kzalloc(sizeof(struct htc_headset_microp_info), GFP_KERNEL);
	if (!hi) {
		HS_ERR("Failed to allocate memory for headset info");
		return -ENOMEM;
	}

	hi->pdata.hpin_int = pdata->hpin_int;
	hi->pdata.hpin_irq = pdata->hpin_irq;
	if (pdata->hpin_mask[0] || pdata->hpin_mask[1] || pdata->hpin_mask[2])
		memcpy(hi->pdata.hpin_mask, pdata->hpin_mask,
		       sizeof(hi->pdata.hpin_mask));

	hi->pdata.remote_int = pdata->remote_int;
	hi->pdata.remote_irq = pdata->remote_irq;
	hi->pdata.remote_enable_pin = pdata->remote_enable_pin;
	hi->pdata.adc_channel = pdata->adc_channel;

	if (pdata->adc_remote[5])
		memcpy(hi->pdata.adc_remote, pdata->adc_remote,
		       sizeof(hi->pdata.adc_remote));

	if (pdata->adc_metrico[0] && pdata->adc_metrico[1])
		memcpy(hi->pdata.adc_metrico, pdata->adc_metrico,
		       sizeof(hi->pdata.adc_metrico));

	hi->hpin_debounce = HS_JIFFIES_ZERO;

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	if (hi->pdata.hpin_int) {
		hi->hpin_gpio_mask = pdata->hpin_mask[0] << 16 |
				     pdata->hpin_mask[1] << 8 |
				     pdata->hpin_mask[2];
	}

	detect_wq = create_workqueue("detect");
	if (detect_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	button_wq = create_workqueue("button");
	if (button_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create button workqueue");
		goto err_create_button_work_queue;
	}

	if (hi->pdata.hpin_int) {
		ret = headset_microp_enable_interrupt(hi->pdata.hpin_int, 1);
		if (ret != 0) {
			HS_ERR("Failed to enable Micro-P HPIN interrupt");
			goto err_enable_microp_hpin_interrupt;
		}
	}

	if (hi->pdata.hpin_irq) {
		ret = request_irq(hi->pdata.hpin_irq,
				  htc_headset_microp_detect_irq,
				  IRQF_TRIGGER_NONE, "HS_MICROP_DETECT", NULL);
		if (ret < 0) {
			ret = -EINVAL;
			HS_ERR("Failed to request Micro-P HPIN IRQ (ERR %d)",
				ret);
			goto err_request_microp_detect_irq;
		}
		ret = set_irq_wake(hi->pdata.hpin_irq, 1);
		if (ret < 0)
			goto err_set_microp_detect_irq_wake;
	}

	if (hi->pdata.adc_remote[5]) {
		memset(data, 0x00, sizeof(data));
		for (i = 0; i < 6; i++)
			data[i + 6] = (uint8_t) hi->pdata.adc_remote[i];
		ret = microp_i2c_write(MICROP_I2C_WCMD_REMOTEKEY_TABLE,
				       data, 12);

		if (ret != 0) {
			ret = -EIO;
			HS_ERR("Failed to write Micro-P ADC table");
			goto err_write_microp_adc_table;
		}
	}

	if (hi->pdata.remote_int) {
		ret = headset_microp_enable_interrupt(hi->pdata.remote_int, 0);
		if (ret != 0) {
			HS_ERR("Failed to disable Micro-P remote interrupt");
			goto err_disable_microp_remote_interrupt;
		}
	}

	if (hi->pdata.remote_irq) {
		ret = request_irq(hi->pdata.remote_irq,
				  htc_headset_microp_button_irq,
				  IRQF_TRIGGER_NONE, "HS_MICROP_BUTTON", NULL);
		if (ret < 0) {
			ret = -EINVAL;
			HS_ERR("Failed to request Micro-P button IRQ (ERR %d)",
				ret);
			goto err_request_microp_button_irq;
		}
		ret = set_irq_wake(hi->pdata.remote_irq, 1);
		if (ret < 0)
			goto err_set_microp_button_irq_wake;
	}

	hs_microp_register();
	hs_notify_driver_ready(DRIVER_NAME);

	if (hi->pdata.hpin_int)
		queue_delayed_work(detect_wq, &detect_microp_work,
				   hi->hpin_debounce);

	HS_LOG("--------------------");

	return 0;

err_set_microp_button_irq_wake:
	if (hi->pdata.remote_irq)
		free_irq(hi->pdata.remote_irq, 0);

err_request_microp_button_irq:
	if (hi->pdata.remote_int)
		headset_microp_enable_interrupt(hi->pdata.remote_int, 0);

err_disable_microp_remote_interrupt:
err_write_microp_adc_table:
err_set_microp_detect_irq_wake:
	if (hi->pdata.hpin_irq)
		free_irq(hi->pdata.hpin_irq, 0);

err_request_microp_detect_irq:
	if (hi->pdata.hpin_int)
		headset_microp_enable_interrupt(hi->pdata.hpin_int, 0);

err_enable_microp_hpin_interrupt:
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_microp_remove(struct platform_device *pdev)
{
	if (hi->pdata.remote_irq)
		free_irq(hi->pdata.remote_irq, 0);

	if (hi->pdata.remote_int)
		headset_microp_enable_interrupt(hi->pdata.remote_int, 0);

	if (hi->pdata.hpin_irq)
		free_irq(hi->pdata.hpin_irq, 0);

	if (hi->pdata.hpin_int)
		headset_microp_enable_interrupt(hi->pdata.hpin_int, 0);

	destroy_workqueue(button_wq);
	destroy_workqueue(detect_wq);

	wake_lock_destroy(&hi->hs_wake_lock);

	kfree(hi);

	return 0;
}

static struct platform_driver htc_headset_microp_driver = {
	.probe	= htc_headset_microp_probe,
	.remove	= htc_headset_microp_remove,
	.driver	= {
		.name	= "HTC_HEADSET_MICROP",
		.owner	= THIS_MODULE,
	},
};

static int __init htc_headset_microp_init(void)
{
	return platform_driver_register(&htc_headset_microp_driver);
}

static void __exit htc_headset_microp_exit(void)
{
	platform_driver_unregister(&htc_headset_microp_driver);
}

module_init(htc_headset_microp_init);
module_exit(htc_headset_microp_exit);

MODULE_DESCRIPTION("HTC Micro-P headset driver");
MODULE_LICENSE("GPL");
