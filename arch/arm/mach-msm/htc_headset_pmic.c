/*
 *
 * /arch/arm/mach-msm/htc_headset_pmic.c
 *
 * HTC PMIC headset driver.
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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/pmic8058.h>

#include <mach/msm_rpcrouter.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_pmic.h>

#define DRIVER_NAME "HS_PMIC"

static struct workqueue_struct *detect_wq;
static void detect_35mm_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(detect_35mm_work, detect_35mm_do_work);

static struct htc_35mm_pmic_info *pmic_info;

static struct msm_rpc_endpoint *endpoint;

static int hs_pmic_hpin_state(void)
{
	return gpio_get_value(pmic_info->pdata.hpin_gpio);
}

static int hs_pmic_remote_adc(int *adc)
{
	int ret = 0;
	struct rpc_request_hdr req;
	struct hs_rpc_client_rep_adc rep;

	HS_DBG_LOG();

	ret = msm_rpc_call_reply(endpoint, HS_RPC_CLIENT_PROC_ADC,
				 &req, sizeof(req), &rep, sizeof(rep),
				 5 * HZ);
	if (ret < 0) {
		HS_LOG("Failed to read remote ADC");
		return 0;
	}

	*adc = (int) be32_to_cpu(rep.adc);
	HS_LOG("Remote ADC %d (0x%X)", *adc, *adc);

	return 1;
}

static int hs_pmic_mic_status(void)
{
	int ret = 0;
	int adc = 0;

	HS_DBG_LOG();

	ret = hs_pmic_remote_adc(&adc);

	if (adc >= pmic_info->pdata.adc_mic)
		ret = HEADSET_MIC;
	else
		ret = HEADSET_NO_MIC;

	return ret;
}

static int hs_pmic_adc_to_keycode(int adc)
{
	int key_code = HS_MGR_KEY_INVALID;

	if (!pmic_info->pdata.adc_remote[5])
		return HS_MGR_KEY_INVALID;

	if (adc >= pmic_info->pdata.adc_remote[0] &&
	    adc <= pmic_info->pdata.adc_remote[1])
		key_code = HS_MGR_KEY_PLAY;
	else if (adc >= pmic_info->pdata.adc_remote[2] &&
		 adc <= pmic_info->pdata.adc_remote[3])
		key_code = HS_MGR_KEY_BACKWARD;
	else if (adc >= pmic_info->pdata.adc_remote[4] &&
		 adc <= pmic_info->pdata.adc_remote[5])
		key_code = HS_MGR_KEY_FORWARD;
	else if (adc >= pmic_info->pdata.adc_mic)
		key_code = HS_MGR_KEY_NONE;

	if (key_code != HS_MGR_KEY_INVALID)
		HS_LOG("Key code %d", key_code);
	else
		HS_LOG("Unknown key code %d", key_code);

	return key_code;
}

static void hs_pmic_rpc_key(int adc)
{
	int key_code = hs_pmic_adc_to_keycode(adc);

	if (key_code != HS_MGR_KEY_INVALID)
		hs_notify_key_event(key_code);
}

static void hs_pmic_key_enable(int enable)
{
	HS_DBG_LOG();

	if (pmic_info->pdata.key_enable_gpio)
		gpio_set_value(pmic_info->pdata.key_enable_gpio, enable);
}

static void detect_35mm_do_work(struct work_struct *work)
{
	int insert = 0;

	HS_DBG_LOG();

	insert = gpio_get_value(pmic_info->pdata.hpin_gpio) ? 0 : 1;
	hs_notify_plug_event(insert);

	pmic_info->hpin_debounce = (insert) ? HS_JIFFIES_REMOVE :
					      HS_JIFFIES_INSERT;
}

static irqreturn_t htc_35mm_pmic_irq(int irq, void *data)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	hs_notify_hpin_irq();

	HS_DBG_LOG();

	pmic_info->hpin_irq_type ^= irq_mask;
	set_irq_type(pmic_info->pdata.hpin_irq, pmic_info->hpin_irq_type);

	wake_lock_timeout(&pmic_info->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(detect_wq, &detect_35mm_work,
			   pmic_info->hpin_debounce);

	return IRQ_HANDLED;
}

static void hs_pmic_register(void)
{
	struct headset_notifier notifier;

	if (pmic_info->pdata.hpin_gpio) {
		notifier.id = HEADSET_REG_HPIN_GPIO;
		notifier.func = hs_pmic_hpin_state;
		headset_notifier_register(&notifier);
	}

	if (pmic_info->pdata.driver_flag & DRIVER_HS_PMIC_RPC_KEY) {
		notifier.id = HEADSET_REG_REMOTE_ADC;
		notifier.func = hs_pmic_remote_adc;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_RPC_KEY;
		notifier.func = hs_pmic_rpc_key;
		headset_notifier_register(&notifier);

		notifier.id = HEADSET_REG_MIC_STATUS;
		notifier.func = hs_pmic_mic_status;
		headset_notifier_register(&notifier);
	}

	if (pmic_info->pdata.key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_pmic_key_enable;
		headset_notifier_register(&notifier);
	}
}

static int htc_35mm_pmic_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct htc_headset_pmic_platform_data *pdata = pdev->dev.platform_data;

	HS_LOG("++++++++++++++++++++");

	pmic_info = kzalloc(sizeof(struct htc_35mm_pmic_info), GFP_KERNEL);
	if (!pmic_info)
		return -ENOMEM;

	pmic_info->pdata.driver_flag = pdata->driver_flag;
	pmic_info->pdata.hpin_gpio = pdata->hpin_gpio;
	pmic_info->pdata.hpin_irq = pdata->hpin_irq;
	pmic_info->pdata.key_enable_gpio = pdata->key_enable_gpio;
	pmic_info->pdata.adc_mic = pdata->adc_mic;

	if (!pmic_info->pdata.adc_mic)
		pmic_info->pdata.adc_mic = HS_DEF_MIC_ADC_16_BIT;

	if (pdata->adc_remote[5])
		memcpy(pmic_info->pdata.adc_remote, pdata->adc_remote,
		       sizeof(pmic_info->pdata.adc_remote));

	if (pdata->adc_metrico[0] && pdata->adc_metrico[1])
		memcpy(pmic_info->pdata.adc_metrico, pdata->adc_metrico,
		       sizeof(pmic_info->pdata.adc_metrico));

	pmic_info->hpin_irq_type = IRQF_TRIGGER_LOW;
	pmic_info->hpin_debounce = HS_JIFFIES_INSERT;

	wake_lock_init(&pmic_info->hs_wake_lock, WAKE_LOCK_SUSPEND,
		       DRIVER_NAME);

	detect_wq = create_workqueue("detection");
	if (detect_wq  == NULL) {
		ret = -ENOMEM;
		goto err_create_detect_work_queue;
	}

	if (pmic_info->pdata.hpin_irq) {
		ret = request_irq(pmic_info->pdata.hpin_irq, htc_35mm_pmic_irq,
				  pmic_info->hpin_irq_type, "HS_PMIC_DETECT",
				  NULL);
		if (ret < 0) {
			HS_LOG("Failed to request PMIC HPIN IRQ (0x%X)", ret);
			goto err_request_detect_irq;
		}

		ret = set_irq_wake(pmic_info->pdata.hpin_irq, 1);
		if (ret < 0)
			HS_LOG("Failed to set PMIC HPIN IRQ wake");
	}

	if (pmic_info->pdata.driver_flag & DRIVER_HS_PMIC_RPC_KEY) {
		/* Register RPC client */
		endpoint = msm_rpc_connect(HS_RPC_CLIENT_PROG,
					   HS_RPC_CLIENT_VERS, 0);
		if (IS_ERR(endpoint)) {
			HS_LOG("Failed to register RPC client");
			goto err_register_rpc_client;
		}
		HS_LOG("Register RPC client successfully");
	}

	hs_pmic_register();
	hs_notify_driver_ready(DRIVER_NAME);

	HS_LOG("--------------------");

	return 0;

err_register_rpc_client:
	if (pmic_info->pdata.hpin_irq)
		free_irq(pmic_info->pdata.hpin_irq, 0);

err_request_detect_irq:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	kfree(pmic_info);

	return ret;
}

static int htc_35mm_pmic_remove(struct platform_device *pdev)
{
	HS_DBG_LOG();

	if (pmic_info->pdata.hpin_irq)
		free_irq(pmic_info->pdata.hpin_irq, 0);

	destroy_workqueue(detect_wq);
	kfree(pmic_info);

	return 0;
}

static struct platform_driver htc_35mm_pmic_driver = {
	.probe	= htc_35mm_pmic_probe,
	.remove	= htc_35mm_pmic_remove,
	.driver	= {
		.name	= "HTC_HEADSET_PMIC",
		.owner	= THIS_MODULE,
	},
};

static int __init htc_35mm_pmic_init(void)
{
	HS_DBG_LOG();
	return platform_driver_register(&htc_35mm_pmic_driver);
}

static void __exit htc_35mm_pmic_exit(void)
{
	HS_DBG_LOG();
	platform_driver_unregister(&htc_35mm_pmic_driver);
}

module_init(htc_35mm_pmic_init);
module_exit(htc_35mm_pmic_exit);

MODULE_DESCRIPTION("HTC PMIC headset driver");
MODULE_LICENSE("GPL");
