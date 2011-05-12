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
#include <linux/rtc.h>
#include <linux/slab.h>

#include <linux/mfd/pmic8058.h>

#include <mach/msm_rpcrouter.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_pmic.h>

#define DRIVER_NAME "HS_PMIC"

static struct workqueue_struct *detect_wq;
static void detect_35mm_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(detect_35mm_work, detect_35mm_do_work);

static struct htc_35mm_pmic_info *hi;

static struct msm_rpc_endpoint *endpoint_adc;
static struct msm_rpc_endpoint *endpoint_current;

static struct hs_pmic_current_threshold current_threshold_lut[] = {
	{
		.adc_max = 14909, /* 0x3A3D */
		.adc_min = 0, /* 0x0000 */
		.current_uA = 500,
	},
	{
		.adc_max = 29825, /* 0x7481 */
		.adc_min = 14910, /* 0x3A3E */
		.current_uA = 600,
	},
	{
		.adc_max = 65535, /* 0xFFFF */
		.adc_min = 29826, /* 0x7482 */
		.current_uA = 500,
	},
};

static int hs_pmic_hpin_state(void)
{
	HS_DBG();

	return gpio_get_value(hi->pdata.hpin_gpio);
}

static int hs_pmic_remote_threshold(uint32_t adc)
{
	int i = 0;
	int ret = 0;
	uint32_t status;
	struct hs_pmic_rpc_request req;
	struct hs_pmic_rpc_reply rep;

	HS_DBG();

	if (!(hi->pdata.driver_flag & DRIVER_HS_PMIC_DYNAMIC_THRESHOLD))
		return 0;

	req.hs_controller = cpu_to_be32(hi->pdata.hs_controller);
	req.hs_switch = cpu_to_be32(hi->pdata.hs_switch);
	req.current_uA = cpu_to_be32(HS_PMIC_HTC_CURRENT_THRESHOLD);

	for (i = 0; i < ARRAY_SIZE(current_threshold_lut); i++) {
		if (adc >= current_threshold_lut[i].adc_min &&
		    adc <= current_threshold_lut[i].adc_max)
			req.current_uA = cpu_to_be32(current_threshold_lut[i].
						     current_uA);
	}

	ret = msm_rpc_call_reply(endpoint_current,
				 HS_PMIC_RPC_CLIENT_PROC_THRESHOLD,
				 &req, sizeof(req), &rep, sizeof(rep),
				 HS_RPC_TIMEOUT);

	if (ret < 0) {
		HS_ERR("Failed to send remote threshold RPC");
		return 0;
	} else {
		status = be32_to_cpu(rep.status);
		if (status != HS_PMIC_RPC_ERR_SUCCESS) {
			HS_ERR("Failed to set remote threshold");
			return 0;
		}
	}

	HS_LOG("Set remote threshold (%u, %u, %u)", hi->pdata.hs_controller,
	       hi->pdata.hs_switch, be32_to_cpu(req.current_uA));

	return 1;
}

static int hs_pmic_remote_adc(int *adc)
{
	int ret = 0;
	struct rpc_request_hdr req;
	struct hs_rpc_client_rep_adc rep;

	HS_DBG();

	ret = msm_rpc_call_reply(endpoint_adc, HS_RPC_CLIENT_PROC_ADC,
				 &req, sizeof(req), &rep, sizeof(rep),
				 HS_RPC_TIMEOUT);
	if (ret < 0) {
		HS_ERR("Failed to read remote ADC");
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

	HS_DBG();

	ret = hs_pmic_remote_adc(&adc);

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_DYNAMIC_THRESHOLD)
		hs_pmic_remote_threshold((unsigned int) adc);


	if (adc >= hi->pdata.adc_mic_bias[0] &&
	    adc <= hi->pdata.adc_mic_bias[1])
		ret = HEADSET_MIC;
	else if (adc < hi->pdata.adc_mic_bias[0])
		ret = HEADSET_NO_MIC;
	else
		ret = HEADSET_UNKNOWN_MIC;

	return ret;
}

static int hs_pmic_adc_to_keycode(int adc)
{
	int key_code = HS_MGR_KEY_INVALID;

	HS_DBG();

	if (!hi->pdata.adc_remote[5])
		return HS_MGR_KEY_INVALID;

	if (adc >= hi->pdata.adc_remote[0] &&
	    adc <= hi->pdata.adc_remote[1])
		key_code = HS_MGR_KEY_PLAY;
	else if (adc >= hi->pdata.adc_remote[2] &&
		 adc <= hi->pdata.adc_remote[3])
		key_code = HS_MGR_KEY_BACKWARD;
	else if (adc >= hi->pdata.adc_remote[4] &&
		 adc <= hi->pdata.adc_remote[5])
		key_code = HS_MGR_KEY_FORWARD;
	else if (adc >= hi->pdata.adc_mic)
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

	HS_DBG();

	if (key_code != HS_MGR_KEY_INVALID)
		hs_notify_key_event(key_code);
}

static void hs_pmic_key_enable(int enable)
{
	HS_DBG();

	if (hi->pdata.key_enable_gpio)
		gpio_set_value(hi->pdata.key_enable_gpio, enable);
}

static void detect_35mm_do_work(struct work_struct *work)
{
	int insert = 0;

	HS_DBG();

	insert = gpio_get_value(hi->pdata.hpin_gpio) ? 0 : 1;
	hs_notify_plug_event(insert);
}

static irqreturn_t htc_35mm_pmic_irq(int irq, void *data)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	hs_notify_hpin_irq();

	HS_DBG();

	hi->hpin_irq_type ^= irq_mask;
	set_irq_type(hi->pdata.hpin_irq, hi->hpin_irq_type);

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(detect_wq, &detect_35mm_work, hi->hpin_debounce);

	return IRQ_HANDLED;
}

static void hs_pmic_register(void)
{
	struct headset_notifier notifier;

	if (hi->pdata.hpin_gpio) {
		notifier.id = HEADSET_REG_HPIN_GPIO;
		notifier.func = hs_pmic_hpin_state;
		headset_notifier_register(&notifier);
	}

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_RPC_KEY) {
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

	if (hi->pdata.key_enable_gpio) {
		notifier.id = HEADSET_REG_KEY_ENABLE;
		notifier.func = hs_pmic_key_enable;
		headset_notifier_register(&notifier);
	}
}

static int htc_headset_pmic_probe(struct platform_device *pdev)
{
	int ret = 0;
	uint32_t vers = 0;
	struct htc_headset_pmic_platform_data *pdata = pdev->dev.platform_data;

	HS_LOG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_35mm_pmic_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	hi->pdata.driver_flag = pdata->driver_flag;
	hi->pdata.hpin_gpio = pdata->hpin_gpio;
	hi->pdata.hpin_irq = pdata->hpin_irq;
	hi->pdata.key_enable_gpio = pdata->key_enable_gpio;
	hi->pdata.hs_controller = pdata->hs_controller;
	hi->pdata.hs_switch = pdata->hs_switch;
	hi->pdata.adc_mic = pdata->adc_mic;

	if (!hi->pdata.adc_mic)
		hi->pdata.adc_mic = HS_DEF_MIC_ADC_16_BIT_MIN;

	if (pdata->adc_mic_bias[0] && pdata->adc_mic_bias[1]) {
		memcpy(hi->pdata.adc_mic_bias, pdata->adc_mic_bias,
		       sizeof(hi->pdata.adc_mic_bias));
		hi->pdata.adc_mic = hi->pdata.adc_mic_bias[0];
	} else {
		hi->pdata.adc_mic_bias[0] = hi->pdata.adc_mic;
		hi->pdata.adc_mic_bias[1] = HS_DEF_MIC_ADC_16_BIT_MAX;
	}

	if (pdata->adc_remote[5])
		memcpy(hi->pdata.adc_remote, pdata->adc_remote,
		       sizeof(hi->pdata.adc_remote));

	if (pdata->adc_metrico[0] && pdata->adc_metrico[1])
		memcpy(hi->pdata.adc_metrico, pdata->adc_metrico,
		       sizeof(hi->pdata.adc_metrico));

	hi->hpin_irq_type = IRQF_TRIGGER_LOW;
	hi->hpin_debounce = HS_JIFFIES_ZERO;

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	detect_wq = create_workqueue("detection");
	if (detect_wq  == NULL) {
		ret = -ENOMEM;
		goto err_create_detect_work_queue;
	}

	if (hi->pdata.hpin_irq) {
		ret = request_irq(hi->pdata.hpin_irq, htc_35mm_pmic_irq,
				  hi->hpin_irq_type, "HS_PMIC_DETECT",
				  NULL);
		if (ret < 0) {
			HS_ERR("Failed to request PMIC HPIN IRQ (0x%X)", ret);
			goto err_request_detect_irq;
		}

		ret = set_irq_wake(hi->pdata.hpin_irq, 1);
		if (ret < 0)
			HS_ERR("Failed to set PMIC HPIN IRQ wake");
	}

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_RPC_KEY) {
		/* Register ADC RPC client */
		endpoint_adc = msm_rpc_connect(HS_RPC_CLIENT_PROG,
					       HS_RPC_CLIENT_VERS, 0);
		if (IS_ERR(endpoint_adc)) {
			hi->pdata.driver_flag &= ~DRIVER_HS_PMIC_RPC_KEY;
			HS_LOG("Failed to register ADC RPC client");
		} else
			HS_LOG("Register ADC RPC client successfully");
	}

	if (hi->pdata.driver_flag & DRIVER_HS_PMIC_DYNAMIC_THRESHOLD) {
		/* Register threshold RPC client */
		vers = HS_PMIC_RPC_CLIENT_VERS_3_1;
		endpoint_current = msm_rpc_connect_compatible(
				   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		if (!endpoint_current) {
			vers = HS_PMIC_RPC_CLIENT_VERS_2_1;
			endpoint_current = msm_rpc_connect(
					   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		}
		if (!endpoint_current) {
			vers = HS_PMIC_RPC_CLIENT_VERS_1_1;
			endpoint_current = msm_rpc_connect(
					   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		}
		if (!endpoint_current) {
			vers = HS_PMIC_RPC_CLIENT_VERS;
			endpoint_current = msm_rpc_connect(
					   HS_PMIC_RPC_CLIENT_PROG, vers, 0);
		}
		if (IS_ERR(endpoint_current)) {
			hi->pdata.driver_flag &=
				~DRIVER_HS_PMIC_DYNAMIC_THRESHOLD;
			HS_LOG("Failed to register threshold RPC client");
		} else
			HS_LOG("Register threshold RPC client successfully"
			       " (0x%X)", vers);
	}

	hs_pmic_register();
	hs_notify_driver_ready(DRIVER_NAME);

	HS_LOG("--------------------");

	return 0;

err_request_detect_irq:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_pmic_remove(struct platform_device *pdev)
{
	if (hi->pdata.hpin_irq)
		free_irq(hi->pdata.hpin_irq, 0);

	destroy_workqueue(detect_wq);
	wake_lock_destroy(&hi->hs_wake_lock);

	kfree(hi);

	return 0;
}

static struct platform_driver htc_headset_pmic_driver = {
	.probe	= htc_headset_pmic_probe,
	.remove	= htc_headset_pmic_remove,
	.driver	= {
		.name	= "HTC_HEADSET_PMIC",
		.owner	= THIS_MODULE,
	},
};

static int __init htc_headset_pmic_init(void)
{
	return platform_driver_register(&htc_headset_pmic_driver);
}

static void __exit htc_headset_pmic_exit(void)
{
	platform_driver_unregister(&htc_headset_pmic_driver);
}

module_init(htc_headset_pmic_init);
module_exit(htc_headset_pmic_exit);

MODULE_DESCRIPTION("HTC PMIC headset driver");
MODULE_LICENSE("GPL");
