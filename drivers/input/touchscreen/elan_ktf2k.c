/* drivers/input/touchscreen/elan_ktf2k.c - ELAN KTF2000 touchscreen driver
 *
 * Copyright (C) 2010 HTC Corporation.
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

/* #define DEBUG */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/elan_ktf2k.h>
#include <linux/device.h>
#include <linux/jiffies.h>

#define ELAN_TS_FUZZ 		0
#define ELAN_TS_FLAT 		0
#define IDX_PACKET_SIZE		18

#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54

#define HELLO_PKT			0x55
#define NORMAL_PKT			0x5D

#define RPT_LOCK_PKT		0x56
#define RPT_UNLOCK_PKT		0xA6

#define RESET_PKT			0x77
#define CALIB_PKT			0xA8

#define IDX_NUM				0x01
#define IDX_FINGER			0x02

struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	int (*power)(int on);
	struct early_suspend early_suspend;
	int intr_gpio;
	int fw_ver;
};

static struct elan_ktf2k_ts_data *private_ts;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf2k_ts_late_resume(struct early_suspend *h);
#endif

static ssize_t elan_ktf2k_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	printk(KERN_DEBUG "GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF3K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		status = gpio_get_value(ts->intr_gpio);
		dev_dbg(&client->dev, "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	dev_dbg(&client->dev, "%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
		    buf[0] != CMD_S_PKT)
			return -EINVAL;
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[4] = { 0 };

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		dev_err(&client->dev, "%s: failed!\n", __func__);
		return -EINVAL;
	}

	rc = i2c_master_recv(client, buf_recv, 4);
	if (rc != 4) {
		dev_err(&client->dev,
			"%s: get hello packet failed!, rc = %d\n",
			__func__, rc);
		return rc;
	} else {
		int i;

		dev_dbg(&client->dev,
			"dump hello packet: %0x, %0x, %0x, %0x\n",
			buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
		for (i = 0; i < 4; i++)
			if (buf_recv[i] != HELLO_PKT)
				return -EINVAL;
	}

	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t buf_recv[4] = {0};

	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	printk(KERN_INFO "%s: firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);

	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc;

	rc = __hello_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev, "%s: hello packet got.\n", __func__);

	rc = __fw_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev, "%s: firmware checking done.\n", __func__);

hand_shake_failed:
	return rc;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "power state = %s\n",
		power_state == PWR_STATE_DEEP_SLEEP ?
		"Deep Sleep" : "Normal/Idle");

	return power_state;
}

#if 0
static int elan_ktf2k_ts_calibration(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CALIB_PKT, CALIB_PKT, CALIB_PKT, CALIB_PKT};

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	dev_dbg(&client->dev,
		"dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_soft_reset(struct i2c_client *client)
{
	uint8_t cmd[] = {RESET_PKT, RESET_PKT, RESET_PKT, RESET_PKT};

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	dev_dbg(&client->dev,
		"dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}
#endif

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf)
{
	/* struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client); */
	int rc, bytes_to_recv = IDX_PACKET_SIZE;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	rc = i2c_master_recv(client, buf, bytes_to_recv);

	if (rc != bytes_to_recv) {
		dev_err(&client->dev,
			"%s: i2c_master_recv error?! \n", __func__);
#if 0
		/* software reset */
		elan_ktf2k_ts_soft_reset(client);

		/* re-initial */
		rc = elan_ktf2k_ts_setup(client);
		if (gpio_get_value(ts->intr_gpio) == 0)
			queue_work(ts->elan_wq, &ts->work);
#endif
		return -EINVAL;
	}

	return rc;
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	static unsigned report_time;
	unsigned report_time2;
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint8_t num, fbits;

	switch (buf[0]) {
	case NORMAL_PKT:
		num = buf[IDX_NUM] & 0x7;
		fbits = buf[IDX_NUM] >> 3;
		if (num == 0) {
			dev_dbg(&client->dev, "no press\n");
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
#else
			input_report_abs(idev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(idev, ABS_MT_POSITION, BIT(31));
#endif
		} else {
			uint8_t i, idx;
			uint8_t reported = 0;

			dev_dbg(&client->dev, "%d fingers\n", num);
			for (i = 0, idx = IDX_FINGER; i < 5; i++, idx += 3) {
				if (!((fbits >> i) & 0x1))
					continue;
				elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
				dev_dbg(&client->dev, "finger %d: x = %d, y = %d\n",
					i, x, y);
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 255);
				input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 8);
				input_report_abs(idev, ABS_MT_POSITION_X, x);
				input_report_abs(idev, ABS_MT_POSITION_Y, y);
				input_mt_sync(idev);
#else
				input_report_abs(idev, ABS_MT_AMPLITUDE, 255 << 16 | 8);
				input_report_abs(idev, ABS_MT_POSITION,
					(reported == num - 1 ? BIT(31) : 0) | x << 16 | y);
#endif
				reported++;
			}
		}
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
		input_sync(ts->input_dev);
#endif
		break;
	default:
		dev_err(&client->dev,
			"%s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	}

	report_time2 = jiffies;
	dev_dbg(&client->dev, "report time = %d\n",
		jiffies_to_msecs(report_time2 - report_time));
	report_time = report_time2;

	return;
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
		container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[IDX_PACKET_SIZE] = { 0 };

	/* this means that we have already serviced it */
	if (gpio_get_value(ts->intr_gpio))
		return;

	rc = elan_ktf2k_ts_recv_data(ts->client, buf);
	if (rc < 0)
		return;

	elan_ktf2k_ts_report_data(ts->client, buf);
	enable_irq(ts->client->irq);

	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "%s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
			IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "%s: request_irq %d failed\n",
				__func__, client->irq);

	return err;
}

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: allocate elan_ktf2k_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		printk(KERN_ERR "%s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (likely(pdata != NULL)) {
		ts->power = pdata->power;
		ts->intr_gpio = pdata->intr_gpio;
	}

	if (ts->power)
		ts->power(1);

	err = elan_ktf2k_ts_setup(client);
	if (err < 0) {
		printk(KERN_INFO "No Elan chip inside\n");
		err = -ENODEV;
		goto err_detect_failed;
	}

/*
	if (pdata) {
		while (pdata->version > ts->fw_ver) {
			printk(KERN_INFO "%s: old tp detected, "
					"panel version = 0x%x\n",
					__func__, ts->fw_ver);
			pdata++;
		}
	}
*/

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		pdata->abs_x_min,  pdata->abs_x_max,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		pdata->abs_y_min,  pdata->abs_y_max,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
		0, 30,
		ELAN_TS_FUZZ, ELAN_TS_FLAT);
#ifndef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
		0, ((255 << 16) | 30), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, (BIT(31) | (pdata->abs_x_max << 16) | pdata->abs_y_max), 0, 0);
#endif

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf2k_ts_register_interrupt(ts->client);

	/* checking the interrupt to avoid missing any interrupt */
	if (gpio_get_value(ts->intr_gpio) == 0) {
		printk(KERN_INFO "%s: handle missed interrupt\n", __func__);
		elan_ktf2k_ts_irq_handler(client->irq, ts);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = elan_ktf2k_ts_early_suspend;
	ts->early_suspend.resume = elan_ktf2k_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	private_ts = ts;

	elan_ktf2k_touch_sysfs_init();

	dev_info(&client->dev, "Start touchscreen %s in interrupt mode\n",
		ts->input_dev->name);

	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;

	printk(KERN_INFO "%s: enter\n", __func__);

	disable_irq(client->irq);

	rc = cancel_work_sync(&ts->work);
	if (rc)
		enable_irq(client->irq);

	rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
/*
	rc = elan_ktf2k_ts_get_power_state(client);
	if (rc < 0 || rc != PWR_STATE_DEEP_SLEEP)
		dev_err(&client->dev,
			"%s: put tp into sleep failed, err = %d!\n",
			__func__, rc);
*/

	return 0;
}

static int elan_ktf2k_ts_resume(struct i2c_client *client)
{
	/* struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client); */
	int rc = 0, retry = 5;

	printk(KERN_INFO "%s: enter\n", __func__);

	do {
		rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_NORMAL);
		rc = elan_ktf2k_ts_get_power_state(client);
		if (rc != PWR_STATE_NORMAL)
			printk(KERN_ERR "%s: wake up tp failed! err = %d\n",
				__func__, rc);
		else
			break;
	} while (--retry);

	enable_irq(client->irq);

/*
	if (gpio_get_value(ts->intr_gpio) == 0)
	{
		printk(KERN_INFO "%s: handle missed interrupt\n", __func__);
		elan_ktf2k_ts_irq_handler(client->irq, ts);
	}
*/

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	elan_ktf2k_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void elan_ktf2k_ts_late_resume(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	elan_ktf2k_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	{ ELAN_KTF2K_NAME, 0 },
	{ }
};

static struct i2c_driver ektf2k_ts_driver = {
	.probe		= elan_ktf2k_ts_probe,
	.remove		= elan_ktf2k_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= elan_ktf2k_ts_suspend,
	.resume		= elan_ktf2k_ts_resume,
#endif
	.id_table	= elan_ktf2k_ts_id,
	.driver		= {
		.name = ELAN_KTF2K_NAME,
	},
};

static int __devinit elan_ktf2k_ts_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
	return i2c_add_driver(&ektf2k_ts_driver);
}

static void __exit elan_ktf2k_ts_exit(void)
{
	i2c_del_driver(&ektf2k_ts_driver);
	return;
}

module_init(elan_ktf2k_ts_init);
module_exit(elan_ktf2k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2000 Touchscreen Driver");
MODULE_LICENSE("GPL");
