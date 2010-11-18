/* drivers/input/touchscreen/synaptics_3k.c - Synaptics 3k serious touch panel driver
 *
 * Copyright (C) 2010 HTC Corporation.
 *
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *syn_wq;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	uint32_t flags;
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	int8_t sensitivity_adjust;
	uint8_t finger_support;
	int (*power)(int on);
	struct early_suspend early_suspend;
	struct page_description page_table[18];
	uint8_t key_pressed;
	int pre_finger_data[2];
	uint8_t debug_log_level;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

static struct synaptics_ts_data *gl_ts;
static const char SYNAPTICSNAME[]	= "Synaptics_3K";
static uint32_t syn_panel_version;

static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s_%#x\n", SYNAPTICSNAME, syn_panel_version);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);

static uint16_t syn_reg_addr;

static ssize_t register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t data;
	struct synaptics_ts_data *ts;
	ts = gl_ts;

	data = i2c_smbus_read_byte_data(ts->client, syn_reg_addr);
	if (data < 0) {
		printk(KERN_WARNING "%s: read fail\n", __func__);
		return ret;
	}
	ret += sprintf(buf, "addr: 0x%X, data: 0x%X\n", syn_reg_addr, data);
	return ret;
}

static ssize_t register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct synaptics_ts_data *ts;
	char buf_tmp[4];
	uint8_t write_da;

	ts = gl_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' &&
		(buf[5] == ':' || buf[5] == '\n')) {
		memcpy(buf_tmp, buf + 2, 3);
		syn_reg_addr = simple_strtol(buf_tmp, NULL, 16);
		printk(KERN_DEBUG "%s: set syn_reg_addr is: 0x%X\n",
						__func__, syn_reg_addr);
		if (buf[0] == 'w' && buf[5] == ':' && buf[9] == '\n') {
			memcpy(buf_tmp, buf + 6, 3);
			write_da = simple_strtol(buf_tmp, NULL, 10);
			printk(KERN_DEBUG "write addr: 0x%X, data: 0x%X\n",
						syn_reg_addr, write_da);
			ret = i2c_smbus_write_byte_data(ts->client,
					syn_reg_addr, write_da);
			if (ret < 0) {
				printk(KERN_ERR "%s: write fail(%d)\n",
								__func__, ret);
			}
		}
	}

	return count;
}

static DEVICE_ATTR(register, 0644, register_show, register_store);

static ssize_t debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;

	return sprintf(buf, "%d\n", ts->debug_log_level);
}

static ssize_t debug_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->debug_log_level = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(debug_level, 0644, debug_level_show, debug_level_store);

static struct kobject *android_touch_kobj;

static int synaptics_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "touch_sysfs_init: sysfs_create_group failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	syn_reg_addr = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	return 0;
}

static void synaptics_touch_sysfs_remove(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	kobject_del(android_touch_kobj);
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret = 0;

	if (ts->sensitivity_adjust) {
		ret = i2c_smbus_write_byte_data(ts->client,
			ts->page_table[2].value + 0x48, ts->sensitivity_adjust); /* Set Sensitivity */
		if (ret < 0)
			printk(KERN_ERR "i2c_smbus_write_byte_data failed for Sensitivity Set\n");
	}

	/* Position Threshold */
	i2c_smbus_write_byte_data(ts->client, ts->page_table[2].value+2, 3);
	i2c_smbus_write_byte_data(ts->client, ts->page_table[2].value+3, 3);

	/* 2D Gesture Enable */
	i2c_smbus_write_byte_data(ts->client, ts->page_table[2].value+10, 0);
	i2c_smbus_write_byte_data(ts->client, ts->page_table[2].value+11, 0);

	/* Configured */
	i2c_smbus_write_byte_data(ts->client, ts->page_table[8].value, 0x80);

	return ret;

}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	struct i2c_msg msg[2];
	uint8_t start_reg;

	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	uint8_t buf[(ts->finger_support * 21 + 11) / 4];

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = ts->page_table[9].value;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0 || ((buf[0] & 0x0F))) {
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_work_func: i2c_transfer failed\n");
		else
			printk(KERN_ERR "synaptics_ts_work_func: Status ERROR: %d\n", buf[0] & 0x0F);
		/* reset touch control */
		if (ts->power) {
			ret = ts->power(0);
			if (ret < 0)
				printk(KERN_ERR "synaptics_ts_resume power off failed\n");
			msleep(10);
			ret = ts->power(1);
			if (ret < 0)
				printk(KERN_ERR	"synaptics_ts_resume power on failed\n");
		} else {
			i2c_smbus_write_byte_data(ts->client, ts->page_table[7].value, 0x01);
			msleep(250);
		}
		synaptics_init_panel(ts);
		if (!ts->use_irq)
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		else
			i2c_smbus_write_byte_data(ts->client, ts->page_table[8].value + 1, 4);
	} else {

		int finger_data[ts->finger_support][4];
		int base = (ts->finger_support + 11) / 4;
		int loop_i, loop_j, finger_pressed = 0;
		uint8_t pos_mask = 0x0f;

		if (ts->debug_log_level & 0x1) {
			printk("Touch:");
			for (i = 0; i < sizeof(buf); i++)
				printk(" %2x", buf[i]);
			printk("\n");
		}
		for (loop_i = 0; loop_i < ts->finger_support; loop_i++)
			if (buf[1 + (ts->finger_support + 3) / 4] >> (loop_i * 2) & 0x03)
				finger_pressed |= 1 << loop_i;
		if (finger_pressed == 0) {
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
			if (ts->key_pressed == 1) {
				ts->key_pressed = 2;
				printk(KERN_INFO "E1@%d, %d\n",
					ts->pre_finger_data[0] , ts->pre_finger_data[1]);
			}

			if (ts->debug_log_level & 0x2)
				printk(KERN_INFO "Finger leave\n");
		} else {
		for (loop_i = 0; loop_i < ts->finger_support; loop_i++) {
			uint32_t flip_flag = SYNAPTICS_FLIP_X;
			if (((finger_pressed >> loop_i) & 1) == 1) {
				finger_pressed &= ~(1 << loop_i);
				pos_mask = 0x0f;
				for (loop_j = 0; loop_j < 2; loop_j++) {
					finger_data[loop_i][loop_j]
						= (buf[base+2] & pos_mask) >> (loop_j * 4) |
						(uint16_t)buf[base + loop_j] << 4;
					if (ts->flags & flip_flag)
						finger_data[loop_i][loop_j]
							= ts->max[loop_j] - finger_data[loop_i][loop_j];
					flip_flag <<= 1;
					pos_mask <<= 4;
				}
				finger_data[loop_i][2] = (buf[base+3] >> 4 & 0x0F) + (buf[base+3] & 0x0F);
				finger_data[loop_i][3] = buf[base+4];
				if (ts->flags & SYNAPTICS_SWAP_XY)
					swap(finger_data[loop_i][0], finger_data[loop_i][1]);
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					finger_data[loop_i][3]);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					finger_data[loop_i][2]);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					finger_data[loop_i][0]);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					finger_data[loop_i][1]);
				input_mt_sync(ts->input_dev);
#else
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
					finger_data[loop_i][3] << 16 | finger_data[loop_i][2]);
				input_report_abs(ts->input_dev, ABS_MT_POSITION,
					(finger_pressed == 0) << 31 |
					finger_data[loop_i][0] << 16 | finger_data[loop_i][1]);
#endif
				if (!ts->key_pressed && finger_data[loop_i][3]) {
					ts->key_pressed = 1;
					printk(KERN_INFO "S1@%d, %d\n",
						finger_data[0][0], finger_data[0][1]);
				}
				if (ts->key_pressed == 1) {
					ts->pre_finger_data[0] = finger_data[0][0];
					ts->pre_finger_data[1] = finger_data[0][1];
				}

				if (ts->debug_log_level & 0x2)
					printk(KERN_INFO "Finger %d=> X:%d, Y:%d w:%d, z:%d\n",
						loop_i + 1, finger_data[loop_i][0],
						finger_data[loop_i][1], finger_data[loop_i][2],
						finger_data[loop_i][3]);

			}
			base += 5;
		}
		}
	}
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_sync(ts->input_dev);
#endif

	if (ts->use_irq)
		enable_irq(ts->client->irq);

}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	queue_work(ts->syn_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->syn_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	uint8_t loop_i, loop_j;
	int ret = 0;
	uint16_t max_x, max_y;
	struct synaptics_i2c_rmi_platform_data *pdata;
	uint32_t panel_version;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power)
		ret = ts->power(1);

	for (loop_i = 0; loop_i < 10; loop_i++) {
		if (i2c_smbus_read_byte_data(ts->client, 0x00DD) >= 0)
			break;
		msleep(10);
	}

	if (loop_i == 10) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}

	for (loop_i = 0x00DD, loop_j = 0; loop_i <= 0x00EE; loop_i++) {
		ts->page_table[loop_j].addr = loop_i;
		ts->page_table[loop_j].value = i2c_smbus_read_byte_data(ts->client, loop_i);
/*		printk("[0x%2.2X]%2.2X\n", ts->page_table[loop_j].addr, ts->page_table[loop_j].value); */
		while (ts->page_table[loop_j].value != i2c_smbus_read_byte_data(ts->client, loop_i))
			ts->page_table[loop_j].value = i2c_smbus_read_byte_data(ts->client, loop_i);
		loop_j++;
	}

	panel_version =
		i2c_smbus_read_byte_data(ts->client, ts->page_table[6].value + 3) |
		i2c_smbus_read_byte_data(ts->client, ts->page_table[6].value + 2) << 8;
	printk(KERN_INFO "%s: panel_version: %x\n", __func__, panel_version);
	syn_panel_version = panel_version;

	if (pdata) {
		while (pdata->version > panel_version) {
			printk(KERN_INFO "synaptics_ts_probe: old tp detected, "
					"panel version = %x\n", panel_version);
			pdata++;
		}
		ts->flags = pdata->flags;
		ts->sensitivity_adjust = pdata->sensitivity_adjust;
		ts->finger_support = pdata->finger_support;
		ts->abs_x_min = pdata->abs_x_min;
		ts->abs_x_max = pdata->abs_x_max;
		ts->abs_y_min = pdata->abs_y_min;
		ts->abs_y_max = pdata->abs_y_max;
	}
	ts->max[0] = max_x =
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+6) |
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+7) << 8;

	ts->max[1] = max_y =
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+8) |
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+9) << 8;
	printk(KERN_INFO"max_x: %X, max_y: %X\n", max_x, max_y);

	if (pdata->abs_x_min == pdata->abs_x_max && pdata->abs_y_min == pdata->abs_y_max) {
		ts->abs_x_min = 0;
		ts->abs_x_max = max_x;
		ts->abs_y_min = 0;
		ts->abs_y_max = max_y;
	}
	ret = synaptics_init_panel(ts);
	if (ret < 0) {
		printk(KERN_ERR "synaptics_init_panel failed\n");
		goto err_detect_failed;
	}

	ts->syn_wq = create_singlethread_workqueue("synaptics_wq");
	if (!ts->syn_wq)
		goto err_create_wq_failed;
	INIT_WORK(&ts->work, synaptics_ts_work_func);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	printk(KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	printk(KERN_INFO "input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		ts->abs_x_min, ts->abs_x_max, ts->abs_y_min, ts->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, ts->abs_x_min, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, ts->abs_y_min, ts->abs_y_max, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
#ifndef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
		0, ((255 << 16) | 15), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, ((1 << 31) | (ts->abs_x_max << 16) | ts->abs_y_max), 0, 0);
#endif

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: "
				"Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	gl_ts = ts;

	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler, IRQF_TRIGGER_LOW,
				client->name, ts);
		if (ret == 0) {
			/* enable abs int */
			ret = i2c_smbus_write_byte_data(ts->client, ts->page_table[8].value + 1, 4);
			if (ret)
				free_irq(client->irq, ts);
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	synaptics_touch_sysfs_init();

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	destroy_workqueue(ts->syn_wq);

err_create_wq_failed:
err_detect_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);

	synaptics_touch_sysfs_remove();

	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);

	ts->key_pressed = 0;

	ret = i2c_smbus_write_byte_data(client, ts->page_table[8].value, 0x01); /* sleep */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(client, ts->page_table[8].value, 0x00); /* wake */
	msleep(100);

	synaptics_init_panel(ts);

#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(ts->input_dev);
#else
	input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
	input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_3K_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_3K_NAME,
	},
};

static int __devinit synaptics_ts_init(void)
{
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
