/* drivers/input/touchscreen/cy8c_tma_ts.c
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

#include <linux/cy8c_tma_ts.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#define CY8C_I2C_RETRY_TIMES 10

struct cy8c_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *cy8c_wq;
	int use_irq;
	struct hrtimer timer;
	struct work_struct work;
	uint16_t version;
	uint8_t id;
	uint16_t intr;
	int (*power) (int on);
	struct early_suspend early_suspend;
	uint8_t debug_log_level;
	uint8_t orient;
	uint8_t diag_command;
	uint8_t first_pressed;
	int pre_finger_data[2];
	uint8_t x_channel;
	uint8_t y_channel;
};

static struct cy8c_ts_data *private_ts;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cy8c_ts_early_suspend(struct early_suspend *h);
static void cy8c_ts_late_resume(struct early_suspend *h);
#endif

static int cy8c_init_panel(struct cy8c_ts_data *ts);

int i2c_cy8c_read(struct i2c_client *client, uint8_t addr, uint8_t *data, uint8_t length)
{
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < CY8C_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(10);
	}
	if (retry == CY8C_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_read_block retry over %d\n",
			CY8C_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

int i2c_cy8c_write(struct i2c_client *client, uint8_t addr, uint8_t *data, uint8_t length)
{
	int retry, loop_i;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = addr;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < CY8C_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry == CY8C_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_write_block retry over %d\n",
			CY8C_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

int i2c_cy8c_write_byte_data(struct i2c_client *client, uint8_t addr, uint8_t value)
{
	return i2c_cy8c_write(client, addr, &value, 1);;
}

static int cy8c_data_toggle(struct cy8c_ts_data *ts)
{
	uint8_t buf;
	i2c_cy8c_read(ts->client, 0x00, &buf, 1);
	return i2c_cy8c_write_byte_data(ts->client, 0x00, buf ^= BIT(7));
}

static ssize_t cy8c_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cy8c_ts_data *ts_data;
	ts_data = private_ts;
	sprintf(buf, "%s_x%4.4X_%2.2X\n", CYPRESS_TMA_NAME,
		ts_data->version, ts_data->id);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, cy8c_vendor_show, NULL);

static uint8_t cy8c_reg_addr;

static ssize_t cy8c_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t ptr[1] = { 0 };
	struct cy8c_ts_data *ts_data;
	ts_data = private_ts;
	if (i2c_cy8c_read(ts_data->client, cy8c_reg_addr, ptr, 1) < 0) {
		printk(KERN_WARNING "%s: read fail\n", __func__);
		return ret;
	}
	ret += sprintf(buf, "addr: %d, data: %d\n", cy8c_reg_addr, ptr[0]);
	return ret;
}

static ssize_t cy8c_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct cy8c_ts_data *ts_data;
	char buf_tmp[4];
	uint8_t write_da;

	ts_data = private_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' &&
		(buf[5] == ':' || buf[5] == '\n')) {
		memcpy(buf_tmp, buf + 2, 3);
		printk(KERN_DEBUG "%s: set cy8c_reg_addr is: %d\n",
						__func__, cy8c_reg_addr);
		if (buf[0] == 'w' && buf[5] == ':' && buf[9] == '\n') {
			memcpy(buf_tmp, buf + 6, 3);
			write_da = simple_strtol(buf_tmp, NULL, 10);
			printk(KERN_DEBUG "write addr: 0x%X, data: 0x%X\n",
						cy8c_reg_addr, write_da);
			ret = i2c_cy8c_write_byte_data(ts_data->client,
						cy8c_reg_addr, write_da);
			if (ret < 0) {
				printk(KERN_ERR "%s: write fail(%d)\n",
								__func__, ret);
			}
		}
	}

	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO),
	cy8c_register_show, cy8c_register_store);


static ssize_t cy8c_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy8c_ts_data *ts_data;
	size_t count = 0;
	ts_data = private_ts;

	count += sprintf(buf, "%d\n", ts_data->debug_log_level);

	return count;
}

static ssize_t cy8c_debug_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cy8c_ts_data *ts_data;
	ts_data = private_ts;
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts_data->debug_log_level = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
	cy8c_debug_level_show, cy8c_debug_level_dump);

static ssize_t cy8c_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy8c_ts_data *ts_data;
	size_t count = 0;
	uint8_t data[250];
	uint8_t loop_i;
	uint16_t num_nodes;
	ts_data = private_ts;
	memset(data, 0x0, sizeof(data));
	num_nodes = ts_data->x_channel * ts_data->y_channel;

	if (ts_data->diag_command < 4 || ts_data->diag_command > 7)
		return count;

	disable_irq(ts_data->client->irq);
	i2c_cy8c_read(ts_data->client, 0x00, &data[0], 1);
	i2c_cy8c_write_byte_data(ts_data->client, 0x00,
		(data[0] & 0x8F) | (ts_data->diag_command << 4));
	msleep(80);
	for (loop_i = 0; loop_i < 20; loop_i++) {
		if (!gpio_get_value(ts_data->intr))
			break;
		msleep(20);
	}
	printk("[%d] change mode to %d\n", loop_i, ts_data->diag_command);
	count += sprintf(buf, "Channel: %d * %d\n", ts_data->x_channel, ts_data->y_channel);

	i2c_cy8c_read(ts_data->client, 0x00, &data[0], 7 + num_nodes);
	if ((data[1] & 0x10) == 0x10) {
		cy8c_init_panel(ts_data);
		i2c_cy8c_read(ts_data->client, 0x00, &data[0], 7 + num_nodes);
	}
	if (ts_data->diag_command == 6 && (data[1] & 0x40) == 0x40) {
		for (loop_i = 0; loop_i < 10; loop_i++) {
			i2c_cy8c_write_byte_data(ts_data->client, 0x00,
				((data[0] ^= BIT(7)) & 0x8F) | (6 << 4));
			msleep(40);

			i2c_cy8c_read(ts_data->client, 0x00, &data[0], 2);
			if (!(data[1] & 0x40))
				break;
			msleep(10);
		}
		i2c_cy8c_read(ts_data->client, 0x00, &data[0], 7 + num_nodes);
	}
	for (loop_i = 7; loop_i < 7 + num_nodes; loop_i++) {
		count += sprintf(buf + count, "%5d", data[loop_i]);
		if ((loop_i - 6) % ts_data->y_channel == 0)
			count += sprintf(buf + count, "\n");
	}
	count += sprintf(buf + count, "\n");
	i2c_cy8c_read(ts_data->client, 0x00, &data[0], 1);
	i2c_cy8c_write_byte_data(ts_data->client, 0x00, ((data[0] ^= BIT(7)) & 0x8F));
	msleep(40);
	enable_irq(ts_data->client->irq);

	return count;
}

static ssize_t cy8c_diag_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cy8c_ts_data *ts_data;
	ts_data = private_ts;

	if (buf[0] >= 0x31 && buf[0] <= 0x34)
		ts_data->diag_command = buf[0] - 0x2D;
	else
		ts_data->diag_command = 0;
	printk(KERN_INFO "ts_data->diag_command = %X\n",  ts_data->diag_command);
	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),
	cy8c_diag_show, cy8c_diag_dump);

static ssize_t cy8c_cal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy8c_ts_data *ts_data;
	size_t count = 0;
	uint8_t diag_cmd = 7;
	uint8_t data[256];
	int i;
	uint16_t num_nodes;
	ts_data = private_ts;
	memset(data, 0x0, sizeof(data));
	num_nodes = ts_data->x_channel * ts_data->y_channel;

	disable_irq(ts_data->client->irq);
	i2c_cy8c_read(ts_data->client, 0x00, &data[0], 1);
	i2c_cy8c_write_byte_data(ts_data->client, 0x00,
		(data[0] & 0x8F) | (diag_cmd << 4));
	mdelay(50);

	i2c_cy8c_read(ts_data->client, 0x00, &data[0], 1);
	printk(KERN_INFO "[cal_show]change mode to 0x%X\n", data[0]);

	i2c_cy8c_read(ts_data->client, 0x07, &data[0], num_nodes);

	for (i = 0; i < num_nodes; i++) {
		count += sprintf(buf + count, "%5d", data[i]);
		if ((i + 1) % ts_data->y_channel == 0)
			count += sprintf(buf + count, "\n");
	}
	count += sprintf(buf + count, "\n");

	/* Enter operation mode */
	i2c_cy8c_read(ts_data->client, 0x00, data, 1);
	if ((data[0] & 0x70) == 0x70)
		i2c_cy8c_write_byte_data(ts_data->client, 0x00, ((data[0] ^= BIT(7)) & 0x8F));
	mdelay(64);

	i2c_cy8c_read(ts_data->client, 0x00, &data[0], 2);
	printk(KERN_INFO "[cal_show]change mode to 0x%X 0x%X\n", data[0], data[1]);
	enable_irq(ts_data->client->irq);
	return count;
}

static ssize_t cy8c_cal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cy8c_ts_data *ts_data;
	uint8_t cal_command[3] = {0x20, 0x00, 0x00};
	uint8_t data[3];
	uint8_t loop_i;
	ts_data = private_ts;

	if (buf[0] == 0x31) {
		i2c_cy8c_read(ts_data->client, 0x00, data, 2);
		if ((data[1] & 0x10) == 0x10) {
			printk(KERN_INFO "Bootloader mode to OP mode2\n");
			cy8c_init_panel(ts_data);
		}

		disable_irq(ts_data->client->irq);

		i2c_cy8c_read(ts_data->client, 0x00, data, 3);
		if ((data[2] & 0x0F) >= 1) {
			printk(KERN_INFO "[cal_store]Number of touches %d\n", data[2] & 0x0F);
			enable_irq(ts_data->client->irq);
			return count;
		}

		/* Enter System mode */
		i2c_cy8c_read(ts_data->client, 0x00, &data[0], 1);
		i2c_cy8c_write_byte_data(ts_data->client, 0x00,
			(data[0] & 0x8F) | (1 << 4));
		mdelay(64);

		i2c_cy8c_write(ts_data->client, 0x02, &cal_command[0], 3);
		cy8c_data_toggle(ts_data);
		mdelay(500);
		for (loop_i = 0; loop_i < 100; loop_i++) {
			i2c_cy8c_read(ts_data->client, 0x00, data, 3);
			if (data[1] == 0x86) {
				printk(KERN_INFO "[cal_store][%d]status return 0x%X\n",
					loop_i + 1, data[1]);
				break;
			} else
				mdelay(10);
		}
		/* Enter operation mode */
		i2c_cy8c_read(ts_data->client, 0x00, data, 1);
		if ((data[0] & 0x70) == 0x10) {
			i2c_cy8c_write_byte_data(ts_data->client, 0x00,
				((data[0] ^= BIT(7)) & 0x8F));
		}
		mdelay(64);

		i2c_cy8c_read(ts_data->client, 0x00, &data[0], 2);
		printk(KERN_INFO "[cal_store]change mode to 0x%X 0x%X\n", data[0], data[1]);
		enable_irq(ts_data->client->irq);
	}
	return count;
}

static DEVICE_ATTR(calibration, (S_IWUSR|S_IRUGO),
	cy8c_cal_show, cy8c_cal_store);


static struct kobject *android_touch_kobj;

static int cy8c_touch_sysfs_init(void)
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
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	cy8c_reg_addr = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_calibration.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	return 0;
}

static void cy8c_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	kobject_del(android_touch_kobj);
}

static int cy8c_init_panel(struct cy8c_ts_data *ts)
{
	uint8_t buf, loop_i;
	uint8_t sec_key[] = {0x00, 0xFF, 0xA5, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	i2c_cy8c_write(ts->client, 0x00, sec_key, 11);
	msleep(80);
	for (loop_i = 0; loop_i < 10; loop_i++) {
		if (!gpio_get_value(ts->intr))
			break;
		msleep(10);
	}
	if (!ts->id)
		i2c_cy8c_read(ts->client, 0x17, &ts->id, 1);
	i2c_cy8c_read(ts->client, 0x00, &buf, 1);
	if ((buf & 0x70) == 0x10)
		i2c_cy8c_write_byte_data(ts->client, 0x00, ((buf ^= BIT(7)) & 0x8F));
	msleep(40);

	return 0;
}

static void cy8c_ts_work_func(struct work_struct *work)
{
	struct cy8c_ts_data *ts = container_of(work, struct cy8c_ts_data, work);
	uint8_t buf[32], loop_i;

	i2c_cy8c_read(ts->client, 0x00, buf, 32);
	if (ts->debug_log_level & 0x1) {
		for (loop_i = 0; loop_i < 32; loop_i++) {
			printk("0x%2.2X ", buf[loop_i]);
			if (loop_i % 16 == 15)
				printk("\n");
		}
	}
	if ((buf[1] & 0x10) == 0x10) {
		printk(KERN_INFO "Bootloader mode to OP mode\n");
		cy8c_init_panel(ts);
	}
	if ((buf[2] & 0x0F) >= 1 && !(buf[2] & 0x10)) {
		uint16_t x, y, z;
		int base = 0x03;
		for (loop_i = 0; (loop_i < (buf[2] & 0x0F)) && (loop_i < 4); loop_i++) {
			x = buf[base] << 8 | buf[base + 1];
			if (ts->orient & 0x01)
				x = 1023 - x;
			y = buf[base + 2] << 8 | buf[base + 3];
			if (ts->orient & 0x02)
				y = 1023 - y;
			if (ts->orient & 0x04)
				swap(x, y);
			z = buf[base + 4];
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, z);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
			input_mt_sync(ts->input_dev);
			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				printk(KERN_INFO "S1@%d,%d\n", x, y);
			}
			if (ts->first_pressed == 1) {
				ts->pre_finger_data[0] = x;
				ts->pre_finger_data[1] = y;
			}

			if (ts->debug_log_level & 0x2)
				printk(KERN_INFO "Finger %d=> X:%d, Y:%d w:%d, z:%d\n",
					loop_i + 1, x, y, z, z);
			if (loop_i % 2 == 1)
				base += 7;
			else
				base += 6;
		}
	} else {
		if ((buf[2] & 0x0F) == 0)
			cy8c_data_toggle(ts);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		if (ts->first_pressed == 1) {
			ts->first_pressed = 2;
			printk(KERN_INFO "E1@%d, %d\n",
				ts->pre_finger_data[0] , ts->pre_finger_data[1]);
		}

		if (ts->debug_log_level & 0x2)
			printk(KERN_INFO "Finger leave\n");
	}
	input_sync(ts->input_dev);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	else
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart cy8c_ts_timer_func(struct hrtimer *timer)
{
	struct cy8c_ts_data *ts;

	ts = container_of(timer, struct cy8c_ts_data, timer);
	queue_work(ts->cy8c_wq, &ts->work);
	return HRTIMER_NORESTART;
}

static irqreturn_t cy8c_ts_irq_handler(int irq, void *dev_id)
{
	struct cy8c_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->cy8c_wq, &ts->work);
	return IRQ_HANDLED;
}

static int cy8c_ts_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct cy8c_ts_data *ts;
	struct cy8c_i2c_platform_data *pdata;
	int ret = 0;
	uint8_t buf[6];
	printk(KERN_INFO "%s: enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct cy8c_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "allocate cy8c_ts_data failed\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;

	if (ts->power) {
		ret = ts->power(1);
		msleep(10);
		if (ret < 0) {
			dev_err(&client->dev, "power on failed\n");
			goto err_power_failed;
		}
	}
	ret = i2c_cy8c_read(ts->client, 0x01, buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "No Cypress chip found\n");
		goto err_detect_failed;
	}
	if ((buf[0] & 0x10) != 0x10) {
		i2c_cy8c_write_byte_data(ts->client, 0x00, 0x01);
		msleep(200);
		i2c_cy8c_read(ts->client, 0x01, buf, 2);
	}

	ret = i2c_cy8c_read(ts->client, 0x0B, &buf[2], 4);
	if (ret < 0) {
		dev_err(&client->dev, "No Cypress chip found\n");
		goto err_detect_failed;
	}
	dev_info(&client->dev, "buf: %x, %x, %x, %x\n", buf[0], buf[1], buf[2], buf[3]);

	ts->version = buf[2] << 8 | buf[3];
	if ((buf[4] + buf[5]) <= 32) {
		ts->x_channel = buf[4];
		ts->y_channel = buf[5];
	} else {
		ts->x_channel = 21;
		ts->y_channel = 11;
	}
	dev_info(&client->dev,
		"application verion = %X, x_channel = %X, y_channel = %X\n",
		ts->version, ts->x_channel, ts->y_channel);
	ret = cy8c_init_panel(ts);

	if (pdata) {
		while (pdata->version > ts->version)
			pdata++;
		ts->intr = pdata->gpio_irq;
		ts->orient = pdata->orient;
		dev_info(&client->dev, "orient: %d\n", ts->orient);
	}

	ts->cy8c_wq = create_singlethread_workqueue("cypress_touch");
	if (!ts->cy8c_wq)
		goto err_create_wq_failed;

	INIT_WORK(&ts->work, cy8c_ts_work_func);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "cy8c-touchscreen";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);

	printk(KERN_INFO "input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		pdata->abs_x_min, pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		pdata->abs_x_min, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		pdata->abs_y_min, pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
		pdata->abs_pressure_min, pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
		pdata->abs_width_min, pdata->abs_width_max, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,
			"cy8c_ts_probe: Unable to register %s input device\n",
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	if (client->irq) {
		ret = request_irq(client->irq, cy8c_ts_irq_handler,
				  IRQF_TRIGGER_LOW, client->name, ts);
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = cy8c_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = cy8c_ts_early_suspend;
	ts->early_suspend.resume = cy8c_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	dev_info(&client->dev, "Start touchscreen %s in %s mode\n",
		 ts->input_dev->name, (ts->use_irq ? "interrupt" : "polling"));

	private_ts = ts;
	cy8c_touch_sysfs_init();

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_create_wq_failed:
err_detect_failed:
err_power_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int cy8c_ts_remove(struct i2c_client *client)
{
	struct cy8c_ts_data *ts = i2c_get_clientdata(client);

	cy8c_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);

	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);

	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int cy8c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cy8c_ts_data *ts = i2c_get_clientdata(client);
	int ret;
	uint8_t buf[2];

	if (ts->use_irq)
		disable_irq_nosync(client->irq);
	else
		hrtimer_cancel(&ts->timer);

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq)
		enable_irq(client->irq);

	i2c_cy8c_read(ts->client, 0x00, buf, 2);
	printk(KERN_INFO "%s: %x, %x\n", __func__, buf[0], buf[1]);

	ts->first_pressed = 0;

	if ((buf[1] & 0x10) == 0x10)
		cy8c_init_panel(ts);
	if (buf[0] & 0x70)
		i2c_cy8c_write_byte_data(ts->client, 0x00, buf[0] & 0x8F);
	i2c_cy8c_write_byte_data(ts->client, 0x00, (buf[0] & 0x8F) | 0x02);

	return 0;
}

static int cy8c_ts_resume(struct i2c_client *client)
{
	struct cy8c_ts_data *ts = i2c_get_clientdata(client);
	uint8_t buf[2];

	i2c_cy8c_read(ts->client, 0x00, buf, 2);
	printk(KERN_INFO "%s: %x, %x\n", __func__, buf[0], buf[1]);

	if ((buf[1] & 0x10) == 0x10)
		cy8c_init_panel(ts);
	i2c_cy8c_write_byte_data(ts->client, 0x00, (buf[0] & 0x8F) | 0x04);

	if (ts->use_irq)
		enable_irq(client->irq);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cy8c_ts_early_suspend(struct early_suspend *h)
{
	struct cy8c_ts_data *ts;
	ts = container_of(h, struct cy8c_ts_data, early_suspend);
	cy8c_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void cy8c_ts_late_resume(struct early_suspend *h)
{
	struct cy8c_ts_data *ts;
	ts = container_of(h, struct cy8c_ts_data, early_suspend);
	cy8c_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id cy8c_ts_i2c_id[] = {
	{CYPRESS_TMA_NAME, 0},
	{}
};

static struct i2c_driver cy8c_ts_driver = {
	.id_table = cy8c_ts_i2c_id,
	.probe = cy8c_ts_probe,
	.remove = cy8c_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = cy8c_ts_suspend,
	.resume = cy8c_ts_resume,
#endif
	.driver = {
		.name = CYPRESS_TMA_NAME,
		.owner = THIS_MODULE,
	},
};

static int __devinit cy8c_ts_init(void)
{
	printk(KERN_INFO "%s: enter\n", __func__);

	return i2c_add_driver(&cy8c_ts_driver);
}

static void __exit cy8c_ts_exit(void)
{
	i2c_del_driver(&cy8c_ts_driver);
}

module_init(cy8c_ts_init);
module_exit(cy8c_ts_exit);

MODULE_DESCRIPTION("Cypress TMA Touchscreen Driver");
MODULE_LICENSE("GPL");

