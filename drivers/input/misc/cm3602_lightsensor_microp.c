/*
 *
 * Copyright (C) 2009 HTC, Inc.
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

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <asm/uaccess.h>
#include <mach/atmega_microp.h>
#include <asm/mach-types.h>

struct microp_ls_info {
	struct microp_function_config *ls_config;
	struct input_dev *ls_input_dev;
	struct early_suspend early_suspend;
	struct i2c_client *client;
	struct workqueue_struct *ls_wq;

	uint32_t als_func;
	uint32_t als_kadc;
	uint32_t als_gadc;
	uint8_t als_calibrating;
	int als_intr_enabled;
	int is_suspend;
	int old_intr_cmd;
};

struct microp_ls_info *ls_info;
static int ls_enable_flag;
static int ls_enable_num;

static void enable_intr_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(enable_intr_work, enable_intr_do_work);

static void lightsensor_do_work(struct work_struct *w);
static DECLARE_WORK(lightsensor_work, lightsensor_do_work);

void set_ls_kvalue(struct microp_ls_info *li)
{

	if (!li) {
		pr_err("%s: ls_info is empty\n", __func__);
		return;
	}

	printk(KERN_INFO "%s: ALS calibrated als_kadc=0x%x\n",
			__func__, als_kadc);
	if (als_kadc >> 16 == ALS_CALIBRATED)
		li->als_kadc = als_kadc & 0xFFFF;
	else {
		li->als_kadc = 0;
		printk(KERN_INFO "%s: no ALS calibrated\n", __func__);
	}

	if (li->als_kadc && li->ls_config->golden_adc > 0) {
		li->als_kadc = (li->als_kadc > 0 && li->als_kadc < 0x400) ?
				li->als_kadc : li->ls_config->golden_adc;
		li->als_gadc = li->ls_config->golden_adc;
	} else {
		li->als_kadc = 1;
		li->als_gadc = 1;
	}
	printk(KERN_INFO "%s: als_kadc=0x%x, als_gadc=0x%x\n",
			__func__, li->als_kadc, li->als_gadc);
}

static int upload_ls_table(struct microp_ls_info *li)
{
	uint8_t data[20];
	int i;
	for (i = 0; i < 10; i++) {
		if (li->ls_config->levels[i] < 0x3FF) {
			data[i] = (uint8_t)(li->ls_config->levels[i]
					* li->als_kadc / li->als_gadc >> 8);
			data[i + 10] = (uint8_t)(li->ls_config->levels[i]
					* li->als_kadc / li->als_gadc);
		} else {
			data[i] = (uint8_t)(li->ls_config->levels[i] >> 8);
			data[i + 10] = (uint8_t)(li->ls_config->levels[i] & 0xFF);
		}
		printk("ls_table: data[%d] , data[%d] = %x, %x\n", i, i, data[i], data[i+10]);
	}

	return microp_i2c_write(MICROP_I2C_WCMD_ADC_TABLE, data, 20);
}

static int get_ls_adc_level(uint8_t *data)
{
	struct microp_ls_info *li = ls_info;
	uint8_t i, adc_level = 0;
	uint16_t adc_value = 0;

	data[0] = 0x00;
	data[1] = li->ls_config->channel;
	if (microp_read_adc(data))
		return -1;

	adc_value = data[0]<<8 | data[1];
	if (adc_value > 0x3FF) {
		printk(KERN_WARNING "%s: get wrong value: 0x%X\n",
			__func__, adc_value);
		return -1;
	} else {
		if (!li->als_calibrating) {
			adc_value = adc_value * li->als_gadc / li->als_kadc;
			if (adc_value > 0x3FF)
				adc_value = 0x3FF;
			data[0] = adc_value >> 8;
			data[1] = adc_value & 0xFF;
		}
		for (i = 0; i < 10; i++) {
			if (adc_value <=
				li->ls_config->levels[i]) {
				adc_level = i;
				if (li->ls_config->levels[i])
					break;
			}
		}
		printk(KERN_DEBUG "ALS value: 0x%X, level: %d #\n",
				adc_value, adc_level);
		data[2] = adc_level;
	}

	return 0;
}

void report_lightseneor_data(void)
{
	uint8_t data[3];
	int ret;
	struct microp_ls_info *li = ls_info;

	ret = get_ls_adc_level(data);
	if (!ret) {
		input_report_abs(li->ls_input_dev,
				ABS_MISC, (int)data[2]);
		input_sync(li->ls_input_dev);
	}
}

static int ls_microp_intr_enable(uint8_t enable)
{

	int ret;
	uint8_t data[2];
	struct microp_ls_info *li = ls_info;

	if (li->old_intr_cmd) {
		data[0] = 0;
		if (enable)
			data[1] = 1;
		else
			data[1] = 0;

		ret = microp_i2c_write(MICROP_I2C_WCMD_AUTO_BL_CTL, data, 2);
	} else {
		ret = microp_write_interrupt(li->client,
			li->ls_config->int_pin, enable);
	}

	return ret;
}

static void enable_intr_do_work(struct work_struct *w)
{
	struct microp_ls_info *li = ls_info;
	int ret;

	if (ls_enable_flag) {
		ret = ls_microp_intr_enable(1);
		if (ret < 0)
			pr_err("%s error\n", __func__);
		else {
			li->als_intr_enabled = 1;
			ls_enable_flag = 0;
			input_report_abs(li->ls_input_dev, ABS_MISC, -1);
			input_sync(li->ls_input_dev);
		}
	}

	report_lightseneor_data();
}

static void lightsensor_do_work(struct work_struct *w)
{
	/* Wait for Framework event polling ready */
	if (ls_enable_num == 0) {
		ls_enable_num = 1;
		msleep(300);
	}

	report_lightseneor_data();
}

static irqreturn_t lightsensor_irq_handler(int irq, void *data)
{
	struct microp_ls_info *li = ls_info;
	queue_work(li->ls_wq, &lightsensor_work);

	return IRQ_HANDLED;
}

static int ls_power(int enable)
{
	struct microp_ls_info *li = ls_info;

	if (li->ls_config->ls_gpio_on)
		gpio_set_value(li->ls_config->ls_gpio_on, enable ? 0 : 1);

	if (li->ls_config->ls_power)
		li->ls_config->ls_power(LS_PWR_ON, enable);

	return 0;
}

static int lightsensor_enable(void)
{
	int ret;
	struct microp_ls_info *li = ls_info;

	pr_info("%s\n", __func__);

	ls_enable_flag = 1;
	if (li->is_suspend) {
		li->als_intr_enabled = 1;
		pr_err("%s: microp is suspended\n", __func__);
		return 0;
	}
	if (!li->als_intr_enabled) {
		ret = ls_microp_intr_enable(1);
		if (ret < 0)
			pr_err("%s: set auto light sensor fail\n", __func__);
		else {
			li->als_intr_enabled = 1;
			/* report an invalid value first to ensure we trigger an event
			* when adc_level is zero.
			*/
			input_report_abs(li->ls_input_dev, ABS_MISC, -1);
			input_sync(li->ls_input_dev);
		}
	}
	return 0;
}

static int lightsensor_disable(void)
{
	/* update trigger data when done */
	struct microp_ls_info *li = ls_info;
	int ret;

	pr_info("%s\n", __func__);
	ls_enable_flag = 0;
	if (li->is_suspend) {
		li->als_intr_enabled = 0;
		pr_err("%s: microp is suspended\n", __func__);
		return 0;
	}

	if (li->als_intr_enabled) {
		ret = ls_microp_intr_enable(0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
		       __func__);
		else
			li->als_intr_enabled = 0;
	}
	return 0;
}

DEFINE_MUTEX(ls_i2c_api_lock);
static int lightsensor_opened;

static int lightsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	pr_debug("%s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	if (lightsensor_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lightsensor_opened = 1;
	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	lightsensor_opened = 0;
	mutex_unlock(&ls_i2c_api_lock);
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct microp_ls_info *li = ls_info;
	mutex_lock(&ls_i2c_api_lock);
	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		pr_info("%s set value = %d\n", __func__, val);
		rc = val ? lightsensor_enable() : lightsensor_disable();
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = li->als_intr_enabled;
		pr_info("%s get enabled status: %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	uint8_t data[3];
	int ret;

	ret = get_ls_adc_level(data);

	ret = sprintf(buf,
				"ADC[0x%03X] => level %d\n",
				(data[0] << 8 | data[1]), data[2]);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0666, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t data[2] = {0, 0};
	int ret;

	microp_i2c_read(MICROP_I2C_RCMD_SPI_BL_STATUS, data, 2);
	ret = sprintf(buf, "Light sensor Auto = %d, SPI enable = %d\n",
				data[0], data[1]);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct microp_ls_info *li = ls_info;
	uint8_t enable = 0;
	int ls_auto;
	int ret;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		enable = 1;
		li->als_calibrating = (ls_auto == 147) ? 1 : 0;
		li->als_intr_enabled = 1;
	} else {
		enable = 0;
		li->als_calibrating = 0;
		li->als_intr_enabled = 0;
	}

	ret = ls_microp_intr_enable(enable);
	if (ret < 0)
		pr_err("%s: ls intr enable fail\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_auto, 0666, ls_enable_show, ls_enable_store);

static ssize_t ls_kadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_ls_info *li = ls_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x, gadc = 0x%x, real kadc = 0x%x\n",
				li->als_kadc, li->als_gadc, als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct microp_ls_info *li = ls_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);
	if (kadc_temp <= 0 || li->ls_config->golden_adc <= 0) {
		printk(KERN_ERR "%s: kadc_temp=0x%x, als_gadc=0x%x\n",
			__func__,
			kadc_temp,
			li->ls_config->golden_adc);
		return -EINVAL;
	}

	li->als_kadc = kadc_temp;
	li->als_gadc = li->ls_config->golden_adc;
	printk(KERN_INFO "%s: als_kadc=0x%x, als_gadc=0x%x\n",
			__func__, li->als_kadc, li->als_gadc);

	if (upload_ls_table(li) < 0)
		printk(KERN_ERR "%s: upload ls table fail\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_kadc, 0666, ls_kadc_show, ls_kadc_store);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void light_sensor_suspend(struct early_suspend *h)
{
	struct microp_ls_info *li = ls_info;
	int ret;

	li->is_suspend = 1;
	cancel_delayed_work(&enable_intr_work);
	if (li->als_intr_enabled) {
		ret = ls_microp_intr_enable(0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
				__func__);
		else
			li->als_intr_enabled = 0;
	}
	ls_power(0);
}

static void light_sensor_resume(struct early_suspend *h)
{
	struct microp_ls_info *li = ls_info;

	ls_power(1);
	queue_delayed_work(li->ls_wq, &enable_intr_work, msecs_to_jiffies(800));
	li->is_suspend = 0;
}
#endif

static int lightsensor_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct microp_ls_info *li;
	struct lightsensor_platform_data *pdata = pdev->dev.platform_data;

	li = kzalloc(sizeof(struct microp_ls_info), GFP_KERNEL);
	if (!li)
		return -ENOMEM;
	ls_info = li;
	li->client = dev_get_drvdata(&pdev->dev);

	if (!li->client) {
		pr_err("%s: can't get microp i2c client\n", __func__);
		return -1;
	}
	li->ls_input_dev = input_allocate_device();
	if (!li->ls_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		return -ENOMEM;
	}
	li->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, li->ls_input_dev->evbit);
	input_set_abs_params(li->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(li->ls_input_dev);
	if (ret < 0) {
		pr_err("%s: can not register input device\n",
				__func__);
		return ret;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("%s: can not register misc device\n",
				__func__);
		return ret;
	}

	li->ls_config = pdata->config;
	irq = pdata->irq;
	li->old_intr_cmd = pdata->old_intr_cmd;
	ret = request_irq(irq, lightsensor_irq_handler, IRQF_TRIGGER_NONE, "lightsensor_microp", li);
	if (ret < 0) {
		pr_err("%s: request_irq(%d) failed for (%d)\n",
			__func__, irq, ret);
		return ret;
	}

	set_ls_kvalue(li);
	ret = upload_ls_table(li);
	if (ret < 0) {
		pr_err("%s: upload ls table fail\n",
				__func__);
		return ret;
	}

	li->ls_wq = create_workqueue("ls_wq");
	if (li->ls_wq == NULL)
			return -ENOMEM;

	if (li->ls_config->ls_gpio_on) {
		ret = gpio_request(li->ls_config->ls_gpio_on,
				"microp_i2c");
		if (ret < 0) {
			pr_err("request gpio ls failed\n");
			return ret;
		}
		ret = gpio_direction_output(li->ls_config->ls_gpio_on, 0);
		if (ret < 0) {
			pr_err("gpio_direction_output ls failed\n");
			return ret;
		}
	}
	ls_power(1);
#ifdef CONFIG_HAS_EARLYSUSPEND
	li->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	li->early_suspend.suspend = light_sensor_suspend;
	li->early_suspend.resume = light_sensor_resume;
	register_early_suspend(&li->early_suspend);
#endif
	ret = device_create_file(&li->client->dev, &dev_attr_ls_adc);
	ret = device_create_file(&li->client->dev, &dev_attr_ls_auto);
	ret = device_create_file(&li->client->dev, &dev_attr_ls_kadc);

	return 0;

}

static struct platform_driver lightsensor_driver = {
	.probe = lightsensor_probe,
	.driver = { .name = "lightsensor_microp", },
};

static int __init light_sensor_init(void)
{
	return platform_driver_register(&lightsensor_driver);
}

static void __exit light_sensor_exit(void)
{
	platform_driver_unregister(&lightsensor_driver);
}

module_init(light_sensor_init);
module_exit(light_sensor_exit);

MODULE_DESCRIPTION("HTC LIGHT SENSOR");
MODULE_LICENSE("GPL");
