/* drivers/input/misc/lightsensor_smd.c
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
#include <mach/msm_iomap.h>

#define D(x...) pr_info(x)

#define LS_SMEM_ADDR (0x000FC000 + 0x2D8 + MSM_SHARED_RAM_BASE)

struct smd_ls_info {
	struct lightsensor_smd_platform_data *pdata;
	struct class *lightsensor_class;
	struct device *ls_dev;

	struct input_dev *ls_input_dev;
	struct early_suspend early_suspend;
	struct workqueue_struct *ls_wq;

	uint32_t als_func;
	uint32_t als_kadc;
	uint32_t als_gadc;
	uint8_t als_calibrating;
	int als_poll_enabled;
	int is_suspend;
	int old_intr_cmd;
	int ls_enable_flag;
	int lightsensor_opened;
	int ls_enable_num;
	uint32_t current_level;
};

struct smd_ls_info *ls_smd_info;

static void enable_poll_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(enable_poll_work, enable_poll_do_work);

static void lightsensor_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(lightsensor_work, lightsensor_do_work);

void set_ls_kvalue_smd(struct smd_ls_info *li)
{
	if (!li) {
		pr_err("%s: smd_ls_info is empty\n", __func__);
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

	if (li->als_kadc && li->pdata->golden_adc > 0) {
		li->als_kadc = (li->als_kadc > 0 && li->als_kadc < 0x10000) ?
				li->als_kadc : li->pdata->golden_adc;
		li->als_gadc = li->pdata->golden_adc;
	} else {
		li->als_kadc = 1;
		li->als_gadc = 1;
	}

	printk(KERN_INFO "%s: als_kadc=0x%x, als_gadc=0x%x\n",
			__func__, li->als_kadc, li->als_gadc);
}

static uint16_t read_smd(void)
{
	uint16_t ls_adc = -1;

	ls_adc = (*(volatile uint16_t *)(LS_SMEM_ADDR));

	return ls_adc;
}

static int get_ls_adc_level(uint8_t *data)
{
	struct smd_ls_info *li = ls_smd_info;
	uint8_t i, adc_level = 0;
	uint32_t adc_value = 0;

	/*D("%s\n", __func__);*/

	adc_value = read_smd();

	if (adc_value > 0xFFFF) {
		printk(KERN_WARNING "%s: get wrong value: 0x%X\n",
			__func__, adc_value);
		return -1;
	} else {
		if (!li->als_calibrating) {
			adc_value = adc_value * li->als_gadc / li->als_kadc;
			if (adc_value > 0xFFFF)
				adc_value = 0xFFFF;
		}

		data[0] = adc_value >> 8;
		data[1] = adc_value & 0xFF;

		for (i = 0; i < 10; i++) {
			if (adc_value <= (*(li->pdata->levels + i))) {
				adc_level = i;
				if ((*(li->pdata->levels + i)))
					break;
			}
		}
		if (li->current_level != adc_level) {
			D("ALS value: 0x%X, level: %d #\n",
				adc_value, adc_level);
			li->current_level = adc_level;
		}

		data[2] = adc_level;
	}

	return 0;
}

void report_lightseneor_data_smd(void)
{
	uint8_t data[3];
	int ret;
	struct smd_ls_info *li = ls_smd_info;

	ret = get_ls_adc_level(data);
	if (!ret) {
		input_report_abs(li->ls_input_dev,
				ABS_MISC, (int)data[2]);
		input_sync(li->ls_input_dev);
	}
}

static int ls_poll_enable(uint8_t enable)
{

	int ret;
	struct smd_ls_info *li = ls_smd_info;

	if (enable)
		ret = queue_delayed_work(li->ls_wq, &lightsensor_work,
			msecs_to_jiffies(1));
	else
		ret = cancel_delayed_work(&lightsensor_work);

	return ret;
}

static void enable_poll_do_work(struct work_struct *w)
{
	struct smd_ls_info *li = ls_smd_info;
	int ret;

	li->is_suspend = 0;
	if (li->ls_enable_flag) {
		ret = ls_poll_enable(1);
		if (ret < 0)
			pr_err("%s error\n", __func__);
		else {
			li->als_poll_enabled = 1;
			li->ls_enable_flag = 0;
			input_report_abs(li->ls_input_dev, ABS_MISC, -1);
			input_sync(li->ls_input_dev);
		}
	}

	report_lightseneor_data_smd();
}

static void lightsensor_do_work(struct work_struct *w)
{
	struct smd_ls_info *li = ls_smd_info;

	/*D("%s\n", __func__);*/

	/* Wait for Framework event polling ready */
	if (li->ls_enable_num == 0) {
		li->ls_enable_num = 1;
		msleep(300);
	}

	report_lightseneor_data_smd();

	queue_delayed_work(li->ls_wq, &lightsensor_work,
		msecs_to_jiffies(500));
}

static int ls_power(int enable)
{
	struct smd_ls_info *li = ls_smd_info;

	if (li->pdata->ls_power)
		li->pdata->ls_power(LS_PWR_ON, enable);

	return 0;
}

static int lightsensor_enable(void)
{
	int ret;
	struct smd_ls_info *li = ls_smd_info;

	D("%s\n", __func__);

	li->ls_enable_flag = 1;

	if (li->is_suspend) {
		li->als_poll_enabled = 1;
		pr_err("%s: System is suspended\n", __func__);
		return 0;
	}

	if (!li->als_poll_enabled) {
		ret = ls_poll_enable(1);
		if (ret < 0)
			pr_err("%s: set auto light sensor fail\n", __func__);
		else {
			li->als_poll_enabled = 1;
			/* report an invalid value first to ensure we
			* trigger an event when adc_level is zero.
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
	struct smd_ls_info *li = ls_smd_info;
	int ret;

	pr_info("%s\n", __func__);

	li->ls_enable_flag = 0;

	if (li->is_suspend) {
		li->als_poll_enabled = 0;
		pr_err("%s: System is suspended\n", __func__);
		return 0;
	}

	if (li->als_poll_enabled) {
		ret = ls_poll_enable(0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
		       __func__);
		else {
			cancel_delayed_work(&lightsensor_work);
			li->als_poll_enabled = 0;
		}
	}

	return 0;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct smd_ls_info *li = ls_smd_info;

	pr_debug("%s\n", __func__);

	if (li->lightsensor_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	li->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct smd_ls_info *li = ls_smd_info;

	pr_debug("%s\n", __func__);
	li->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct smd_ls_info *li = ls_smd_info;

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
		val = li->als_poll_enabled;
		pr_info("%s get enabled status: %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
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

	ret = sprintf(buf, "ADC[0x%03X] => level %d\n",
				(data[0] << 8 | data[1]), data[2]);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0666, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct smd_ls_info *li = ls_smd_info;
	int ret;

	ret = sprintf(buf, "Light sensor Auto = %d\n",
			li->als_poll_enabled);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct smd_ls_info *li = ls_smd_info;
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
		li->als_poll_enabled = 1;
	} else {
		enable = 0;
		li->als_calibrating = 0;
		li->als_poll_enabled = 0;
	}

	ret = ls_poll_enable(enable);
	if (ret < 0)
		pr_err("%s: ls intr enable fail\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_auto, 0666, ls_enable_show, ls_enable_store);

static ssize_t ls_kadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct smd_ls_info *li = ls_smd_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x, gadc = 0x%x, real kadc = 0x%x\n",
				li->als_kadc, li->als_gadc, als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct smd_ls_info *li = ls_smd_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);
	if (kadc_temp <= 0 || li->pdata->golden_adc <= 0) {
		printk(KERN_ERR "%s: kadc_temp=0x%x, als_gadc=0x%x\n",
			__func__,
			kadc_temp,
			li->pdata->golden_adc);
		return -EINVAL;
	}

	li->als_kadc = kadc_temp;
	li->als_gadc = li->pdata->golden_adc;
	printk(KERN_INFO "%s: als_kadc=0x%x, als_gadc=0x%x\n",
			__func__, li->als_kadc, li->als_gadc);

	return count;
}

static DEVICE_ATTR(ls_kadc, 0666, ls_kadc_show, ls_kadc_store);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void light_sensor_suspend(struct early_suspend *h)
{
	struct smd_ls_info *li = ls_smd_info;
	int ret;

	li->is_suspend = 1;
	cancel_delayed_work(&enable_poll_work);
	if (li->als_poll_enabled) {
		ret = ls_poll_enable(0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
				__func__);
		else
			li->als_poll_enabled = 0;
	}

	ls_power(0);
}

static void light_sensor_resume(struct early_suspend *h)
{
	struct smd_ls_info *li = ls_smd_info;

	ls_power(1);
	queue_delayed_work(li->ls_wq, &enable_poll_work, msecs_to_jiffies(800));
}
#endif

static int lightsensor_probe(struct platform_device *pdev)
{
	int ret;
	struct smd_ls_info *li;
	struct lightsensor_smd_platform_data *pdata = pdev->dev.platform_data;

	D("%s\n", __func__);

	li = kzalloc(sizeof(struct smd_ls_info), GFP_KERNEL);
	if (!li)
		return -ENOMEM;

	ls_smd_info = li;
	li->pdata = pdata;

	li->ls_input_dev = input_allocate_device();
	if (!li->ls_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_allocate_device;
	}

	li->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, li->ls_input_dev->evbit);
	input_set_abs_params(li->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(li->ls_input_dev);
	if (ret < 0) {
		pr_err("%s: can not register input device\n",
				__func__);
		goto err_input_register_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("%s: can not register misc device\n",
				__func__);
		goto err_misc_register;
	}

	li->ls_wq = create_workqueue("ls_wq");
	if (li->ls_wq == NULL) {
		ret = -ENOMEM;
		goto err_create_workqueue;
	}

	set_ls_kvalue_smd(li);

	ret = ls_power(1);
	if (ret < 0) {
		pr_err("Fail to power on the lightsensor\n");
		goto err_ls_power;
	}

	li->lightsensor_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(li->lightsensor_class)) {
		ret = PTR_ERR(li->lightsensor_class);
		li->lightsensor_class = NULL;
		goto err_create_class;
	}

	li->ls_dev = device_create(li->lightsensor_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(li->ls_dev))) {
		ret = PTR_ERR(li->ls_dev);
		li->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(li->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(li->ls_dev, &dev_attr_ls_auto);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(li->ls_dev, &dev_attr_ls_kadc);
	if (ret)
		goto err_create_ls_device_file;

#ifdef CONFIG_HAS_EARLYSUSPEND
	li->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	li->early_suspend.suspend = light_sensor_suspend;
	li->early_suspend.resume = light_sensor_resume;
	register_early_suspend(&li->early_suspend);
#endif

	return 0;

err_create_ls_device_file:
	device_unregister(li->ls_dev);
err_create_ls_device:
	class_destroy(li->lightsensor_class);
err_create_class:
err_ls_power:
err_create_workqueue:
	misc_deregister(&lightsensor_misc);
err_misc_register:
	input_unregister_device(li->ls_input_dev);
err_input_register_device:
	input_free_device(li->ls_input_dev);
err_input_allocate_device:
	kfree(li);
	return ret;
}

static struct platform_driver lightsensor_driver = {
	.probe = lightsensor_probe,
	.driver = { .name = "lightsensor_smd", },
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
