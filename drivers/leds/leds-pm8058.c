/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/pwm.h>
#include <linux/android_alarm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pm8058.h>
#include <linux/wakelock.h>

#ifdef CONFIG_HTC_HEADSET_MISC
#include <mach/htc_headset_misc.h>
#endif

#ifdef CONFIG_HTC_HEADSET_MISC
#define charming_led_enable(enable) headset_indicator_enable(enable)
#else
#define charming_led_enable(enable) {}
#endif

#define LED_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[LED]" fmt, ##__VA_ARGS__)
#define LED_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[LED]" fmt, ##__VA_ARGS__)
#define LED_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[LED][ERR]" fmt, ##__VA_ARGS__)

/* static struct pw8058_pwm_config pwm_conf; */
static struct workqueue_struct *g_led_work_queue;
static int duties[64];
static struct pm8058_led_data  *for_key_led_data;
static int flag_hold_virtual_key = 0;
static int virtual_key_state;
struct wake_lock pmic_led_wake_lock;

static int bank_to_id(int bank)
{
	int id;

	switch (bank) {
	case 3:
		id = PM_PWM_LED_KPD;
		break;
	case 4:
		id = PM_PWM_LED_0;
		break;
	case 5:
		id = PM_PWM_LED_1;
		break;
	case 6:
		id = PM_PWM_LED_2;
		break;
	default:
		id = -1;
	}

	return id;
}
void button_backlight_flash(int brightness_key)
{
	int milliamps;
	int id, mode;

	LED_INFO_LOG("%s brightness_key: %d\n", __func__,brightness_key);
	pwm_disable(for_key_led_data->pwm_led);
	id = bank_to_id(for_key_led_data->bank);
	mode = (id == PM_PWM_LED_KPD) ? PM_PWM_CONF_PWM1 :
					PM_PWM_CONF_PWM1 + (for_key_led_data->bank - 4);

	if (brightness_key) {
		flag_hold_virtual_key = 1;
		milliamps = (for_key_led_data->flags & PM8058_LED_DYNAMIC_BRIGHTNESS_EN) ?
	    for_key_led_data->out_current * brightness_key / LED_FULL :
	    for_key_led_data->out_current;
		pm8058_pwm_config_led(for_key_led_data->pwm_led, id, mode, milliamps);
		pwm_config(for_key_led_data->pwm_led, 320000, 640000);
		pwm_enable(for_key_led_data->pwm_led);
		LED_INFO_LOG("%s Button_backlight flash on\n", __func__);
	} else {
		pwm_disable(for_key_led_data->pwm_led);
		pwm_disable(for_key_led_data->pwm_led);
		pm8058_pwm_config_led(for_key_led_data->pwm_led, id, mode, 0);
		LED_INFO_LOG("%s Button_backlight flash off\n", __func__);
		if (virtual_key_state != 0){
			milliamps = (for_key_led_data->flags & PM8058_LED_DYNAMIC_BRIGHTNESS_EN) ?
			for_key_led_data->out_current * brightness_key / LED_FULL :
			for_key_led_data->out_current;
			pm8058_pwm_config_led(for_key_led_data->pwm_led, id, mode, milliamps);
			pwm_config(for_key_led_data->pwm_led, 64000, 64000);
			pwm_enable(for_key_led_data->pwm_led);
			LED_INFO_LOG("%s Button_backlight state resume\n", __func__);
		}
		flag_hold_virtual_key = 0;
	}
}
static void pwm_lut_delayed_fade_out(struct work_struct *work)
{
	struct pm8058_led_data *ldata;
	int id, mode;
	ldata = container_of(work, struct pm8058_led_data,
			     led_delayed_work.work);
	id = bank_to_id(ldata->bank);
	mode = (id == PM_PWM_LED_KPD) ? PM_PWM_CONF_PWM1 :
					PM_PWM_CONF_PWM1 + (ldata->bank - 4);
	LED_INFO_LOG("%s \n", __func__);
	pm8058_pwm_lut_enable(ldata->pwm_led, 0);
	pm8058_pwm_config_led(ldata->pwm_led, id, mode, 0);
}

static void led_blink_do_work(struct work_struct *work)
{
	struct pm8058_led_data *ldata;

	ldata = container_of(work, struct pm8058_led_data,
			     led_delayed_work.work);

	LED_INFO_LOG("%s \n", __func__);
	pwm_config(ldata->pwm_led, ldata->duty_time_ms * 1000,
		   ldata->period_us);
	pwm_enable(ldata->pwm_led);
}

static void led_work_func(struct work_struct *work)
{
	struct pm8058_led_data *ldata;

	ldata = container_of(work, struct pm8058_led_data, led_work);
	pwm_disable(ldata->pwm_led);

	if (strcmp(ldata->ldev.name, "charming-led") == 0)
		charming_led_enable(0);
}

static void led_alarm_handler(struct alarm *alarm)
{
	struct pm8058_led_data *ldata;

	ldata = container_of(alarm, struct pm8058_led_data, led_alarm);
	queue_work(g_led_work_queue, &ldata->led_work);
}

static void pm8058_pwm_led_brightness_set(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	struct pm8058_led_data *ldata;
	int enable = 0;

	/* struct pwm_device* pwm_led; */
	ldata = container_of(led_cdev, struct pm8058_led_data, ldev);

	pwm_disable(ldata->pwm_led);

	brightness = (brightness > LED_FULL) ? LED_FULL : brightness;
	brightness = (brightness < LED_OFF) ? LED_OFF : brightness;
	LED_INFO_LOG("%s: bank %d brightness %d\n", __func__,
	       ldata->bank, brightness);

	enable = (brightness) ? 1 : 0;
	if (strcmp(ldata->ldev.name, "charming-led") == 0)
		charming_led_enable(enable);

	if (brightness) {
		pwm_config(ldata->pwm_led, 64000, 64000);
#if 0
		pwm_conf.pwm_size = ldata->pwm_size;
		pwm_conf.clk = ldata->clk;
		pwm_conf.pre_div = ldata->pre_div;
		pwm_conf.pre_div_exp = ldata->pre_div_exp;
		pwm_conf.pwm_value = ldata->pwm_value;
		pwm_conf.bypass_lut = 1;
		pwm_configure(ldata->pwm_led, &pwm_conf);
#endif
		pwm_enable(ldata->pwm_led);
	}
}

static void pm8058_drvx_led_brightness_set(struct led_classdev *led_cdev,
					   enum led_brightness brightness)
{
	struct pm8058_led_data *ldata;
	int *pduties;
	int id, mode;
	int lut_flag;
	int milliamps;
	int enable = 0;

	ldata = container_of(led_cdev, struct pm8058_led_data, ldev);

	pwm_disable(ldata->pwm_led);
	cancel_delayed_work_sync(&ldata->led_delayed_work);

	id = bank_to_id(ldata->bank);
	mode = (id == PM_PWM_LED_KPD) ? PM_PWM_CONF_PWM1 :
					PM_PWM_CONF_PWM1 + (ldata->bank - 4);

	brightness = (brightness > LED_FULL) ? LED_FULL : brightness;
	brightness = (brightness < LED_OFF) ? LED_OFF : brightness;
	LED_INFO_LOG("%s: bank %d brightness %d +\n", __func__,
	       ldata->bank, brightness);

	enable = (brightness) ? 1 : 0;
	if (strcmp(ldata->ldev.name, "charming-led") == 0)
		charming_led_enable(enable);

	lut_flag = ldata->lut_flag & ~(PM_PWM_LUT_LOOP | PM_PWM_LUT_REVERSE);
	virtual_key_state = enable;
	if (flag_hold_virtual_key == 1) {
		LED_INFO_LOG("%s, Return control by button_backlight flash \n", __func__);
		return;
	}

	if (brightness) {
		milliamps = (ldata->flags & PM8058_LED_DYNAMIC_BRIGHTNESS_EN) ?
			    ldata->out_current * brightness / LED_FULL :
			    ldata->out_current;
		pm8058_pwm_config_led(ldata->pwm_led, id, mode, milliamps);
		if (ldata->flags & PM8058_LED_LTU_EN) {
			pduties = &duties[ldata->start_index];
			pm8058_pwm_lut_config(ldata->pwm_led,
					      ldata->period_us,
					      pduties,
					      ldata->duty_time_ms,
					      ldata->start_index,
					      ldata->duites_size,
					      0, 0,
					      lut_flag);
			pm8058_pwm_lut_enable(ldata->pwm_led, 0);
			pm8058_pwm_lut_enable(ldata->pwm_led, 1);
		} else {
			pwm_config(ldata->pwm_led, 64000, 64000);
			pwm_enable(ldata->pwm_led);
		}
	} else {
		if (ldata->flags & PM8058_LED_LTU_EN) {
			wake_lock_timeout(&pmic_led_wake_lock,HZ*2);
			pduties = &duties[ldata->start_index +
					  ldata->duites_size];
			pm8058_pwm_lut_config(ldata->pwm_led,
					      ldata->period_us,
					      pduties,
					      ldata->duty_time_ms,
					      ldata->start_index +
					      ldata->duites_size,
					      ldata->duites_size,
					      0, 0,
					      lut_flag);
			pm8058_pwm_lut_enable(ldata->pwm_led, 1);
			queue_delayed_work(g_led_work_queue,
					   &ldata->led_delayed_work,
					   msecs_to_jiffies(ldata->duty_time_ms * ldata->duites_size));

			LED_INFO_LOG("%s: bank %d fade out brightness %d -\n", __func__,
			ldata->bank, brightness);
			return;
		} else
			pwm_disable(ldata->pwm_led);
		pm8058_pwm_config_led(ldata->pwm_led, id, mode, 0);
	}
	LED_INFO_LOG("%s: bank %d brightness %d -\n", __func__, ldata->bank, brightness);
}

static ssize_t pm8058_led_blink_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct pm8058_led_data *ldata;
	int id, mode;
	int val;
	int enable = 0;
#ifdef CONFIG_HTC_HEADSET_MISC
	int *pduties;
#endif

/*struct timespec ts1, ts2;*/

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < 0 || val > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct pm8058_led_data, ldev);

	id = bank_to_id(ldata->bank);
	mode = (id == PM_PWM_LED_KPD) ? PM_PWM_CONF_PWM1 :
					PM_PWM_CONF_PWM1 + (ldata->bank - 4);

	if (ldata->flags & PM8058_LED_BLINK_EN)
		pm8058_pwm_config_led(ldata->pwm_led, id, mode,
				      ldata->out_current);

	LED_INFO_LOG("%s: bank %d blink %d\n", __func__, ldata->bank, val);

	enable = (val > 0) ? 1 : 0;
	if (strcmp(ldata->ldev.name, "charming-led") == 0)
		charming_led_enable(enable);

	switch (val) {
	case -1: /* stop flashing */
		pwm_disable(ldata->pwm_led);
		if (ldata->flags & PM8058_LED_BLINK_EN)
			pm8058_pwm_config_led(ldata->pwm_led, id, mode, 0);
		break;
	case 0:
		pwm_disable(ldata->pwm_led);
		if (led_cdev->brightness) {
			pwm_config(ldata->pwm_led, 64000, 64000);
			pwm_enable(ldata->pwm_led);
		} else {
			if (ldata->flags & PM8058_LED_BLINK_EN)
				pm8058_pwm_config_led(ldata->pwm_led, id,
						      mode, 0);
		}
		break;
	case 1:
		pwm_disable(ldata->pwm_led);
		pwm_config(ldata->pwm_led, 64000, 2000000);
		pwm_enable(ldata->pwm_led);
		break;
	case 2:
		cancel_delayed_work_sync(&ldata->led_delayed_work);
		pwm_disable(ldata->pwm_led);
		ldata->duty_time_ms = 64;
		ldata->period_us = 2000000;
		queue_delayed_work(g_led_work_queue, &ldata->led_delayed_work,
				   msecs_to_jiffies(310));
		break;
	case 3:
		cancel_delayed_work_sync(&ldata->led_delayed_work);
		pwm_disable(ldata->pwm_led);
		ldata->duty_time_ms = 64;
		ldata->period_us = 2000000;
		queue_delayed_work(g_led_work_queue, &ldata->led_delayed_work,
				   msecs_to_jiffies(1000));
		break;
	case 4:
		pwm_disable(ldata->pwm_led);
		pwm_config(ldata->pwm_led, 1000000, 2000000);
#if 0
		pwm_conf.pwm_size = 9;
		pwm_conf.clk = PM_PWM_CLK_1KHZ;
		pwm_conf.pre_div = PM_PWM_PREDIVIDE_2;
		pwm_conf.pre_div_exp = 1;
		pwm_conf.pwm_value = 512/2;
		pwm_conf.bypass_lut = 1;
		pwm_configure(ldata->pwm_led, &pwm_conf);
#endif
		pwm_enable(ldata->pwm_led);
		break;
	case 5:
		pwm_disable(ldata->pwm_led);
		pwm_config(ldata->pwm_led, 100000, 200000);
		pwm_enable(ldata->pwm_led);
		break;
#ifdef CONFIG_HTC_HEADSET_MISC
	case 6:
		pm8058_pwm_config_led(ldata->pwm_led, id, mode,
				      ldata->out_current);
		pduties = &duties[ldata->start_index];
		pm8058_pwm_lut_config(ldata->pwm_led, ldata->period_us,
				      pduties, ldata->duty_time_ms,
				      ldata->start_index, ldata->duites_size,
				      0, 0, ldata->lut_flag);
		pm8058_pwm_lut_enable(ldata->pwm_led, 0);
		pm8058_pwm_lut_enable(ldata->pwm_led, 1);
		break;
	case 7:
		pwm_disable(ldata->pwm_led);
		pwm_config(ldata->pwm_led, 64000, 4000000);
		pwm_enable(ldata->pwm_led);
		break;
#endif
	default:
		LED_ERR_LOG(KERN_INFO "%s: bank %d not support blink %d\n", __func__, ldata->bank, val);
		return -EINVAL;
	}
	return count;
}

static ssize_t pm8058_led_blink_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int ret = 0;

	return ret;
}
static DEVICE_ATTR(blink, 0644, pm8058_led_blink_show,
				pm8058_led_blink_store);

static ssize_t pm8058_led_off_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;

	return ret;
}

static ssize_t pm8058_led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct pm8058_led_data *ldata;
	int min, sec;
	uint16_t off_timer;
	ktime_t interval;
	ktime_t next_alarm;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);

	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct pm8058_led_data, ldev);

	LED_INFO_LOG("Setting %s off_timer to %d min %d sec\n",
					   led_cdev->name, min, sec);

	off_timer = min * 60 + sec;

	alarm_cancel(&ldata->led_alarm);
	cancel_work_sync(&ldata->led_work);
	if (off_timer) {
		interval = ktime_set(off_timer, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&ldata->led_alarm, next_alarm, next_alarm);
	}
	return count;
}

static DEVICE_ATTR(off_timer, 0644, pm8058_led_off_timer_show,
				      pm8058_led_off_timer_store);

static ssize_t pm8058_led_currents_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *led_cdev;
	struct pm8058_led_data *ldata;

	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct pm8058_led_data, ldev);

	return sprintf(buf, "%d\n", ldata->out_current);
}

static ssize_t pm8058_led_currents_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int currents = 0;
	struct led_classdev *led_cdev;
	struct pm8058_led_data *ldata;

	sscanf(buf, "%d", &currents);
	if (currents < 0)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct pm8058_led_data, ldev);

	LED_INFO_LOG("%s: bank %d currents %d\n", __func__, ldata->bank,
	       currents);

	ldata->out_current = currents;

	ldata->ldev.brightness_set(led_cdev, 0);
	if (currents)
		ldata->ldev.brightness_set(led_cdev, 255);

	return count;
}

static DEVICE_ATTR(currents, 0644, pm8058_led_currents_show,
		   pm8058_led_currents_store);

static int pm8058_led_probe(struct platform_device *pdev)
{
	struct pm8058_led_platform_data *pdata;
	struct pm8058_led_data *ldata;
	int i, ret;

	ret = -ENOMEM;

	pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		LED_ERR_LOG("%s: platform data is NULL\n", __func__);
		return -ENODEV;
	}

	if (!pdata->num_leds) {
		LED_ERR_LOG("%s: LED num is 0\n", __func__);
		return 0;
	}

	ldata = kzalloc(sizeof(struct pm8058_led_data)
			* pdata->num_leds, GFP_KERNEL);
	if (ldata == NULL) {
		ret = -ENOMEM;
		LED_ERR_LOG("%s: failed on allocate ldata\n", __func__);
		goto err_exit;
	}

	dev_set_drvdata(&pdev->dev, ldata);

	wake_lock_init(&pmic_led_wake_lock, WAKE_LOCK_SUSPEND, "pmic_led");

	g_led_work_queue = create_workqueue("led");
	if (!g_led_work_queue)
		goto err_create_work_queue;

	for (i = 0; i < 64; i++)
		duties[i] = pdata->duties[i];

	for (i = 0; i < pdata->num_leds; i++) {
		ldata[i].led_config = pdata->led_config + i;
		ldata[i].ldev.name = pdata->led_config[i].name;
		ldata[i].bank = pdata->led_config[i].bank;
		ldata[i].flags =  pdata->led_config[i].flags;
		ldata[i].pwm_size =  pdata->led_config[i].pwm_size;
		ldata[i].clk =  pdata->led_config[i].clk;
		ldata[i].pre_div =  pdata->led_config[i].pre_div;
		ldata[i].pre_div_exp =  pdata->led_config[i].pre_div_exp;
		ldata[i].pwm_value =  pdata->led_config[i].pwm_value;
		ldata[i].period_us =  pdata->led_config[i].period_us;
		ldata[i].start_index =  pdata->led_config[i].start_index;
		ldata[i].duites_size =  pdata->led_config[i].duites_size;
		ldata[i].duty_time_ms =  pdata->led_config[i].duty_time_ms;
		ldata[i].lut_flag =  pdata->led_config[i].lut_flag;
		ldata[i].out_current =  pdata->led_config[i].out_current;
		switch (pdata->led_config[i].type) {
		case PM8058_LED_CURRENT:
			if (ldata[i].flags & PM8058_LED_BLINK_EN)
				INIT_DELAYED_WORK(&ldata[i].led_delayed_work,
						  led_blink_do_work);
			else
				INIT_DELAYED_WORK(&ldata[i].led_delayed_work,
						  pwm_lut_delayed_fade_out);
			ldata[i].pwm_led = pwm_request(ldata[i].bank,
						ldata[i].ldev.name);
			ldata[i].ldev.brightness_set =
					pm8058_drvx_led_brightness_set;
			break;
		case PM8058_LED_RGB:
			INIT_DELAYED_WORK(&ldata[i].led_delayed_work,
					  led_blink_do_work);
		case PM8058_LED_PWM:
			ldata[i].pwm_led = pwm_request(ldata[i].bank,
						       ldata[i].ldev.name);
			ldata[i].ldev.brightness_set =
					pm8058_pwm_led_brightness_set;
			break;
		case PM8058_LED_DRVX:
			if (ldata[i].flags & PM8058_LED_BLINK_EN)
				INIT_DELAYED_WORK(&ldata[i].led_delayed_work,
						  led_blink_do_work);
			else
				INIT_DELAYED_WORK(&ldata[i].led_delayed_work,
						  pwm_lut_delayed_fade_out);
			ldata[i].pwm_led = pwm_request(ldata[i].bank,
						ldata[i].ldev.name);
			ldata[i].ldev.brightness_set =
					pm8058_drvx_led_brightness_set;
			break;
		}

		ret = led_classdev_register(&pdev->dev, &ldata[i].ldev);
		if (ret < 0) {
			LED_ERR_LOG("%s: failed on led_classdev_register [%s]\n",
				__func__, ldata[i].ldev.name);
			goto err_register_led_cdev;
		}
		if (ldata[i].flags & PM8058_LED_LTU_EN)
			for_key_led_data = &ldata[i];
	}

	for (i = 0; i < pdata->num_leds; i++) {
		if (pdata->led_config[i].type == PM8058_LED_RGB ||
		    ldata[i].flags & PM8058_LED_BLINK_EN) {
			ret = device_create_file(ldata[i].ldev.dev,
						 &dev_attr_blink);
			if (ret < 0) {
				LED_ERR_LOG("%s: Failed to create attr blink"
				       " [%d]\n", __func__, i);
				goto err_register_attr_blink;
			}
		}
	}

	for (i = 0; i < pdata->num_leds; i++) {
		if (pdata->led_config[i].type == PM8058_LED_RGB ||
		    ldata[i].flags & PM8058_LED_BLINK_EN) {
			ret = device_create_file(ldata[i].ldev.dev,
						 &dev_attr_off_timer);
			if (ret < 0) {
				LED_ERR_LOG("%s: Failed to create attr off timer"
				       " [%d]\n", __func__, i);
				goto err_register_attr_off_timer;
			}
			INIT_WORK(&ldata[i].led_work, led_work_func);
			alarm_init(&ldata[i].led_alarm,
				   ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
				   led_alarm_handler);
		}
	}

	for (i = 0; i < pdata->num_leds; i++) {
		if (ldata[i].bank < 3)
			continue;
		ret = device_create_file(ldata[i].ldev.dev, &dev_attr_currents);
		if (ret < 0) {
			LED_ERR_LOG("%s: Failed to create attr blink [%d]\n",
			       __func__, i);
			goto err_register_attr_currents;
		}
	}

	return 0;

err_register_attr_currents:
	for (i--; i >= 0; i--) {
		if (ldata[i].bank < 3)
			continue;
		device_remove_file(ldata[i].ldev.dev, &dev_attr_currents);
	}
	i = pdata->num_leds;

err_register_attr_off_timer:
	for (i--; i >= 0; i--) {
		if (pdata->led_config[i].type == PM8058_LED_RGB ||
		    ldata[i].flags & PM8058_LED_BLINK_EN) {
			device_remove_file(ldata[i].ldev.dev,
					   &dev_attr_off_timer);
		}
	}
	i = pdata->num_leds;

err_register_attr_blink:
	for (i--; i >= 0; i--) {
		if (pdata->led_config[i].type == PM8058_LED_RGB ||
		    ldata[i].flags & PM8058_LED_BLINK_EN) {
			device_remove_file(ldata[i].ldev.dev, &dev_attr_blink);
		}
	}
	i = pdata->num_leds;

err_register_led_cdev:
	for (i--; i >= 0; i--) {
		switch (pdata->led_config[i].type) {
		case PM8058_LED_RGB:
		case PM8058_LED_PWM:
		case PM8058_LED_DRVX:
			pwm_free(ldata[i].pwm_led);
			break;
		}
		led_classdev_unregister(&ldata[i].ldev);
	}
	destroy_workqueue(g_led_work_queue);

err_create_work_queue:
	kfree(ldata);

err_exit:
	wake_lock_destroy(&pmic_led_wake_lock);
	return ret;
}

static int __devexit pm8058_led_remove(struct platform_device *pdev)
{
	struct pm8058_led_platform_data *pdata;
	struct pm8058_led_data *ldata;
	int i;

	pdata = pdev->dev.platform_data;
	ldata = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&ldata[i].ldev);
		switch (pdata->led_config[i].type) {
		case PM8058_LED_RGB:
			pwm_free(ldata[i].pwm_led);
			break;
		case PM8058_LED_PWM:
		case PM8058_LED_DRVX:
			pwm_free(ldata[i].pwm_led);
			break;
		}

		if (pdata->led_config[i].type == PM8058_LED_RGB ||
		    ldata[i].flags & PM8058_LED_BLINK_EN) {
			device_remove_file(ldata[i].ldev.dev,
					   &dev_attr_blink);
			device_remove_file(ldata[i].ldev.dev,
					   &dev_attr_off_timer);
		}

		if (ldata[i].bank >= 3)
			device_remove_file(ldata[i].ldev.dev,
					   &dev_attr_currents);
	}

	wake_lock_destroy(&pmic_led_wake_lock);
	destroy_workqueue(g_led_work_queue);
	kfree(ldata);

	return 0;
}

static struct platform_driver pm8058_led_driver = {
	.probe = pm8058_led_probe,
	.remove = __devexit_p(pm8058_led_remove),
	.driver = {
		   .name = "leds-pm8058",
		   .owner = THIS_MODULE,
		   },
};

int __init pm8058_led_init(void)
{
	return platform_driver_register(&pm8058_led_driver);
}

void pm8058_led_exit(void)
{
	platform_driver_unregister(&pm8058_led_driver);
}

module_init(pm8058_led_init);
module_exit(pm8058_led_exit);

MODULE_DESCRIPTION("pm8058 led driver");
MODULE_LICENSE("GPL");
