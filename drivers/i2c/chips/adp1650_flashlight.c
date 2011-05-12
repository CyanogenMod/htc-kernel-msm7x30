/* drivers/i2c/chips/adp1650_flashlight.c
 *
 * Copyright (C) 2008-2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/msm_iomap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <mach/msm_flashlight.h>
#include <linux/adp1650_flashlight.h>

#define FLT_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[FLT]" fmt, ##__VA_ARGS__)
#define FLT_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[FLT]" fmt, ##__VA_ARGS__)
#define FLT_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[FLT][ERR]" fmt, ##__VA_ARGS__)

#define DEBUG 1
#define ADP1650_RETRY_COUNT 10

static struct {
	enum flashlight_mode_flags	mode;
	char 				*name;
	enum led_brightness		brightness;
} fl_mode_t[6] = {
	{FL_MODE_OFF, 		"off",			0},
	{FL_MODE_TORCH,		"torch",		LED_HALF},
	{FL_MODE_FLASH,		"flash",		LED_FULL},
	{FL_MODE_PRE_FLASH,	"pre-flash",		(LED_HALF+1)},
	{FL_MODE_TORCH_LEVEL_1,	"torch level 1",	(LED_HALF-2)},
	{FL_MODE_TORCH_LEVEL_2,	"torch level 2",	(LED_HALF-1)},
};

struct adp1650_data {
	struct led_classdev 		fl_lcdev;
	struct early_suspend		fl_early_suspend;
	enum flashlight_mode_flags 	mode_status;
	uint32_t			flash;
	uint32_t			timeout;
	struct hrtimer			timer;
	spinlock_t			spin_lock;
};

static struct i2c_client *this_client;
static struct adp1650_data *this_adp1650;
static ktime_t ktime;

static int ADP1650_I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < ADP1650_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= ADP1650_RETRY_COUNT) {
		FLT_ERR_LOG("%s retry over %d\n", __func__,
							ADP1650_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int ADP1650_I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < ADP1650_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= ADP1650_RETRY_COUNT) {
		FLT_ERR_LOG("%s retry over %d\n", __func__,
							ADP1650_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int chip_info(uint8_t reg)
{
	uint8_t buffer[2];
	int ret;

	buffer[0] = reg;
	/* Read data */
	ret = ADP1650_I2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;

	FLT_INFO_LOG("%s: %x\n", __func__, buffer[0]);
	return ret;
}

static int assist_light(uint8_t i_tor)
{
	uint8_t buffer[2];
	int ret;

	buffer[0] = CURRENT_SET_REG;
	buffer[1] = i_tor;
	ret = ADP1650_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	buffer[0] = OUTPUT_MODE_REG;
	buffer[1] = OUTPUT_MODE_DEF | OUTPUT_MODE_ASSIST;
	ret = ADP1650_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	return 0;
}

static int flash_light(uint8_t i_fl, uint8_t fl_tim)
{
	uint8_t buffer[2];
	int ret;

	buffer[0] = CURRENT_SET_REG;
	buffer[1] = i_fl;
	ret = ADP1650_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	buffer[0] = TIMER_REG;
	buffer[1] = fl_tim;
	ret = ADP1650_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	buffer[0] = OUTPUT_MODE_REG;
	buffer[1] = OUTPUT_MODE_DEF | OUTPUT_MODE_FLASH;
	ret = ADP1650_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	return 0;
}

static int enable_low_batt_support(uint8_t value)
{
	uint8_t buffer[2];
	int ret;

	buffer[0] = BATT_LOW_REG;
	buffer[1] = value;
	ret = ADP1650_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	return 0;
}

static int turn_off(void)
{
	uint8_t buffer[2];
	int ret;

	if (this_adp1650->mode_status == FL_MODE_OFF)
		return 0;

	buffer[0] = OUTPUT_MODE_REG;
	buffer[1] = OUTPUT_MODE_DEF;
	ret = ADP1650_I2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	this_adp1650->mode_status = FL_MODE_OFF;
	this_adp1650->fl_lcdev.brightness = 0;
	return 0;
}

int adp1650_flashlight_control(int mode)
{
	int ret = 0;
	unsigned long flag = 0;
	uint32_t prev_mode = this_adp1650->mode_status;

	switch (mode) {
	case FL_MODE_OFF:
		turn_off();
		break;

	case FL_MODE_TORCH:
		assist_light(CUR_TOR_75MA);
		break;

	case FL_MODE_FLASH:
		if (prev_mode == FL_MODE_FLASH) {
			spin_lock_irqsave(&this_adp1650->spin_lock, flag);
			hrtimer_cancel(&this_adp1650->timer);
			spin_unlock_irqrestore(&this_adp1650->spin_lock, flag);
		}

		flash_light(CUR_FL_700MA, TIMER_600MS);
		spin_lock_irqsave(&this_adp1650->spin_lock, flag);
		gpio_set_value(this_adp1650->flash, 1);
		hrtimer_start(&this_adp1650->timer, ktime, HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&this_adp1650->spin_lock, flag);
		break;

	case FL_MODE_PRE_FLASH:
		assist_light(CUR_TOR_125MA);
		break;

	case FL_MODE_TORCH_LEVEL_1:
		assist_light(CUR_TOR_25MA);
		break;

	case FL_MODE_TORCH_LEVEL_2:
		assist_light(CUR_TOR_50MA);
		break;

	default:
		turn_off();
		FLT_ERR_LOG("%s: unknown mode %d\n", __func__, mode);
		return -EINVAL;
	}

	if (mode != FL_MODE_FLASH) {
		spin_lock_irqsave(&this_adp1650->spin_lock, flag);
		if (prev_mode == FL_MODE_FLASH) {
			if (hrtimer_cancel(&this_adp1650->timer))
				gpio_set_value(this_adp1650->flash, 0);
		}
		spin_unlock_irqrestore(&this_adp1650->spin_lock, flag);
	}

	this_adp1650->mode_status = mode;

	FLT_INFO_LOG("%s: mode: %d\n", FLASHLIGHT_NAME, mode);
	return ret;
}

static void fl_lcdev_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness brightness)
{
	enum flashlight_mode_flags mode = FL_MODE_OFF;
	int i = 0, ret = -1;
	char *mode_name = NULL;

	if (brightness < 0 || brightness > LED_FULL) {
		FLT_ERR_LOG("%s: invalid brightness %d\n",
						__func__, brightness);
	}

	for (i = 0; i < ARRAY_SIZE(fl_mode_t); i++) {
		if (brightness == fl_mode_t[i].brightness) {
			mode = fl_mode_t[i].mode;
			mode_name = fl_mode_t[i].name;
			break;
		}
	}

	if (!mode_name) {
		FLT_ERR_LOG("%s: no matching brightness for %d\n",
						__func__, brightness);
		return;
	}

	ret = adp1650_flashlight_control(mode);
	if (ret) {
		FLT_ERR_LOG("%s: control failure rc:%d\n", __func__, ret);
		return;
	}

	this_adp1650->fl_lcdev.brightness = brightness;
}

static void flashlight_early_suspend(struct early_suspend *handler)
{
	struct adp1650_data *fl_str = container_of(handler,
				struct adp1650_data, fl_early_suspend);
	unsigned long flag = 0;
	uint32_t prev_mode = fl_str->mode_status;
	FLT_INFO_LOG("%s\n", __func__);

	if (fl_str != NULL && fl_str->mode_status) {
		turn_off();
		spin_lock_irqsave(&fl_str->spin_lock, flag);
		if (prev_mode == FL_MODE_FLASH) {
			if (hrtimer_cancel(&fl_str->timer))
				gpio_set_value(fl_str->flash, 0);
		}
		spin_unlock_irqrestore(&fl_str->spin_lock, flag);
	}
}

static void flashlight_late_resume(struct early_suspend *handler)
{

}

static enum hrtimer_restart flash_gpio_timer_func(struct hrtimer *timer)
{
	struct adp1650_data *adp1650 = container_of(timer,
				struct adp1650_data, timer);
	unsigned long flag = 0;

	spin_lock_irqsave(&this_adp1650->spin_lock, flag);
	if (gpio_get_value(adp1650->flash))
		gpio_set_value(adp1650->flash, 0);
	spin_unlock_irqrestore(&this_adp1650->spin_lock, flag);

	return HRTIMER_NORESTART;
}

static int adp1650_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct adp1650_data *adp1650;
	struct flashlight_platform_data *pdata;
	int err = 0;

	FLT_INFO_LOG("%s:\n", __func__);
	pdata = client->dev.platform_data;
	if (!pdata) {
		FLT_ERR_LOG("%s: Assign platform_data error!!\n", __func__);
		return -EINVAL;
	}
	if (pdata->gpio_init)
		pdata->gpio_init();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto check_functionality_failed;
	}

	adp1650 = kzalloc(sizeof(struct adp1650_data), GFP_KERNEL);
	if (!adp1650) {
		FLT_ERR_LOG("%s: kzalloc fail !!!\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, adp1650);
	this_client = client;

	/* Register led class device */
	adp1650->fl_lcdev.name           = client->name;
	adp1650->fl_lcdev.brightness     = 0;
	adp1650->fl_lcdev.brightness_set = fl_lcdev_brightness_set;
	adp1650->flash                   = pdata->flash;
	adp1650->timeout                 = pdata->flash_duration_ms;
	if (adp1650->timeout <= 0)
		adp1650->timeout = 600;
	ktime = ktime_set(adp1650->timeout / 1000,
			(adp1650->timeout % 1000) * 1000000);
	hrtimer_init(&adp1650->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	adp1650->timer.function = flash_gpio_timer_func;
	spin_lock_init(&adp1650->spin_lock);

	err = led_classdev_register(&client->dev, &adp1650->fl_lcdev);
	if (err < 0) {
		FLT_ERR_LOG("%s: failed on led_classdev_register\n", __func__);
		goto platform_data_null;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	adp1650->fl_early_suspend.suspend = flashlight_early_suspend;
	adp1650->fl_early_suspend.resume  = flashlight_late_resume;
	register_early_suspend(&adp1650->fl_early_suspend);
#endif

	this_adp1650 = adp1650;

	chip_info(0);
	enable_low_batt_support(0x81);
	return 0;


platform_data_null:
	if (adp1650->flash) {
		gpio_free(adp1650->flash);
		adp1650->flash = 0;
	}
	kfree(adp1650);
check_functionality_failed:
	return err;
}

static int adp1650_remove(struct i2c_client *client)
{
	struct adp1650_data *adp1650 = i2c_get_clientdata(client);

	hrtimer_cancel(&adp1650->timer);
	if (adp1650->flash) {
		gpio_free(adp1650->flash);
		adp1650->flash = 0;
	}
	led_classdev_unregister(&adp1650->fl_lcdev);
	//i2c_detach_client(client);
	kfree(adp1650);

	FLT_INFO_LOG("%s:\n", __func__);
	return 0;
}

static const struct i2c_device_id adp1650_id[] = {
	{ FLASHLIGHT_NAME, 0 },
	{ }
};

static struct i2c_driver adp1650_driver = {
	.probe		= adp1650_probe,
	.remove		= adp1650_remove,
	.id_table	= adp1650_id,
	.driver		= {
		.name = FLASHLIGHT_NAME,
	},
};

static int __init adp1650_init(void)
{
	FLT_INFO_LOG("adp1650 Led Flash driver: init\n");
	return i2c_add_driver(&adp1650_driver);
}

static void __exit adp1650_exit(void)
{
	i2c_del_driver(&adp1650_driver);
}

module_init(adp1650_init);
module_exit(adp1650_exit);

MODULE_DESCRIPTION("ADP1650 Led Flash driver");
MODULE_LICENSE("GPL");
