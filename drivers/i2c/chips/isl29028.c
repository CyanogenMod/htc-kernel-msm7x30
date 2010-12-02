/* drivers/i2c/chips/isl29028.c - isl29028 optical sensors driver
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
#include <asm/mach-types.h>
#include <linux/isl29028.h>
#include <linux/capella_cm3602.h>
#include <asm/setup.h>
#include <linux/wakelock.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

/*#define DEBUG_PROXIMITY*/

#define INTR_DEFAULT 0x04
#define CONFIG_DEFAULT 0x62

#define INTR_MASK 0x77
#define CONFIG_MASK 0x7B

static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

static void report_near_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_near_work, report_near_do_work);

#ifdef DEBUG_PROXIMITY
static void info_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(info_work, info_do_work);
#endif

struct isl29028_info {
	struct class *isl29028_class;
	struct device *ls_dev;
	struct device *ps_dev;

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;

	struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;

	int als_enable;
	int ls_enable_flag;

	int ps_enable;
	int ps_irq_flag;
	uint8_t ps_lt;
	uint8_t ps_ht;
	uint8_t ps_B_val;
	uint8_t ps_C_val;
	uint8_t ps_A_val;
	uint8_t ps_X_val;
	int led;
	uint8_t default_ps_lt;
	uint8_t default_ps_ht;

	uint16_t *adc_table;
	uint16_t cali_table[10];
	int irq;

	int is_suspend;
	int ls_calibrate;

	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t als_kadc;
	uint32_t als_gadc;
	uint16_t golden_adc;

	struct wake_lock ps_wake_lock;
	int psensor_opened;
	int lightsensor_opened;

	uint8_t debounce;
};

struct isl29028_info *lp_info;

/*static uint32_t als_kadc;*/
static uint32_t ps_threshold;

static int I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
		 .addr = lp_info->i2c_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = lp_info->i2c_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = lp_info->i2c_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static uint16_t get_ls_adc_value(uint16_t *raw_adc_value)
{
	uint16_t value, tmp_value, ret;
	struct isl29028_info *lpi = lp_info;
	char buffer[3];

	buffer[0] = ISL29028_LS_DATA1;
	ret = I2C_RxData(buffer, 2);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail\n", __func__);
		return ret;
	}
	value = buffer[0];
	tmp_value = buffer[1];

	/*D("%s: value = 0x%03X, tmp_value = 0x%03X\n",
		__func__, value, tmp_value);*/

	value = value | tmp_value << 8;
	*raw_adc_value = value;

	if (value > 0xFFF) {
		printk(KERN_WARNING "%s: get wrong value: 0x%X\n",
			__func__, value);
		return -1;
	} else {
		if (!lpi->ls_calibrate) {
			value = value * lpi->als_gadc / lpi->als_kadc;
			if (value > 0xFFF)
				value = 0xFFF;
		}
	}

	return value & 0x0FFF;
}

static uint16_t get_ps_adc_value(void)
{
	uint16_t value;
	struct isl29028_info *lpi = lp_info;
	value = i2c_smbus_read_byte_data(lpi->i2c_client,
			ISL29028_PROX_DATA);
	return value & 0xff;
}

static int _isl29028_set_reg_bit(struct i2c_client *client, u8 set,
				u8 cmd, u8 data)
{
	char buffer[2];
	u8 value;
	int ret = 0;

	buffer[0] = cmd;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail\n", __func__);
		return ret;
	}
	value = buffer[0];
	/*printk(KERN_DEBUG "Andy, %s: I2C_RxData[0x%x] = 0x%x\n",
		__func__, cmd, value);*/

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_TxData(buffer, 2);
	if (ret < 0) {
		pr_err("%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	/* Debug use */
	/*value = i2c_smbus_read_byte_data(client, cmd);
	printk(KERN_DEBUG "Andy, %s: After set reg bit[0x%x] = 0x%x\n",
		__func__, cmd, value);*/

	return ret;
}

static int set_lsensor_range(uint16_t lt, uint16_t ht)
{
	uint16_t value = 0;
	int ret;
	char buffer[4];

	value = (lt >> 8) & 0x0f;
	value |= ht << 4;
	value &= 0xff;

	buffer[0] = ISL29028_LS_TH1;
	buffer[1] = lt & 0xFF;
	buffer[2] = value;
	buffer[3] = (ht >> 4) & 0xFF;
	ret = I2C_TxData(buffer, 4);
	if (ret < 0) {
		printk(KERN_ERR "%s : I2C_TxData fail\n", __func__);
		return ret;
	}

/*
	value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29028_LS_TH1);
	D("TH1--------->0x%03X\n", value);
	value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29028_LS_TH2);
	D("TH2--------->0x%03X\n", value);
	value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29028_LS_TH3);
	D("TH3--------->0x%03X\n", value);
*/
	return ret;
}

static int set_psensor_range(u8 lt, u8 ht)
{
	int ret;
	char buffer[3];
	struct isl29028_info *lpi = lp_info;

	if (!lpi) {
		pr_err("%s: ls_info is empty\n", __func__);
		return -EINVAL;
	}

	D("%s: PS ps_threshold = 0x%x\n",
			__func__, ps_threshold);

	if (ps_threshold >> 16 == PS_CALIBRATED) {
		lt = (ps_threshold >> 8) & 0xFF;
		ht = ps_threshold & 0xFF;
	}

	D("%s: Setting psensor range (0x%X, 0x%X)\n",
			__func__, lt, ht);

	buffer[0] = ISL29028_PROX_LT;
	buffer[1] = lt;
	buffer[2] = ht;
	ret = I2C_TxData(buffer, 3);
	if (ret < 0) {
		printk(KERN_ERR "%s : I2C_TxData fail\n", __func__);
		return ret;
	}

	return ret;
}

static void report_psensor_input_event(struct isl29028_info *lpi,
					uint16_t ps_adc)
{
	int val, ret;

	if (lpi->debounce == 1)
		cancel_delayed_work(&report_near_work);

	if (ps_adc > lpi->ps_ht)
		val = 0;
	else if (ps_adc <= lpi->ps_ht)
		val = 1;
	else
		D("%s: Proximity adc value not as expected!\n", __func__);

	D("proximity %s\n", val ? "FAR" : "NEAR");

	if (lpi->debounce == 1) {
		if (val == 0) {
			queue_delayed_work(lpi->lp_wq, &report_near_work,
				msecs_to_jiffies(500));
			return;
		} else {
			/* dummy report */
			input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, -1);
			input_sync(lpi->ps_input_dev);
		}
	}

	/* 0 is close, 1 is far */
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
	input_sync(lpi->ps_input_dev);

	ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
		ISL29028_INTERRUPT, ISL29028_INT_ALS_FLAG);
	if (ret < 0)
		pr_err("%s: clear lsensor intr flag fail\n", __func__);

	wake_lock_timeout(&(lpi->ps_wake_lock), 2*HZ);
}

static void report_lsensor_input_event(struct isl29028_info *lpi)
{
	uint16_t adc_value, raw_adc_value;
	int level = 0, i, ret;

	adc_value = get_ls_adc_value(&raw_adc_value);

	for (i = 0; i < 10; i++) {
		if (adc_value <= (*(lpi->adc_table + i))) {
			level = i;
			if (*(lpi->adc_table + i))
				break;
		}
	}

	ret = set_lsensor_range((i == 0) ? 0 :
			*(lpi->cali_table + (i - 1)) + 1,
		*(lpi->cali_table + i));
	if (ret < 0)
		printk(KERN_ERR "%s fail\n", __func__);

	D("ALS_ADC = 0x%03X, Level = %d \n", adc_value, level);
	/*D("%s: RAW ADC = 0x%03X\n", __func__, raw_adc_value);*/
	input_report_abs(lpi->ls_input_dev, ABS_MISC, level);
	input_sync(lpi->ls_input_dev);

	ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
		ISL29028_INTERRUPT, ISL29028_INT_ALS_FLAG);
	if (ret < 0)
		pr_err("%s: clear lsensor intr flag fail\n", __func__);

}

static int lightsensor_real_enable(struct isl29028_info *lpi)
{
	int ret = -1;

	/*D("%s\n", __func__);*/

	ret = _isl29028_set_reg_bit(lpi->i2c_client, 1,
		ISL29028_CONFIGURE, ISL29028_ALS_EN);
	if (ret < 0)
		pr_err("%s: error\n", __func__);
	else {
		lpi->als_enable = 1;
		input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
		input_sync(lpi->ls_input_dev);

		report_lsensor_input_event(lpi);
	}

	return ret;
}

static int is_only_ls_enabled(uint8_t reg_config)
{
	int ret = ((reg_config & ISL29028_ALS_EN) &&
		(!(reg_config & ISL29028_PROX_EN)));

	return ret;
}

static void __report_psensor_near(struct isl29028_info *lpi, uint16_t ps_adc)
{
	uint8_t ret;

	set_irq_type(lpi->irq, IRQF_TRIGGER_HIGH);

	if (lpi->als_enable) {
		ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: disable auto light sensor"
			" fail\n", __func__);
		else
			lpi->als_enable = 0;
	}

	report_psensor_input_event(lpi, ps_adc);
	lpi->ps_irq_flag = 1;
}

static void judge_and_enable_lightsensor(struct isl29028_info *lpi)
{
	int ret;

	if (lpi->ls_enable_flag && (lpi->ps_irq_flag == 0)) {
		ret = lightsensor_real_enable(lpi);
		if (ret < 0)
			pr_err("%s: lightsensor_real_enable: not enabled!\n",
				__func__);
	}
}

static void __report_psensor_far(struct isl29028_info *lpi, uint16_t ps_adc)
{
	set_irq_type(lpi->irq, IRQF_TRIGGER_LOW);

	report_psensor_input_event(lpi, ps_adc);

	lpi->ps_irq_flag = 0;

	judge_and_enable_lightsensor(lpi);
}

static void clear_intr_flags(uint8_t intrrupt, struct isl29028_info *lpi)
{
	uint8_t ret;

	if (intrrupt & ISL29028_INT_ALS_FLAG) {
		ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
			ISL29028_INTERRUPT, ISL29028_INT_ALS_FLAG);
		if (ret < 0)
			pr_err("%s: clear lsensor intr flag fail\n",
				__func__);
	}
	if (intrrupt & ISL29028_INT_PROX_FLAG) {
		ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
			ISL29028_INTERRUPT, ISL29028_INT_PROX_FLAG);
		if (ret < 0)
			pr_err("%s: clear psensor intr flag fail\n",
				__func__);
	}
}

static void check_and_recover(struct isl29028_info *lpi)
{
	uint8_t intrrupt, reg_config;
	uint8_t def_intrrupt, def_reg_config;
	char buffer[2];
	int ret = 0;
	uint8_t orig_enabled;

	buffer[0] = ISL29028_INTERRUPT;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail (ISL29028_INTERRUPT)\n",
			__func__);
		return;
	}
	intrrupt = buffer[0];

	buffer[0] = ISL29028_CONFIGURE;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail (ISL29028_CONFIGURE)\n",
			__func__);
		return;
	}
	reg_config = buffer[0];

	def_intrrupt = (intrrupt & INTR_MASK);
	def_reg_config = (reg_config & CONFIG_MASK);
	orig_enabled = ((reg_config & ISL29028_PROX_EN) |
			(reg_config & ISL29028_ALS_EN));

	if (def_intrrupt != INTR_DEFAULT) {
		buffer[0] = ISL29028_INTERRUPT;
		buffer[1] = INTR_DEFAULT;
		ret = I2C_TxData(buffer, 2);
		if (ret < 0)
			printk(KERN_ERR "%s : Set LPS INTR fail\n", __func__);
	}

	if (def_reg_config != CONFIG_DEFAULT) {
		buffer[0] = ISL29028_CONFIGURE;
		buffer[1] = (CONFIG_DEFAULT | orig_enabled);
		ret = I2C_TxData(buffer, 2);
		if (ret < 0)
			printk(KERN_ERR "%s : Set LPS Configuration fail\n",
				__func__);
	}
}

static void sensor_irq_do_work(struct work_struct *work)
{
	uint8_t intrrupt, ret, reg_config;
	struct isl29028_info *lpi = lp_info;
	char buffer[2];

	int value1 = -1;

	uint16_t ps_adc = 0;

	value1 = gpio_get_value(lpi->intr_pin);
	/*D("%s: lpi->intr_pin = %d\n", __func__, value1);*/

	buffer[0] = ISL29028_INTERRUPT;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail (ISL29028_INTERRUPT)\n",
			__func__);
		return;
	}
	intrrupt = buffer[0];
	D("isl_irq: INT = 0x%x\n", intrrupt);

	buffer[0] = ISL29028_CONFIGURE;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail (ISL29028_CONFIGURE)\n",
			__func__);
		return;
	}
	reg_config = buffer[0];
	D("isl_irq: CONFIG = 0x%x, INT_PIN = %d\n", reg_config, value1);

	if (reg_config & ISL29028_PROX_EN) {

		ps_adc = get_ps_adc_value();
		D("isl_irq: ps_adc = 0x%02X, ps_lt = 0x%02X, ps_ht = 0x%02X\n",
			ps_adc, lpi->ps_lt, lpi->ps_ht);

		if (reg_config & ISL29028_ALS_EN) {
			if (ps_adc > lpi->ps_ht)
				__report_psensor_near(lpi, ps_adc);
			else if (intrrupt & ISL29028_INT_ALS_FLAG)
				report_lsensor_input_event(lpi);
			else {
				D("%s: Suppose ps and ls enabled or "
					"LS interrupt, but NO!\n", __func__);
				clear_intr_flags(intrrupt, lpi);
			}
		} else {
			if (ps_adc < lpi->ps_lt)
				__report_psensor_far(lpi, ps_adc);
			else if (ps_adc > lpi->ps_ht)
				__report_psensor_near(lpi, ps_adc);
			else {
				if (value1 == 0)
					__report_psensor_near(lpi,
						(lpi->ps_ht + 1));
				else
					__report_psensor_far(lpi,
						(lpi->ps_lt - 1));
			}
		}
	} else if (is_only_ls_enabled(reg_config))
		report_lsensor_input_event(lpi);
	else {
		D("%s: ps and ls both disabled!\n", __func__);
		clear_intr_flags(intrrupt, lpi);
	}

	check_and_recover(lpi);

	enable_irq(lpi->irq);
}

static void report_near_do_work(struct work_struct *w)
{
	struct isl29028_info *lpi = lp_info;
	int ret;

	D("%s\n", __func__);

	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 0);
	input_sync(lpi->ps_input_dev);

	ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
		ISL29028_INTERRUPT, ISL29028_INT_ALS_FLAG);
	if (ret < 0)
		pr_err("%s: clear lsensor intr flag fail\n", __func__);

	wake_lock_timeout(&(lpi->ps_wake_lock), 2*HZ);
}

/*#ifdef DEBUG_PROXIMITY*/
static void info_do_work(struct work_struct *w)
{
	struct isl29028_info *lpi = lp_info;
	uint8_t intrrupt, reg_config;
	int ret, i = 0, level = 0;
	int value1;
	uint16_t value, TH_value;
	char buffer[2];
	uint16_t adc_value, raw_adc_value;

	value1 = gpio_get_value(lpi->intr_pin);
	D("\n%s: intr_pin = %d, value of intr_pin = %d\n",
		__func__, lpi->intr_pin, value1);

	value = get_ps_adc_value();

	D("%s: PS_ADC[0x%03X], ENABLE = %d, intr_pin = %d\n",
		__func__, value, lpi->ps_enable, value1);

	adc_value = get_ls_adc_value(&raw_adc_value);

	for (i = 0; i < 10; i++) {
		if (adc_value <= (*(lpi->adc_table + i))) {
			level = i;
			if (*(lpi->adc_table + i))
				break;
		}
	}

	D("%s: LS_ADC = 0x%03X, Level = %d \n", __func__, adc_value, level);
	D("%s: LS_RAW ADC = 0x%03X\n", __func__, raw_adc_value);

	TH_value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29028_LS_TH1);
	D("%s: LS_TH1--------->0x%03X\n", __func__, TH_value);
	TH_value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29028_LS_TH2);
	D("%s: LS_TH2--------->0x%03X\n", __func__, TH_value);
	TH_value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29028_LS_TH3);
	D("%s: LS_TH3--------->0x%03X\n", __func__, TH_value);

	TH_value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29028_PROX_LT);
	D("%s: PROX_LT--------->0x%03X\n", __func__, TH_value);
	TH_value = i2c_smbus_read_byte_data(lpi->i2c_client, ISL29828_PROX_HT);
	D("%s: PROX_HT--------->0x%03X\n", __func__, TH_value);

	buffer[0] = ISL29028_INTERRUPT;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail (ISL29028_INTERRUPT)\n",
			__func__);
		return;
	}
	intrrupt = buffer[0];

	buffer[0] = ISL29028_CONFIGURE;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail (ISL29028_CONFIGURE)\n",
			__func__);
		return;
	}
	reg_config = buffer[0];

	D("%s: reg_config = 0x%x\n", __func__, reg_config);
	D("%s: intrrupt = 0x%x\n", __func__, intrrupt);

	/*queue_delayed_work(lpi->lp_wq, &info_work,
		msecs_to_jiffies(3000));*/
}
/*#endif*/

static irqreturn_t isl29028_irq_handler(int irq, void *data)
{
	struct isl29028_info *lpi = data;

	/*int value1;
	value1 = gpio_get_value(lpi->intr_pin);
	D("\n%s: intr_pin = %d, value of intr_pin = %d\n",
		__func__, lpi->intr_pin, value1);*/

	disable_irq_nosync(lpi->irq);

	/*D("%s\n", __func__);*/

	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct isl29028_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static int psensor_enable(struct isl29028_info *lpi)
{
	int ret;
	uint16_t ps_adc = 0;

	D("%s\n", __func__);
	if (lpi->ps_enable) {
		D("%s: already enabled\n", __func__);
		return 0;
	}

	/* dummy report */
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, -1);
	input_sync(lpi->ps_input_dev);

	if (lpi->power)
		lpi->power(PS_PWR_ON, 1);

	ret = _isl29028_set_reg_bit(lpi->i2c_client, 1,
		ISL29028_CONFIGURE, ISL29028_PROX_EN);
	if (ret < 0) {
		pr_err("%s: enable psensor fail\n", __func__);
		return ret;
	}

#ifdef DEBUG_PROXIMITY
	queue_delayed_work(lpi->lp_wq, &info_work,
		msecs_to_jiffies(3000));
#endif

	msleep(15);
	ps_adc = get_ps_adc_value();
	report_psensor_input_event(lpi, ps_adc);

	lpi->ps_enable = 1;

	ret = set_irq_wake(lpi->irq, 1);
	if (ret < 0) {
		pr_err("%s: failed to disable irq %d as a wake interrupt\n",
			__func__, lpi->irq);
		return ret;
	}

	return ret;
}

static int psensor_disable(struct isl29028_info *lpi)
{
	int ret = -EIO;

#ifdef DEBUG_PROXIMITY
	cancel_delayed_work(&info_work);
#endif

	lpi->ps_irq_flag = 0;

	judge_and_enable_lightsensor(lpi);

	D("%s\n", __func__);
	if (!lpi->ps_enable) {
		D("%s: already disabled\n", __func__);
		return 0;
	}

	ret = set_irq_wake(lpi->irq, 0);
	if (ret < 0) {
		pr_err("%s: failed to disable irq %d as a wake interrupt\n",
			__func__, lpi->irq);
		return ret;
	}

	ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
		ISL29028_CONFIGURE, ISL29028_PROX_EN);
	if (ret < 0) {
		pr_err("%s: disable psensor fail\n", __func__);
		return ret;
	}

	ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
		ISL29028_INTERRUPT, ISL29028_INT_PROX_FLAG);
	if (ret < 0) {
		pr_err("%s: clear proximity INT flag fail\n", __func__);
		return ret;
	}

	set_irq_type(lpi->irq, IRQF_TRIGGER_LOW);

	if (lpi->power)
		lpi->power(PS_PWR_ON, 0);

	lpi->ps_enable = 0;
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct isl29028_info *lpi = lp_info;

	D("%s\n", __func__);

	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct isl29028_info *lpi = lp_info;

	D("%s\n", __func__);

	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val;
	struct isl29028_info *lpi = lp_info;

	D("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case CAPELLA_CM3602_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val)
			return psensor_enable(lpi);
		else
			return psensor_disable(lpi);
		break;
	case CAPELLA_CM3602_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &psensor_fops
};

void lightsensor_set_kvalue(struct isl29028_info *lpi)
{
	if (!lpi) {
		pr_err("%s: ls_info is empty\n", __func__);
		return;
	}

	D("%s: ALS calibrated als_kadc=0x%x\n",
			__func__, als_kadc);

	if (als_kadc >> 16 == ALS_CALIBRATED)
		lpi->als_kadc = als_kadc & 0xFFFF;
	else {
		lpi->als_kadc = 0;
		D("%s: no ALS calibrated\n", __func__);
	}

	if (lpi->als_kadc && lpi->golden_adc > 0) {
		lpi->als_kadc = (lpi->als_kadc > 0 && lpi->als_kadc < 0x1000) ?
				lpi->als_kadc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	} else {
		lpi->als_kadc = 1;
		lpi->als_gadc = 1;
	}
	D("%s: als_kadc=0x%x, als_gadc=0x%x\n",
		__func__, lpi->als_kadc, lpi->als_gadc);
}

void psensor_set_kvalue(struct isl29028_info *lpi)
{
	D("%s: PS calibrated ps_kparam1 = 0x%x, ps_kparam2 = 0x%x\n",
			__func__, ps_kparam1, ps_kparam2);

	if (ps_kparam1 >> 16 == PS_CALIBRATED) {

		lpi->ps_B_val = (ps_kparam1 >> 8) & 0xFF;
		lpi->ps_C_val = ps_kparam1 & 0xFF;
		lpi->ps_A_val = (ps_kparam2 >> 24) & 0xFF;
		lpi->ps_X_val = (ps_kparam2 >> 16) & 0xFF;
		lpi->ps_lt = (ps_kparam2 >> 8) & 0xFF;
		lpi->ps_ht = ps_kparam2 & 0xFF;

		D("%s: PS calibrated ps_B_val = 0x%x, ps_C_val = 0x%x"
			", ps_A_val = 0x%x, ps_X_val = 0x%x, ps_lt = 0x%x"
			", ps_ht = 0x%x\n",
				__func__, lpi->ps_B_val, lpi->ps_C_val,
				lpi->ps_A_val, lpi->ps_X_val, lpi->ps_lt,
				lpi->ps_ht);

		ps_threshold = (PS_CALIBRATED << 16) |
			((lpi->ps_lt << 8) | lpi->ps_ht);

		set_psensor_range(lpi->ps_lt, lpi->ps_ht);
	} else
		D("%s: Proximity not calibrated\n", __func__);
}

static int lightsensor_update_table(struct isl29028_info *lpi)
{
	uint16_t data[10];
	int i;
	for (i = 0; i < 10; i++) {
		if (*(lpi->adc_table + i) < 0xFFF) {
			data[i] = *(lpi->adc_table + i)
					* lpi->als_kadc / lpi->als_gadc;
		} else {
			data[i] = *(lpi->adc_table + i);
		}
		D("%s: Calibrated adc_table: data[%d] = %x\n",
			__func__, i, data[i]);
	}
	memcpy(lpi->cali_table, data, 20);
	return 0;
}

static int lightsensor_enable(struct isl29028_info *lpi)
{
	int ret;

	pr_info("%s\n", __func__);

	lpi->ls_enable_flag = 1;
	if (lpi->is_suspend || lpi->ps_irq_flag == 1) {
		pr_err("%s: system is suspended or proximity is NEAR\n",
			__func__);
		return 0;
	}

	if (!lpi->als_enable) {
		ret = _isl29028_set_reg_bit(lpi->i2c_client, 1,
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: set auto light sensor fail\n", __func__);
		else {
			lpi->als_enable = 1;
			/* report an invalid value first to ensure we
			* trigger an event when adc_level is zero.
			*/
			input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
			input_sync(lpi->ls_input_dev);

			report_lsensor_input_event(lpi);
		}
	}
	return 0;
}

static int lightsensor_disable(struct isl29028_info *lpi)
{
	int ret;

	pr_info("%s\n", __func__);

	lpi->ls_enable_flag = 0;
	if (lpi->is_suspend) {
		pr_err("%s: system is suspended\n", __func__);
		return 0;
	}

	if (lpi->als_enable) {
		ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
				__func__);
		else
			lpi->als_enable = 0;

		ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
			ISL29028_INTERRUPT, ISL29028_INT_ALS_FLAG);
		if (ret < 0)
			pr_err("%s: clear lsensor intr flag fail\n", __func__);

	}
	return 0;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct isl29028_info *lpi = lp_info;
	int rc = 0;

	D("%s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct isl29028_info *lpi = lp_info;

	D("%s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct isl29028_info *lpi = lp_info;

	D("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		/*D("%s value = %d\n", __func__, val);*/
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->ls_enable_flag;
		/*D("%s enabled %d\n", __func__, val);*/
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

static ssize_t ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;
	struct isl29028_info *lpi = lp_info;

	int value1;

/*#ifdef DEBUG_PROXIMITY*/
	uint16_t value_of_test1, value_of_test2;

	value_of_test1 = i2c_smbus_read_byte_data(lpi->i2c_client,
			0x0E);
	value_of_test2 = i2c_smbus_read_byte_data(lpi->i2c_client,
			0x0F);
	D("\n%s: value_of_test1 = 0x%02x, value_of_test2 = 0x%02x\n",
		__func__, value_of_test1, value_of_test2);

	info_do_work(NULL);
	/*cancel_delayed_work(&info_work);*/
/*#endif*/

	value1 = gpio_get_value(lpi->intr_pin);
	D("\n%s: intr_pin = %d, value of intr_pin = %d\n",
		__func__, lpi->intr_pin, value1);

	value = get_ps_adc_value();

	ret = sprintf(buf, "ADC[0x%03X], ENABLE = %d, intr_pin = %d\n",
		value, lpi->ps_enable, value1);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct isl29028_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1)
		return -EINVAL;

	if (ps_en) {
		psensor_enable(lpi);
	} else {
		psensor_disable(lpi);
	}

	return count;
}

static DEVICE_ATTR(ps_adc, 0666, ps_adc_show, ps_enable_store);

static ssize_t ps_kadc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct isl29028_info *lpi = lp_info;

	ret = sprintf(buf, "(B_value, C_value, A_value, X_value, THL, THH)"
		" = (0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
		lpi->ps_B_val, lpi->ps_C_val, lpi->ps_A_val, lpi->ps_X_val,
		lpi->ps_lt, lpi->ps_ht);

	return ret;
}

static ssize_t ps_kadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint8_t lt, ht;
	int param1, param2;
	int ret;
	struct isl29028_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &param1, &param2);

	D("%s: store value = 0x%X, 0x%X\n", __func__, param1, param2);

	lt = (param2 >> 8) & 0xFF;
	ht = param2 & 0xFF;

	ps_threshold =  (PS_CALIBRATED << 16) | (param2 & 0xFFFF);

	D("%s: lt = 0x%X, ht = 0x%X\n", __func__, lt, ht);
	D("%s: ps_threshold = 0x%X\n", __func__, ps_threshold);

	if (lt < ht) {

		lpi->ps_B_val = (param1 >> 8) & 0xFF;
		lpi->ps_C_val = param1 & 0xFF;
		lpi->ps_A_val = (param2 >> 24) & 0xFF;
		lpi->ps_X_val = (param2 >> 16) & 0xFF;
		lpi->ps_lt = (param2 >> 8) & 0xFF;
		lpi->ps_ht = param2 & 0xFF;

		ret = set_psensor_range(lt, ht);
		if (ret < 0) {
			printk(KERN_ERR "%s : write ISL29028_PS_LT_HT fail\n",
				__func__);
			return -1;
		}
		lpi->ps_lt = lt;
		lpi->ps_ht = ht;
	} else
		D("%s: Setting PS calibration ERROR!\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_kadc, 0666, ps_kadc_show, ps_kadc_store);

static ssize_t ps_led_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char buffer[2];
	uint8_t value;
	int ret = 0;
	struct isl29028_info *lpi = lp_info;

	buffer[0] = ISL29028_CONFIGURE;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail\n", __func__);
		return ret;
	}
	value = buffer[0];
	D("%s: Configure reg = 0x%x\n", __func__, value);

	lpi->led = (value & 0x08) >> 3;
	D("%s: lpi->led = 0x%x\n", __func__, lpi->led);

	ret = sprintf(buf, "Current led setting: %s\n",
		(lpi->led == 0) ? "100mA" : "200mA");

	return ret;
}

static ssize_t ps_led_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int param;
	int ret;
	struct isl29028_info *lpi = lp_info;

	sscanf(buf, "%d", &param);

	D("%s: store value = %d\n", __func__, param);

	if (param == 0 || param == 1) {
		ret = _isl29028_set_reg_bit(lpi->i2c_client,
			param, ISL29028_CONFIGURE, ISL29028_PROX_DR);
		if (ret < 0) {
			printk(KERN_ERR "%s : Set LED  fail\n",
				__func__);
			return -1;
		}
		lpi->led = param;
	} else
		D("%s: Setting PS LED ERROR, please use 0 or 1!\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_led, 0666, ps_led_show, ps_led_store);

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint16_t value, raw_adc_value;
	int ret, i, level = -1;
	struct isl29028_info *lpi = lp_info;

	value = get_ls_adc_value(&raw_adc_value);

	for (i = 0; i < 10; i++) {
		if (value <= (*(lpi->adc_table + i))) {
			level = i;
			if (*(lpi->adc_table + i))
				break;
		}
	}

	ret = sprintf(buf, "ADC[0x%03X] => level %d\n", value, level);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0666, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	/*struct isl29028_info *lpi = lp_info;*/
	char buffer[2];

	buffer[0] = ISL29028_CONFIGURE;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: I2C_RxData fail\n", __func__);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			(value & 0x04) ? 1 : 0);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{

	int ls_auto;
	int want_enable;
	int ret;
	struct isl29028_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		want_enable = 1;
	} else {
		lpi->ls_calibrate = 0;
		want_enable = 0;
	}

	D("%s: want_enable = %d!\n", __func__, want_enable);

	ret = _isl29028_set_reg_bit(lpi->i2c_client,
		((want_enable) ? 1 : 0),
		ISL29028_CONFIGURE, ISL29028_ALS_EN);
	if (ret < 0)
		pr_err("%s: ls enable fail\n", __func__);
	else
		lpi->als_enable = (want_enable) ? 1 : 0;

	return count;
}

static DEVICE_ATTR(ls_auto, 0666,
	ls_enable_show, ls_enable_store);

static ssize_t ls_kadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct isl29028_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x, gadc = 0x%x, kadc while this boot"
			" = 0x%x\n",
			lpi->als_kadc, lpi->als_gadc, als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct isl29028_info *lpi = lp_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);
	if (kadc_temp <= 0 || lpi->golden_adc <= 0) {
		printk(KERN_ERR "%s: kadc_temp=0x%x, als_gadc=0x%x\n",
			__func__, kadc_temp, lpi->golden_adc);
		return -EINVAL;
	}

	lpi->als_kadc = kadc_temp;
	lpi->als_gadc = lpi->golden_adc;
	printk(KERN_INFO "%s: als_kadc=0x%x, als_gadc=0x%x\n",
			__func__, lpi->als_kadc, lpi->als_gadc);

	if (lightsensor_update_table(lpi) < 0)
		printk(KERN_ERR "%s: update ls table fail\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_kadc, 0666, ls_kadc_show, ls_kadc_store);

static int lightsensor_setup(struct isl29028_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err("%s: could not allocate ls input device\n", __func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int psensor_setup(struct isl29028_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err("%s: could not allocate ps input device\n", __func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err("%s: could not register ps input device\n", __func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err("%s: could not register ps misc device\n", __func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}


static int isl29028_setup(struct isl29028_info *lpi)
{
	int ret = 0;
	char buffer[2];

	ret = gpio_request(lpi->intr_pin, "gpio_isl29028_intr");
	if (ret < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	ret = request_irq(lpi->irq,
			isl29028_irq_handler,
			IRQF_TRIGGER_LOW,
			"isl29028",
			lpi);
	if (ret < 0) {
		pr_err("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	buffer[0] = ISL29028_CONFIGURE;
	buffer[1] = CONFIG_DEFAULT;
	ret = I2C_TxData(buffer, 2);
	if (ret < 0) {
		printk(KERN_ERR "%s : failed to set LPS Configuration fail\n",
			__func__);
		goto err_configure;
	}
	lpi->led = 0;

	ret = set_lsensor_range(0x0, 0x0);
	if (ret < 0) {
		printk(KERN_ERR "%s : write ISL29028_LS_TH123 fail\n",
			__func__);
		goto err_set_lsensor_range;
	}

	lpi->ps_lt = lpi->default_ps_lt;
	lpi->ps_ht = lpi->default_ps_ht;
	ret = set_psensor_range(lpi->ps_lt, lpi->ps_ht);
	if (ret < 0) {
		printk(KERN_ERR "%s : write ISL29028_PS_LT_HT fail\n",
			__func__);
		goto err_set_psensor_range;
	}

	buffer[0] = ISL29028_INTERRUPT;
	buffer[1] = INTR_DEFAULT;
	ret = I2C_TxData(buffer, 2);
	if (ret < 0) {
		pr_err("%s: set ALS PRST fail\n", __func__);
		goto err_set_als_prst;
	}

	return ret;

err_set_als_prst:
err_set_psensor_range:
err_set_lsensor_range:
err_configure:
	free_irq(lpi->irq, lpi);
fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

static void isl29028_early_suspend(struct early_suspend *h)
{

	int ret;
	struct isl29028_info *lpi = lp_info;

	D("%s\n", __func__);

	lpi->is_suspend = 1;

	if (lpi->als_enable) {
		/*D("%s: lpi->ps_enable = %d\n", __func__, lpi->ps_enable);
		buffer[0] = ISL29028_CONFIGURE;
		buffer[1] = (lpi->ps_enable) ? 0xE2 : 0x62;
		ret = I2C_TxData(buffer, 2);
		if (ret < 0) {
			printk(KERN_ERR "%s : Disable sensors fail\n",
				__func__);
		} else
			lpi->als_enable = 0;*/

		ret = _isl29028_set_reg_bit(lpi->i2c_client, 0,
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
				__func__);
		else
			lpi->als_enable = 0;
	}
}

static void isl29028_late_resume(struct early_suspend *h)
{
	struct isl29028_info *lpi = lp_info;

	D("isl_resume: ls_enable_flag = %d, ps_irq_flag = %d\n",
		lpi->ls_enable_flag, lpi->ps_irq_flag);

	lpi->is_suspend = 0;
	judge_and_enable_lightsensor(lpi);
}

static int isl29028_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	char buffer[2];
	struct isl29028_info *lpi;
	struct isl29028_platform_data *pdata;

	D("%s\n", __func__);

	lpi = kzalloc(sizeof(struct isl29028_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/*D("%s: client->irq = %d\n", __func__, client->irq);*/

	lpi->i2c_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("%s: Assign platform_data error!!\n", __func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}

	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);
	lpi->intr_pin = pdata->intr;
	lpi->adc_table = pdata->levels;
	lpi->golden_adc = pdata->golden_adc;
	lpi->power = pdata->power;
	lpi->default_ps_lt = pdata->lt;
	lpi->default_ps_ht = pdata->ht;
	lpi->debounce = pdata->debounce;

	lp_info = lpi;

	als_power(1);

	buffer[0] = ISL29028_CHIPID;
	ret = I2C_RxData(buffer, 1);
	if (ret < 0) {
		pr_err("%s: Read Chip ID fail\n", __func__);
		goto err_platform_data_null;
	}

	if (buffer[0] != 0xA1) {
		pr_err("%s: Error Chip ID or i2c error\n", __func__);
		goto err_platform_data_null;
	}

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("%s: lightsensor_setup error!!\n", __func__);
		goto err_lightsensor_setup;
	}

	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("%s: lightsensor_setup error!!\n", __func__);
		goto err_psensor_setup;
	}

	lightsensor_set_kvalue(lpi);

	ret = lightsensor_update_table(lpi);
	if (ret < 0) {
		pr_err("%s: update ls table fail\n",
			__func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("isl29028_wq");
	if (!lpi->lp_wq) {
		pr_err("%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	ret = isl29028_setup(lpi);
	if (ret < 0) {
		pr_err("%s: isl29028_setup error!\n", __func__);
		goto err_isl29028_setup;
	}

	psensor_set_kvalue(lpi);

	lpi->isl29028_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->isl29028_class)) {
		ret = PTR_ERR(lpi->isl29028_class);
		lpi->isl29028_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->isl29028_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_kadc);
	if (ret)
		goto err_create_ls_device_file;

	lpi->ps_dev = device_create(lpi->isl29028_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ls_device_file;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_kadc);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_led);
	if (ret)
		goto err_create_ps_device;

	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = isl29028_early_suspend;
	lpi->early_suspend.resume = isl29028_late_resume;
	register_early_suspend(&lpi->early_suspend);

	D("%s: Probe success!\n", __func__);
	return ret;

err_create_ps_device:
	device_unregister(lpi->ps_dev);
err_create_ls_device_file:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->isl29028_class);
err_create_class:
err_isl29028_setup:
	destroy_workqueue(lpi->lp_wq);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
err_psensor_setup:
err_lightsensor_setup:
err_platform_data_null:
	kfree(lpi);
	return ret;
}
/*
#define ATAG_PS        0x29028290
static int __init parse_tag_ps_threshold(const struct tag *tags)
{
	int found = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_PS) {
			found = 1;
			break;
		}
	}

	if (found)
		ps_threshold = t->u.revision.rev;
	D("%s: ps_threshold = 0x%x\n", __func__, ps_threshold);
	return 0;
}
__tagtable(ATAG_PS, parse_tag_ps_threshold);
*/
static const struct i2c_device_id isl29028_i2c_id[] = {
	{ISL29028_I2C_NAME, 0},
	{}
};

static struct i2c_driver isl29028_driver = {
	.id_table = isl29028_i2c_id,
	.probe = isl29028_probe,
	.driver = {
		.name = ISL29028_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init isl29028_init(void)
{
	return i2c_add_driver(&isl29028_driver);
}

static void __exit isl29028_exit(void)
{
	i2c_del_driver(&isl29028_driver);
}

module_init(isl29028_init);
module_exit(isl29028_exit);

MODULE_DESCRIPTION("ISL29028 Driver");
MODULE_LICENSE("GPL");
