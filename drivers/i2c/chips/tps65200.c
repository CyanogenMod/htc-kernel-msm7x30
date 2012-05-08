/* drivers/i2c/chips/tps65200.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Josh Hsiao <Josh_Hsiao@htc.com>
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/tps65200.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/android_alarm.h>
#include <linux/usb/android_composite.h>
#include <mach/board_htc.h>
#include <mach/board.h>
#if defined(CONFIG_MACH_HOLIDAY)
#define AC_CURRENT_SWTICH_DELAY_200MS		200
#define AC_CURRENT_SWTICH_DELAY_100MS		100
#endif	/* CONFIG_MACH_HOLIDAY */
#define pr_tps_fmt(fmt) "[BATT][tps65200] " fmt
#define pr_tps_err_fmt(fmt) "[BATT][tps65200] err:" fmt
#define pr_tps_info(fmt, ...) \
	printk(KERN_INFO pr_tps_fmt(fmt), ##__VA_ARGS__)
#define pr_tps_err(fmt, ...) \
	printk(KERN_ERR pr_tps_err_fmt(fmt), ##__VA_ARGS__)

/* there is a 32 seconds hw safety timer for boost mode */
#define TPS65200_CHECK_INTERVAL (15)
/* Delay 200ms to set original VDPM for preventing vbus voltage dropped & pass MHL spec */
#define DELAY_MHL_INIT	msecs_to_jiffies(200)
#define DELAY_KICK_TPS	msecs_to_jiffies(10)
#define SET_VDPM_AS_476	1

static struct workqueue_struct *tps65200_wq;
static struct work_struct chg_stat_work;
static struct delayed_work set_vdpm_work;
static struct delayed_work kick_dog;

static struct alarm tps65200_check_alarm;
static struct work_struct check_alarm_work;

static int chg_stat_int;
static unsigned int chg_stat_enabled;
static spinlock_t chg_stat_lock;

static struct tps65200_chg_int_data *chg_int_data;
static LIST_HEAD(tps65200_chg_int_list);
static DEFINE_MUTEX(notify_lock);

static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

static int tps65200_initial = -1;
static int tps65200_low_chg;
static int tps65200_vdpm_chg = 0;

#ifdef CONFIG_SUPPORT_DQ_BATTERY
static int htc_is_dq_pass;
#endif

u8 batt_charging_state;

static void tps65200_set_check_alarm(void)
{
	ktime_t interval;
	ktime_t next_alarm;

	interval = ktime_set(TPS65200_CHECK_INTERVAL, 0);
	next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
	alarm_start_range(&tps65200_check_alarm, next_alarm, next_alarm);
}

/**
 * Insmod parameters
 */
/* I2C_CLIENT_INSMOD_1(tps65200); */

static int tps65200_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
#if 0
static int tps65200_detect(struct i2c_client *client, int kind,
			 struct i2c_board_info *info);
#endif
static int tps65200_remove(struct i2c_client *client);

/* Supersonic for Switch charger */
struct tps65200_i2c_client {
	struct i2c_client *client;
	u8 address;
	/* max numb of i2c_msg required is for read =2 */
	struct i2c_msg xfer_msg[2];
	/* To lock access to xfer_msg */
	struct mutex xfer_lock;
};
static struct tps65200_i2c_client tps65200_i2c_module;

/**
Function:tps65200_i2c_write
Target:	Write a byte to Switch charger
Timing:	TBD
INPUT: 	value-> write value
		reg  -> reg offset
		num-> number of byte to write
return :TRUE-->OK
		FALSE-->Fail
 */
static int tps65200_i2c_write(u8 *value, u8 reg, u8 num_bytes)
{
	int ret;
	struct tps65200_i2c_client *tps;
	struct i2c_msg *msg;

	tps = &tps65200_i2c_module;

	mutex_lock(&tps->xfer_lock);
	/*
	 * [MSG1]: fill the register address data
	 * fill the data Tx buffer
	 */
	msg = &tps->xfer_msg[0];
	msg->addr = tps->address;
	msg->len = num_bytes + 1;
	msg->flags = 0;
	msg->buf = value;
	/* over write the first byte of buffer with the register address */
	*value = reg;
	ret = i2c_transfer(tps->client->adapter, tps->xfer_msg, 1);
	mutex_unlock(&tps->xfer_lock);

	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret >= 0)
		ret = 0;
	return ret;
}


/**
Function:tps65200_i2c_read
Target:	Read a byte from Switch charger
Timing:	TBD
INPUT: 	value-> store buffer
		reg  -> reg offset to read
		num-> number of byte to read
return :TRUE-->OK
		FALSE-->Fail
 */
static int tps65200_i2c_read(u8 *value, u8 reg, u8 num_bytes)
{
	int ret;
	u8 val;
	struct tps65200_i2c_client *tps;
	struct i2c_msg *msg;

	tps = &tps65200_i2c_module;

	mutex_lock(&tps->xfer_lock);
	/* [MSG1] fill the register address data */
	msg = &tps->xfer_msg[0];
	msg->addr = tps->address;
	msg->len = 1;
	msg->flags = 0; /* Read the register value */
	val = reg;
	msg->buf = &val;
	/* [MSG2] fill the data rx buffer */
	msg = &tps->xfer_msg[1];
	msg->addr = tps->address;
	msg->flags = I2C_M_RD;  /* Read the register value */
	msg->len = num_bytes;   /* only n bytes */
	msg->buf = value;
	ret = i2c_transfer(tps->client->adapter, tps->xfer_msg, 2);
	mutex_unlock(&tps->xfer_lock);

	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret >= 0)
		ret = 0;
	return ret;
}


/**
Function:tps65200_i2c_write_byte
Target:	Write a byte from Switch charger
Timing:	TBD
INPUT: 	value-> store buffer
		reg  -> reg offset to read
return :TRUE-->OK
		FALSE-->Fail
 */
static int tps65200_i2c_write_byte(u8 value, u8 reg)
{
	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	int result;
	int i;
	u8 temp_buffer[2] = { 0 };
	/* offset 1 contains the data */
	temp_buffer[1] = value;
	/*  There is workaround here to retry when returns fail since of
	    I2C resume order changed in kernel 3.0

	    Max retry times : 10
	    Will break when writing byte successfully (return 0)
	*/
	for (i = 0; i < 10; i++) {
		result = tps65200_i2c_write(temp_buffer, reg, 1);
		if (result == 0)
			break;
		pr_tps_err("TPS65200 I2C write retry count = %d, result = %d\n", i+1, result);
		msleep(10);
	}
	if (result != 0)
		pr_tps_err("TPS65200 I2C write fail = %d\n", result);

	return result;
}

/**
Function:tps65200_i2c_read_byte
Target:	Read a byte from Switch charger
Timing:	TBD
INPUT: 	value-> store buffer
		reg  -> reg offset to read
return :TRUE-->OK
		FALSE-->Fail
 */
static int tps65200_i2c_read_byte(u8 *value, u8 reg)
{
	int result = 0;
	int i;
	/*  Similar to the case of tps65200_i2c_write_byte

	    Max retry times : 10
	    Will break when reading byte successfully (return 0)
	*/
	for (i = 0; i < 10; i++) {
		result = tps65200_i2c_read(value, reg, 1);
		if (result == 0)
			break;
		pr_tps_err("TPS65200 I2C read retry count = %d, result = %d\n", i+1, result);
		msleep(10);
	}
	if (result != 0)
		pr_tps_err("TPS65200 I2C read fail = %d\n", result);

	return result;
}

int tps_register_notifier(struct tps65200_chg_int_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_lock);
	list_add(&notifier->notifier_link,
		&tps65200_chg_int_list);
	mutex_unlock(&notify_lock);
	return 0;
}
EXPORT_SYMBOL(tps_register_notifier);

static void send_tps_chg_int_notify(int int_reg, int value)
{
	static struct tps65200_chg_int_notifier *notifier;

	mutex_lock(&notify_lock);
	list_for_each_entry(notifier,
		&tps65200_chg_int_list,
		notifier_link) {
		if (notifier->func != NULL)
			notifier->func(int_reg, value);
	}
	mutex_unlock(&notify_lock);
}

static int tps65200_set_chg_stat(unsigned int ctrl)
{
	unsigned long flags;
	if (!chg_stat_int)
		return -1;
	spin_lock_irqsave(&chg_stat_lock, flags);
	chg_stat_enabled = ctrl;
	spin_unlock_irqrestore(&chg_stat_lock, flags);

	return 0;
}

static int tps65200_dump_register(void)
{
	u8 regh0 = 0, regh1 = 0, regh2 = 0, regh3 = 0;
	int result = 0;
	tps65200_i2c_read_byte(&regh1, 0x01);
	tps65200_i2c_read_byte(&regh0, 0x00);
	tps65200_i2c_read_byte(&regh3, 0x03);
	tps65200_i2c_read_byte(&regh2, 0x02);
	pr_tps_info("regh 0x00=%x, regh 0x01=%x, regh 0x02=%x, regh 0x03=%x\n",
			regh0, regh1, regh2, regh3);
	tps65200_i2c_read_byte(&regh0, 0x06);
	tps65200_i2c_read_byte(&regh1, 0x08);
	tps65200_i2c_read_byte(&regh2, 0x09);
	result = tps65200_i2c_read_byte(&regh3, 0x0A);
	pr_tps_info("regh 0x06=%x, 0x08=%x, regh 0x09=%x, regh 0x0A=%x\n",
			regh0, regh1, regh2, regh3);

	return result;
}

u32 htc_fake_charger_for_testing(u32 ctl)
{
	u32 new_ctl = POWER_SUPPLY_ENABLE_FAST_CHARGE;

	if((ctl > POWER_SUPPLY_ENABLE_INTERNAL) || (ctl == POWER_SUPPLY_DISABLE_CHARGE))
		return ctl;

#if defined(CONFIG_MACH_VERDI_LTE)
	new_ctl = POWER_SUPPLY_ENABLE_9VAC_CHARGE;
#else
	/* set charger to 1A AC  by default */
#endif

	pr_tps_info("[BATT] %s(%d -> %d)\n", __func__, ctl , new_ctl);
	batt_charging_state = new_ctl;
	return new_ctl;
}

static void set_vdpm(struct work_struct *work)
{
	if (tps65200_vdpm_chg)
		tps_set_charger_ctrl(VDPM_ORIGIN_V);
}

#ifdef CONFIG_MACH_GOLFU
static void set_golfu_regh(void)
{
	u8 regh;
	tps65200_i2c_write_byte(0x00, 0x0F);
	tps65200_i2c_read_byte(&regh, 0x03);
	regh |= 0x20;
	regh &= 0xBF;
	tps65200_i2c_write_byte(regh, 0x03);
}
#endif

int tps_set_charger_ctrl(u32 ctl)
{
	int result = 0;
	u8 status = 0;
	u8 regh = 0;
	u8 regh1 = 0, regh2 = 0, regh3 = 0;
	int i2c_status = 0;

	if (get_kernel_flag() & KERNEL_FLAG_ENABLE_FAST_CHARGE)
		ctl = htc_fake_charger_for_testing(ctl);

	if (tps65200_initial < 0)
		return 0;

	switch (ctl) {
	case POWER_SUPPLY_DISABLE_CHARGE:
		pr_tps_info("Switch charger OFF\n");
		tps65200_set_chg_stat(0);
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x28, 0x00);

#if SET_VDPM_AS_476
		cancel_delayed_work_sync(&set_vdpm_work);
		tps65200_vdpm_chg = 0;
		tps65200_i2c_write_byte(0x87, 0x03); /* VDPM = 4.76V */
		#ifdef CONFIG_MACH_GOLFU
		set_golfu_regh();
		#endif
#endif /* SET_VDPM_AS_476 */

		/* cancel CHECK_CHG alarm */
		alarm_cancel(&tps65200_check_alarm);
		break;
	case POWER_SUPPLY_ENABLE_SLOW_CHARGE:
	case POWER_SUPPLY_ENABLE_WIRELESS_CHARGE:
		tps65200_dump_register();
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);
#if SET_VDPM_AS_476
		regh = 0x87; /* VDPM = 4.76V */
#else
		/* set DPM regulation voltage to 4.44V */
		regh = 0x83;
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			/* set DPM regulation voltage to 4.6V */
			regh = 0x85;
#endif
#endif /* SET_VDPM_AS_476 */
		if (tps65200_low_chg)
			regh |= 0x08;	/* enable low charge curent */
		tps65200_i2c_write_byte(regh, 0x03);
		#ifdef CONFIG_MACH_GOLFU
		set_golfu_regh();
		#endif
		regh = 0x63;
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			regh = 0x6A;
#endif
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh1, 0x03);
		tps65200_i2c_read_byte(&regh2, 0x02);
		pr_tps_info("Switch charger ON (SLOW): regh 0x03=%x, "
				"regh 0x02=%x\n", regh1, regh2);
		tps65200_set_chg_stat(1);
#if SET_VDPM_AS_476
		tps65200_vdpm_chg = 1;
		result = schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		if (!result) {
			cancel_delayed_work(&set_vdpm_work);
			schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		}
#endif /* SET_VDPM_AS_476 */
		break;
	case POWER_SUPPLY_ENABLE_FAST_CHARGE:
		tps65200_dump_register();
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);

#if SET_VDPM_AS_476
		regh = 0x87;
#else
		/* set DPM regulation voltage to 4.44V */
		regh = 0x83;
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			/* set DPM regulation voltage to 4.6V */
			regh = 0x85;
#endif
#endif /* SET_VDPM_AS_476 */

		if (tps65200_low_chg)
			regh |= 0x08;	/* enable low charge current */
		tps65200_i2c_write_byte(regh, 0x03);
		#ifdef CONFIG_MACH_GOLFU
		set_golfu_regh();
		#endif

		regh = 0xA3;
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			regh = 0xAA;
#endif
		tps65200_i2c_write_byte(regh, 0x02);

		tps65200_i2c_read_byte(&regh, 0x01);
		tps65200_i2c_read_byte(&regh1, 0x00);
		tps65200_i2c_read_byte(&regh2, 0x03);
		tps65200_i2c_read_byte(&regh3, 0x02);
		pr_tps_info("Switch charger ON (FAST): regh 0x01=%x, "
				"regh 0x00=%x, regh 0x03=%x, regh 0x02=%x\n",
				regh, regh1, regh2, regh3);
		tps65200_set_chg_stat(1);
#if SET_VDPM_AS_476
		tps65200_vdpm_chg = 1;
		result = schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		if (!result) {
			cancel_delayed_work(&set_vdpm_work);
			schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		}
#endif /* SET_VDPM_AS_476 */
		break;
	case POWER_SUPPLY_ENABLE_SLOW_HV_CHARGE:
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);
		regh = 0x85;
		if (tps65200_low_chg)
			regh |= 0x08;
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_write_byte(0x6A, 0x02);
		tps65200_i2c_read_byte(&regh, 0x03);
		tps65200_i2c_read_byte(&regh1, 0x02);
		pr_tps_info("Switch charger ON (SLOW_HV): regh 0x03=%x, "
				"regh 0x02=%x\n", regh, regh1);
		break;
	case POWER_SUPPLY_ENABLE_FAST_HV_CHARGE:
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);
		regh = 0x85;
		if (tps65200_low_chg)
			regh |= 0x08;
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_write_byte(0xAA, 0x02);
		tps65200_i2c_read_byte(&regh, 0x01);
		tps65200_i2c_read_byte(&regh1, 0x00);
		tps65200_i2c_read_byte(&regh2, 0x03);
		tps65200_i2c_read_byte(&regh3, 0x02);
		pr_tps_info("Switch charger ON (FAST_HV): regh 0x01=%x, "
				"regh 0x01=%x, regh 0x03=%x, regh 0x02=%x\n",
				regh, regh1, regh2, regh3);
		break;
	case ENABLE_LIMITED_CHG:
		tps65200_i2c_read_byte(&regh, 0x03);
		regh |= 0x08;
		tps65200_i2c_write_byte(regh, 0x03);
		#ifdef CONFIG_MACH_GOLFU
		set_golfu_regh();
		#endif
		tps65200_low_chg = 1;
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Switch charger ON (LIMITED): regh 0x03=%x\n", regh);
		break;
	case CLEAR_LIMITED_CHG:
		tps65200_i2c_read_byte(&regh, 0x03);
		regh &= 0xF7;
		tps65200_i2c_write_byte(regh, 0x03);
		#ifdef CONFIG_MACH_GOLFU
		set_golfu_regh();
		#endif
		tps65200_low_chg = 0;
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Switch charger OFF (LIMITED): regh 0x03=%x\n", regh);
		break;
	case CHECK_CHG:
		i2c_status = tps65200_dump_register();
		if (i2c_status == -5) {
			pr_tps_info("Delay 200ms to kick tps watchdog!\n");
			schedule_delayed_work(&kick_dog, DELAY_KICK_TPS);
		}
		break;
	case SET_ICL500:
		pr_tps_info("Switch charger SET_ICL500 \n");
		tps65200_i2c_write_byte(0xA3, 0x02);
		break;
	case SET_ICL100:
		pr_tps_info("Switch charger SET_ICL100 \n");
		tps65200_i2c_write_byte(0x23, 0x02);
		break;
	case CHECK_INT1:
		tps65200_i2c_read_byte(&status, 0x08);
		pr_tps_info("Switch charger CHECK_INT1: regh 0x08h=%x\n", status);
		result = (int)status;
		break;
	case CHECK_INT2:
		tps65200_i2c_read_byte(&status, 0x09);
		pr_tps_info("TPS65200 INT2 %x\n", status);
		result = (int)status;
		break;
	case CHECK_CONTROL:
		tps65200_i2c_read_byte(&status, 0x00);
		pr_tps_info("TPS65200 check control, regh 0x00=%x\n", status);
		break;
	case OVERTEMP_VREG:
		tps65200_i2c_read_byte(&regh, 0x02);
		regh = (regh & 0xC0) | 0x1C;
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh, 0x02);
		pr_tps_info("Switch charger OVERTEMP_VREG_4060: regh 0x02=%x\n", regh);
		break;
	case NORMALTEMP_VREG:
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		tps65200_i2c_read_byte(&regh, 0x04);
		pr_tps_info("Switch charger CONFIG_D: regh 0x04=%x\n", regh);
		if (htc_is_dq_pass) {
			tps65200_i2c_read_byte(&regh, 0x02);
			regh = (regh & 0xC0) | 0X2A;
			tps65200_i2c_write_byte(regh, 0x02);
			tps65200_i2c_read_byte(&regh, 0x02);
			pr_tps_info("Switch charger NORMALTEMP_VREG_4340: regh 0x02=%x\n", regh);
			break;
		}
#endif
		tps65200_i2c_read_byte(&regh, 0x02);
		regh = (regh & 0xC0) | 0X23;
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh, 0x02);
		pr_tps_info("Switch charger NORMALTEMP_VREG_4200: regh 0x02=%x\n", regh);
		break;
	case NORMALTEMP_VREG_HV:
		tps65200_i2c_read_byte(&regh, 0x02);
		regh = (regh & 0xC0) | 0x2A;
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh, 0x02);
		pr_tps_info("Switch charger NORMALTEMP_VREG_4200: regh 0x02=%x\n", regh);
		break;
	case POWER_SUPPLY_ENABLE_INTERNAL:
		/* Boost Mode */
		tps65200_i2c_read_byte(&regh, 0x00);
		pr_tps_info("regh 0x00=%x\n", regh);
		regh |= 0x01;
		regh &= 0xfd;
		tps65200_i2c_write_byte(regh, 0x00);
		/* set alarm for CHECK_CHG */
		tps65200_set_check_alarm();
		tps65200_i2c_read_byte(&regh, 0x00);
		pr_tps_info("Switch charger to Boost mode: regh 0x00=%x\n", regh);
		break;
	case VDPM_ORIGIN_V:
		tps65200_i2c_read_byte(&regh, 0x03);
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			regh = (regh & 0xF8) | 0x05; /* set VDPM as 4.6V */
		else
			regh = (regh & 0xF8) | 0x03; /* set VDPM as 4.4V */
#else
		regh = (regh & 0xF8) | 0x03; /* set VDPM as 4.4V */
#endif
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Set VDPM back to original V: regh 0x03=%x\n", regh);
		tps65200_vdpm_chg = 0;
		break;
	case VDPM_476V:
		tps65200_i2c_read_byte(&regh, 0x03);
		regh = (regh & 0xF8) | 0x07;
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Set VDPM as 4.76V: regh 0x03=%x\n", regh);
		break;
	default:
		pr_tps_info("%s: Not supported battery ctr called.!", __func__);
		result = -EINVAL;
		break;
	}

	return result;
}
EXPORT_SYMBOL(tps_set_charger_ctrl);

#if 0
static int tps65200_detect(struct i2c_client *client, int kind,
			 struct i2c_board_info *info)
{
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_BYTE))
		return -ENODEV;

	strlcpy(info->type, "tps65200", I2C_NAME_SIZE);

	return 0;
}
#endif

static irqreturn_t chg_stat_handler(int irq, void *data)
{
	pr_tps_info("interrupt chg_stat is triggered. "
			"chg_stat_enabled:%u\n", chg_stat_enabled);

	if (chg_stat_enabled)
		queue_work(tps65200_wq, &chg_stat_work);

	return IRQ_HANDLED;
}

static irqreturn_t chg_int_handler(int irq, void *data)
{
	pr_tps_info("interrupt chg_int is triggered.\n");

	disable_irq_nosync(chg_int_data->gpio_chg_int);
	chg_int_data->tps65200_reg = 0;
	schedule_delayed_work(&chg_int_data->int_work, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static void tps65200_int_func(struct work_struct *work)
{
	int fault_bit;

	switch (chg_int_data->tps65200_reg) {
	case CHECK_INT1:
		/*  read twice. First read to trigger TPS65200 clear fault bit
		    on INT1. Second read to make sure that fault bit is cleared
		    and call off ovp function.	*/
		fault_bit = tps_set_charger_ctrl(CHECK_INT1);
		fault_bit = tps_set_charger_ctrl(CHECK_INT1);

		if (fault_bit & 0x40) {
			send_tps_chg_int_notify(CHECK_INT1, 1);
			schedule_delayed_work(&chg_int_data->int_work,
						msecs_to_jiffies(5000));
			pr_tps_info("over voltage fault bit "
				"on TPS65200 is raised: %x\n", fault_bit);

		} else {
			send_tps_chg_int_notify(CHECK_INT1, 0);
			cancel_delayed_work(&chg_int_data->int_work);
			enable_irq(chg_int_data->gpio_chg_int);
		}
		break;
	default:
		fault_bit = tps_set_charger_ctrl(CHECK_INT2);
		pr_tps_info("Read register INT2 value: %x\n", fault_bit);
		if (fault_bit & 0x80) {
			fault_bit = tps_set_charger_ctrl(CHECK_INT2);
			fault_bit = tps_set_charger_ctrl(CHECK_INT2);
			pr_tps_info("Reverse current protection happened.\n");
			tps65200_set_chg_stat(0);
			tps65200_i2c_write_byte(0x29, 0x01);
			tps65200_i2c_write_byte(0x28, 0x00);
			send_tps_chg_int_notify(CHECK_INT2, 1);
			cancel_delayed_work(&chg_int_data->int_work);
			enable_irq(chg_int_data->gpio_chg_int);
		} else {
			fault_bit = tps_set_charger_ctrl(CHECK_INT1);
			if (fault_bit & 0x40) {
				chg_int_data->tps65200_reg = CHECK_INT1;
				schedule_delayed_work(&chg_int_data->int_work,
							msecs_to_jiffies(200));
			} else {
				pr_tps_err("CHG_INT should not be triggered "
					"without fault bit!\n");
				enable_irq(chg_int_data->gpio_chg_int);
			}
		}
	}
}

static void chg_stat_work_func(struct work_struct *work)
{
	tps65200_set_chg_stat(0);
	tps65200_i2c_write_byte(0x29, 0x01);
	tps65200_i2c_write_byte(0x28, 0x00);
	return;
}
static void kick_tps_watchdog(struct work_struct *work)
{
	pr_tps_info("To kick tps watchgod through delay workqueue !\n");
	tps65200_dump_register();
	return;
}

static void tps65200_check_alarm_handler(struct alarm *alarm)
{
	queue_work(tps65200_wq, &check_alarm_work);
}

static void check_alarm_work_func(struct work_struct *work)
{
	tps_set_charger_ctrl(CHECK_CHG);
	tps65200_dump_register();
	tps65200_set_check_alarm();
}

static int tps65200_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc = 0;
	struct tps65200_i2c_client   *data = &tps65200_i2c_module;
	struct tps65200_platform_data *pdata =
					client->dev.platform_data;
	pr_tps_info("%s\n",__func__);

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		pr_tps_err("I2C fail\n");
		return -EIO;
	}

	tps65200_wq = create_singlethread_workqueue("tps65200");
	if (!tps65200_wq) {
		pr_tps_err("Failed to create tps65200 workqueue.");
		return -ENOMEM;
	}

	/* for boost mode safety timer */
	INIT_WORK(&check_alarm_work, check_alarm_work_func);
	alarm_init(&tps65200_check_alarm,
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			tps65200_check_alarm_handler);

#ifdef CONFIG_SUPPORT_DQ_BATTERY
	htc_is_dq_pass = pdata->dq_result;
	if (htc_is_dq_pass)
		pr_tps_info("HV battery is detected.\n");
	else
		pr_tps_info("not HV battery.\n");

#endif
	/*  For chg_stat interrupt initialization. */
	chg_stat_int = 0;
	if (pdata->gpio_chg_stat > 0) {
		rc = request_any_context_irq(pdata->gpio_chg_stat,
					chg_stat_handler,
					IRQF_TRIGGER_RISING,
					"chg_stat", NULL);

		if (rc < 0)
			pr_tps_err("request chg_stat irq failed!\n");
		else {
			INIT_WORK(&chg_stat_work, chg_stat_work_func);
			chg_stat_int = pdata->gpio_chg_stat;
		}
	}

	/*  For chg_int interrupt initialization. */
	if (pdata->gpio_chg_int > 0) {
		chg_int_data = (struct tps65200_chg_int_data *)
				kmalloc(sizeof(struct tps65200_chg_int_data),
					GFP_KERNEL);
		if (!chg_int_data) {
			pr_tps_err("No memory for chg_int_data!\n");
			return -1;
		}

		chg_int_data->gpio_chg_int = 0;
		INIT_DELAYED_WORK(&chg_int_data->int_work,
				tps65200_int_func);

		rc = request_any_context_irq(
				pdata->gpio_chg_int,
				chg_int_handler,
				IRQF_TRIGGER_FALLING,
				"chg_int", NULL);
		if (rc < 0)
			pr_tps_err("request chg_int irq failed!\n");
		else {
			pr_tps_info("init chg_int interrupt.\n");
			chg_int_data->gpio_chg_int =
				pdata->gpio_chg_int;
		}
	}
	INIT_DELAYED_WORK(&set_vdpm_work, set_vdpm);

	pr_tps_info("To init delay workqueue to kick tps watchdog!\n");
	INIT_DELAYED_WORK(&kick_dog, kick_tps_watchdog);

	data->address = client->addr;
	data->client = client;
	mutex_init(&data->xfer_lock);
	tps65200_initial = 1;
#if SET_VDPM_AS_476
	tps_set_charger_ctrl(VDPM_476V);
#endif
	pr_tps_info("[TPS65200]: Driver registration done\n");
	return 0;
}

static int tps65200_remove(struct i2c_client *client)
{
	struct tps65200_i2c_client   *data = i2c_get_clientdata(client);
	pr_tps_info("TPS65200 remove\n");
	if (data->client && data->client != client)
		i2c_unregister_device(data->client);
	tps65200_i2c_module.client = NULL;
	if (tps65200_wq)
		destroy_workqueue(tps65200_wq);
	return 0;
}

static void tps65200_shutdown(struct i2c_client *client)
{
	u8 regh = 0;

	pr_tps_info("TPS65200 shutdown\n");
	tps65200_i2c_read_byte(&regh, 0x00);
	/* disable shunt monitor to decrease 0.035mA of current */
	regh &= 0xDF;
	tps65200_i2c_write_byte(regh, 0x00);
}

static const struct i2c_device_id tps65200_id[] = {
	{ "tps65200", 0 },
	{  },
};
static struct i2c_driver tps65200_driver = {
	.driver.name    = "tps65200",
	.id_table   = tps65200_id,
	.probe      = tps65200_probe,
	.remove     = tps65200_remove,
	.shutdown   = tps65200_shutdown,
};

static int __init sensors_tps65200_init(void)
{
	int res;

	tps65200_low_chg = 0;
	chg_stat_enabled = 0;
	spin_lock_init(&chg_stat_lock);
	res = i2c_add_driver(&tps65200_driver);
	if (res)
		pr_tps_err("[TPS65200]: Driver registration failed \n");

	return res;
}

static void __exit sensors_tps65200_exit(void)
{
	kfree(chg_int_data);
	i2c_del_driver(&tps65200_driver);
}

MODULE_AUTHOR("Josh Hsiao <Josh_Hsiao@htc.com>");
MODULE_DESCRIPTION("tps65200 driver");
MODULE_LICENSE("GPL");

fs_initcall(sensors_tps65200_init);
module_exit(sensors_tps65200_exit);
