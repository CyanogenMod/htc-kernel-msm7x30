/*
 * Copyright (C) 2007 HTC Incorporated
 * Author: Jay Tu (jay_tu@htc.com)
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
#ifndef _HTC_BATTERY_H_
#define _HTC_BATTERY_H_
#include <linux/notifier.h>
#include <linux/power_supply.h>

#define BATT_EVENT_SUSPEND	0x01

#define CHECK_CHG           0X64
#define SET_ICL500		0X65
#define SET_ICL100		0X66
#define CHECK_INT2		0X67
#define OVERTEMP_VREG_4060	0XC8
#define NORMALTEMP_VREG_4200	0XC9
#define CHECK_INT1		0XCA
#define CHECK_CONTROL           0xCB
/* information about the system we're running on */
extern unsigned int system_rev;

enum batt_ctl_t {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG
};

/* This order is the same as htc_power_supplies[]
 * And it's also the same as htc_cable_status_update()
 */
enum charger_type_t {
	CHARGER_UNKNOWN = -1,
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
};

enum {
	GUAGE_NONE,
	GUAGE_MODEM,
	GUAGE_DS2784,
	GUAGE_DS2746,
};

enum {
	LINEAR_CHARGER,
	SWITCH_CHARGER,
};

struct battery_info_reply {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	s32 batt_temp;		/* Battery Temperature (C) from formula and ADC */
	s32 batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 full_bat;		/* Full capacity of battery (mAh) */
	u32 full_level;		/* Full Level */
	u32 over_vchg;		/* 0:normal, 1:over voltage charger */
	s32 eval_current;	/* System loading current from ADC */
};

struct htc_battery_tps65200_int {
	int chg_int;
	int tps65200_reg;
	struct delayed_work int_work;
};

struct htc_battery_platform_data {
	int (*func_show_batt_attr)(struct device_attribute *attr,
					 char *buf);
	int gpio_mbat_in;
	int gpio_usb_id;
	int gpio_mchg_en_n;
	int gpio_iset;
	int guage_driver;
	int m2a_cable_detect;
	int charger;
	struct htc_battery_tps65200_int int_data;
};

#if CONFIG_HTC_BATTCHG
extern int register_notifier_cable_status(struct notifier_block *nb);
extern int unregister_notifier_cable_status(struct notifier_block *nb);
#else
static int register_notifier_cable_status(struct notifier_block *nb) { return 0; }
static int unregister_notifier_cable_status(struct notifier_block *nb) { return 0; }
#endif

#ifdef CONFIG_BATTERY_DS2784
extern int battery_charging_ctrl(enum batt_ctl_t ctl);
#endif
extern int get_cable_status(void);
#ifdef CONFIG_HTC_BATTCHG
extern int batt_register_client(struct notifier_block *nb);
extern int batt_unregister_client(struct notifier_block *nb);
extern int batt_notifier_call_chain(unsigned long val, void *v);
#else
static int batt_register_client(struct notifier_block *nb)
{
	return 0;
}

static int batt_unregister_client(struct notifier_block *nb)
{
	return 0;
}

static int batt_notifier_call_chain(unsigned long val, void *v)
{
	return 0;
}
#endif

extern unsigned int batt_get_status(enum power_supply_property psp);
#endif
