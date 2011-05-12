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

/* information about the system we're running on */
extern unsigned int system_rev;

enum batt_ctl_t {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG,
	ENABLE_SUPER_CHG,
	CHARGER_CHK,
	TOGGLE_CHARGER,
	ENABLE_MIN_TAPER,
	DISABLE_MIN_TAPER
};

/* This order is the same as htc_power_supplies[]
 * And it's also the same as htc_cable_status_update()
 */
enum charger_type_t {
	CHARGER_UNKNOWN = -1,
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC,
	CHARGER_SUPER_AC
};

enum charger_control_flag {
	STOP_CHARGER = 0,
	ENABLE_CHARGER,
	ENABLE_LIMIT_CHARGER,
	DISABLE_LIMIT_CHARGER,
	END_CHARGER
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
	SWITCH_CHARGER_TPS65200,
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
	int gpio_adp_9v;
	int guage_driver;
	int m2a_cable_detect;
	int charger;
	struct htc_battery_tps65200_int int_data;
	int (*func_is_support_super_charger)(void);
	int (*func_battery_charging_ctrl)(enum batt_ctl_t ctl);
	int (*func_battery_gpio_init)(void);
};

#ifdef CONFIG_HTC_BATTCHG
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

extern unsigned int batt_get_status(enum power_supply_property psp);
extern int htc_battery_charger_disable(void);

#endif
