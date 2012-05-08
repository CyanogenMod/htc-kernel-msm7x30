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
#ifndef _HTC_BATTERY_COMMON_H_
#define _HTC_BATTERY_COMMON_H_
/* Common battery terms are defined in this file. */


/* enum definition */

/* This order is the same as htc_power_supplies[]
 * And it's also the same as htc_cable_status_update()
 */
enum charger_type_t {
	CHARGER_CLEAR = -2,
	CHARGER_UNKNOWN = -1,
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC,
	CHARGER_9V_AC,
	CHARGER_WIRELESS
};

enum power_supplies_type {
	BATTERY_SUPPLY,
	USB_SUPPLY,
	AC_SUPPLY,
	WIRELESS_SUPPLY
};

enum charger_control_flag {
	STOP_CHARGER = 0,
	ENABLE_CHARGER,
	ENABLE_LIMIT_CHARGER,
	DISABLE_LIMIT_CHARGER,
	END_CHARGER
};
/* context event */
enum batt_context_event {
	EVENT_TALK_START = 0,
	EVENT_TALK_STOP,
	EVENT_NETWORK_SEARCH_START,
	EVENT_NETWORK_SEARCH_STOP
};

/* interface function declaration */

int htc_battery_charger_disable(void);
int htc_battery_get_zcharge_mode(void);

#endif
