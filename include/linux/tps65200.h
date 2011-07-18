/*
 * Copyright (C) 2007 HTC Incorporated
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
#ifndef _TPS65200_H_
#define _TPS65200_H_
#include <linux/notifier.h>
#include <mach/htc_battery.h>

enum wled_ctl_t {
	WLED_DISABLE = 0,
	WLED_ENABLE,
	WLED_STATUS
};

struct tps65200_platform_data {
	int charger_check;
};

#if defined(CONFIG_TPS65200) || defined(CONFIG_TPS65200_VIVO)
extern int tps_set_charger_ctrl(u32 ctl);
#else
static int tps_set_charger_ctrl(u32 ctl) {return 0 ; }
#endif
#endif
