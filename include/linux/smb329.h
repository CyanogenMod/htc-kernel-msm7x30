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
#ifndef _SMB329_H_
#define _SMB329_H_
#include <linux/notifier.h>
#include <mach/htc_battery.h>

#ifdef CONFIG_SMB329
extern int set_charger_ctrl(u32 ctl);
#else
static int set_charger_ctrl(u32 ctl) {return 0 ; }
#endif
#endif
