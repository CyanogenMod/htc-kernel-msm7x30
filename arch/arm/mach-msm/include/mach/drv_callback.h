/* linux/arch/arm/mach-msm/drv_callback.h
 *
 * Copyright (C) 2009 HTC Corporation.
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

#include <linux/list.h>
#include <linux/spinlock.h>

struct cnf_driver {
	const char		*name;
	int (*func)		(void *);

	/* configurable driver list lock */
	rwlock_t		cnfdrv_list_lock;
	struct list_head        next_drv;
};

int cnf_driver_register(struct cnf_driver *);
int cnf_driver_event(const char *, void *argu);
