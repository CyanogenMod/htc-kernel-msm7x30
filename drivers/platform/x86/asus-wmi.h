/*
 * Asus PC WMI hotkey driver
 *
 * Copyright(C) 2010 Intel Corporation.
 * Copyright(C) 2010-2011 Corentin Chary <corentin.chary@gmail.com>
 *
 * Portions based on wistron_btns.c:
 * Copyright (C) 2005 Miloslav Trmac <mitr@volny.cz>
 * Copyright (C) 2005 Bernhard Rosenkraenzer <bero@arklinux.org>
 * Copyright (C) 2005 Dmitry Torokhov <dtor@mail.ru>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _ASUS_WMI_H_
#define _ASUS_WMI_H_

#include <linux/platform_device.h>

struct module;
struct key_entry;
struct asus_wmi;

struct asus_wmi_driver {
	bool			hotplug_wireless;

	const char		*name;
	struct module		*owner;

	const char		*event_guid;

	const struct key_entry	*keymap;
	const char		*input_name;
	const char		*input_phys;

	int (*probe) (struct platform_device *device);
	void (*quirks) (struct asus_wmi_driver *driver);

	struct platform_driver	platform_driver;
	struct platform_device *platform_device;
};

int asus_wmi_register_driver(struct asus_wmi_driver *driver);
void asus_wmi_unregister_driver(struct asus_wmi_driver *driver);

#endif /* !_ASUS_WMI_H_ */
