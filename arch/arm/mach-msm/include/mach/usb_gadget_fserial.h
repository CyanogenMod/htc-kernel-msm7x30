/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_USB_GADGET_FSERIAL_H__
#define __LINUX_USB_GADGET_FSERIAL_H__

#include <linux/platform_device.h>

enum transport_type {
	USB_GADGET_FSERIAL_TRANSPORT_TTY,
	USB_GADGET_FSERIAL_TRANSPORT_SDIO,
	USB_GADGET_FSERIAL_TRANSPORT_SMD,
};

enum fserial_func_type {
	USB_FSER_FUNC_NONE,
	USB_FSER_FUNC_SERIAL,
	USB_FSER_FUNC_MODEM,
	USB_FSER_FUNC_MODEM_MDM,
};

#define GSERIAL_NO_PORTS 5
struct usb_gadget_fserial_platform_data {
	enum transport_type	transport[GSERIAL_NO_PORTS];
	enum fserial_func_type	func_type[GSERIAL_NO_PORTS];
	unsigned		no_ports;
};

struct usb_gadget_facm_pdata {
	enum transport_type	transport[GSERIAL_NO_PORTS];
	enum fserial_func_type	func_type[GSERIAL_NO_PORTS];
	unsigned		no_ports;
};
#endif
