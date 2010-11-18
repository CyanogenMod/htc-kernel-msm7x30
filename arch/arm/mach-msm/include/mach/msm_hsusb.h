/* linux/include/asm-arm/arch-msm/hsusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_HSUSB_H
#define __ASM_ARCH_MSM_HSUSB_H

#include <linux/types.h>

/* platform device data for msm_hsusb driver */

#ifdef CONFIG_USB_FUNCTION
/* matches a product ID to a list of enabled functions */
struct msm_hsusb_product {
	/* product ID for usb_device_descriptor.idProduct */
	__u16 product_id;

	/* bit mask of enabled usb_functions, matching ordering
	** in msm_hsusb_platform_data.functions
	*/
	__u32 functions;
};
#endif

struct msm_hsusb_platform_data {
	/* hard reset the ULPI PHY */
	void (*phy_reset)(void);
	void (*phy_shutdown)(void);

	/* (de)assert the reset to the usb core */
	void (*hw_reset)(bool enable);

	/* for notification when USB is connected or disconnected */
	void (*usb_connected)(int);
	/* 1 : uart, 0 : usb */
	void (*usb_uart_switch)(int);
	void (*config_usb_id_gpios)(bool enable);
	void (*usb_hub_enable)(bool);
	void (*serial_debug_gpios)(int);
	int (*china_ac_detect)(void);
	void (*disable_usb_charger)(void);
	/* val, reg pairs terminated by -1 */
	int *phy_init_seq;

#ifdef CONFIG_USB_FUNCTION
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID.
	** This can be overridden dynamically based on the disabled
	** state of the functions using the product_table.
	*/
	__u16 product_id;

	__u16 version;
	char *product_name;
	char *manufacturer_name;

	/* list of function drivers to bind to this configuration */
	int num_functions;
	char **functions;

	/* if num_products is zero, then the default value in product_id
	** is used for the configuration descriptor.
	*/
	int num_products;
	struct msm_hsusb_product *products;
#endif
	char *serial_number;
	int usb_id_pin_gpio;
	int dock_pin_gpio;
	int id_pin_irq;
	bool enable_car_kit_detect;
	__u8 accessory_detect;
	bool dock_detect;
};

int usb_get_connect_type(void);
#endif
