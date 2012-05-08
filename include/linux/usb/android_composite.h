/*
 * Platform data for Android USB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#ifndef	__LINUX_USB_ANDROID_H
#define	__LINUX_USB_ANDROID_H

#include <linux/usb/composite.h>
#include <linux/if_ether.h>

#if 0
struct android_usb_function {
	struct list_head	list;
	char			*name;
	int 			(*bind_config)(struct usb_configuration *c);
};
#endif

struct android_usb_product {
	/* Vendor ID for this set of functions.
	 * Default vendor_id in platform data will be used if this is zero.
	 */
	__u16 vendor_id;

	/* Default product ID. */
	__u16 product_id;

	/* List of function names associated with this product.
	 * This is used to compute the USB product ID dynamically
	 * based on which functions are enabled.
	 */
	int num_functions;
	char **functions;
};

struct android_usb_platform_data {
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID. */
	__u16 product_id;

	__u16 version;

	char *product_name;
	char *manufacturer_name;
	char *serial_number;

	/* List of available USB products.
	 * This is used to compute the USB product ID dynamically
	 * based on which functions are enabled.
	 * if num_products is zero or no match can be found,
	 * we use the default product ID
	 */
	int num_products;
	struct android_usb_product *products;

	/* List of all supported USB functions.
	 * This list is used to define the order in which
	 * the functions appear in the configuration's list of USB interfaces.
	 * This is necessary to avoid depending upon the order in which
	 * the individual function drivers are initialized.
	 */
	int num_functions;
	char **functions;

	void (*enable_fast_charge)(bool enable);
	bool RndisDisableMPDecision;

	/* To indicate the GPIO num for USB id
	 */
	int usb_id_pin_gpio;

	/* For QCT diag
	 */
	int (*update_pid_and_serial_num)(uint32_t, const char *);

	/* For multiple serial function support
	 * "Ex: tty:serial[,sdio:modem_mdm][,smd:modem]"
	 */
	char *fserial_init_string;

	/* The gadget driver need to initial at beginning
	 */
	unsigned char diag_init:1;
	unsigned char modem_init:1;
	unsigned char rmnet_init:1;
	unsigned char reserved:5;

	/* ums initial parameters */

	/* number of LUNS */
	int nluns;
	/* bitmap of lun to indicate cdrom disk.
	 * NOTE: Only support one cdrom disk
	 * and it must be located in last lun */
	int cdrom_lun;
	int cdrom_cttype;


	/* Re-match the product ID.
	 * In some devices, the product id is specified by vendor request.
	 *
	 * @param product_id: the common product id
	 * @param intrsharing: 1 for internet sharing, 0 for internet pass through
	 */
	int (*match)(int product_id, int intrsharing);

	/* request usb controller/phy reset during function switch
	 * in some platfrom, we need this behavior to improve the USB stability
	 */
	int req_reset_during_switch_func;
};

/* Platform data for "usb_mass_storage" driver. */
struct usb_mass_storage_platform_data {
	/* Contains values for the SC_INQUIRY SCSI command. */
	char *vendor;
	char *product;
	int release;

	char can_stall;
	/* number of LUNS */
	int nluns;
};

/* Platform data for USB ethernet driver. */
struct usb_ether_platform_data {
	u8	ethaddr[ETH_ALEN];
	u32	vendorID;
	const char *vendorDescr;
};

#if defined(CONFIG_MACH_HOLIDAY)
extern u8 in_usb_tethering;
#endif

#if 0
extern void android_register_function(struct android_usb_function *f);
extern int android_enable_function(struct usb_function *f, int enable);
#endif


#endif	/* __LINUX_USB_ANDROID_H */
