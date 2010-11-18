/*
 * Copyright (C) 2010 HTC, Inc.
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
#ifndef __ASM_ARCH_MSM_HTC_USB_H
#define __ASM_ARCH_MSM_HTC_USB_H

#ifdef CONFIG_ARCH_QSD8X50
void msm_hsusb_8x50_phy_reset(void);
#endif

#ifdef CONFIG_USB_ANDROID
#ifdef ERROR
#undef ERROR
#endif
#include <linux/usb/android_composite.h>

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"usb_mass_storage",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_ECM
static char *usb_functions_ecm[] = {
	"cdc_ethernet",
};
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
static char *usb_functions_rndis[] = {
	"ether",
};
static char *usb_functions_rndis_adb[] = {
	"ether",
	"adb",
};
#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_rndis_diag[] = {
	"ether",
	"diag",
};
static char *usb_functions_rndis_adb_diag[] = {
	"ether",
	"adb",
	"diag",
};
#endif
#endif

#ifdef CONFIG_USB_ANDROID_PROJECTOR
static char *usb_functions_projector[] = {
	"usb_mass_storage",
	"projector",
};
static char *usb_functions_adb_projector[] = {
	"usb_mass_storage",
	"adb",
	"projector",
};
#endif

#ifdef CONFIG_USB_ANDROID_MTP
static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};

#endif

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_diag[] = {
	"usb_mass_storage",
	"diag",
};
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

#ifdef CONFIG_USB_ANDROID_SERIAL
static char *usb_functions_modem[] = {
	"usb_mass_storage",
	"modem",
};
static char *usb_functions_adb_modem[] = {
	"usb_mass_storage",
	"adb",
	"modem",
};
#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_diag_modem[] = {
	"usb_mass_storage",
	"diag",
	"modem",
};
static char *usb_functions_adb_diag_modem[] = {
	"usb_mass_storage",
	"adb",
	"diag",
	"modem",
};
static char *usb_functions_adb_diag_serial[] = {
	"usb_mass_storage",
	"adb",
	"diag",
	"serial",
};
static char *usb_functions_diag_serial[] = {
	"usb_mass_storage",
	"diag",
	"serial",
};
#endif
#endif

#ifdef CONFIG_USB_ANDROID_ACM
static char *usb_functions_adb_acm[] = {
	"usb_mass_storage",
	"adb",
	"acm",
};
static char *usb_functions_acm[] = {
	"acm",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"ether",
#endif
#ifdef CONFIG_USB_ANDROID_MTP
	"mtp",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_SERIAL
	"serial",
#endif
#ifdef CONFIG_USB_ANDROID_PROJECTOR
	"projector",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		 .product_id = 0x0c02, /* vary by board */
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
	{
		.product_id	= 0x0ff9,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
#ifdef CONFIG_USB_ANDROID_ACM
	{
		.product_id	= 0x0ff4,
		.num_functions	= ARRAY_SIZE(usb_functions_acm),
		.functions	= usb_functions_acm,
	},
	{
		.product_id	= 0x0ff5,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_acm),
		.functions	= usb_functions_adb_acm,
	},
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	{
		.product_id	= 0x0ff8,
		.num_functions	= ARRAY_SIZE(usb_functions_ecm),
		.functions	= usb_functions_ecm,
	},
#endif
#ifdef CONFIG_USB_ANDROID_SERIAL
	{
		.product_id	= 0x0c03,
		.num_functions	= ARRAY_SIZE(usb_functions_modem),
		.functions	= usb_functions_modem,
	},
	{
		.product_id	= 0x0c04,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_modem),
		.functions	= usb_functions_adb_modem,
	},
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x0c88,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem),
		.functions	= usb_functions_adb_diag_modem,
	},
	{
		.product_id	= 0x0c89,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial),
		.functions	= usb_functions_diag_serial,
	},
	{
		.product_id	= 0x0c8a,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial),
		.functions	= usb_functions_adb_diag_serial,
	},
	{
		.product_id	= 0x0ffb,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem),
		.functions	= usb_functions_diag_modem,
	},
#endif
#endif
#ifdef CONFIG_USB_ANDROID_PROJECTOR
	{
		.product_id	= 0x0c05,
		.num_functions	= ARRAY_SIZE(usb_functions_projector),
		.functions	= usb_functions_projector,
	},
	{
		.product_id	= 0x0c06,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_projector),
		.functions	= usb_functions_adb_projector,
	},
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x0c07,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
	{
		.product_id	= 0x0c08,
		.num_functions	= ARRAY_SIZE(usb_functions_diag),
		.functions	= usb_functions_diag,
	},
#endif
#ifdef CONFIG_USB_ANDROID_MTP
	{
		.product_id	= 0x0ca8,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb),
		.functions	= usb_functions_mtp_adb,
	},
	{
		.product_id	= 0x0c93,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp),
		.functions	= usb_functions_mtp,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0ffc,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x0ff6,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag),
		.functions	= usb_functions_rndis_adb_diag,
	},
	{
		.product_id	= 0x0ff7,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag),
		.functions	= usb_functions_rndis_diag,
	},
#endif
#endif
};
#endif

#endif
