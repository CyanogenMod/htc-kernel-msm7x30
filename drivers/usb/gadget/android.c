/*
 * Gadget Driver for Android
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"
#include <linux/wakelock.h>
#include <mach/perflock.h>

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";
static struct wake_lock usb_rndis_idle_wake_lock;
static struct perf_lock usb_rndis_perf_lock;

enum {
	USB_FUNCTION_UMS = 0,
	USB_FUNCTION_ADB = 1,
	USB_FUNCTION_RNDIS,
	USB_FUNCTION_DIAG,
	USB_FUNCTION_SERIAL,
	USB_FUNCTION_PROJECTOR,
	USB_FUNCTION_FSYNC,
	USB_FUNCTION_MTP,
	USB_FUNCTION_MODEM,
	USB_FUNCTION_ECM,
	USB_FUNCTION_ACM,
};

#define PID_RNDIS		0x0ffe
#define PID_ECM			0x0ff8
#define PID_ACM			0x0ff4
#ifdef CONFIG_USB_ANDROID_MTP
#define MS_VENDOR_CODE	0x0b
#define FEATURE_DESC_SIZE	64
#define PID_MTP			0x0c93
#define PID_MTP_ADB		0x0ca8
struct ms_comp_feature_descriptor {
	__le32 dwLength;
	__le16 bcdVersion;
	__le16 wIndex;
	__u8 bCount;
	__u8 resv1[7];
	/* for MTP */
	__u8 bFirstInterfaceNumber;
	__u8 resv2;
	__u8 compatibleID[8];
	__u8 subCompatibleID[8];
	__u8 resv3[6];
	/* for adb */
	__u8 bFirstInterfaceNumber2;
	__u8 resv4;
	__u8 compatibleID2[8];
	__u8 subCompatibleID2[8];
	__u8 resv5[6];
} __attribute__ ((packed));


static struct ms_comp_feature_descriptor ms_comp_desc = {
	.dwLength = __constant_cpu_to_le32(FEATURE_DESC_SIZE),
	.bcdVersion = __constant_cpu_to_le16(0x0100),
	.wIndex = __constant_cpu_to_le16(0x0004),
	.bCount = 0x02,
	/* for MTP */
	.bFirstInterfaceNumber = 0,
	.resv2 = 1,
	.compatibleID = "MTP",
	/* for adb */
	.bFirstInterfaceNumber2 = 1,
	.resv4 = 1,
};

#endif

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

struct android_dev {
	struct usb_composite_dev *cdev;
	struct usb_configuration *config;
	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **functions;

	int product_id;
	int version;
};

static struct android_dev *_android_dev;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct list_head _functions = LIST_HEAD_INIT(_functions);
static int _registered_function_count = 0;
static int get_product_id(struct android_dev *dev);

void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function	*f;
	list_for_each_entry(f, &_functions, list) {
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function	*f;
	char **functions = dev->functions;
	int	product_id;
	int i;

	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
		if (f)
			f->bind_config(dev->config);
		else
			printk(KERN_ERR "function %s not found in bind_functions\n", name);
	}
	product_id = get_product_id(dev);
	printk(KERN_INFO "usb: product_id=0x%x\n", product_id);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	if (dev->cdev)
		dev->cdev->desc.idProduct = device_desc.idProduct;
}

static int __init android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_DEBUG "android_bind_config\n");
	dev->config = c;

	/* bind our functions if they have all registered */
	if (_registered_function_count == dev->num_functions)
		bind_functions(dev);

	return 0;
}

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;
#ifdef CONFIG_USB_ANDROID_MTP
	struct usb_composite_dev *cdev = c->cdev;
	struct usb_request	*req = cdev->req;
	struct android_dev *dev = _android_dev;
	int product_id;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	/* vendor request to GET OS feature descriptor */
	if (((ctrl->bRequestType << 8) | ctrl->bRequest) ==
		(((USB_DIR_IN | USB_TYPE_VENDOR) << 8) | MS_VENDOR_CODE)) {

		printk(KERN_DEBUG "usb: get OS feature descriptor\n");
		if (w_index != 0x0004 || w_length > FEATURE_DESC_SIZE)
			return ret;

		product_id = get_product_id(dev);
		printk(KERN_INFO "mtp: product_id = 0x%x\n", product_id);
		if (product_id == PID_MTP) {
			ms_comp_desc.bCount = 1;
			ms_comp_desc.dwLength =
				__constant_cpu_to_le32(40);
			ret = w_length;
		} else if (product_id == PID_MTP_ADB) {
			ms_comp_desc.bCount = 2;
			ms_comp_desc.dwLength =
				__constant_cpu_to_le32(64);
			ret = w_length;
		}
		/* respond with data transfer or status phase? */
		if (ret >= 0) {
			memcpy(req->buf, (char *)&ms_comp_desc, w_length);
			req->zero = 0;
			req->length = ret;
			ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
			if (ret < 0)
				ERROR(cdev, "get feature desc err %d\n", ret);
			return ret;
		}
	}
#endif
	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup) {
			ret = android_config_driver.interface[i]->setup(
				android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

static int product_has_function(struct android_usb_product *p,
		struct usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		if (!strcmp(name, *functions++))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p)
{
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (product_has_function(p, f) == !!f->hidden)
			return 0;
	}
	return 1;
}

static int get_product_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p))
				return p->product_id;
		}
	}
	/* use default product ID */
	return dev->product_id;
}

static int __init android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, product_id, ret;

	printk(KERN_INFO "android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	if (gadget->ops->wakeup)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
	product_id = get_product_id(dev);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	cdev->desc.idProduct = device_desc.idProduct;

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.enable_function = android_enable_function,
};

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_register_function %s\n", f->name);
	list_add_tail(&f->list, &_functions);
	_registered_function_count++;

	/* bind our functions if they have all registered
	 * and the main driver has bound.
	 */
	if (dev->config && _registered_function_count == dev->num_functions)
		bind_functions(dev);
}
int android_show_function(char *buf)
{
	unsigned length = 0;
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		length += sprintf(buf + length, "%s:%s\n", f->name,
			(f->hidden)?"disable":"enable");
	}
	return length;
}

int android_switch_function(unsigned func)
{
	struct usb_function		*f;
	struct android_dev *dev = _android_dev;
	int product_id;

	printk(KERN_INFO "%s: %u\n", __func__, func);

	list_for_each_entry(f, &android_config_driver.functions, list) {
		if ((func & (1 << USB_FUNCTION_UMS)) &&
			!strcmp(f->name, "usb_mass_storage"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_ADB)) &&
			!strcmp(f->name, "adb"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_ECM)) &&
			!strcmp(f->name, "cdc_ethernet"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_ACM)) &&
			!strcmp(f->name, "acm"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_RNDIS)) &&
			!strcmp(f->name, "ether")) {
			if (f->hidden) {
				printk("%s: rndis perf lock\n", __func__);
				wake_lock(&usb_rndis_idle_wake_lock);
				if (!is_perf_lock_active(&usb_rndis_perf_lock))
					perf_lock(&usb_rndis_perf_lock);
			}
			f->hidden = 0;
		} else if ((func & (1 << USB_FUNCTION_DIAG)) &&
			!strcmp(f->name, "diag"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_MODEM)) &&
			!strcmp(f->name, "modem"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_SERIAL)) &&
			!strcmp(f->name, "serial"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_MTP)) &&
			!strcmp(f->name, "mtp"))
			f->hidden = 0;
		/* also enable adb with MTP function */
		else if ((func & (1 << USB_FUNCTION_MTP)) &&
			!strcmp(f->name, "adb"))
			f->hidden = 0;
		else if ((func & (1 << USB_FUNCTION_PROJECTOR)) &&
			!strcmp(f->name, "projector"))
			f->hidden = 0;
		else {
			if (!strcmp(f->name, "ether") && !f->hidden) {
				printk("%s: rndis perf unlock\n", __func__);
				wake_unlock(&usb_rndis_idle_wake_lock);
				if (is_perf_lock_active(&usb_rndis_perf_lock))
					perf_unlock(&usb_rndis_perf_lock);
			}
			f->hidden = 1;
		}
	}
	product_id = get_product_id(dev);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	if (dev->cdev)
		dev->cdev->desc.idProduct = device_desc.idProduct;

	/* We need to specify the COMM class in the device descriptor
	* if we are using RNDIS.
	*/
	if (product_id == PID_RNDIS || product_id == PID_ECM || product_id == PID_ACM)
		dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
	else
		dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;

#ifdef CONFIG_USB_GADGET_MSM_72K
	msm_hsusb_request_reset();
#else
	/* force reenumeration */
	if (dev->cdev && dev->cdev->gadget &&
			dev->cdev->gadget->speed != USB_SPEED_UNKNOWN) {
		usb_gadget_disconnect(dev->cdev->gadget);
		msleep(10);
		usb_gadget_connect(dev->cdev->gadget);
	}
#endif
	return 0;
}

void android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	int product_id;

	if (!!f->hidden != disable) {
		f->hidden = disable;
		product_id = get_product_id(dev);
		device_desc.idProduct = __constant_cpu_to_le16(product_id);
		if (dev->cdev)
			dev->cdev->desc.idProduct = device_desc.idProduct;

#ifdef CONFIG_USB_ANDROID_RNDIS
		/* We need to specify the COMM class in the device descriptor
		* if we are using RNDIS.
		*/
		if (product_id == PID_RNDIS)
			dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
		else
			dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
#endif

		if (product_id == PID_ECM || product_id == PID_ACM)
			dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
		else
			dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
#ifdef CONFIG_USB_GADGET_MSM_72K
		msm_hsusb_request_reset();
#else
		/* force reenumeration */
		if (dev->cdev && dev->cdev->gadget &&
				dev->cdev->gadget->speed != USB_SPEED_UNKNOWN) {
			usb_gadget_disconnect(dev->cdev->gadget);
			msleep(10);
			usb_gadget_connect(dev->cdev->gadget);
		}
#endif
	}
}
void android_set_serialno(char *serialno)
{
	strings_dev[STRING_SERIAL_IDX].s = serialno;
}

int android_get_model_id(void)
{
	struct android_dev *dev = _android_dev;
	return dev->product_id;
}


static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_probe pdata: %p\n", pdata);

	if (pdata) {
		dev->products = pdata->products;
		dev->num_products = pdata->num_products;
		dev->functions = pdata->functions;
		dev->num_functions = pdata->num_functions;
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
	}

	return usb_composite_register(&android_usb_driver);
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;

	printk(KERN_INFO "android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	_android_dev = dev;

	wake_lock_init(&usb_rndis_idle_wake_lock, WAKE_LOCK_IDLE, "rndis_idle_lock");
	perf_lock_init(&usb_rndis_perf_lock, PERF_LOCK_HIGHEST, "rndis");

	return platform_driver_register(&android_platform_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
