/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android_composite.h>

#include "gadget_chips.h"
#include <linux/wakelock.h>
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif

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

#include "f_diag.c"
#if defined(CONFIG_USB_ANDROID_RMNET_SMD)
#include "f_rmnet_smd.c"
#elif defined(CONFIG_USB_ANDROID_RMNET_SDIO)
#include "f_rmnet_sdio.c"
#elif defined(CONFIG_USB_ANDROID_RMNET_SMD_SDIO)
#include "f_rmnet_smd_sdio.c"
#elif defined(CONFIG_USB_ANDROID_RMNET_BAM)
#include "f_rmnet.c"
#endif
#include "f_mass_storage.c"
#include "u_serial.c"
#include "u_sdio.c"
#include "u_smd.c"
#include "u_bam.c"
#include "u_rmnet_ctrl_smd.c"
#include "f_serial.c"
#ifdef CONFIG_USB_ANDROID_ACM
#include "f_acm.c"
#endif
#include "f_adb.c"
#if 0
#include "f_ccid.c"
#endif
#ifdef CONFIG_USB_ANDROID_MTP
#include "f_mtp.c"
#endif
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#ifdef CONFIG_USB_ANDROID_RNDIS
#include "f_rndis.c"
#include "rndis.c"
#endif
#ifdef CONFIG_USB_ANDROID_ECM
#include "f_ecm.c"
#endif
#include "u_ether.c"
#ifdef CONFIG_USB_ANDROID_USBNET
#include "f_usbnet.c"
#endif

#include <linux/usb/htc_info.h>

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug(fmt, args...) \
	printk(KERN_DEBUG "[USB] " pr_fmt(fmt), ## args)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, args...) \
	printk(KERN_ERR "[USB] " pr_fmt(fmt), ## args)

#ifdef pr_warning
#undef pr_warning
#endif
#define pr_warning(fmt, args...) \
	printk(KERN_WARNING "[USB] " pr_fmt(fmt), ## args)

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, args...) \
	printk(KERN_INFO "[USB] " pr_fmt(fmt), ## args)

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

static bool connect2pc;

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *, struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *, struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured
	 *	and/or before the function is enabled */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);

	/* for performance requirement */
	int performance_lock;
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;
	struct android_usb_platform_data *pdata;

	bool enabled;
	bool connected;
	bool sw_connected;
	struct work_struct work;
	struct delayed_work init_work;
	/* waiting for enabling functions */
	struct list_head function_list;

	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **in_house_functions;

	int product_id;
	void (*enable_fast_charge)(bool enable);
	bool RndisDisableMPDecision;
	int (*match)(int product_id, int intrsharing);
};

static struct class *android_class;
static struct android_dev *_android_dev;

#ifdef CONFIG_USB_ANDROID_PROJECTOR
#include "f_projector.c"
#endif

static struct wake_lock android_usb_idle_wake_lock;
#ifdef CONFIG_PERFLOCK
static struct perf_lock android_usb_perf_lock;
#endif


static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);


/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
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

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE /* | USB_CONFIG_ATT_SELFPOWER */,
	.bMaxPower	= 0xFA, /* 500ma */
};

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	unsigned long flags;
	struct android_usb_function *f;
	int count = 0;

	/* release performance related locks first */
	if (wake_lock_active(&android_usb_idle_wake_lock))
		wake_unlock(&android_usb_idle_wake_lock);
#ifdef CONFIG_PERFLOCK
	if (is_perf_lock_active(&android_usb_perf_lock))
		perf_unlock(&android_usb_perf_lock);
#endif


	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config) {
		spin_unlock_irqrestore(&cdev->lock, flags);
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
							configured);
		pr_info("USB_STATE=CONFIGURED");

		/* hold perflock, wakelock for performance consideration */
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->performance_lock) {
				pr_info("Performance lock for '%s'\n", f->name);
				count++;
			}
		}
		if (count) {
			if (!wake_lock_active(&android_usb_idle_wake_lock))
				wake_lock(&android_usb_idle_wake_lock);
#ifdef CONFIG_PERFLOCK
			if (!is_perf_lock_active(&android_usb_perf_lock))
				perf_lock(&android_usb_perf_lock);
#endif
		}

		if (!connect2pc && dev->connected && !is_mtp_enabled) {
			connect2pc = true;
			switch_set_state(&cdev->sw_connect2pc, 1);
			pr_info("set usb_connect2pc = 1\n");
		}
		return;
	}

	if (dev->connected != dev->sw_connected) {
		dev->sw_connected = dev->connected;
		spin_unlock_irqrestore(&cdev->lock, flags);
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
				dev->sw_connected ? connected : disconnected);

		pr_info("%s\n", dev->connected ? connected[0] : disconnected[0]);
	} else {
		spin_unlock_irqrestore(&cdev->lock, flags);
	}

	if (connect2pc && !dev->connected && !is_mtp_enabled) {
		connect2pc = false;
		switch_set_state(&cdev->sw_connect2pc, 0);
		pr_info("set usb_connect2pc = 0\n");
	}
}


/*-------------------------------------------------------------------------*/
/* Supported functions initialization */
static ssize_t func_en_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct android_usb_function *func = dev_get_drvdata(dev);
	struct android_usb_function *f;
	int ebl = 0;

	list_for_each_entry(f, &_android_dev->enabled_functions, enabled_list) {
		if (!strcmp(func->name, f->name)) {
			ebl = 1;
			break;
		}
	}
	return sprintf(buf, "%d", ebl);
}

static ssize_t func_en_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct android_usb_function *func = dev_get_drvdata(dev);
	struct android_usb_function *f;
	int ebl = 0;
	int value;

	sscanf(buf, "%d", &value);
	list_for_each_entry(f, &_android_dev->enabled_functions, enabled_list) {
		if (!strcmp(func->name, f->name)) {
			ebl = 1;
			break;
		}
	}
	if (!!value == ebl) {
		pr_info("%s function is already %s\n", func->name
			, ebl ? "enable" : "disable");
		return size;
	}

	if (value)
		htc_usb_enable_function(func->name, 1);
	else
		htc_usb_enable_function(func->name, 0);

	return size;
}
static DEVICE_ATTR(on, S_IRUGO | S_IWUSR | S_IWGRP, func_en_show, func_en_store);

#if defined(CONFIG_USB_ANDROID_RMNET_SMD)
/* RMNET_SMD */
static int rmnet_smd_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_bind_config(c);
}

static struct android_usb_function rmnet_smd_function = {
	.name		= "rmnet",
	.bind_config	= rmnet_smd_function_bind_config,
	.performance_lock = 1,
};
#elif defined(CONFIG_USB_ANDROID_RMNET_SDIO)

/* RMNET_SDIO */
static int rmnet_sdio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_sdio_function_add(c);
}

static struct android_usb_function rmnet_sdio_function = {
	.name		= "rmnet",
	.bind_config	= rmnet_sdio_function_bind_config,
	.performance_lock = 1,
};

#elif defined(CONFIG_USB_ANDROID_RMNET_SMD_SDIO)
/* RMNET_SMD_SDIO */
static int rmnet_smd_sdio_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return rmnet_smd_sdio_init();
}

static void rmnet_smd_sdio_function_cleanup(struct android_usb_function *f)
{
	rmnet_smd_sdio_cleanup();
}

static int rmnet_smd_sdio_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_sdio_function_add(c);
}

static struct device_attribute *rmnet_smd_sdio_attributes[] = {
					&dev_attr_transport, NULL };

static struct android_usb_function rmnet_smd_sdio_function = {
	.name		= "rmnet",
	.init		= rmnet_smd_sdio_function_init,
	.cleanup	= rmnet_smd_sdio_function_cleanup,
	.bind_config	= rmnet_smd_sdio_bind_config,
	.attributes	= rmnet_smd_sdio_attributes,
	.performance_lock = 1,
};
#elif defined(CONFIG_USB_ANDROID_RMNET_BAM)
/* RMNET - used with BAM */
#define MAX_RMNET_INSTANCES 1
static int rmnet_instances = 1;
static int rmnet_function_init(struct android_usb_function *f,
					 struct usb_composite_dev *cdev)
{
	return frmnet_init_port(MAX_RMNET_INSTANCES);
}

static void rmnet_function_cleanup(struct android_usb_function *f)
{
	frmnet_cleanup();
}

static int rmnet_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int i;
	int ret = 0;

	for (i = 0; i < rmnet_instances; i++) {
		ret = frmnet_bind_config(c, i);
		if (ret) {
			pr_err("Could not bind rmnet%u config\n", i);
			break;
		}
	}

	return ret;
}

static ssize_t rmnet_instances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", rmnet_instances);
}

static ssize_t rmnet_instances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	pr_info("%s, buff: %s\n", __func__, buf);
	sscanf(buf, "%d", &value);
	if (value > MAX_RMNET_INSTANCES)
		value = MAX_RMNET_INSTANCES;
	rmnet_instances = value;
	return size;
}

static DEVICE_ATTR(instances, S_IRUGO | S_IWUSR, rmnet_instances_show,
						 rmnet_instances_store);
static struct device_attribute *rmnet_function_attributes[] = {
					&dev_attr_instances, NULL };

static struct android_usb_function rmnet_function = {
	.name		= "rmnet",
	.init		= rmnet_function_init,
	.cleanup	= rmnet_function_cleanup,
	.bind_config	= rmnet_function_bind_config,
	.attributes	= rmnet_function_attributes,
	.performance_lock = 1,
};
#endif

/* DIAG */
static char diag_clients[32];	    /*enabled DIAG clients- "diag[,diag_mdm]" */
static ssize_t clients_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	pr_info("%s, buff: %s\n", __func__, buff);
	strlcpy(diag_clients, buff, sizeof(diag_clients));

	return size;
}

static DEVICE_ATTR(clients, S_IWUSR, NULL, clients_store);
static struct device_attribute *diag_function_attributes[] =
					 { &dev_attr_clients, NULL };

static int diag_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return diag_setup();
}

static void diag_function_cleanup(struct android_usb_function *f)
{
	diag_cleanup();
}

static int diag_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
#if 1
	int err;
	int (*notify)(uint32_t, const char *);

	notify = _android_dev->pdata->update_pid_and_serial_num;

	err = diag_function_add(c, "diag", notify);
	if (err)
		pr_err("diag: Cannot open channel 'diag'");
#else
	char *name;
	char buf[32], *b;
	int once = 0, err = -1;
	int (*notify)(uint32_t, const char *);

	strlcpy(buf, diag_clients, sizeof(buf));
	b = strim(buf);

	while (b) {
		notify = NULL;
		name = strsep(&b, ",");
		/* Allow only first diag channel to update pid and serial no */
		if (_android_dev->pdata && !once++)
			notify = _android_dev->pdata->update_pid_and_serial_num;

		if (name) {
			err = diag_function_add(c, name, notify);
			if (err)
				pr_err("diag: Cannot open channel '%s'", name);
		}
	}
#endif
	return err;
}

static struct android_usb_function diag_function = {
	.name		= "diag",
	.init		= diag_function_init,
	.cleanup	= diag_function_cleanup,
	.bind_config	= diag_function_bind_config,
	.attributes	= diag_function_attributes,
};

#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
static int diag_mdm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err;
	int (*notify)(uint32_t, const char *);

	notify = NULL;

	err = diag_function_add(c, "diag_mdm", notify);
	if (err)
		pr_err("diag: Cannot open channel 'diag_mdm'");

	return 0;
}

static struct android_usb_function diag_mdm_function = {
	.name		= "diag_mdm",
	.bind_config	= diag_mdm_function_bind_config,
};
#endif

/* Serial, Modem */
static int serial_driver_initial(struct usb_configuration *c)
{
	char *name, *str[2];
	char buf[80], *b;
	int err = -1;
	static int serial_initialized = 0, ports = 0;
	char *init_string;

	if (serial_initialized) {
		pr_info("%s: already initial\n", __func__);
		return ports;
	}
	serial_initialized = 1;

	init_string = _android_dev->pdata->fserial_init_string ?
		_android_dev->pdata->fserial_init_string :
		"smd:modem,tty,tty,tty:serial";

	strncpy(buf, init_string, sizeof(buf));
	buf[79] = 0;
	pr_info("%s: init string: %s\n", __func__, buf);

	b = strim(buf);

	while (b) {
		str[0] = str[1] = 0;
		name = strsep(&b, ",");
		if (name) {
			str[0] = strsep(&name, ":");
			if (str[0])
				str[1] = strsep(&name, ":");
		}
		err = gserial_init_port(ports, str[0], str[1]);
		if (err) {
			pr_err("serial: Cannot open port '%s'\n", str[0]);
			goto out;
		}
		ports++;
	}

	err = gport_setup(c);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}
	return ports;

out:
	return err;
}

/* Modem */
static void modem_function_cleanup(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	/* ToDo: need to cleanup by different channel */
	gsmd_cleanup(dev->cdev->gadget, 1);
}

static int modem_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports;

	ports = serial_driver_initial(c);
	if (ports < 0)
		goto out;


	for (i = 0; i < ports; i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_MODEM) {
			err = gser_bind_config(c, i);
			if (err) {
				pr_err("serial: bind_config failed for port %d", i);
				goto out;
			}
		}
	}

out:
	return err;
}

static struct android_usb_function modem_function = {
	.name		= "modem",
	.cleanup	= modem_function_cleanup,
	.bind_config	= modem_function_bind_config,
	.performance_lock = 1,
};

#ifdef CONFIG_USB_ANDROID_MDM9K_MODEM
/* Modem_Mdm */
static void modem_mdm_function_cleanup(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	/* ToDo: need to cleanup by different channel */
	gsmd_cleanup(dev->cdev->gadget, 1);
}

static int modem_mdm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports;

	ports = serial_driver_initial(c);
	if (ports < 0)
		goto out;


	for (i = 0; i < ports; i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_MODEM_MDM) {
			err = gser_bind_config(c, i);
			if (err) {
				pr_err("serial: bind_config failed for port %d", i);
				goto out;
			}
		}
	}

out:
	return err;
}

static struct android_usb_function modem_mdm_function = {
	.name		= "modem_mdm",
	.cleanup	= modem_mdm_function_cleanup,
	.bind_config	= modem_mdm_function_bind_config,
	.performance_lock = 1,
};
#endif

/* SERIAL */
static char serial_transports[32];	/*enabled FSERIAL ports - "tty[,sdio]"*/
static ssize_t serial_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	pr_info("%s: %s\n", __func__, buff);
	strlcpy(serial_transports, buff, sizeof(serial_transports));

	return size;
}

static DEVICE_ATTR(transports, S_IWUSR, NULL, serial_transports_store);
static struct device_attribute *serial_function_attributes[] =
					 { &dev_attr_transports, NULL };

static void serial_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int serial_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
#if 1
	int err = -1;
	int i, ports;

	ports = serial_driver_initial(c);
	if (ports < 0)
		goto out;
	for (i = 0; i < ports; i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_SERIAL) {
			err = gser_bind_config(c, i);
			if (err) {
				pr_err("serial: bind_config failed for port %d", i);
				goto out;
			}
		}
	}
#else
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int serial_initialized = 0, ports = 0;

	if (serial_initialized)
		goto bind_config;

	serial_initialized = 1;
	strlcpy(buf, serial_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = gserial_init_port(ports, name);
			if (err) {
				pr_err("serial: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = gport_setup(c);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) {
		err = gser_bind_config(c, i);
		if (err) {
			pr_err("serial: bind_config failed for port %d", i);
			goto out;
		}
	}
#endif

out:
	return err;
}

static struct android_usb_function serial_function = {
	.name		= "serial",
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.attributes	= serial_function_attributes,
};

/* ADB */
static int adb_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
}

static int adb_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

/* CCID */
#if 0
static int ccid_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return ccid_setup();
}

static void ccid_function_cleanup(struct android_usb_function *f)
{
	ccid_cleanup();
}

static int ccid_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return ccid_bind_config(c);
}

static struct android_usb_function ccid_function = {
	.name		= "ccid",
	.init		= ccid_function_init,
	.cleanup	= ccid_function_cleanup,
	.bind_config	= ccid_function_bind_config,
};
#endif

#ifdef CONFIG_USB_ANDROID_ACM
#define MAX_ACM_INSTANCES 4
struct acm_function_config {
	int instances;
};

static int acm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	struct acm_function_config *config;
	f->config = kzalloc(sizeof(struct acm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	config = f->config;
	config->instances = 1;
	return gserial_setup(cdev->gadget, MAX_ACM_INSTANCES);
}

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
	kfree(f->config);
	f->config = NULL;
}

static int acm_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	int i;
	int ret = 0;
	struct acm_function_config *config = f->config;

	for (i = 0; i < config->instances; i++) {
		ret = acm_bind_config(c, i);
		if (ret) {
			pr_err("Could not bind acm%u config\n", i);
			break;
		}
	}

	return ret;
}

static ssize_t acm_instances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->instances);
}

static ssize_t acm_instances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_ACM_INSTANCES)
		value = MAX_ACM_INSTANCES;
	config->instances = value;
	return size;
}

static DEVICE_ATTR(instances, S_IRUGO | S_IWUSR, acm_instances_show, acm_instances_store);
static struct device_attribute *acm_function_attributes[] = { &dev_attr_instances, NULL };

static struct android_usb_function acm_function = {
	.name		= "acm",
	.init		= acm_function_init,
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};
#endif

#ifdef CONFIG_USB_ANDROID_MTP
static int mtp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int mtp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int ptp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int ptp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};
#endif

#ifdef CONFIG_USB_ANDROID_ECM
/* ECM */
struct ecm_function_config {
	u8      ethaddr[ETH_ALEN];
};

static int ecm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	struct ecm_function_config *ecm;
	f->config = kzalloc(sizeof(struct ecm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	ecm = f->config;
	return 0;
}

static void ecm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int ecm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct ecm_function_config *ecm = f->config;

	if (!ecm) {
		pr_err("%s: ecm_pdata\n", __func__);
		return -1;
	}


	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "usb");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}


	return ecm_bind_config(c, ecm->ethaddr);
}

static void ecm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t ecm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
}

static ssize_t ecm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ecm->ethaddr[0], (int *)&ecm->ethaddr[1],
		    (int *)&ecm->ethaddr[2], (int *)&ecm->ethaddr[3],
		    (int *)&ecm->ethaddr[4], (int *)&ecm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ecm_ethaddr, S_IRUGO | S_IWUSR, ecm_ethaddr_show,
					       ecm_ethaddr_store);

static struct device_attribute *ecm_function_attributes[] = {
	&dev_attr_ecm_ethaddr,
	NULL
};

static struct android_usb_function ecm_function = {
	.name		= "cdc_ethernet",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_function_bind_config,
	.unbind_config	= ecm_function_unbind_config,
	.attributes	= ecm_function_attributes,
	.performance_lock = 1,
};
#endif

#ifdef CONFIG_USB_ANDROID_RNDIS
/* RNDIS */
struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

static int rndis_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	struct rndis_function_config *rndis;
	struct android_dev *dev = _android_dev;

	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	rndis = f->config;

	strncpy(rndis->manufacturer, dev->pdata->manufacturer_name, sizeof(rndis->manufacturer));
	rndis->vendorID = dev->pdata->vendor_id;

	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int rndis_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "usb");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config(c, rndis->ethaddr, rndis->vendorID,
				    rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%255s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
	.performance_lock = 1,
};
#endif

struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	struct android_dev *dev = _android_dev;
	int i;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;


	if (dev->pdata->nluns) {
		config->fsg.nluns = dev->pdata->nluns;
		if (config->fsg.nluns > FSG_MAX_LUNS)
			config->fsg.nluns = FSG_MAX_LUNS;
		for (i = 0; i < config->fsg.nluns; i++) {
			if (dev->pdata->cdrom_lun & (1 << i)) {
				config->fsg.luns[i].cdrom = 1;
				config->fsg.luns[i].removable = 1;
				config->fsg.luns[i].ro = 1;
			} else {
				config->fsg.luns[i].cdrom = 0;
				config->fsg.luns[i].removable = 1;
				config->fsg.luns[i].ro = 0;
			}
		}
	} else {
		/* default value */
		config->fsg.nluns = 1;
		config->fsg.luns[0].removable = 1;
	}

	config->fsg.vendor_name = dev->pdata->manufacturer_name;
	config->fsg.product_name= dev->pdata->product_name;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	for (i = 0; i < config->fsg.nluns; i++) {
		err = sysfs_create_link(&f->dev->kobj,
					&common->luns[i].dev.kobj,
					common->luns[i].dev.kobj.name);
		if (err) {
			fsg_common_release(&common->ref);
			kfree(config);
			return err;
		}
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%28s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

#ifdef CONFIG_USB_ANDROID_PROJECTOR
static int projector_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct htcmode_protocol), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return projector_setup(f->config);
}

static void projector_function_cleanup(struct android_usb_function *f)
{
	if (f->config) {
		kfree(f->config);
		f->config = NULL;
	}
}

static int projector_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return projector_bind_config(c);
}

static int projector_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return projector_ctrlrequest(cdev, c);
}


static ssize_t projector_product_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PRODUCT_NAME_MAX, "%s\n", config->product_name);
}

static DEVICE_ATTR(product, S_IRUGO | S_IWUSR, projector_product_show,
						    NULL);

static ssize_t projector_car_model_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, CAR_MODEL_NAME_MAX, "%s\n", config->car_model);
}

static DEVICE_ATTR(car_model, S_IRUGO | S_IWUSR, projector_car_model_show,
						    NULL);

static ssize_t projector_width_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.width);
}

static DEVICE_ATTR(width, S_IRUGO | S_IWUSR, projector_width_show,
						    NULL);

static ssize_t projector_height_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.height);
}

static DEVICE_ATTR(height, S_IRUGO | S_IWUSR, projector_height_show,
						    NULL);

static ssize_t projector_rotation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", (config->client_info.display_conf & CLIENT_INFO_SERVER_ROTATE_USED));
}

static DEVICE_ATTR(rotation, S_IRUGO | S_IWUSR, projector_rotation_show,
						    NULL);

static ssize_t projector_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->version);
}

static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, projector_version_show,
						    NULL);


static struct device_attribute *projector_function_attributes[] = {
	&dev_attr_product,
	&dev_attr_car_model,
	&dev_attr_width,
	&dev_attr_height,
	&dev_attr_rotation,
	&dev_attr_version,
	NULL
};


struct android_usb_function projector_function = {
	.name		= "projector",
	.init		= projector_function_init,
	.cleanup	= projector_function_cleanup,
	.bind_config	= projector_function_bind_config,
	.ctrlrequest = projector_function_ctrlrequest,
	.attributes = projector_function_attributes
};
#endif

#ifdef CONFIG_USB_ANDROID_USBNET
static int usbnet_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return usbnet_setup();
}

static int usbnet_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return usbnet_bind_config(c);
}

static int usbnet_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return usbnet_ctrlrequest(cdev, c);
}

struct android_usb_function usbnet_function = {
	.name		= "usbnet",
	.init		= usbnet_function_init,
	.bind_config	= usbnet_function_bind_config,
	.ctrlrequest	= usbnet_function_ctrlrequest,
};
#endif

static struct android_usb_function *supported_functions[] = {
#if 1
#ifdef CONFIG_USB_ANDROID_RNDIS
	&rndis_function,
#endif
	&accessory_function,
#ifdef CONFIG_USB_ANDROID_MTP
	&mtp_function,
	&ptp_function,
#endif
	&mass_storage_function,
	&adb_function,
#ifdef CONFIG_USB_ANDROID_ECM
	&ecm_function,
#endif
	&diag_function,

	&modem_function,
#ifdef CONFIG_USB_ANDROID_MDM9K_MODEM
	&modem_mdm_function,
#endif
	&serial_function,
#ifdef CONFIG_USB_ANDROID_PROJECTOR
	&projector_function,
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	&acm_function,
#endif
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	&diag_mdm_function,
#endif
#if defined(CONFIG_USB_ANDROID_RMNET_SMD)
	&rmnet_smd_function,
#elif defined(CONFIG_USB_ANDROID_RMNET_SDIO)
	&rmnet_sdio_function,
#elif defined(CONFIG_USB_ANDROID_RMNET_SMD_SDIO)
	&rmnet_smd_sdio_function,
#elif defined(CONFIG_USB_ANDROID_RMNET_BAM)
	&rmnet_function,
#endif
#if 0
	&ccid_function,
#endif
#ifdef CONFIG_USB_ANDROID_USBNET
	&usbnet_function,
#endif
	NULL
#endif
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 0;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (device_create_file(f->dev, &dev_attr_on) < 0) {
			pr_err("%s: Failed to create dev file %s", __func__,
							f->dev_name);
			goto err_create;
		}


		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
		pr_info("%s %s init\n", __func__, f->name);
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

	static int
android_bind_enabled_functions(struct android_dev *dev,
		struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		pr_info("%s bind name: %s\n", __func__, f->name);
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			pr_info("%s: %s enabled\n", __func__, name);
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			return 0;
		}
	}
	pr_info("%s: %s failed\n", __func__, name);
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += snprintf(buff, PAGE_SIZE, "%s,", f->name);
	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

/* TODO: replace by switch function and enable function */
static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	int err;

	pr_info("%s: buff: %s\n", __func__, buff);
	return size;

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	int enabled = 0;

	sscanf(buff, "%d", &enabled);

	if (enabled)
		htc_usb_enable_function("adb", 1);

	pr_info("%s, buff: %s\n", __func__, buff);

	/* temporaily return immediately to prevent framework change usb behavior
	 */
	return size;

	if (enabled && !dev->enabled) {
		/* update values in composite driver's copy of device descriptor */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		if (usb_add_config(cdev, &android_config_driver,
							android_bind_config))
			return size;

		usb_gadget_connect(cdev->gadget);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		usb_gadget_disconnect(cdev->gadget);
		usb_remove_config(cdev, &android_config_driver);
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
        if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	int value;					       		\
	pr_info("%s: %s\n", __func__, buf);				\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	pr_info("%s: %s\n", __func__, buf);				\
	if (size >= sizeof(buffer)) return -EINVAL;			\
	if (sscanf(buf, "%255s", buffer) == 1) {			\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show, functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	NULL
};

#include "htc_attr.c"

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_platform_data *pdata = _android_dev->pdata;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

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

	dev->products = pdata->products;
	dev->num_products = pdata->num_products;
	dev->in_house_functions = pdata->functions;
	dev->num_functions = pdata->num_functions;
	dev->match = pdata->match;

	/* default String */
	if (pdata->product_name)
		strlcpy(product_string, pdata->product_name,
			sizeof(product_string) - 1);
	if (pdata->manufacturer_name)
		strlcpy(manufacturer_string, pdata->manufacturer_name,
			sizeof(manufacturer_string) - 1);
	if (pdata->serial_number)
		strlcpy(serial_string, pdata->serial_number,
			sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

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


	cdev->sw_connect2pc.name = "usb_connect2pc";
	ret = switch_dev_register(&cdev->sw_connect2pc);
	if (ret < 0)
		pr_err("switch_dev_register fail:usb_connect2pc\n");


	schedule_delayed_work(&dev->init_work, HZ);
	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	switch_dev_unregister(&cdev->sw_connect2pc);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
};

#ifdef CONFIG_USB_ANDROID_USBNET
static struct work_struct reenumeration_work;
static void do_reenumeration_work(struct work_struct *w)
{
	struct android_dev *dev = _android_dev;
	int err, funcs, product_id;

	if (dev->enabled != true) {
		pr_info("%s: USB driver is not initialize\n", __func__);
		return;
	}

	mutex_lock(&function_bind_sem);

	funcs = htc_usb_get_func_combine_value();
	usb_gadget_disconnect(dev->cdev->gadget);
	usb_remove_config(dev->cdev, &android_config_driver);

	INIT_LIST_HEAD(&dev->enabled_functions);

	if (funcs & (1 << USB_FUNCTION_ADB)) {
		err = android_enable_function(dev, "adb");
		if (err)
			pr_err("android_usb: Cannot enable '%s'", "adb");
	}

	err = android_enable_function(dev, "usbnet");
	if (err)
		pr_err("android_usb: Cannot enable '%s'", "usbnet");

	product_id = get_product_id(dev, &dev->enabled_functions);

	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	dev->cdev->desc.idProduct = device_desc.idProduct;
	printk(KERN_INFO "%s:product_id = 0x%04x\n", __func__, product_id);

	usb_add_config(dev->cdev, &android_config_driver, android_bind_config);
	mdelay(100);
	usb_gadget_connect(dev->cdev->gadget);
	dev->enabled = true;

	mutex_unlock(&function_bind_sem);
}

static int handle_mode_switch(u16 switchIndex, struct usb_composite_dev *cdev)
{
	switch (switchIndex) {
	case 0x1F:
		/* Enable the USBNet function and disable all others but adb */
		printk(KERN_INFO "[USBNET] %s: 0x%02x\n", __func__, switchIndex);
		cdev->desc.bDeviceClass = USB_CLASS_COMM;
		break;
		/* Add other switch functions */
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int android_switch_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *c)
{
	int value = -EOPNOTSUPP;
	u16 wIndex = le16_to_cpu(c->wIndex);
	u16 wValue = le16_to_cpu(c->wValue);
	u16 wLength = le16_to_cpu(c->wLength);
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct usb_request *req = cdev->req;
	/* struct android_dev *dev = _android_dev; */

	switch (c->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_VENDOR:
		/* If the request is a mode switch , handle it */
		if ((c->bRequest == 1) && (wValue == 0) && (wLength == 0)) {
			value = handle_mode_switch(wIndex, cdev);
			if (value != 0)
				return value;

			req->zero = 0;
			req->length = value;
			if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				printk(KERN_ERR "ep0 in queue failed\n");

			/* force reenumeration */
			schedule_work(&reenumeration_work);
		}
		break;
		/* Add Other type of requests here */
	default:
		break;
	}
	return value;
}
#endif

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

#ifdef CONFIG_USB_ANDROID_USBNET
	value = android_switch_setup(gadget, c);
	if (value >= 0)
		return value;
#endif

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

#ifdef CONFIG_USB_ANDROID_PROJECTOR
	/*
	 * The projector needs to handle control requests before it's enabled.
	 */
	if (value < 0)
		value = projector_ctrlrequest(cdev, c);
#endif

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	}
	else if (c->bRequest == USB_REQ_SET_CONFIGURATION && cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);

	/*android_switch_function is not called if removing usb cable. Without it, connect2pc may be blocked by is_mtp_enabled*/
	is_mtp_enabled = false;
}

static void android_mute_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);

	/*changes USB_STATE only for MTP*/
	if (is_mtp_enabled) {
		spin_lock_irqsave(&cdev->lock, flags);
		dev->connected = 0;
		schedule_work(&dev->work);
		spin_unlock_irqrestore(&cdev->lock, flags);
	}
}

static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	dev->pdata = pdata;

	init_mfg_serialno();
	if (sysfs_create_group(&pdev->dev.kobj, &android_usb_attr_group))
		pr_err("%s: fail to create sysfs\n", __func__);

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb"},
};

static void android_usb_init_work(struct work_struct *data)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_platform_data *pdata = dev->pdata;
	struct usb_composite_dev *cdev = dev->cdev;
	int ret = 0;
	__u16 product_id;

	/* initial ums+adb by default */
	ret = android_enable_function(dev, "mass_storage");
	if (ret)
		pr_err("android_usb: Cannot enable '%s'", "mass_storage");

#if 0
	ret = android_enable_function(dev, "adb");
	if (ret)
		pr_err("android_usb: Cannot enable '%s'", "adb");
#endif

	/* initial function depends on radio flag */
	if (pdata->diag_init) {
		ret = android_enable_function(dev, "diag");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "diag");
	}
	if (pdata->modem_init) {
		ret = android_enable_function(dev, "modem");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "modem");
#if defined(CONFIG_USB_ANDROID_MDM9K_MODEM)
		ret = android_enable_function(dev, "modem_mdm");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "modem_mdm");
#endif
	}

#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	if (pdata->diag_init) {
		ret = android_enable_function(dev, "diag_mdm");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "diag_mdm");
	}
#endif

	if (pdata->rmnet_init) {
		ret = android_enable_function(dev, "rmnet");
		if (ret)
			pr_err("android_usb: Cannot enable '%s'", "rmnet");
	}


	cdev->desc.idVendor = __constant_cpu_to_le16(pdata->vendor_id),
	product_id = get_product_id(dev, &dev->enabled_functions);

	if (dev->match)
		product_id = dev->match(product_id, intrsharing);

	cdev->desc.idProduct = __constant_cpu_to_le16(product_id),
	cdev->desc.bcdDevice = device_desc.bcdDevice;
	cdev->desc.bDeviceClass = device_desc.bDeviceClass;
	cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
	cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;

	device_desc.idVendor = cdev->desc.idVendor;
	device_desc.idProduct = cdev->desc.idProduct;

	ret = usb_add_config(cdev, &android_config_driver,
				android_bind_config);

	usb_gadget_connect(cdev->gadget);
	dev->enabled = true;
	pr_info("%s: ret: %d\n", __func__, ret);
}

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	connect2pc = false;
	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
				__func__);
		class_destroy(android_class);
		return -ENOMEM;
	}
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	INIT_DELAYED_WORK(&dev->init_work, android_usb_init_work);
#ifdef CONFIG_USB_ANDROID_USBNET
	INIT_WORK(&reenumeration_work, do_reenumeration_work);
#endif

	ret = android_create_device(dev);
	if (ret) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;
	}

	_android_dev = dev;


	wake_lock_init(&android_usb_idle_wake_lock, WAKE_LOCK_IDLE,
					"android_usb_idle");

#ifdef CONFIG_PERFLOCK
	perf_lock_init(&android_usb_perf_lock, PERF_LOCK_HIGHEST, "android_usb");
#endif

	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;
	composite_driver.mute_disconnect = android_mute_disconnect;

	ret = platform_driver_probe(&android_platform_driver, android_probe);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "platform driver\n", __func__);
		goto err_probe;
	}
	ret = usb_composite_probe(&android_usb_driver, android_bind);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "composite driver\n", __func__);
		platform_driver_unregister(&android_platform_driver);
		goto err_probe;
	}
	return ret;
err_probe:
	android_destroy_device(dev);
err_dev:
	kfree(dev);
	class_destroy(android_class);
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{

	wake_lock_destroy(&android_usb_idle_wake_lock);

	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
