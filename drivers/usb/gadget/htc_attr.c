/*
 * Copyright (C) 2011 HTC, Inc.
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

#include <mach/board.h>
#include <linux/gpio.h>

enum {
	USB_FUNCTION_UMS = 0,
	USB_FUNCTION_ADB = 1,
	USB_FUNCTION_RNDIS,
	USB_FUNCTION_DIAG,
	USB_FUNCTION_SERIAL,
	USB_FUNCTION_PROJECTOR,
	USB_FUNCTION_FSYNC,
	USB_FUNCTION_MTP,
	USB_FUNCTION_MODEM, /* 8 */
	USB_FUNCTION_ECM,
	USB_FUNCTION_ACM,
	USB_FUNCTION_DIAG_MDM, /* 11 */
	USB_FUNCTION_RMNET,
	USB_FUNCTION_ACCESSORY,
	USB_FUNCTION_MODEM_MDM, /* 14 */
	USB_FUNCTION_MTP36,
	USB_FUNCTION_USBNET,
	USB_FUNCTION_RNDIS_IPT = 31,
};

struct usb_string_node{
	u32 usb_function_flag;
	char *name;
};

static struct usb_string_node usb_string_array[] = {
	{
		.usb_function_flag = 1 << USB_FUNCTION_UMS,
		.name = "mass_storage",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ADB,
		.name = "adb",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_RNDIS,
		.name = "rndis",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_DIAG,
		.name = "diag",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_SERIAL,
		.name = "serial",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_PROJECTOR,
		.name = "projector",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_MODEM,
		.name = "modem",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ECM,
		.name = "cdc_ethernet",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ACM,
		.name = "acm",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_DIAG_MDM,
		.name = "diag_mdm",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_RMNET,
		.name = "rmnet",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ACCESSORY,
		.name = "accessory",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_MODEM_MDM,
		.name = "modem_mdm",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_MTP,
		.name = "mtp",
	},

};

static int use_mfg_serialno;
static char mfg_df_serialno[16];
static int intrsharing;

#define PID_RNDIS		0x0ffe
#define PID_ECM			0x0ff8
#define PID_ACM			0x0ff4
#define PID_USBNET		0x0fcd

/* for htc in-house device attribute, htc_usb_attr.c */
void android_force_reset(void)
{
	if (_android_dev && _android_dev->cdev)
		usb_composite_force_reset(_android_dev->cdev);
}

static bool isFunctionDisabled(struct android_usb_function *function)
{
	struct android_usb_function *f;
	struct list_head *list = &_android_dev->enabled_functions;

	list_for_each_entry(f, list, enabled_list) {
		if (!strcmp(function->name, f->name))
			return false;
	}
	return true;
}

static int product_has_function(struct android_usb_product *p,
		struct android_usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		if (!strncmp(name, functions[i], strlen(name)))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p,
	struct list_head *list)
{
	int count = 0;
	struct android_usb_function             *f;
	list_for_each_entry(f, list, enabled_list) {
		count++;
		if (product_has_function(p, f) == isFunctionDisabled(f))
			return 0;
	}

	if (count == p->num_functions)
		return 1;
	else
		return 0;
}

static int get_product_id(struct android_dev *dev, struct list_head *list)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p, list))
				return p->product_id;
		}
	}
	/* use default product ID */
	return dev->pdata->product_id;
}

static struct android_usb_product *get_product(struct android_dev *dev, struct list_head *list)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p, list))
				return p;
		}
	}
	return NULL;
}


static unsigned int htc_usb_get_func_combine_value(void)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	int i;
	unsigned int val = 0;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		for (i = 0; i < ARRAY_SIZE(usb_string_array); i++)
			if (!strcmp(usb_string_array[i].name, f->name)) {
				val |= usb_string_array[i].usb_function_flag;
				break;
			}
	}
	return val;
}
static DEFINE_MUTEX(function_bind_sem);

int htc_usb_enable_function(char *name, int ebl)
{
	int i;
	unsigned val;

	mutex_lock(&function_bind_sem);

	val = htc_usb_get_func_combine_value();

	for (i = 0; i < ARRAY_SIZE(usb_string_array); i++) {
		if (!strcmp(usb_string_array[i].name, name)) {
			if (ebl) {
				if (val & usb_string_array[i].usb_function_flag) {
					pr_info("%s: '%s' is already enabled\n", __func__, name);
					mutex_unlock(&function_bind_sem);
					return 0;
				}
				val |= usb_string_array[i].usb_function_flag;
			} else {
				if (!(val & usb_string_array[i].usb_function_flag)) {
					pr_info("%s: '%s' is already disabled\n", __func__, name);
					mutex_unlock(&function_bind_sem);
					return 0;
				}

				val &= ~usb_string_array[i].usb_function_flag;
			}
			break;
		}
	}
	mutex_unlock(&function_bind_sem);
	return android_switch_function(val);
}


int android_show_function(char *buf)
{
	unsigned length = 0;
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	char *ebl_str[2] = {"disable", "enable"};
	char *p;
	int i;

	for (i = 0; dev->functions[i] != NULL; i++) {

		p = ebl_str[0];
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (!strcmp(dev->functions[i]->name, f->name)) {
				p = ebl_str[1];
				break;
			}
		}

		length += sprintf(buf + length, "%s:%s\n",
				dev->functions[i]->name, p);

	}
	return length;
}


int android_switch_function(unsigned func)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	struct android_usb_product *product;
	int product_id, vendor_id;
	unsigned val;

	/* framework may try to enable adb before android_usb_init_work is done.*/
       if (dev->enabled != true) {
              pr_info("%s: USB driver is not initialize\n", __func__);
              return 0;
       }

	mutex_lock(&function_bind_sem);

	val = htc_usb_get_func_combine_value();

	pr_info("%s: %u, before %u\n", __func__, func, val);

	if (func == val) {
		pr_info("%s: SKIP due the function is the same ,%u\n"
			, __func__, func);
		mutex_unlock(&function_bind_sem);
		return 0;
	}

	usb_gadget_disconnect(dev->cdev->gadget);
	usb_remove_config(dev->cdev, &android_config_driver);

	INIT_LIST_HEAD(&dev->enabled_functions);

	is_mtp_enabled = false;
	while ((f = *functions++)) {
		if ((func & (1 << USB_FUNCTION_UMS)) &&
				!strcmp(f->name, "mass_storage"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
		else if ((func & (1 << USB_FUNCTION_ADB)) &&
				!strcmp(f->name, "adb"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
		else if ((func & (1 << USB_FUNCTION_ECM)) &&
				!strcmp(f->name, "cdc_ethernet"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
		else if ((func & (1 << USB_FUNCTION_ACM)) &&
				!strcmp(f->name, "acm"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
		else if ((func & (1 << USB_FUNCTION_RNDIS)) &&
				!strcmp(f->name, "rndis")) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			intrsharing = !((func >> USB_FUNCTION_RNDIS_IPT) & 1);
		} else if ((func & (1 << USB_FUNCTION_DIAG)) &&
				!strcmp(f->name, "diag")) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
#ifdef CONFIG_USB_ANDROID_MDM9K_DIAG
			func |= 1 << USB_FUNCTION_DIAG_MDM;
#endif
		} else if ((func & (1 << USB_FUNCTION_MODEM)) &&
				!strcmp(f->name, "modem")) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
#ifdef CONFIG_USB_ANDROID_MDM9K_MODEM
			func |= 1 << USB_FUNCTION_MODEM_MDM;
#endif
		} else if ((func & (1 << USB_FUNCTION_SERIAL)) &&
				!strcmp(f->name, "serial"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
		else if ((func & (1 << USB_FUNCTION_MTP)) &&
				!strcmp(f->name, "mtp")) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			is_mtp_enabled = true;
		}
		else if ((func & (1 << USB_FUNCTION_ACCESSORY)) &&
				!strcmp(f->name, "accessory"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
		else if ((func & (1 << USB_FUNCTION_PROJECTOR)) &&
				!strcmp(f->name, "projector"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
#ifdef CONFIG_USB_ANDROID_MDM9K_DIAG
		else if ((func & (1 << USB_FUNCTION_DIAG_MDM)) &&
				!strcmp(f->name, "diag_mdm")) {
			if (func & (1 << USB_FUNCTION_DIAG))
				list_add_tail(&f->enabled_list, &dev->enabled_functions);
			else
				func &= ~(1 << USB_FUNCTION_DIAG_MDM);
		}
#endif
		else if ((func & (1 << USB_FUNCTION_RMNET)) &&
				!strcmp(f->name, "rmnet"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
#ifdef CONFIG_USB_ANDROID_MDM9K_MODEM
		else if ((func & (1 << USB_FUNCTION_MODEM_MDM)) &&
				!strcmp(f->name, "modem_mdm")) {
			if (func & (1 << USB_FUNCTION_MODEM))
				list_add_tail(&f->enabled_list, &dev->enabled_functions);
			else
				func &= ~(1 << USB_FUNCTION_MODEM_MDM);
		}
#endif
#ifdef CONFIG_USB_ANDROID_USBNET
		else if ((func & (1 << USB_FUNCTION_USBNET)) &&
				!strcmp(f->name, "usbnet"))
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
#endif
	}

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		pr_debug("# %s\n", f->name);

	product = get_product(dev, &dev->enabled_functions);

	if (product) {
		vendor_id = product->vendor_id ? product->vendor_id : dev->pdata->vendor_id;
		product_id = product->product_id;
	} else {
		vendor_id = dev->pdata->vendor_id;
		product_id =  dev->pdata->product_id;
	}

	/* We need to specify the COMM class in the device descriptor
	 * if we are using RNDIS.
	 */
	if (product_id == PID_RNDIS || product_id == PID_ECM
		|| product_id == PID_ACM || product_id == PID_USBNET)
		dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
	else
		dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;

	if (dev->match)
		product_id = dev->match(product_id, intrsharing);

	pr_info("%s: vendor_id=0x%x, product_id=0x%x\n",
			__func__, vendor_id, product_id);

	device_desc.idVendor = __constant_cpu_to_le16(vendor_id);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);

	dev->cdev->desc.idVendor = device_desc.idVendor;
	dev->cdev->desc.idProduct = device_desc.idProduct;

	device_desc.bDeviceClass = dev->cdev->desc.bDeviceClass;

	usb_add_config(dev->cdev, &android_config_driver, android_bind_config);

	/* reset usb controller/phy for USB stability */
	if(dev->pdata && dev->pdata->req_reset_during_switch_func)
		usb_gadget_request_reset(dev->cdev->gadget);

	mdelay(100);
	usb_gadget_connect(dev->cdev->gadget);
	dev->enabled = true;

	mutex_unlock(&function_bind_sem);
	return 0;
}

void android_set_serialno(char *serialno)
{
	strings_dev[STRING_SERIAL_IDX].s = serialno;
}

void android_switch_adb_ums(void)
{
	android_switch_function((1 << USB_FUNCTION_ADB) |
				(1 << USB_FUNCTION_UMS));
}

void android_switch_htc_mode(void)
{
	android_switch_function((1 << USB_FUNCTION_ADB) |
				(1 << USB_FUNCTION_PROJECTOR) |
				(1 << USB_FUNCTION_SERIAL) |
				(1 << USB_FUNCTION_UMS));
}


void init_mfg_serialno(void)
{
	char *serialno = "000000000000";

	use_mfg_serialno = (board_mfg_mode() == 1) ? 1 : 0;
	strncpy(mfg_df_serialno, serialno, strlen(serialno));
}

static ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d",
			(usb_get_connect_type() == CONNECT_TYPE_USB)?1:0);
	return length;
}

static ssize_t show_usb_function_switch(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return android_show_function(buf);
}

static ssize_t store_usb_function_switch(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned u;
	ssize_t  ret;

	ret = strict_strtoul(buf, 10, (unsigned long *)&u);
	if (ret < 0) {
		USB_ERR("%s: %d\n", __func__, ret);
		return 0;
	}

	ret = android_switch_function(u);

	if (ret == 0)
		return count;
	else
		return 0;
}

static ssize_t show_USB_ID_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_platform_data *pdata = dev->platform_data;
	int value = 1;
	unsigned length;
	printk(KERN_INFO "[USB] id pin: %d\n", pdata->usb_id_pin_gpio);

	if (pdata->usb_id_pin_gpio != 0) {
		value = gpio_get_value(pdata->usb_id_pin_gpio);
		printk(KERN_INFO"[USB] id pin status %d\n", value);
	}

	length = sprintf(buf, "%d", value);
	return length;
}

static ssize_t show_usb_serial_number(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct android_usb_platform_data *pdata = dev->platform_data;

	length = sprintf(buf, "%s", pdata->serial_number);
	return length;
}

static ssize_t store_usb_serial_number(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct android_usb_platform_data *pdata = dev->platform_data;
	char *serialno = "000000000000";

	if (buf[0] == '0' || buf[0] == '1') {
		memset(mfg_df_serialno, 0x0, sizeof(mfg_df_serialno));
		if (buf[0] == '0') {
			strncpy(mfg_df_serialno, serialno, strlen(serialno));
			use_mfg_serialno = 1;
			android_set_serialno(mfg_df_serialno);
		} else {
			strncpy(mfg_df_serialno, pdata->serial_number,
					strlen(pdata->serial_number));
			use_mfg_serialno = 0;
			android_set_serialno(pdata->serial_number);
		}
		/* reset_device */
		android_force_reset();
	}

	return count;
}

static ssize_t show_dummy_usb_serial_number(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct android_usb_platform_data *pdata = dev->platform_data;

	if (use_mfg_serialno)
		length = sprintf(buf, "%s", mfg_df_serialno); /* dummy */
	else
		length = sprintf(buf, "%s", pdata->serial_number); /* Real */
	return length;
}

static ssize_t store_dummy_usb_serial_number(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int data_buff_size = (sizeof(mfg_df_serialno) > strlen(buf))?
		strlen(buf):sizeof(mfg_df_serialno);
	int loop_i;

	/* avoid overflow, mfg_df_serialno[16] always is 0x0 */
	if (data_buff_size == 16)
		data_buff_size--;

	for (loop_i = 0; loop_i < data_buff_size; loop_i++)     {
		if (buf[loop_i] >= 0x30 && buf[loop_i] <= 0x39) /* 0-9 */
			continue;
		else if (buf[loop_i] >= 0x41 && buf[loop_i] <= 0x5A) /* A-Z */
			continue;
		if (buf[loop_i] == 0x0A) /* Line Feed */
			continue;
		else {
			printk(KERN_INFO "%s(): get invaild char (0x%2.2X)\n",
					__func__, buf[loop_i]);
			return -EINVAL;
		}
	}

	use_mfg_serialno = 1;
	memset(mfg_df_serialno, 0x0, sizeof(mfg_df_serialno));
	strncpy(mfg_df_serialno, buf, data_buff_size);
	android_set_serialno(mfg_df_serialno);
	/*device_reset */
	android_force_reset();

	return count;
}

static ssize_t
show_usb_car_kit_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length;
	int value = 0;
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
#include <mach/cable_detect.h>
	value = (cable_get_accessory_type() == DOCK_STATE_UNDOCKED) ? 0 : 1;
	printk(KERN_INFO "USB_car_kit_enable %d\n", cable_get_accessory_type());
#else
	value = 0;
	printk(KERN_INFO "USB_car_kit_enable: CABLE_DETECT_ACCESSORY was not defined\n");
#endif

	length = sprintf(buf, "%d", value);
	return length;
}

static ssize_t show_usb_phy_setting(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return otg_show_usb_phy_setting(buf);
}
static ssize_t store_usb_phy_setting(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	return otg_store_usb_phy_setting(buf, count);
}

#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
void msm_otg_set_id_state(int id);
static ssize_t store_usb_host_mode(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	unsigned u, enable;
	ssize_t  ret;

	ret = strict_strtoul(buf, 10, (unsigned long *)&u);
	if (ret < 0) {
		USB_ERR("%s: %d\n", __func__, ret);
		return 0;
	}

	enable = u ? 1 : 0;
	msm_otg_set_id_state(!enable);

	USB_INFO("%s USB host\n", enable ? "Enable" : "Disable");

	return count;
}
static DEVICE_ATTR(host_mode, 0220,
		NULL, store_usb_host_mode);
#endif

static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL);
static DEVICE_ATTR(usb_function_switch, 0664,
		show_usb_function_switch, store_usb_function_switch);
static DEVICE_ATTR(USB_ID_status, 0444, show_USB_ID_status, NULL);
static DEVICE_ATTR(usb_serial_number, 0644,
		show_usb_serial_number, store_usb_serial_number);
static DEVICE_ATTR(dummy_usb_serial_number, 0644,
		show_dummy_usb_serial_number, store_dummy_usb_serial_number);
static DEVICE_ATTR(usb_car_kit_enable, 0444, show_usb_car_kit_enable, NULL);
static DEVICE_ATTR(usb_phy_setting, 0664,
		show_usb_phy_setting, store_usb_phy_setting);

static struct attribute *android_htc_usb_attributes[] = {
	&dev_attr_usb_cable_connect.attr,
	&dev_attr_usb_function_switch.attr,
	&dev_attr_USB_ID_status.attr, /* for MFG */
	&dev_attr_usb_serial_number.attr, /* for MFG */
	&dev_attr_dummy_usb_serial_number.attr, /* for MFG */
	&dev_attr_usb_car_kit_enable.attr,
	&dev_attr_usb_phy_setting.attr,
#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
	&dev_attr_host_mode.attr,
#endif
	NULL
};

static const struct attribute_group android_usb_attr_group = {
	.attrs = android_htc_usb_attributes,
};

