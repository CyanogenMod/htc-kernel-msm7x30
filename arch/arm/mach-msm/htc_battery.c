/* arch/arm/mach-msm/htc_battery.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <mach/board_htc.h>
#include <mach/msm_fb.h> /* Jay, to register display notifier */
#include <mach/htc_battery.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>
#include <linux/tps65200.h>
#ifdef CONFIG_HTC_BATTCHG_SMEM
#include "smd_private.h"
#endif

#if defined(CONFIG_TROUT_BATTCHG_DOCK)
#include <mach/htc_one_wire.h>
#endif
#ifdef CONFIG_BATTERY_DS2784
#include <linux/ds2784_battery.h>
#elif defined(CONFIG_BATTERY_DS2746)
#include <linux/ds2746_battery.h>
#endif

static struct wake_lock vbus_wake_lock;

enum {
	HTC_BATT_DEBUG_M2A_RPC = 1U << 0,
	HTC_BATT_DEBUG_A2M_RPC = 1U << 1,
	HTC_BATT_DEBUG_UEVT = 1U << 2,
	HTC_BATT_DEBUG_USER_QUERY = 1U << 3,
	HTC_BATT_DEBUG_USB_NOTIFY = 1U << 4,
	HTC_BATT_DEBUG_SMEM = 1U << 5,
};
static int htc_batt_debug_mask = HTC_BATT_DEBUG_M2A_RPC | HTC_BATT_DEBUG_A2M_RPC
	| HTC_BATT_DEBUG_UEVT | HTC_BATT_DEBUG_USB_NOTIFY | HTC_BATT_DEBUG_SMEM;
module_param_named(debug_mask, htc_batt_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define BATT_LOG(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_INFO "[BATT] " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_ERR(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "[BATT] err:" x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

/* rpc related */
#define APP_BATT_PDEV_NAME		"rs30100001:00000000"
#define APP_BATT_PROG			0x30100001
#define APP_BATT_VER			MSM_RPC_VERS(0,0)
#define HTC_PROCEDURE_BATTERY_NULL	0
#define HTC_PROCEDURE_GET_BATT_LEVEL	1
#define HTC_PROCEDURE_GET_BATT_INFO	2
#define HTC_PROCEDURE_GET_CABLE_STATUS	3
#define HTC_PROCEDURE_SET_BATT_DELTA	4
#define HTC_PROCEDURE_CHARGER_SWITCH    6
#define HTC_PROCEDURE_SET_FULL_LEVEL	7
#define HTC_PROCEDURE_GET_USB_ACCESSORY_ADC_LEVEL	10

const char *charger_tags[] = {"none", "USB", "AC"};

struct htc_battery_info {
	int device_id;
	int present;
	unsigned long update_time;

	/* lock to protect the battery info */
	struct mutex lock;

	/* lock held while calling the arm9 to query the battery info */
	struct mutex rpc_lock;
	struct battery_info_reply rep;
	int (*func_show_batt_attr)(struct device_attribute *attr, char *buf);
	int gpio_mbat_in;
	int gpio_usb_id;
	int gpio_mchg_en_n;
	int gpio_iset;
	int guage_driver;
	int m2a_cable_detect;
	int charger;
	int gpio_adp_9v;
	int (*func_battery_charging_ctrl)(enum batt_ctl_t ctl);
};

static struct msm_rpc_endpoint *endpoint;

static struct htc_battery_info htc_batt_info;

/* Remove cache mechanism to prevent cable status not sync. */
static unsigned int cache_time;

static int htc_battery_initial = 0;
static int htc_full_level_flag = 0;

static enum power_supply_property htc_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property htc_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

/* HTC dedicated attributes */
static ssize_t htc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static int htc_power_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static struct power_supply htc_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = htc_battery_properties,
		.num_properties = ARRAY_SIZE(htc_battery_properties),
		.get_property = htc_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
};

static int update_batt_info(void);
static void usb_status_notifier_func(int online);
//static int g_usb_online;
static struct t_usb_status_notifier usb_status_notifier = {
	.name = "htc_battery",
	.func = usb_status_notifier_func,
};

/* Move cable detection/notification to standard PMIC RPC. */
static BLOCKING_NOTIFIER_HEAD(cable_status_notifier_list);
int register_notifier_cable_status(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&cable_status_notifier_list, nb);
}

int unregister_notifier_cable_status(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&cable_status_notifier_list, nb);
}

/* -------------------------------------------------------------------------- */
/* For sleep charging screen. */
static int zcharge_enabled;
static int htc_is_zcharge_enabled(void)
{
	return zcharge_enabled;
}
static int __init enable_zcharge_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	zcharge_enabled = cal;
	return 1;
}
__setup("enable_zcharge=", enable_zcharge_setup);

static int htc_is_cable_in(void)
{
	if (!htc_batt_info.update_time) {
		BATT_ERR("%s: battery driver hasn't been initialized yet.", __func__);
		return -EINVAL;
	}
	return (htc_batt_info.rep.charging_source != CHARGER_BATTERY) ? 1 : 0;
}

/**
 * htc_power_policy - check if it obeys our policy
 * return 0 for no errors, to indicate it follows policy.
 * non zero otherwise.
 **/
static int __htc_power_policy(void)
{
	if (!zcharge_enabled)
		return 0;

	if (htc_is_cable_in())
		return 1;

	return 0;
}

/*
 * Jay, 7/1/09'
 */
static int htc_power_policy(struct notifier_block *nfb,
		unsigned long action, void *ignored)
{
	int rc;
	switch (action) {
	case NOTIFY_POWER:
		pr_info("[BATT] %s: enter.\n", __func__);
		rc = __htc_power_policy();
		if (rc)
			return NOTIFY_STOP;
		else
			return NOTIFY_OK;
	}
	return NOTIFY_DONE; /* we did not care other action here */
}

unsigned int batt_get_status(enum power_supply_property psp)
{
	union power_supply_propval val;

	if (update_batt_info())
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&htc_batt_info.lock);
		val.intval = htc_batt_info.rep.level;
		mutex_unlock(&htc_batt_info.lock);
		/* prevent shutdown before battery driver ready. */
		if (htc_batt_info.device_id == 0)
			val.intval = 55; /* 55 == ?? */
		break;
	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&htc_batt_info.lock);
		val.intval = htc_batt_info.rep.batt_temp;
		mutex_unlock(&htc_batt_info.lock);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&htc_batt_info.lock);
		val.intval = htc_batt_info.rep.batt_vol;
		mutex_unlock(&htc_batt_info.lock);
		break;
	default:
		return -EINVAL;
	}

	return val.intval;
}

#if defined(CONFIG_DEBUG_FS)
static int htc_battery_set_charging(enum batt_ctl_t ctl);
static int batt_debug_set(void *data, u64 val)
{
	return htc_battery_set_charging((enum batt_ctl_t) val);
}

static int batt_debug_get(void *data, u64 *val)
{
	return -ENOSYS;
}

DEFINE_SIMPLE_ATTRIBUTE(batt_debug_fops, batt_debug_get, batt_debug_set, "%llu\n");
static int __init batt_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("htc_battery", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("charger_state", 0644, dent, NULL, &batt_debug_fops);

	return 0;
}

device_initcall(batt_debug_init);
#endif

static int init_batt_gpio(void)
{

	if (htc_batt_info.gpio_mbat_in > 0 &&
		gpio_request(htc_batt_info.gpio_mbat_in, "batt_detect") < 0)
		goto gpio_failed;
	if (htc_batt_info.gpio_mchg_en_n > 0 &&
		gpio_request(htc_batt_info.gpio_mchg_en_n, "charger_en") < 0)
		goto gpio_failed;
	if (htc_batt_info.gpio_iset > 0 &&
		gpio_request(htc_batt_info.gpio_iset, "charge_current") < 0)
		goto gpio_failed;
	if (htc_batt_info.gpio_adp_9v > 0 &&
		gpio_request(htc_batt_info.gpio_adp_9v, "super_charge_current") < 0)
		goto gpio_failed;

	return 0;

gpio_failed:
	return -EINVAL;

}

/*
 *	battery_charging_ctrl - battery charing control.
 * 	@ctl:			battery control command
 *
 */
int battery_charging_ctrl(enum batt_ctl_t ctl)
{
	int result = 0;

	switch (ctl) {
	case DISABLE:
		BATT_LOG("charger OFF");
		/* 0 for enable; 1 disable */
		result = gpio_direction_output(htc_batt_info.gpio_mchg_en_n, 1);
		break;
	case ENABLE_SLOW_CHG:
		BATT_LOG("charger ON (SLOW)");
		result = gpio_direction_output(htc_batt_info.gpio_iset, 0);
		result = gpio_direction_output(htc_batt_info.gpio_mchg_en_n, 0);
		if (htc_batt_info.gpio_adp_9v > 0)
			result = gpio_direction_output(htc_batt_info.gpio_adp_9v, 0);
		break;
	case ENABLE_FAST_CHG:
		BATT_LOG("charger ON (FAST)");
		result = gpio_direction_output(htc_batt_info.gpio_iset, 1);
		result = gpio_direction_output(htc_batt_info.gpio_mchg_en_n, 0);
		if (htc_batt_info.gpio_adp_9v > 0)
			result = gpio_direction_output(htc_batt_info.gpio_adp_9v, 0);
		break;
	case ENABLE_SUPER_CHG:
		BATT_LOG("charger ON (SUPER)");
		result = gpio_direction_output(htc_batt_info.gpio_iset, 1);
		result = gpio_direction_output(htc_batt_info.gpio_mchg_en_n, 0);
		if (htc_batt_info.gpio_adp_9v > 0)
			result = gpio_direction_output(htc_batt_info.gpio_adp_9v, 1);
		break;
	default:
		BATT_ERR("%s: Not supported battery ctr called.!", __func__);
		result = -EINVAL;
		break;
	}

	return result;
}

static int htc_battery_set_charging(enum batt_ctl_t ctl)
{
	int rc;

	if ((rc = battery_charging_ctrl(ctl)) < 0)
		goto result;

	if (!htc_battery_initial) {
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
	} else {
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
		mutex_unlock(&htc_batt_info.lock);
	}
result:
	return rc;
}

static int htc_battery_status_update(u32 curr_level)
{
	int notify;
	if (!htc_battery_initial)
		return 0;

	mutex_lock(&htc_batt_info.lock);
	notify = (htc_batt_info.rep.level != curr_level);
	htc_batt_info.rep.level = curr_level;
	mutex_unlock(&htc_batt_info.lock);
#if 0
	if (notify) {
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
			BATT_LOG("power_supply_changed: battery");
	}
#else
	/* we don't check level here for charging over temp RPC call */
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
	if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
		BATT_LOG("power_supply_changed: battery");
#endif
	return 0;
}

static void update_wake_lock(int status)
{
	if (status == CHARGER_USB) {
		wake_lock(&vbus_wake_lock);
	} else if (__htc_power_policy()) {
		/* Lock suspend for DOPOD charging animation */
		wake_lock(&vbus_wake_lock);
	} else {
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ * 5);
	}
}

#ifdef CONFIG_HTC_BATTCHG_SMEM
static int htc_set_smem_cable_type(u32 cable_type);
#else
static int htc_set_smem_cable_type(u32 cable_type) { return -1; }
#endif
#if 1 //JH //this is for packet filter (notify port list while USB in/out)
int update_port_list_charging_state(int enable);
#endif 
static int htc_cable_status_update(int status)
{
	int rc = 0;
//	unsigned last_source;

	if (!htc_battery_initial)
		return 0;

	if (status < CHARGER_BATTERY || status > CHARGER_SUPER_AC) {
		BATT_ERR("%s: Not supported cable status received!", __func__);
		return -EINVAL;
	}

	mutex_lock(&htc_batt_info.lock);
#if 1
	pr_info("[BATT] %s: %d -> %d\n", __func__, htc_batt_info.rep.charging_source, status);
	if (status == htc_batt_info.rep.charging_source) {
	/* When cable overvoltage(5V => 7V) A9 will report the same source, so only sent the uevent */
		if (status == CHARGER_USB) {
		power_supply_changed(&htc_power_supplies[CHARGER_USB]);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
		BATT_LOG("(htc_cable_status_update)power_supply_changed: OverVoltage");
	}
		mutex_unlock(&htc_batt_info.lock);
		return 0;
	}

	/* TODO: replace charging_source to vbus_present */
	htc_batt_info.rep.charging_source = status;
	/* ARM9 should know the status it notifies,
	 * keep this code for old projects. */
	/* htc_set_smem_cable_type(status); */

	update_wake_lock(status);
	/*ARM9 report CHARGER_AC while plug in htc_adaptor which is identify by usbid*/
	/*don't need to notify usb driver*/
	if ((htc_batt_info.guage_driver == GUAGE_MODEM) && (status == CHARGER_AC)) {
		htc_set_smem_cable_type(CHARGER_AC);
		power_supply_changed(&htc_power_supplies[CHARGER_AC]);
	} else
		msm_hsusb_set_vbus_state(!!htc_batt_info.rep.charging_source);

	/* TODO: use power_supply_change to notify battery drivers. */
	if (htc_batt_info.guage_driver == GUAGE_DS2784 ||
		htc_batt_info.guage_driver == GUAGE_DS2746)
		blocking_notifier_call_chain(&cable_status_notifier_list,
			status, NULL);

	if (status == CHARGER_BATTERY) {
		htc_set_smem_cable_type(CHARGER_BATTERY);
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
		BATT_LOG("(htc_cable_status_update)power_supply_changed: battery");
	}

#else
	/* A9 reports USB charging when helf AC cable in and China AC charger. */
	/* notify userspace USB charging first,
	and then usb driver will notify AC while D+/D- Line short. */
	/* China AC detection:
	 * Write SMEM as USB first, and update SMEM to AC
	 * if receives AC notification */
	last_source = htc_batt_info.rep.charging_source;
	if (status == CHARGER_USB && g_usb_online == 0) {
		htc_set_smem_cable_type(CHARGER_USB);
		htc_batt_info.rep.charging_source = CHARGER_USB;
	} else {
		htc_set_smem_cable_type(status);
		htc_batt_info.rep.charging_source  = status;
		/* usb driver will not notify usb offline. */
		if (status == CHARGER_BATTERY && g_usb_online != 0)
			g_usb_online = 0;
	}

	msm_hsusb_set_vbus_state(status == CHARGER_USB);
	if (htc_batt_info.guage_driver == GUAGE_DS2784 ||
		htc_batt_info.guage_driver == GUAGE_DS2746)
		blocking_notifier_call_chain(&cable_status_notifier_list,
			htc_batt_info.rep.charging_source, NULL);

	if (htc_batt_info.rep.charging_source != last_source) {
#if 1 //JH //this is for packet filter (notify port list while USB in/out)
		update_port_list_charging_state(!(htc_batt_info.rep.charging_source == CHARGER_BATTERY));
#endif
		/* Lock suspend only when USB in for ADB or other USB functions. */
		if (htc_batt_info.rep.charging_source == CHARGER_USB) {
			wake_lock(&vbus_wake_lock);
		} else if (__htc_power_policy()) {
			/* Lock suspend for DOPOD charging animation */
			wake_lock(&vbus_wake_lock);
		} else {
			if (htc_batt_info.rep.charging_source == CHARGER_AC
				&& last_source == CHARGER_USB)
				BATT_ERR("%s: USB->AC\n", __func__);
			/* give userspace some time to see the uevent and update
			 * LED state or whatnot...
			 */
			wake_lock_timeout(&vbus_wake_lock, HZ * 5);
		}
		if (htc_batt_info.rep.charging_source == CHARGER_BATTERY || last_source == CHARGER_BATTERY)
			power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		if (htc_batt_info.rep.charging_source == CHARGER_USB || last_source == CHARGER_USB)
			power_supply_changed(&htc_power_supplies[CHARGER_USB]);
		if (htc_batt_info.rep.charging_source == CHARGER_AC || last_source == CHARGER_AC)
			power_supply_changed(&htc_power_supplies[CHARGER_AC]);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
			BATT_LOG("power_supply_changed: %s -> %s",
				charger_tags[last_source], charger_tags[htc_batt_info.rep.charging_source]);
	}
#endif
	mutex_unlock(&htc_batt_info.lock);

	return rc;
}

#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
int htc_get_usb_accessory_adc_level(uint32_t *buffer)
{
	struct rpc_request_hdr req;

	struct htc_get_usb_adc_value_rep {
		struct rpc_reply_hdr hdr;
		uint32_t adc_value;
	} rep;

	int rc;
	printk(KERN_INFO "[BATT] %s\n", __func__);

	if (buffer == NULL) {
		printk(KERN_INFO "[BATT] %s: buffer null\n", __func__);
		return -EINVAL;
	}

	rc = msm_rpc_call_reply(endpoint, HTC_PROCEDURE_GET_USB_ACCESSORY_ADC_LEVEL,
				&req, sizeof(req),
				&rep, sizeof(rep),
				5 * HZ);
	if (rc < 0) {
		printk(KERN_INFO "[BATT] %s: msm_rpc_call_reply fail\n", __func__);
		return rc;
	}
	*buffer 		= be32_to_cpu(rep.adc_value);

	printk(KERN_INFO "[BATT] %s: adc = %d\n", __func__, *buffer);
	return 0;
}
EXPORT_SYMBOL(htc_get_usb_accessory_adc_level);
#endif

/* A9 reports USB charging when helf AC cable in and China AC charger. */
/* notify userspace USB charging first,
and then usb driver will notify AC while D+/D- Line short. */
static void usb_status_notifier_func(int online)
{
#if 1
	pr_info("[BATT] online=%d",online);
	/* TODO: replace charging_source to usb_status */
	htc_batt_info.rep.charging_source = online;
	htc_set_smem_cable_type(htc_batt_info.rep.charging_source);

	/* TODO: use power_supply_change to notify battery drivers. */
	if (htc_batt_info.guage_driver == GUAGE_DS2784 || htc_batt_info.guage_driver == GUAGE_DS2746)
		blocking_notifier_call_chain(&cable_status_notifier_list,
			htc_batt_info.rep.charging_source, NULL);

	if (htc_battery_initial) {
		power_supply_changed(&htc_power_supplies[CHARGER_AC]);
		power_supply_changed(&htc_power_supplies[CHARGER_USB]);
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
	} else {
		pr_err("\n\n[BATT] ### htc_battery_code is not inited yet! ###\n\n");
	}
	update_wake_lock(htc_batt_info.rep.charging_source);
#else
	mutex_lock(&htc_batt_info.lock);
	if (htc_batt_debug_mask & HTC_BATT_DEBUG_USB_NOTIFY)
		BATT_LOG("%s: online=%d, g_usb_online=%d", __func__, online, g_usb_online);
	if (g_usb_online != online) {
		g_usb_online = online;
		if (online == CHARGER_AC && htc_batt_info.rep.charging_source == CHARGER_USB) {
			mutex_unlock(&htc_batt_info.lock);
			htc_cable_status_update(CHARGER_AC);
			mutex_lock(&htc_batt_info.lock);
		}
	}
	mutex_unlock(&htc_batt_info.lock);
#endif
}

static int htc_get_batt_info(struct battery_info_reply *buffer)
{
	struct rpc_request_hdr req;

	struct htc_get_batt_info_rep {
		struct rpc_reply_hdr hdr;
		struct battery_info_reply info;
	} rep;

	int rc;

	if (buffer == NULL)
		return -EINVAL;

	rc = msm_rpc_call_reply(endpoint, HTC_PROCEDURE_GET_BATT_INFO,
				&req, sizeof(req),
				&rep, sizeof(rep),
				5 * HZ);
	if ( rc < 0 )
		return rc;

	mutex_lock(&htc_batt_info.lock);
	buffer->batt_id 		= be32_to_cpu(rep.info.batt_id);
	buffer->batt_vol 		= be32_to_cpu(rep.info.batt_vol);
	buffer->batt_temp 		= be32_to_cpu(rep.info.batt_temp);
	buffer->batt_current 		= be32_to_cpu(rep.info.batt_current);
	buffer->level 			= be32_to_cpu(rep.info.level);
	/* Move the rules of charging_source to cable_status_update. */
	/* buffer->charging_source 	= be32_to_cpu(rep.info.charging_source); */
	buffer->charging_enabled 	= be32_to_cpu(rep.info.charging_enabled);
	buffer->full_bat 		= be32_to_cpu(rep.info.full_bat);
	/* Over_vchg only update in SMEM from A9 */
	/* buffer->over_vchg 		= be32_to_cpu(rep.info.over_vchg); */
	mutex_unlock(&htc_batt_info.lock);

	if (htc_batt_debug_mask & HTC_BATT_DEBUG_A2M_RPC)
		BATT_LOG("A2M_RPC: get_batt_info: batt_id=%d, batt_vol=%d, batt_temp=%d, "
			"batt_current=%d, level=%d, charging_source=%d, "
			"charging_enabled=%d, full_bat=%d, over_vchg=%d",
			buffer->batt_id, buffer->batt_vol, buffer->batt_temp,
			buffer->batt_current, buffer->level, buffer->charging_source,
			buffer->charging_enabled, buffer->full_bat, buffer->over_vchg);

	return 0;
}

#ifdef CONFIG_HTC_BATTCHG_SMEM
struct htc_batt_info_full {
	u32 batt_id;
	u32 batt_vol;
	u32 batt_vol_last;
	u32 batt_temp;
	s32 batt_current;
	s32 batt_current_last;
	u32 batt_discharge_current;

	u32 VREF_2;
	u32 VREF;
	u32 ADC4096_VREF;

	u32 Rtemp;
	s32  Temp;
	s32  Temp_last;

	u32 pd_M;
	u32 MBAT_pd;
	s32 I_MBAT;

	u32 pd_temp;
	u32 percent_last;
	u32 percent_update;
	u32 dis_percent;

	u32 vbus;
	u32 usbid;
	u32 charging_source;

	u32 MBAT_IN;
	u32 full_bat;

	u32 eval_current;
	u32 eval_current_last;
	u32 charging_enabled;

	u32 timeout;
	u32 fullcharge;
	u32 level;
	u32 delta;

	u32 chg_time;
	s32 level_change;
	u32 sleep_timer_count;
	u32 OT_led_on;
	u32 overloading_charge;

	u32 a2m_cable_type;
	u32 vchg;	// VCHG => 0: Not, 1: In
	u32 over_vchg;	/*over voltage charger detection, 0:VCHG normal(below 6V) 1:VCHG over(upper 6V)*/
	u32 reserve4;
	u32 hv_enabled; /* Battery => 0: RV, 1: HV */
};

/* SMEM_BATT_INFO is allocated by A9 after first A2M RPC is sent. */
static struct htc_batt_info_full *smem_batt_info;

static int htc_get_batt_info_smem(struct battery_info_reply *buffer)
{
	if (!smem_batt_info) {
		smem_batt_info = smem_alloc(SMEM_BATT_INFO,
			sizeof(struct htc_batt_info_full));
		if (!smem_batt_info) {
			BATT_ERR("battery SMEM allocate fail, "
				"use RPC instead of");
			return htc_get_batt_info(buffer);
		}
	}

	if (!buffer)
		return -EINVAL;

	mutex_lock(&htc_batt_info.lock);
	buffer->batt_id = smem_batt_info->batt_id;
	buffer->batt_vol = smem_batt_info->batt_vol;
	buffer->batt_temp = smem_batt_info->Temp;
	buffer->batt_current = smem_batt_info->batt_current;
	buffer->eval_current = smem_batt_info->eval_current;
	/* Fix issue that recharging percent drop to 99%. */
	/* The level in SMEM is for A9 internal use,
	 * always use value reported by M2A level update RPC. */
#if 0
	buffer->level 	= smem_batt_info->percent_update;
#endif
	/* Move the rules of charging_source to cable_status_update. */
	/* buffer->charging_source 	= be32_to_cpu(smem_batt_info->charging_source); */
	buffer->charging_enabled = smem_batt_info->charging_enabled;
	buffer->full_bat = smem_batt_info->full_bat;
	buffer->over_vchg = smem_batt_info->over_vchg;
	mutex_unlock(&htc_batt_info.lock);

	if (htc_batt_debug_mask & HTC_BATT_DEBUG_SMEM)
		BATT_LOG("SMEM_BATT: get_batt_info: batt_id=%d, batt_vol=%d, batt_temp=%d, "
			"batt_current=%d, eval_current=%d, level=%d, charging_source=%d, "
			"charging_enabled=%d, full_bat=%d, over_vchg=%d hv_enabled=%d",
			buffer->batt_id, buffer->batt_vol, buffer->batt_temp,
			buffer->batt_current, buffer->eval_current, buffer->level, buffer->charging_source,
			buffer->charging_enabled, buffer->full_bat, buffer->over_vchg,
			smem_batt_info->hv_enabled);

	return 0;
}

static ssize_t htc_battery_show_smem(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int len = 0;

	if (!smem_batt_info) {
		smem_batt_info = smem_alloc(SMEM_BATT_INFO,
			sizeof(struct htc_batt_info_full));
		if (!smem_batt_info) {
			BATT_ERR("Show SMEM: allocate fail");
			return 0;
		}
	}

	if (!strcmp(attr->attr.name, "smem_raw")) {
		len = sizeof(struct htc_batt_info_full);
		memcpy(buf, smem_batt_info, len);
	} else if (!strcmp(attr->attr.name, "smem_text")) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"batt_id: %d\n"
			"batt_vol: %d\n"
			"batt_vol_last: %d\n"
			"batt_temp: %d\n"
			"batt_current: %d\n"
			"batt_current_last: %d\n"
			"batt_discharge_current: %d\n"
			"VREF_2: %d\n"
			"VREF: %d\n"
			"ADC4096_VREF: %d\n"
			"Rtemp: %d\n"
			"Temp: %d\n"
			"Temp_last: %d\n"
			"pd_M: %d\n"
			"MBAT_pd: %d\n"
			"I_MBAT: %d\n"
			"pd_temp: %d\n"
			"percent_last: %d\n"
			"percent_update: %d\n"
			"dis_percent: %d\n"
			"vbus: %d\n"
			"usbid: %d\n"
			"charging_source: %d\n"
			"MBAT_IN: %d\n"
			"full_bat: %d\n"
			"eval_current: %d\n"
			"eval_current_last: %d\n"
			"charging_enabled: %d\n"
			"timeout: %d\n"
			"fullcharge: %d\n"
			"level: %d\n"
			"delta: %d\n"
			"chg_time: %d\n"
			"level_change: %d\n"
			"sleep_timer_count: %d\n"
			"OT_led_on: %d\n"
			"overloading_charge: %d\n"
			"a2m_cable_type: %d\n"
			"vchg: %d\n"
			"over_vchg: %d\n"
			"hv_enabled: %d\n",
			smem_batt_info->batt_id,
			smem_batt_info->batt_vol,
			smem_batt_info->batt_vol_last,
			smem_batt_info->batt_temp,
			smem_batt_info->batt_current,
			smem_batt_info->batt_current_last,
			smem_batt_info->batt_discharge_current,
			smem_batt_info->VREF_2,
			smem_batt_info->VREF,
			smem_batt_info->ADC4096_VREF,
			smem_batt_info->Rtemp,
			smem_batt_info->Temp,
			smem_batt_info->Temp_last,
			smem_batt_info->pd_M,
			smem_batt_info->MBAT_pd,
			smem_batt_info->I_MBAT,
			smem_batt_info->pd_temp,
			smem_batt_info->percent_last,
			smem_batt_info->percent_update,
			smem_batt_info->dis_percent,
			smem_batt_info->vbus,
			smem_batt_info->usbid,
			smem_batt_info->charging_source,
			smem_batt_info->MBAT_IN,
			smem_batt_info->full_bat,
			smem_batt_info->eval_current,
			smem_batt_info->eval_current_last,
			smem_batt_info->charging_enabled,
			smem_batt_info->timeout,
			smem_batt_info->fullcharge,
			smem_batt_info->level,
			smem_batt_info->delta,
			smem_batt_info->chg_time,
			smem_batt_info->level_change,
			smem_batt_info->sleep_timer_count,
			smem_batt_info->OT_led_on,
			smem_batt_info->overloading_charge,
			smem_batt_info->a2m_cable_type,
			smem_batt_info->vchg,
			smem_batt_info->over_vchg,
			smem_batt_info->hv_enabled);
	}

	return len;
}

static int htc_set_smem_cable_type(u32 cable_type)
{
	if (!smem_batt_info) {
		smem_batt_info = smem_alloc(SMEM_BATT_INFO,
				sizeof(struct htc_batt_info_full));
		if (!smem_batt_info) {
			BATT_ERR("Update SMEM: allocate fail");
			return -EINVAL;
		}
	}

	smem_batt_info->a2m_cable_type = cable_type;
	BATT_LOG("Update SMEM: cable type %d", cable_type);

	return 0;
}
#endif
static ssize_t htc_battery_show_batt_attr(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	switch (htc_batt_info.guage_driver) {
	case GUAGE_MODEM:
#ifdef CONFIG_HTC_BATTCHG_SMEM
		return htc_battery_show_smem(dev, attr, buf);
#endif
		break;
	case GUAGE_DS2784:
	case GUAGE_DS2746:
		return htc_batt_info.func_show_batt_attr(attr, buf);
		break;
	}
	return 0;
}

/* -------------------------------------------------------------------------- */
static int htc_power_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	enum charger_type_t charger;

	mutex_lock(&htc_batt_info.lock);

	charger = htc_batt_info.rep.charging_source;
	/* ARM9 decides charging_enabled value by battery id */
	/* if (htc_batt_info.rep.batt_id == 255)
		charger = CHARGER_BATTERY; */

	mutex_unlock(&htc_batt_info.lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (charger == CHARGER_AC || charger == CHARGER_SUPER_AC)
				val->intval = 1;
			else
				val->intval = 0;
			if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
				BATT_LOG("%s: %s: online=%d", __func__, psy->name, val->intval);
		} else if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
			if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
				BATT_LOG("%s: %s: online=%d", __func__, psy->name, val->intval);
		} else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Once charge full, set this flag */
static int htc_charge_full = 0;

static int htc_battery_get_charging_status(void)
{
	u32 level;
	enum charger_type_t charger;
	int ret;

	mutex_lock(&htc_batt_info.lock);

	charger = htc_batt_info.rep.charging_source;

	/* ARM9 decides charging_enabled value by battery id */
	if (htc_batt_info.rep.batt_id == 255)
		charger = CHARGER_UNKNOWN;

	switch (charger) {
	case CHARGER_BATTERY:
		htc_charge_full = 0;
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
	case CHARGER_SUPER_AC:
#if !defined(CONFIG_BATTERY_DS2746)
		if ((htc_charge_full) && (htc_batt_info.rep.full_level == 100)) {
			htc_batt_info.rep.level = 100;
		}
#endif
		level = htc_batt_info.rep.level;
		if (level == 100){
			htc_charge_full = 1;}
		else
			htc_charge_full = 0;

		if (htc_charge_full)
			ret = POWER_SUPPLY_STATUS_FULL;
		else if (htc_batt_info.rep.charging_enabled != 0)
			ret = POWER_SUPPLY_STATUS_CHARGING;
		else
			ret = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	mutex_unlock(&htc_batt_info.lock);
	return ret;
}

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = htc_battery_get_charging_status();
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: status=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (machine_is_paradise()) {
			if (htc_batt_info.rep.batt_temp >= 500 ||
				htc_batt_info.rep.batt_temp <= 0)
				val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		} else if (machine_is_spade() || machine_is_flyer()) {
			if (htc_batt_info.rep.batt_temp >= 450 ||
				htc_batt_info.rep.batt_temp <= 0)
				val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			if (htc_batt_info.rep.batt_temp >= 480 ||
				htc_batt_info.rep.batt_temp <= 0)
				val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		}

		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: health=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = htc_batt_info.present;
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: present=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: technology=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&htc_batt_info.lock);
		val->intval = htc_batt_info.rep.level;
		/* prevent shutdown before battery driver ready. */
		if (htc_batt_info.device_id == 0)
			val->intval = 55; /* 55 == ?? */
		mutex_unlock(&htc_batt_info.lock);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: capacity=%d", __func__, psy->name, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define HTC_BATTERY_ATTR(_name)							\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },	\
	.show = htc_battery_show_property,					\
	.store = NULL,								\
}

static struct device_attribute htc_battery_attrs[] = {
	HTC_BATTERY_ATTR(batt_id),
	HTC_BATTERY_ATTR(batt_vol),
	HTC_BATTERY_ATTR(batt_temp),
	HTC_BATTERY_ATTR(batt_current),
	HTC_BATTERY_ATTR(charging_source),
	HTC_BATTERY_ATTR(charging_enabled),
	HTC_BATTERY_ATTR(full_bat),
	HTC_BATTERY_ATTR(over_vchg),
/*[FIXME]__ATTR(batt_attr_raw, S_IRUGO, htc_battery_show_batt_attr, NULL),*/
#ifdef CONFIG_HTC_BATTCHG_SMEM
	__ATTR(smem_raw, S_IRUGO, htc_battery_show_smem, NULL),
	__ATTR(smem_text, S_IRUGO, htc_battery_show_smem, NULL),
#endif
	__ATTR(batt_attr_text, S_IRUGO, htc_battery_show_batt_attr, NULL),
};

enum {
	BATT_ID = 0,
	BATT_VOL,
	BATT_TEMP,
	BATT_CURRENT,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
	OVER_VCHG,
};

static int htc_rpc_set_delta(unsigned delta)
{
	int ret = 0;
	struct set_batt_delta_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	req.data = cpu_to_be32(delta);
	ret = msm_rpc_call(endpoint, HTC_PROCEDURE_SET_BATT_DELTA,
			    &req, sizeof(req), 5 * HZ);
	if (ret < 0)
		BATT_ERR("%s: msm_rpc_call failed!", __func__);

	return ret;
}

static int htc_rpc_charger_switch(unsigned enable)
{
	int ret = 0;
	struct charger_switch_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	BATT_LOG("%s: switch charger to mode: %u", __func__, enable);
	if (enable == ENABLE_LIMIT_CHARGER)
		ret = tps_set_charger_ctrl(ENABLE_LIMITED_CHG);
	else if (enable == DISABLE_LIMIT_CHARGER)
		ret = tps_set_charger_ctrl(CLEAR_LIMITED_CHG);
	else {
		req.data = cpu_to_be32(enable);
		ret = msm_rpc_call(endpoint, HTC_PROCEDURE_CHARGER_SWITCH,
				&req, sizeof(req), 5 * HZ);
		if (ret < 0)
			BATT_ERR("%s: msm_rpc_call failed!", __func__);
	}

	return ret;
}

static int htc_rpc_set_full_level(unsigned level)
{
	int ret = 0;
	struct set_batt_full_level_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	req.data = cpu_to_be32(level);
	ret = msm_rpc_call(endpoint, HTC_PROCEDURE_SET_FULL_LEVEL,
			    &req, sizeof(req), 5 * HZ);
	if (ret < 0)
		BATT_ERR("%s: msm_rpc_call failed!", __func__);

	return ret;
}

static ssize_t htc_battery_set_delta(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int rc;
	unsigned long delta = 0;

	delta = simple_strtoul(buf, NULL, 10);

	if (delta > 100)
		return -EINVAL;

	mutex_lock(&htc_batt_info.rpc_lock);
	rc = htc_rpc_set_delta(delta);
	mutex_unlock(&htc_batt_info.rpc_lock);
	if (rc < 0)
		return rc;
	return count;
}

/*
*	For PA and QA test
*	0x10-> fake temp to 250
*	0x11->TBD if needed
*	0x12->TBD if needed
*	....
*/
static ssize_t htc_battery_debug_flag(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long debug_flag;
	debug_flag = simple_strtoul(buf, NULL, 10);

	if (debug_flag > 100 || debug_flag == 0)
		return -EINVAL;

	mutex_lock(&htc_batt_info.lock);
	blocking_notifier_call_chain(&cable_status_notifier_list,
		debug_flag, 0);
	mutex_unlock(&htc_batt_info.lock);
	return 0;

}

int htc_battery_charger_disable()
{
	int rc;

	mutex_lock(&htc_batt_info.rpc_lock);
	rc = htc_rpc_charger_switch(0);
	mutex_unlock(&htc_batt_info.rpc_lock);
	return rc;
}

static ssize_t htc_battery_charger_switch(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int rc;
	unsigned long enable = 0;

	enable = simple_strtoul(buf, NULL, 10);

	if (enable >= END_CHARGER)
		return -EINVAL;

	mutex_lock(&htc_batt_info.rpc_lock);
	rc = htc_rpc_charger_switch(enable);
	mutex_unlock(&htc_batt_info.rpc_lock);
	if (rc < 0)
		return rc;
	return count;
}


static ssize_t htc_battery_set_full_level(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int rc = 0;
	unsigned long percent = 100;

	percent = simple_strtoul(buf, NULL, 10);

	if (percent > 100 || percent == 0)
		return -EINVAL;

	switch (htc_batt_info.guage_driver) {
	case GUAGE_MODEM:
		mutex_lock(&htc_batt_info.rpc_lock);
		htc_batt_info.rep.full_level = percent;
		rc = htc_rpc_set_full_level(percent);
		mutex_unlock(&htc_batt_info.rpc_lock);
		break;
	case GUAGE_DS2784:
	case GUAGE_DS2746:
		if (htc_full_level_flag == 0) {
	mutex_lock(&htc_batt_info.lock);
	htc_full_level_flag = 1;
	htc_batt_info.rep.full_level = percent;
	blocking_notifier_call_chain(&cable_status_notifier_list,
		0xff, (void *) &htc_batt_info.rep.full_level);
	mutex_unlock(&htc_batt_info.lock);
			}
	rc = 0;
		break;
		}
	if (rc < 0)
		return rc;
	return rc;
}

static struct device_attribute htc_set_delta_attrs[] = {
	__ATTR(delta, S_IWUSR | S_IWGRP, NULL, htc_battery_set_delta),
	__ATTR(full_level, S_IWUSR | S_IWGRP, NULL, htc_battery_set_full_level),
	__ATTR(batt_debug_flag,S_IWUSR | S_IWGRP, NULL, htc_battery_debug_flag),
	__ATTR(charger_control, S_IWUSR | S_IWGRP, NULL, htc_battery_charger_switch),
};

static int htc_battery_create_attrs(struct device * dev)
{
	int i = 0, j = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
		rc = device_create_file(dev, &htc_battery_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}

	for (j = 0; j < ARRAY_SIZE(htc_set_delta_attrs); j++) {
		rc = device_create_file(dev, &htc_set_delta_attrs[j]);
		if (rc)
			goto htc_delta_attrs_failed;
	}

	goto succeed;

htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htc_battery_attrs[i]);
htc_delta_attrs_failed:
	while (j--)
		device_remove_file(dev, &htc_set_delta_attrs[j]);
succeed:
	return rc;
}

static int update_batt_info(void)
{
	int ret = 0;

	/* FIXME */
	switch (htc_batt_info.guage_driver) {
	case GUAGE_MODEM:
#ifdef CONFIG_HTC_BATTCHG_SMEM
		if (htc_get_batt_info_smem(&htc_batt_info.rep) < 0) {
			BATT_ERR("%s: smem read failed!!!", __func__);
			ret = -1;
		}
#else
		if (htc_get_batt_info(&htc_batt_info.rep) < 0) {
			BATT_ERR("%s: rpc failed!!!", __func__);
			ret = -1;
		}
#endif
		break;
#ifdef CONFIG_BATTERY_DS2784
	case GUAGE_DS2784:
		if (ds2784_get_battery_info(&htc_batt_info.rep)) {
			BATT_ERR("%s: ds2784 read failed!!!", __func__);
			ret = -1;
		}
		break;
#elif defined(CONFIG_BATTERY_DS2746)
	case GUAGE_DS2746:
		if (ds2746_get_battery_info(&htc_batt_info.rep)) {
			BATT_ERR("%s: ds2746 read failed!!!", __func__);
			ret = -1;
		}
		break;
#endif

	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t htc_battery_show_property(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - htc_battery_attrs;

	/* rpc lock is used to prevent two threads from calling
	 * into the get info rpc at the same time
	 */

	mutex_lock(&htc_batt_info.rpc_lock);
	/* check cache time to decide if we need to update */
	if (htc_batt_info.update_time &&
            time_before(jiffies, htc_batt_info.update_time +
			msecs_to_jiffies(cache_time))) {
		BATT_LOG("%s: use cached values", __func__);
                goto dont_need_update;
	}

	if (!update_batt_info())
		htc_batt_info.update_time = jiffies;

dont_need_update:
	mutex_unlock(&htc_batt_info.rpc_lock);

	mutex_lock(&htc_batt_info.lock);
	switch (off) {
	case BATT_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_id);
		break;
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_vol);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_temp);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_current);
		break;
	case CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_source);
		break;
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_enabled);
		break;
	case FULL_BAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.full_bat);
		break;
	case OVER_VCHG:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.over_vchg);
		break;
	default:
		i = -EINVAL;
	}
	mutex_unlock(&htc_batt_info.lock);

	if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY) {
		if (i < 0)
			BATT_LOG("%s: battery: attribute is not supported: %d", __func__, off);
		else
			BATT_LOG("%s: battery: %s=%s", __func__, attr->attr.name, buf);
	}
	return i;
}

static irqreturn_t tps65200_int_detection(int irq, void *data)
{
	struct htc_battery_tps65200_int *ip = data;

	BATT_LOG("%s: over voltage is detected.", __func__);

	disable_irq_nosync(ip->chg_int);

	ip->tps65200_reg = 0;

	schedule_delayed_work(&ip->int_work, msecs_to_jiffies(200));

	return IRQ_HANDLED;
}

static void htc_battery_tps65200_int_func(struct work_struct *work)
{
	struct htc_battery_tps65200_int *ip;
	int fault_bit;
	ip = container_of(work, struct htc_battery_tps65200_int,
			int_work.work);
#ifdef CONFIG_HTC_BATTCHG_SMEM
	if (!smem_batt_info) {
		smem_batt_info = smem_alloc(SMEM_BATT_INFO,
				sizeof(struct htc_batt_info_full));
		if (!smem_batt_info) {
			BATT_ERR("Update SMEM: allocate fail");
			return;
		}
	}
#endif
	switch (ip->tps65200_reg) {
	case CHECK_INT1:
		/* read twice. First read to trigger TPS65200 clear fault bit
		   on INT1. Second read to make sure that fault bit is cleared
		   and call off ovp function.*/
		fault_bit = tps_set_charger_ctrl(CHECK_INT1);
		BATT_LOG("INT1 value: %d", fault_bit);
		fault_bit = tps_set_charger_ctrl(CHECK_INT1);

		if (fault_bit) {
#ifdef CONFIG_HTC_BATTCHG_SMEM
			smem_batt_info->over_vchg = 1;
#else
			htc_batt_info.rep.over_vchg = 1;
#endif
			power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
			schedule_delayed_work(&ip->int_work,
					    msecs_to_jiffies(5000));
			BATT_LOG("OVER_VOLTAGE: "
				"over voltage fault bit on TPS65200 is raised:"
				" %d", fault_bit);
		} else {
#ifdef CONFIG_HTC_BATTCHG_SMEM
			smem_batt_info->over_vchg = 0;
#else
			htc_batt_info.rep.over_vchg = 0;
#endif
			cancel_delayed_work(&ip->int_work);
			enable_irq(ip->chg_int);
		}
		break;
	default:
		fault_bit = tps_set_charger_ctrl(CHECK_INT2);
		BATT_LOG("Read TPS65200 INT2 register value: %x", fault_bit);
		if (fault_bit & 0x80) {
			fault_bit = tps_set_charger_ctrl(CHECK_INT2);
			BATT_LOG("Read TPS65200 INT2 register value: %x"
				, fault_bit);
			fault_bit = tps_set_charger_ctrl(CHECK_INT2);
			BATT_LOG("Read TPS65200 INT2 register value: %x"
				, fault_bit);
			fault_bit = tps_set_charger_ctrl(CHECK_CONTROL);
#ifdef CONFIG_HTC_BATTCHG_SMEM
			smem_batt_info->reserve4 = 1;
#endif
			cancel_delayed_work(&ip->int_work);
			enable_irq(ip->chg_int);
		} else {
			fault_bit = tps_set_charger_ctrl(CHECK_INT1);
			BATT_LOG("Read TPS65200 INT1 register value: %x"
				, fault_bit);
			if (fault_bit & 0x40) {
				ip->tps65200_reg = CHECK_INT1;
				schedule_delayed_work(&ip->int_work,
						msecs_to_jiffies(200));
			}
		}
		break;
	}
}

static int htc_battery_core_probe(struct platform_device *pdev)
{
	int i, rc;

	/* init battery gpio */
	if (htc_batt_info.charger == LINEAR_CHARGER) {
	if ((rc = init_batt_gpio()) < 0) {
		BATT_ERR("%s: init battery gpio failed!", __func__);
		return rc;
	}
	}

	/* init structure data member */
	htc_batt_info.update_time 	= jiffies;
	/* A9 will shutdown the phone if battery is pluged out, so this value is always 1.
	htc_batt_info.present 		= gpio_get_value(GPIO_TROUT_MBAT_IN);
	*/
	htc_batt_info.present 		= 1;

	/* init rpc */
	endpoint = msm_rpc_connect(APP_BATT_PROG, APP_BATT_VER, 0);
	if (IS_ERR(endpoint)) {
		BATT_ERR("%s: init rpc failed! rc = %ld",
		       __func__, PTR_ERR(endpoint));
		return -EINVAL;
	}

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(htc_power_supplies); i++) {
		rc = power_supply_register(&pdev->dev, &htc_power_supplies[i]);
		if (rc)
			BATT_ERR("%s: Failed to register power supply (%d)", __func__, rc);
	}

	/* create htc detail attributes */
	htc_battery_create_attrs(htc_power_supplies[CHARGER_BATTERY].dev);

	/* After battery driver gets initialized, send rpc request to inquiry
	 * the battery status in case of we lost some info
	 */
	htc_battery_initial = 1;

	mutex_lock(&htc_batt_info.rpc_lock);
	htc_batt_info.rep.charging_source = CHARGER_BATTERY;
	if (htc_get_batt_info(&htc_batt_info.rep) < 0)
		BATT_ERR("%s: get info failed", __func__);

	if (htc_rpc_set_delta(1) < 0)
		BATT_ERR("%s: set delta failed", __func__);
	htc_batt_info.update_time = jiffies;
	mutex_unlock(&htc_batt_info.rpc_lock);

	return 0;
}

static struct platform_driver htc_battery_core_driver = {
	.probe	= htc_battery_core_probe,
	.driver	= {
		.name	= APP_BATT_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};

/* batt_mtoa server definitions */
#define BATT_MTOA_PROG				0x30100000
#define BATT_MTOA_VERS				0
#define RPC_BATT_MTOA_NULL			0
#define RPC_BATT_MTOA_SET_CHARGING_PROC		1
#define RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC 	2
#define RPC_BATT_MTOA_LEVEL_UPDATE_PROC		3

struct rpc_batt_mtoa_set_charging_args {
	int enable;
};

struct rpc_batt_mtoa_cable_status_update_args {
	int status;
};

struct rpc_dem_battery_update_args {
	uint32_t level;
};

static int handle_battery_call(struct msm_rpc_server *server,
			       struct rpc_request_hdr *req, unsigned len)
{
	switch (req->procedure) {
	case RPC_BATT_MTOA_NULL:
		return 0;

	case RPC_BATT_MTOA_SET_CHARGING_PROC: {
		struct rpc_batt_mtoa_set_charging_args *args;
		args = (struct rpc_batt_mtoa_set_charging_args *)(req + 1);
		args->enable = be32_to_cpu(args->enable);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_M2A_RPC)
			BATT_LOG("M2A_RPC: set_charging: %d", args->enable);
		if (htc_batt_info.charger == SWITCH_CHARGER_TPS65200)
			tps_set_charger_ctrl(args->enable);
		else if (htc_batt_info.charger == SWITCH_CHARGER)
			blocking_notifier_call_chain(&cable_status_notifier_list,
				args->enable, NULL);
		else {
			htc_battery_set_charging(args->enable);
		}
		return 0;
	}
	case RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC: {
		struct rpc_batt_mtoa_cable_status_update_args *args;
		args = (struct rpc_batt_mtoa_cable_status_update_args *)(req + 1);
		args->status = be32_to_cpu(args->status);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_M2A_RPC)
			BATT_LOG("M2A_RPC: cable_update: %s", charger_tags[args->status]);
#if 0
		/* FIXME: work arround for usb function, remove it after battery driver ready */
		if (machine_is_incrediblec() && args->status == CHARGER_AC)
			args->status = CHARGER_USB;
#endif
		htc_cable_status_update(args->status);
	#if defined(CONFIG_TROUT_BATTCHG_DOCK)
		dock_detect_start(args->status);
	#endif
		return 0;
	}
	case RPC_BATT_MTOA_LEVEL_UPDATE_PROC: {
		struct rpc_dem_battery_update_args *args;
		args = (struct rpc_dem_battery_update_args *)(req + 1);
		args->level = be32_to_cpu(args->level);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_M2A_RPC)
			BATT_LOG("M2A_RPC: level_update: %d", args->level);
		htc_battery_status_update(args->level);
		return 0;
	}
	default:
		BATT_ERR("%s: program 0x%08x:%d: unknown procedure %d",
		       __func__, req->prog, req->vers, req->procedure);
		return -ENODEV;
	}
}

static struct msm_rpc_server battery_server = {
	.prog = BATT_MTOA_PROG,
	.vers = BATT_MTOA_VERS,
	.rpc_call = handle_battery_call,
};

#if defined(CONFIG_BATTERY_DS2784)
static int ds2784_notifier_func(struct notifier_block *nfb,
		unsigned long action, void *param)
{
	u8 arg = 0;

	if (param)
		arg = *(u8 *)param;

	BATT_LOG("ds2784_notify: %ld %d", action, arg);
	switch (action) {
	case DS2784_CHARGING_CONTROL:
		if (htc_batt_info.charger == LINEAR_CHARGER)
			battery_charging_ctrl(arg);
//		else if(htc_batt_info.charger == SWITCH_CHARGER) 
//			set_charger_ctrl(arg);
		break;
	case DS2784_LEVEL_UPDATE:
		htc_battery_status_update(arg);
		break;
	case DS2784_BATTERY_FAULT:
	case DS2784_OVER_TEMP:
		htc_battery_status_update(htc_batt_info.rep.level);
		break;
	default:
		return NOTIFY_BAD;
	}

	return NOTIFY_OK; /* we did not care other action here */
}

static struct notifier_block ds2784_notifier = {
	.notifier_call = ds2784_notifier_func,
};

#elif defined(CONFIG_BATTERY_DS2746)
static int ds2746_notifier_func(struct notifier_block *nfb, unsigned long action, void *param)
{
	u8 arg = 0;

	if (param)
		arg = *(u8 *)param;

	BATT_LOG("ds2746_notify: %ld %d", action, arg);
	switch (action) {
	case DS2746_CHARGING_CONTROL:
		if (htc_batt_info.charger == LINEAR_CHARGER)
			htc_batt_info.func_battery_charging_ctrl(arg);
		break;
	case DS2746_LEVEL_UPDATE:
		htc_battery_status_update(arg);
		break;
	default:
		return NOTIFY_BAD;
	}

	return NOTIFY_OK; /* we did not care other action here */
}

static struct notifier_block ds2746_notifier = {
	.notifier_call = ds2746_notifier_func,
};

#endif

static int htc_battery_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct htc_battery_platform_data *pdata = pdev->dev.platform_data;

	htc_batt_info.device_id = pdev->id;
	htc_batt_info.gpio_usb_id = pdata->gpio_usb_id;
	htc_batt_info.guage_driver = pdata->guage_driver;
	htc_batt_info.m2a_cable_detect = pdata->m2a_cable_detect;
	htc_batt_info.func_show_batt_attr = pdata->func_show_batt_attr;
	htc_batt_info.charger = pdata->charger;
	htc_batt_info.rep.full_level = 100;

	if (htc_batt_info.charger == LINEAR_CHARGER) {
		htc_batt_info.gpio_mbat_in = pdata->gpio_mbat_in;
		htc_batt_info.gpio_mchg_en_n = pdata->gpio_mchg_en_n;
		htc_batt_info.gpio_iset = pdata->gpio_iset;
	}

	if (pdata->func_is_support_super_charger != NULL) {
		if (pdata->func_is_support_super_charger() == 1)
			htc_batt_info.gpio_adp_9v = pdata->gpio_adp_9v;
	}

	if (pdata->func_battery_charging_ctrl != NULL && pdata->func_battery_gpio_init != NULL) {
		htc_batt_info.func_battery_charging_ctrl = pdata->func_battery_charging_ctrl;
		pdata->func_battery_gpio_init();
	} else
		htc_batt_info.func_battery_charging_ctrl = battery_charging_ctrl;

	if (pdata->guage_driver == GUAGE_MODEM ||
		pdata->m2a_cable_detect)
		msm_rpc_create_server(&battery_server);
#ifdef CONFIG_BATTERY_DS2784
	if (pdata->guage_driver == GUAGE_DS2784)
		ds2784_register_notifier(&ds2784_notifier);
#elif defined(CONFIG_BATTERY_DS2746)
	if (pdata->guage_driver == GUAGE_DS2746)
		ds2746_register_notifier(&ds2746_notifier);
#endif

	if (pdata->int_data.chg_int) {
		BATT_LOG("init over voltage interrupt detection.");
		INIT_DELAYED_WORK(&pdata->int_data.int_work,
				htc_battery_tps65200_int_func);

		rc = request_irq(pdata->int_data.chg_int,
				tps65200_int_detection,
				IRQF_TRIGGER_LOW,
				"over_voltage_interrupt",
				&pdata->int_data);

		if (rc) {
			BATT_LOG("request irq failed");
			return rc;
		}
	}

	return 0;
}

int get_cable_status(void)
{
//	if(htc_batt_info.rep.charging_source == CHARGER_AC || htc_batt_info.rep.charging_source == CHARGER_USB)
//		htc_cable_status_update(htc_batt_info.rep.charging_source);
	return htc_batt_info.rep.charging_source;
}

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= "htc_battery",
		.owner	= THIS_MODULE,
	},
};

static struct notifier_block batt_notify = {
	.notifier_call = htc_power_policy,
};

static BLOCKING_NOTIFIER_HEAD(battery_notifier_list);
int batt_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&battery_notifier_list, nb);
}

int batt_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&battery_notifier_list, nb);
}

int batt_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&battery_notifier_list, val, v);
}

static int __init htc_battery_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	mutex_init(&htc_batt_info.lock);
	mutex_init(&htc_batt_info.rpc_lock);
	usb_register_notifier(&usb_status_notifier);
	platform_driver_register(&htc_battery_driver);
	platform_driver_register(&htc_battery_core_driver);
	batt_register_client(&batt_notify);
	/* Jay, The msm_fb need to consult htc_battery for power policy */
	display_notifier(htc_power_policy, NOTIFY_POWER);
	return 0;
}

module_init(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");
EXPORT_SYMBOL(htc_is_cable_in);
EXPORT_SYMBOL(htc_is_zcharge_enabled);
