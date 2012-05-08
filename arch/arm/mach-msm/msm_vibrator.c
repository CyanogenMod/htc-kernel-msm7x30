/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include "pmic.h"
#include "timed_output.h"
#include <linux/debug_by_vibrator.h>

#include <mach/msm_rpcrouter.h>
#define VIB_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[VIB]" fmt, ##__VA_ARGS__)
#define VIB_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[VIB][ERR]" fmt, ##__VA_ARGS__)

#define PM_LIBPROG      0x30000061
#ifdef  CONFIG_RPC_VER_60001
#define PM_LIBVERS	0x60001
#else
#define PM_LIBVERS	0x30001
#endif
#define VIB_MAX_LEVEL_mV	3100
#define VIB_MIN_LEVEL_mV	1200
#define PMIC_VIBRATOR_LEVEL (3000)
#define HTC_PROCEDURE_SET_VIB_ON_OFF	22
static struct work_struct vibrator_work;
static int vibe_state;
static spinlock_t vibe_lock;
static struct hrtimer vibe_timer;
static int pmic_vibrator_level;


#ifdef CONFIG_PM8XXX_RPC_VIBRATOR
static void set_pmic_vibrator(int on)
{
	int rc;

	rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	if (rc) {
		pr_err("%s: Vibrator set mode failed", __func__);
		return;
	}

	if (on)
		rc = pmic_vib_mot_set_volt(pmic_vibrator_level);
	else
		rc = pmic_vib_mot_set_volt(0);

	if (rc)
		pr_err("%s: Vibrator set voltage level failed", __func__);
}
#else

static void set_pmic_vibrator(int on)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;
	int rc;


	if (!vib_endpoint) {
#ifdef CONFIG_ARCH_MSM7X30
		vib_endpoint = msm_rpc_connect_compatible(PM_LIBPROG, PM_LIBVERS, 0);
#else
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
#endif
		if (IS_ERR(vib_endpoint)) {
			VIB_ERR_LOG("init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}
	if (on)
		req.data = cpu_to_be32(pmic_vibrator_level);
	else
		req.data = cpu_to_be32(0);
	rc = msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);

	if (rc < 0)
		VIB_ERR_LOG("msm_rpc_call failed (%d)!\n", rc);
	else if (on)
		pr_info("[ATS][set_vibration][successful]\n");

}
#endif
static void update_vibrator(struct work_struct *work)
{
	set_pmic_vibrator(vibe_state);
}


static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long	flags;

retry:
	spin_lock_irqsave(&vibe_lock, flags);
	if (hrtimer_try_to_cancel(&vibe_timer) < 0) {
		spin_unlock_irqrestore(&vibe_lock, flags);
		cpu_relax();
		goto retry;
	}

	VIB_INFO_LOG("vibrator_enable, %s(parent:%s): vibrates %d msec\n",
				current->comm, current->parent->comm, value);

	if (value == 0)
		vibe_state = 0;
	else {
		value = (value > 15000 ? 15000 : value);
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vibe_lock, flags);
	schedule_work(&vibrator_work);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	VIB_INFO_LOG("%s\n", __func__);
	vibe_state = 0;
	schedule_work(&vibrator_work);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

#if defined(CONFIG_DEBUG_BY_VIBRATOR)	 //HTC_CSP_START
#define DEBUG_VIBRATOR_TIME	(3000)
/**
* debug_by_vibrator -debug interface.
* @mode: debug mode.
*	ERR_MODE mode: Common error,vibrate 3 seconds, with the log: [VIB]: Kernel ERROR!!Root cause module is XXX.
*	CRASH_MODE mode: Crash mode or fatal error, be careful to use!vibrate always, with the log: [VIB]:FATAL ERROR!!Root cause is XXX.
*@name: the device name to use and to print in log.
*
* Using the interface, below steps shouled be followed.
*	1. Include the file: #include<linux/debug_by_vibrator.h>
*	2. Call the interface: debug_by_vibrator(mode,your_module_name);
*	3. Return values: when 0,function called correctly; when -1,mode number error.
*/
int debug_by_vibrator(int mode, const char *name)
{
	int ret = 0;
	unsigned long flags;

	VIB_INFO_LOG("debug_by_vibrator used by %s, %s(parent:%s): vibrates in mode %d\n", name,
			current->comm, current->parent->comm, mode);

	if(mode == DISABLE){
		hrtimer_cancel(&vibe_timer);
		spin_lock_irqsave(&vibe_lock, flags);
		vibe_state = 0;
		spin_unlock_irqrestore(&vibe_lock, flags);
		schedule_work(&vibrator_work);
		ret = 1;
	}else if(mode == ERR_MODE){
		hrtimer_cancel(&vibe_timer);
		spin_lock_irqsave(&vibe_lock, flags);
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			      ktime_set(DEBUG_VIBRATOR_TIME / 1000, 0),
			      HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&vibe_lock, flags);
		schedule_work(&vibrator_work);
		VIB_INFO_LOG(": Kernel ERROR!!Root cause module is %s \n\n",name);
		ret = 0;
	}
	else if(mode== CRASH_MODE){
		hrtimer_cancel(&vibe_timer);
		spin_lock_irqsave(&vibe_lock, flags);
		vibe_state = 1;
		spin_unlock_irqrestore(&vibe_lock, flags);
		schedule_work(&vibrator_work);
		VIB_INFO_LOG(": FATAL ERROR!!Root cause module is %s \n\n",name);
		ret = 0;
	}
	else{
		VIB_INFO_LOG(": Using a incrrect mode in DEBUG_BY_VIBRATOR interface!\n");
		ret = -1;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(debug_by_vibrator);

int get_vibrator_enabled(void)
{
	return vibe_state;
}
EXPORT_SYMBOL_GPL(get_vibrator_enabled);
#endif	//HTC_CSP_END

#if defined(CONFIG_DEBUG_BY_VIBRATOR)	//HTC_CSP_START
static ssize_t debug_by_vibrator_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int mode = 0;
	char name[0];

	sscanf(buf,"%d %s",&mode,name);

	switch (mode){
	case 0:
		debug_by_vibrator(DISABLE,name);
		break;
	case 1:
		debug_by_vibrator(ERR_MODE,name);
		break;
	case 2:
		debug_by_vibrator(CRASH_MODE,name);
		break;
	default:
		printk(KERN_DEBUG"[VIB] Wrong useage!\n");
		return -EINVAL;
	}
	printk(KERN_DEBUG"[VIB]Calling the interface debug_by_vibrator \n");

	return size;
}

static ssize_t debug_by_vibrator_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int vib_enabled = 0;
	int ret = 0;
	vib_enabled = get_vibrator_enabled();

	ret = sprintf(buf,"%d",vib_enabled);
	return ret;
}

static DEVICE_ATTR(debug_by_vibrator, S_IRUGO | S_IWUSR, debug_by_vibrator_show, debug_by_vibrator_store);
#endif	//HTC_CSP_END

static ssize_t voltage_level_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", pmic_vibrator_level);
}

static ssize_t voltage_level_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	if (value < VIB_MIN_LEVEL_mV || value > VIB_MAX_LEVEL_mV)
		value = VIB_MAX_LEVEL_mV;
	pmic_vibrator_level = value ;
	return size;
}

static DEVICE_ATTR(voltage_level, S_IRUGO | S_IWUSR, voltage_level_show, voltage_level_store);

void __init msm_init_pmic_vibrator(int level)
{
	int rc;
	INIT_WORK(&vibrator_work, update_vibrator);
	spin_lock_init(&vibe_lock);
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;
	pmic_vibrator_level = level;
	timed_output_dev_register(&pmic_vibrator);
	rc = device_create_file(pmic_vibrator.dev, &dev_attr_voltage_level);
	if (rc < 0)
		VIB_ERR_LOG("%s, create voltage_level fail\n", __func__);
#if defined(CONFIG_DEBUG_BY_VIBRATOR)	//HTC_CSP_START

	rc = device_create_file(pmic_vibrator.dev, &dev_attr_debug_by_vibrator);
	if (rc < 0) {
		VIB_ERR_LOG("%s, create debug sysfs fail\n", __func__);
/*		goto err_create_debug_flag; */
	}
#endif	//HTC_CSP_END
	VIB_INFO_LOG("%s, init pmic vibrator!",__func__);
	return;
/*err_create_debug_flag:
	device_remove_attrs(pmic_vibrator.dev);*/
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

