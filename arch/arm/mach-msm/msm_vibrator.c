/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
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
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_rpc_version.h>

#define VIB_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[VIB]" fmt, ##__VA_ARGS__)
#define VIB_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[VIB][ERR]" fmt, ##__VA_ARGS__)

#define PM_LIBPROG	  0x30000061

static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;
static int pmic_vibrator_level;

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
		VIB_ERR_LOG("msm_rpc_call failed!\n");
	else if (on)
		pr_info("[ATS][set_vibration][successful]\n");
}

static void update_vibrator(struct work_struct *work)
{
	set_pmic_vibrator(vibe_state);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long	flags;

	spin_lock_irqsave(&vibe_lock, flags);
	hrtimer_cancel(&vibe_timer);

	VIB_INFO_LOG(" %s(parent:%s): vibrates %d msec\n",
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

static void set_vibrator_level(struct timed_output_dev *dev, int level)
{
	pmic_vibrator_level = level;
}

static int get_vibrator_level(struct timed_output_dev *dev)
{
	return pmic_vibrator_level;
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
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
	.set_level = set_vibrator_level,
	.get_level = get_vibrator_level,
};

void __init msm_init_pmic_vibrator(int level)
{
	INIT_WORK(&vibrator_work, update_vibrator);

	spin_lock_init(&vibe_lock);
	pmic_vibrator_level = level;
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

