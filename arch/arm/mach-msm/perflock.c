/* arch/arm/mach-msm/perflock.c
 *
 * Copyright (C) 2008 HTC Corporation
 * Author: Eiven Peng <eiven_peng@htc.com>
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>
#include <linux/timer.h>
#include <mach/perflock.h>
#include "proc_comm.h"
#include "acpuclock.h"

#define PERF_LOCK_INITIALIZED	(1U << 0)
#define PERF_LOCK_ACTIVE	(1U << 1)

enum {
	PERF_LOCK_DEBUG = 1U << 0,
	PERF_EXPIRE_DEBUG = 1U << 1,
	PERF_CPUFREQ_NOTIFY_DEBUG = 1U << 2,
	PERF_CPUFREQ_LOCK_DEBUG = 1U << 3,
	PERF_SCREEN_ON_POLICY_DEBUG = 1U << 4,
};

static LIST_HEAD(active_perf_locks);
static LIST_HEAD(inactive_perf_locks);
static DEFINE_SPINLOCK(list_lock);
static DEFINE_SPINLOCK(policy_update_lock);
static int initialized;
static unsigned int *perf_acpu_table;
static unsigned int table_size;
static unsigned int curr_lock_speed;
static struct cpufreq_policy *cpufreq_policy;

#ifdef CONFIG_PERF_LOCK_DEBUG
static int debug_mask = PERF_LOCK_DEBUG | PERF_EXPIRE_DEBUG |
	PERF_CPUFREQ_NOTIFY_DEBUG | PERF_CPUFREQ_LOCK_DEBUG;
#else
static int debug_mask = PERF_CPUFREQ_LOCK_DEBUG | PERF_SCREEN_ON_POLICY_DEBUG;
#endif
module_param_call(debug_mask, param_set_int, param_get_int,
		&debug_mask, S_IWUSR | S_IRUGO);

static unsigned int get_perflock_speed(void);
static void print_active_locks(void);

#ifdef CONFIG_PERFLOCK_SCREEN_POLICY
/* Increase cpufreq minumum frequency when screen on.
    Pull down to lowest speed when screen off. */
static unsigned int screen_off_policy_req;
static unsigned int screen_on_policy_req;
static void perflock_early_suspend(struct early_suspend *handler)
{
	unsigned long irqflags;

	spin_lock_irqsave(&policy_update_lock, irqflags);
	if (screen_on_policy_req) {
		screen_on_policy_req--;
		spin_unlock_irqrestore(&policy_update_lock, irqflags);
		return;
	}
	screen_off_policy_req++;
	spin_unlock_irqrestore(&policy_update_lock, irqflags);

	if (cpufreq_policy)
		cpufreq_update_policy(cpufreq_policy->cpu);
}

static void perflock_late_resume(struct early_suspend *handler)
{
	unsigned long irqflags;

/*
 * This workaround is for hero project
 * May cause potential bug:
 * Accidentally set cpu in high freq in screen off mode.
 * senario: in screen off early suspended state, runs the following sequence:
 * 1.perflock_late_resume():acpuclk_set_rate(high freq);screen_on_pilicy_req=1;
 * 2.perflock_early_suspend():if(screen_on_policy_req) return;
 * 3.perflock_notifier_call(): only set policy's min and max
 */
#ifdef CONFIG_MACH_HERO
	/* Work around for display driver,
	 * need to increase cpu speed immediately.
	 */
	unsigned int lock_speed = get_perflock_speed() / 1000;
	if (lock_speed > CONFIG_PERFLOCK_SCREEN_ON_MIN)
		acpuclk_set_rate(lock_speed * 1000, 0);
	else
		acpuclk_set_rate(CONFIG_PERFLOCK_SCREEN_ON_MIN * 1000, 0);
#endif

	spin_lock_irqsave(&policy_update_lock, irqflags);
	if (screen_off_policy_req) {
		screen_off_policy_req--;
		spin_unlock_irqrestore(&policy_update_lock, irqflags);
		return;
	}
	screen_on_policy_req++;
	spin_unlock_irqrestore(&policy_update_lock, irqflags);

	if (cpufreq_policy)
		cpufreq_update_policy(cpufreq_policy->cpu);
}

static struct early_suspend perflock_power_suspend = {
	.suspend = perflock_early_suspend,
	.resume = perflock_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};

/* 7k projects need to raise up cpu freq before panel resume for stability */
#if defined(CONFIG_HTC_ONMODE_CHARGING) && \
	(defined(CONFIG_ARCH_MSM7225) || \
	defined(CONFIG_ARCH_MSM7227) || \
	defined(CONFIG_ARCH_MSM7201A))
static struct early_suspend perflock_onchg_suspend = {
	.suspend = perflock_early_suspend,
	.resume = perflock_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};
#endif

static int __init perflock_screen_policy_init(void)
{
	register_early_suspend(&perflock_power_suspend);
/* 7k projects need to raise up cpu freq before panel resume for stability */
#if defined(CONFIG_HTC_ONMODE_CHARGING) && \
	(defined(CONFIG_ARCH_MSM7225) || \
	defined(CONFIG_ARCH_MSM7227) || \
	defined(CONFIG_ARCH_MSM7201A))
	register_onchg_suspend(&perflock_onchg_suspend);
#endif
	screen_on_policy_req++;
	if (cpufreq_policy)
		cpufreq_update_policy(cpufreq_policy->cpu);

	return 0;
}

late_initcall(perflock_screen_policy_init);
#endif

#if 0
static unsigned int policy_min = CONFIG_MSM_CPU_FREQ_ONDEMAND_MIN;
static unsigned int policy_max = CONFIG_MSM_CPU_FREQ_ONDEMAND_MAX;
#else
static unsigned int policy_min;
static unsigned int policy_max;
#endif
static int param_set_cpu_min_max(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	if (cpufreq_policy)
		cpufreq_update_policy(cpufreq_policy->cpu);
	return ret;
}

module_param_call(min_cpu_khz, param_set_cpu_min_max, param_get_int,
	&policy_min, S_IWUSR | S_IRUGO);
module_param_call(max_cpu_khz, param_set_cpu_min_max, param_get_int,
	&policy_max, S_IWUSR | S_IRUGO);

static int perflock_notifier_call(struct notifier_block *self,
			       unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int lock_speed;
	unsigned long irqflags;

	spin_lock_irqsave(&policy_update_lock, irqflags);
	if (debug_mask & PERF_CPUFREQ_NOTIFY_DEBUG)
		pr_info("%s: event=%ld, policy->min=%d, policy->max=%d",
			__func__, event, policy->min, policy->max);

	if (event == CPUFREQ_START)
		cpufreq_policy = policy;
	else if (event == CPUFREQ_NOTIFY) {
		/* Each time cpufreq_update_policy,
		 * min/max will reset, need to set it again. */
#ifdef CONFIG_PERFLOCK_SCREEN_POLICY
		if (screen_on_policy_req) {
			if (debug_mask & PERF_SCREEN_ON_POLICY_DEBUG)
				pr_info("%s: screen_on_policy_req %d,"
					"policy_min %d\n", __func__,
					screen_on_policy_req,
					CONFIG_PERFLOCK_SCREEN_ON_MIN);
			policy_min = CONFIG_PERFLOCK_SCREEN_ON_MIN;
			policy_max = CONFIG_PERFLOCK_SCREEN_ON_MAX;
			screen_on_policy_req--;
		} else if (screen_off_policy_req) {
			if (debug_mask & PERF_SCREEN_ON_POLICY_DEBUG)
				pr_info("%s: screen_off_policy_req %d,"
					"policy_min %d\n", __func__,
					screen_off_policy_req,
					CONFIG_MSM_CPU_FREQ_ONDEMAND_MIN);
			policy_min = CONFIG_PERFLOCK_SCREEN_OFF_MIN;
			policy_max = CONFIG_PERFLOCK_SCREEN_OFF_MAX;
			screen_off_policy_req--;
		}
#endif
		lock_speed = get_perflock_speed() / 1000;
		if (lock_speed) {
			policy->min = lock_speed;
			policy->max = lock_speed;
			if (debug_mask & PERF_CPUFREQ_LOCK_DEBUG) {
				pr_info("%s: cpufreq lock speed %d\n",
					__func__, lock_speed);
				print_active_locks();
			}
		} else {
			policy->min = policy_min;
			policy->max = policy_max;
			if (debug_mask & PERF_CPUFREQ_LOCK_DEBUG)
				pr_info("%s: cpufreq recover policy %d %d\n",
					__func__, policy->min, policy->max);
		}
		curr_lock_speed = lock_speed;
	}
	spin_unlock_irqrestore(&policy_update_lock, irqflags);

	return 0;
}

static struct notifier_block perflock_notifier = {
	.notifier_call = perflock_notifier_call,
};

static unsigned int get_perflock_speed(void)
{
	unsigned long irqflags;
	struct perf_lock *lock;
	unsigned int perf_level = 0;

	/* Get the maxmimum perf level. */
	if (list_empty(&active_perf_locks))
		return 0;

	spin_lock_irqsave(&list_lock, irqflags);
	list_for_each_entry(lock, &active_perf_locks, link) {
		if (lock->level > perf_level)
			perf_level = lock->level;
	}
	spin_unlock_irqrestore(&list_lock, irqflags);

	return perf_acpu_table[perf_level];
}

static void print_active_locks(void)
{
	unsigned long irqflags;
	struct perf_lock *lock;

	spin_lock_irqsave(&list_lock, irqflags);
	list_for_each_entry(lock, &active_perf_locks, link) {
		pr_info("active perf lock '%s'\n", lock->name);
	}
	spin_unlock_irqrestore(&list_lock, irqflags);
}

/**
 * perf_lock_init - acquire a perf lock
 * @lock: perf lock to acquire
 * @level: performance level of @lock
 * @name: the name of @lock
 *
 * Acquire @lock with @name and @level. (It doesn't activate the lock.)
 */
void perf_lock_init(struct perf_lock *lock,
			unsigned int level, const char *name)
{
	unsigned long irqflags = 0;

	WARN_ON(!name);
	WARN_ON(level >= PERF_LOCK_INVALID);
	WARN_ON(lock->flags & PERF_LOCK_INITIALIZED);

	if ((!name) || (level >= PERF_LOCK_INVALID) ||
			(lock->flags & PERF_LOCK_INITIALIZED)) {
		pr_err("%s: ERROR \"%s\" flags %x level %d\n",
			__func__, name, lock->flags, level);
		return;
	}
	lock->name = name;
	lock->flags = PERF_LOCK_INITIALIZED;
	lock->level = level;

	INIT_LIST_HEAD(&lock->link);
	spin_lock_irqsave(&list_lock, irqflags);
	list_add(&lock->link, &inactive_perf_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);
}
EXPORT_SYMBOL(perf_lock_init);

/**
 * perf_lock - activate a perf lock
 * @lock: perf lock to activate
 *
 * Activate @lock.(Need to init_perf_lock before activate)
 */
void perf_lock(struct perf_lock *lock)
{
	unsigned long irqflags;

	WARN_ON(!initialized);
	WARN_ON((lock->flags & PERF_LOCK_INITIALIZED) == 0);
	WARN_ON(lock->flags & PERF_LOCK_ACTIVE);

	spin_lock_irqsave(&list_lock, irqflags);
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s: '%s', flags %d level %d\n",
			__func__, lock->name, lock->flags, lock->level);
	if (lock->flags & PERF_LOCK_ACTIVE) {
		pr_err("%s: over-locked\n", __func__);
		return;
	}
	lock->flags |= PERF_LOCK_ACTIVE;
	list_del(&lock->link);
	list_add(&lock->link, &active_perf_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);

	/* Update cpufreq policy - scaling_min/scaling_max */
	if (cpufreq_policy &&
			(curr_lock_speed != (get_perflock_speed() / 1000)))
		cpufreq_update_policy(cpufreq_policy->cpu);
}
EXPORT_SYMBOL(perf_lock);

#define PERF_UNLOCK_DELAY		(HZ)
static void do_expire_perf_locks(struct work_struct *work)
{
	if (debug_mask & PERF_EXPIRE_DEBUG)
		pr_info("%s: timed out to unlock\n", __func__);

	if (cpufreq_policy &&
			(curr_lock_speed != (get_perflock_speed() / 1000))) {
		if (debug_mask & PERF_EXPIRE_DEBUG)
			pr_info("%s: update cpufreq policy\n", __func__);
		cpufreq_update_policy(cpufreq_policy->cpu);
	}
}
static DECLARE_DELAYED_WORK(work_expire_perf_locks, do_expire_perf_locks);

/**
 * perf_unlock - de-activate a perf lock
 * @lock: perf lock to de-activate
 *
 * de-activate @lock.
 */
void perf_unlock(struct perf_lock *lock)
{
	unsigned long irqflags;

	WARN_ON(!initialized);
	WARN_ON((lock->flags & PERF_LOCK_ACTIVE) == 0);

	spin_lock_irqsave(&list_lock, irqflags);
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s: '%s', flags %d level %d\n",
			__func__, lock->name, lock->flags, lock->level);
	if (!(lock->flags & PERF_LOCK_ACTIVE)) {
		pr_err("%s: under-locked\n", __func__);
		return;
	}
	lock->flags &= ~PERF_LOCK_ACTIVE;
	list_del(&lock->link);
	list_add(&lock->link, &inactive_perf_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);

	/* Prevent lock/unlock quickly, add a timeout to release perf_lock */
	if (cpufreq_policy &&
			(curr_lock_speed != (get_perflock_speed() / 1000)))
		schedule_delayed_work(&work_expire_perf_locks,
			PERF_UNLOCK_DELAY);
}
EXPORT_SYMBOL(perf_unlock);

/**
 * is_perf_lock_active - query if a perf_lock is active or not
 * @lock: target perf lock
 * RETURN: 0: inactive; 1: active
 *
 * query if @lock is active or not
 */
inline int is_perf_lock_active(struct perf_lock *lock)
{
	return (lock->flags & PERF_LOCK_ACTIVE);
}
EXPORT_SYMBOL(is_perf_lock_active);

/**
 * is_perf_locked - query if there is any perf lock activates
 * RETURN: 0: no perf lock activates 1: at least a perf lock activates
 */
int is_perf_locked(void)
{
	return (!list_empty(&active_perf_locks));
}
EXPORT_SYMBOL(is_perf_locked);


#ifdef CONFIG_PERFLOCK_BOOT_LOCK
/* Stop cpufreq and lock cpu, shorten boot time. */
#define BOOT_LOCK_TIMEOUT	(60 * HZ)
static struct perf_lock boot_perf_lock;

static void do_expire_boot_lock(struct work_struct *work)
{
	perf_unlock(&boot_perf_lock);
	pr_info("Release 'boot-time' perf_lock\n");
}
static DECLARE_DELAYED_WORK(work_expire_boot_lock, do_expire_boot_lock);
#endif

static void perf_acpu_table_fixup(void)
{
	int i;
	for (i = 0; i < table_size; ++i) {
		if (perf_acpu_table[i] > policy_max * 1000)
			perf_acpu_table[i] = policy_max * 1000;
		else if (perf_acpu_table[i] < policy_min * 1000)
			perf_acpu_table[i] = policy_min * 1000;
	}
}

void __init perflock_init(struct perflock_platform_data *pdata)
{
	struct cpufreq_policy policy;
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(smp_processor_id());

	BUG_ON(cpufreq_frequency_table_cpuinfo(&policy, table));
	policy_min = policy.cpuinfo.min_freq;
	policy_max = policy.cpuinfo.max_freq;

	if (!pdata)
		goto invalid_config;

	perf_acpu_table = pdata->perf_acpu_table;
	table_size = pdata->table_size;
	if (!perf_acpu_table || !table_size)
		goto invalid_config;
	if (table_size < PERF_LOCK_INVALID)
		goto invalid_config;

	perf_acpu_table_fixup();
	cpufreq_register_notifier(&perflock_notifier, CPUFREQ_POLICY_NOTIFIER);

	initialized = 1;

#ifdef CONFIG_PERFLOCK_BOOT_LOCK
	/* Stop cpufreq and lock cpu, shorten boot time. */
	perf_lock_init(&boot_perf_lock, PERF_LOCK_HIGHEST, "boot-time");
	perf_lock(&boot_perf_lock);
	schedule_delayed_work(&work_expire_boot_lock, BOOT_LOCK_TIMEOUT);
	pr_info("Acquire 'boot-time' perf_lock\n");
#endif

	return;

invalid_config:
	pr_err("%s: invalid configuration data, %p %d %d\n", __func__,
		perf_acpu_table, table_size, PERF_LOCK_INVALID);
}
