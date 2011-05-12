/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
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

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "power.h"

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_NO_SUSPEND = 1U << 3,
};
#ifdef CONFIG_NO_SUSPEND
static int debug_mask = DEBUG_USER_STATE | DEBUG_NO_SUSPEND;
#else
static int debug_mask = DEBUG_USER_STATE;
#endif
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPENDED_ON = 0x0,
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;
#ifdef CONFIG_HTC_ONMODE_CHARGING
static LIST_HEAD(onchg_suspend_handlers);
static void onchg_suspend(struct work_struct *work);
static void onchg_resume(struct work_struct *work);

static DECLARE_WORK(onchg_suspend_work, onchg_suspend);
static DECLARE_WORK(onchg_resume_work, onchg_resume);

static int state_onchg;
#endif

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

#ifdef CONFIG_SYS_SYNC_BLOCKING_DEBUG
void sys_sync_debug(void);
#endif

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	pr_info("[R] early_suspend start\n");
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED) {
		state |= SUSPENDED;
#ifdef CONFIG_HTC_ONMODE_CHARGING
		state_onchg = SUSPEND_REQUESTED_AND_SUSPENDED;
#endif
	}
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL)
			pos->suspend(pos);
	}
	mutex_unlock(&early_suspend_lock);

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: sync\n");

	pr_info("[R] early_suspend: sync\n");

#ifdef CONFIG_SYS_SYNC_BLOCKING_DEBUG
	sys_sync_debug();
#else
	sys_sync();
#endif

	if (debug_mask & DEBUG_NO_SUSPEND) {
		pr_info("DEBUG_NO_SUSPEND set, will not suspend\n");
		wake_lock(&no_suspend_wake_lock);
	}

abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
	pr_info("[R] early_suspend end\n");
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	pr_info("[R] late_resume start\n");
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED) {
		state &= ~SUSPENDED;
#ifdef CONFIG_HTC_ONMODE_CHARGING
		state_onchg &= ~SUSPEND_REQUESTED_AND_SUSPENDED;
#endif
	}
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link)
		if (pos->resume != NULL)
			pos->resume(pos);
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");

	wake_unlock(&no_suspend_wake_lock);

abort:
	mutex_unlock(&early_suspend_lock);
	pr_info("[R] late_resume end\n");
}

#ifdef CONFIG_HTC_ONMODE_CHARGING
void register_onchg_suspend(struct early_suspend *handler)
{
	struct list_head *pos;
	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &onchg_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_onchg_suspend);

void unregister_onchg_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_onchg_suspend);


static void onchg_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	pr_info("[R] onchg_suspend start\n");
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED &&
	    state_onchg == SUSPEND_REQUESTED)
		state_onchg |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("onchg_suspend: abort, state %d, state_onchg: %d\n", state, state_onchg);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("onchg_suspend: call handlers\n");

	list_for_each_entry(pos, &onchg_suspend_handlers, link) {
		if (pos->suspend != NULL)
			pos->suspend(pos);
	}
	mutex_unlock(&early_suspend_lock);

abort:
	pr_info("[R] onchg_suspend end\n");
}

static void onchg_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	pr_info("[R] onchg_resume start\n");
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if ( state == SUSPEND_REQUESTED_AND_SUSPENDED &&
	     state_onchg == SUSPENDED)
		state_onchg &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("onchg_resume: abort, state %d, state_onchg: %d\n", state, state_onchg);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("onchg_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &onchg_suspend_handlers, link)
		if (pos->resume != NULL)
			pos->resume(pos);
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("onchg_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
	pr_info("[R] onchg_resume end\n");
}

void request_onchg_state(int on)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_onchg_state: %s (%d.%d)->%d at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			on == 1 ? "on" : "off",
			state,
			!(state_onchg & SUSPEND_REQUESTED),
			on,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED) {
		old_sleep = state_onchg & SUSPEND_REQUESTED;
		if (!old_sleep && on == 0) {
			state_onchg |= SUSPEND_REQUESTED;
			queue_work(suspend_work_queue, &onchg_suspend_work);
		}
		else if (old_sleep && on ==1) {
			state_onchg &= ~SUSPEND_REQUESTED;
			queue_work(suspend_work_queue, &onchg_resume_work);
		}
	}
	spin_unlock_irqrestore(&state_lock, irqflags);
}

int get_onchg_state(void)
{
	return state_onchg;
}
#endif

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
