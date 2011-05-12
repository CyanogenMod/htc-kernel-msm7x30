/* arch/arm/mach-msm/htc_set_perflock.c
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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <mach/perflock.h>

static	struct mutex lock;
static 	struct perf_lock media_perf_lock;
static	uint32_t num = 0;

static int perflock_open(struct inode *inode, struct file *file)
{
	mutex_lock(&lock);
	if (num == 0) {
		perf_lock(&media_perf_lock);
		printk(KERN_DEBUG "[perflock] Perflock enabled.\n");
	}
	num++;
	printk(KERN_DEBUG "[perflock] Perflock node is opened by [%s]/[PID=%d],numbers of opened nodes = [%d].\n",
		current->comm, current->pid, num);
	mutex_unlock(&lock);

	return 0;
}

static int perflock_release(struct inode *inode, struct file *file)
{
	mutex_lock(&lock);
	num--;
	printk(KERN_DEBUG "[perflock] Perflock node is closed by [%s]/[PID=%d], numbers of opened nodes = [%d].\n",
		current->comm, current->pid, num);
	if (num == 0) {
		perf_unlock(&media_perf_lock);
		printk(KERN_DEBUG "[perflock] Perflock disabled.\n");
	}
	mutex_unlock(&lock);

	return 0;
}

static struct file_operations perflock_fops = {
	.owner		= THIS_MODULE,
	.open		= perflock_open,
	.release	= perflock_release,
};

struct miscdevice perflock_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "perflock",
	.fops	= &perflock_fops,
};

static int __init set_perflock_init(void) {
	mutex_init(&lock);
	perf_lock_init(&media_perf_lock, PERF_LOCK_HIGHEST, "media");
	return misc_register(&perflock_misc);
}

device_initcall(set_perflock_init);
