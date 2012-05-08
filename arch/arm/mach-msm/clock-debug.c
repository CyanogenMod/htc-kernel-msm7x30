/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2011, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/clkdev.h>

#include "clock.h"

static int clock_debug_rate_set(void *data, u64 val)
{
	struct clk *clock = data;
	int ret;

	/* Only increases to max rate will succeed, but that's actually good
	 * for debugging purposes so we don't check for error. */
	if (clock->flags & CLKFLAG_MAX)
		clk_set_max_rate(clock, val);
	ret = clk_set_rate(clock, val);
	if (ret)
		pr_err("clk_set_rate failed (%d)\n", ret);

	return ret;
}

static int clock_debug_rate_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = clk_get_rate(clock);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(clock_rate_fops, clock_debug_rate_get,
			clock_debug_rate_set, "%llu\n");

static struct clk *measure;

static int clock_debug_measure_get(void *data, u64 *val)
{
	int ret;
	struct clk *clock = data;

	ret = clk_set_parent(measure, clock);
	if (!ret)
		*val = clk_get_rate(measure);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(clock_measure_fops, clock_debug_measure_get,
			NULL, "%lld\n");

static int clock_debug_enable_set(void *data, u64 val)
{
	struct clk *clock = data;
	int rc = 0;

	if (val)
		rc = clk_enable(clock);
	else
		clk_disable(clock);

	return rc;
}

static int clock_debug_enable_get(void *data, u64 *val)
{
	struct clk *clock = data;
	int enabled;

	if (clock->ops->is_enabled)
		enabled = clock->ops->is_enabled(clock);
	else
		enabled = !!(clock->count);

	*val = enabled;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(clock_enable_fops, clock_debug_enable_get,
			clock_debug_enable_set, "%lld\n");

static int clock_debug_local_get(void *data, u64 *val)
{
	struct clk *clock = data;

	*val = clock->ops->is_local(clock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(clock_local_fops, clock_debug_local_get,
			NULL, "%llu\n");

struct clk *clock_debug_parent_get(void *data)
{
	struct clk *clock = data;

	if (clock->ops->get_parent)
		return clock->ops->get_parent(clock);

	return 0;
}

static struct dentry *debugfs_base;
static u32 debug_suspend;
static struct clk_lookup *msm_clocks;
static size_t num_msm_clocks;

int htc_clock_dump(struct clk *clock, struct seq_file *m)
{
	int len = 0;
	u64 value = 0;
	struct clk *parent;
	char nam_buf[20];
	char en_buf[20];
	char hz_buf[20];
	char loc_buf[20];
	char par_buf[20];

	if (!clock)
		return 0;

	memset(nam_buf,  ' ', sizeof(nam_buf));
	nam_buf[19] = 0;
	memset(en_buf, 0, sizeof(en_buf));
	memset(hz_buf, 0, sizeof(hz_buf));
	memset(loc_buf, 0, sizeof(loc_buf));
	memset(par_buf,  ' ', sizeof(par_buf));
	par_buf[19] = 0;

	len = strlen(clock->dbg_name);
	if (len > 19)
		len = 19;
	memcpy(nam_buf, clock->dbg_name, len);

	clock_debug_enable_get(clock, &value);
	if (value)
		sprintf(en_buf, "Y");
	else
		sprintf(en_buf, "N");

	clock_debug_rate_get(clock, &value);
	sprintf(hz_buf, "%llu", value);

	clock_debug_local_get(clock, &value);
	if (value)
		sprintf(loc_buf, "Y");
	else
		sprintf(loc_buf, "N");

	parent = clock_debug_parent_get(clock);
	if (parent) {
		len = strlen(parent->dbg_name);
		if (len > 19)
			len = 19;
		memcpy(par_buf, parent->dbg_name, len);
	} else
		memcpy(par_buf, "NULL", 4);

	if (m)
		seq_printf(m, "%s: [EN]%s, [LOC]%s, [SRC]%s, [FREQ]%s\n", nam_buf, en_buf, loc_buf, par_buf, hz_buf);
	else
		pr_info("%s: [EN]%s, [LOC]%s, [SRC]%s, [FREQ]%s\n", nam_buf, en_buf, loc_buf, par_buf, hz_buf);

	return 0;
}

static int list_clocks_show(struct seq_file *m, void *unused)
{
	int index;
	char *title_msg = "------------ HTC Clock -------------\n";

	seq_printf(m, title_msg);
	for (index = 0; index < num_msm_clocks; index++)
		htc_clock_dump(msm_clocks[index].clk, m);
	return 0;
}

static int list_clocks_open(struct inode *inode, struct file *file)
{
	return single_open(file, list_clocks_show, inode->i_private);
}

static const struct file_operations list_clocks_fops = {
	.open		= list_clocks_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static struct dentry *debugfs_clock_base;

int __init htc_clock_status_debug_init(void)
{
	int err = 0;

	debugfs_clock_base = debugfs_create_dir("htc_clock", NULL);
	if (!debugfs_clock_base)
		return -ENOMEM;

	if (!debugfs_create_file("list_clocks", S_IRUGO, debugfs_clock_base,
				&msm_clocks, &list_clocks_fops))
		return -ENOMEM;

	return err;
}

int __init clock_debug_init(struct clock_init_data *data)
{
	int ret = 0;

	debugfs_base = debugfs_create_dir("clk", NULL);
	if (!debugfs_base)
		return -ENOMEM;
	if (!debugfs_create_u32("debug_suspend", S_IRUGO | S_IWUSR,
				debugfs_base, &debug_suspend)) {
		debugfs_remove_recursive(debugfs_base);
		return -ENOMEM;
	}
	msm_clocks = data->table;
	num_msm_clocks = data->size;

	measure = clk_get_sys("debug", "measure");
	if (IS_ERR(measure)) {
		ret = PTR_ERR(measure);
		measure = NULL;
	}
	htc_clock_status_debug_init();

	return ret;
}


static int clock_debug_print_clock(struct clk *c)
{
	size_t ln = 0;
	char s[128];

	if (!c || !c->count)
		return 0;

	ln += snprintf(s, sizeof(s), "\t%s", c->dbg_name);
	while (ln < sizeof(s) && (c = clk_get_parent(c)))
		ln += snprintf(s + ln, sizeof(s) - ln, " -> %s", c->dbg_name);
	pr_info("%s\n", s);
	return 1;
}

void clock_debug_print_enabled(void)
{
	unsigned i;
	int cnt = 0;

/*
	if (likely(!debug_suspend))
		return;
*/

	pr_info("Enabled clocks:\n");
	for (i = 0; i < num_msm_clocks; i++)
		cnt += clock_debug_print_clock(msm_clocks[i].clk);

	if (cnt)
		pr_info("Enabled clock count: %d\n", cnt);
	else
		pr_info("No clocks enabled.\n");

}

static int list_rates_show(struct seq_file *m, void *unused)
{
	struct clk *clock = m->private;
	int rate, level, fmax = 0, i = 0;

	/* Find max frequency supported within voltage constraints. */
	if (!clock->vdd_class) {
		fmax = INT_MAX;
	} else {
		for (level = 0; level < ARRAY_SIZE(clock->fmax); level++)
			if (clock->fmax[level])
				fmax = clock->fmax[level];
	}

	/*
	 * List supported frequencies <= fmax. Higher frequencies may appear in
	 * the frequency table, but are not valid and should not be listed.
	 */
	while ((rate = clock->ops->list_rate(clock, i++)) >= 0) {
		if (rate <= fmax)
			seq_printf(m, "%u\n", rate);
	}

	return 0;
}

static int list_rates_open(struct inode *inode, struct file *file)
{
	return single_open(file, list_rates_show, inode->i_private);
}

static const struct file_operations list_rates_fops = {
	.open		= list_rates_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

int __init clock_debug_add(struct clk *clock)
{
	char temp[50], *ptr;
	struct dentry *clk_dir;

	if (!debugfs_base)
		return -ENOMEM;

	strlcpy(temp, clock->dbg_name, ARRAY_SIZE(temp));
	for (ptr = temp; *ptr; ptr++)
		*ptr = tolower(*ptr);

	clk_dir = debugfs_create_dir(temp, debugfs_base);
	if (!clk_dir)
		return -ENOMEM;

	if (!debugfs_create_file("rate", S_IRUGO | S_IWUSR, clk_dir,
				clock, &clock_rate_fops))
		goto error;

	if (!debugfs_create_file("enable", S_IRUGO | S_IWUSR, clk_dir,
				clock, &clock_enable_fops))
		goto error;

	if (!debugfs_create_file("is_local", S_IRUGO, clk_dir, clock,
				&clock_local_fops))
		goto error;

	if (measure &&
	    !clk_set_parent(measure, clock) &&
	    !debugfs_create_file("measure", S_IRUGO, clk_dir, clock,
				&clock_measure_fops))
		goto error;

	if (clock->ops->list_rate)
		if (!debugfs_create_file("list_rates",
				S_IRUGO, clk_dir, clock, &list_rates_fops))
			goto error;

	return 0;
error:
	debugfs_remove_recursive(clk_dir);
	return -ENOMEM;
}
