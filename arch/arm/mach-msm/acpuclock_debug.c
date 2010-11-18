/* arch/arm/mach-msm/acpuclock_debug.c
 *
 * Copyright (C) 2010 HTC Corporation
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

#include <linux/debugfs.h>
#include <mach/acpuclock_debug.h>

#if defined(CONFIG_DEBUG_FS)

enum {
	ATTR_CURRENT_VDD,
	ATTR_WFI_RAMP_DOWN,
	ATTR_PWRC_RAMP_DOWN,
};

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];
static unsigned int last_freq;
static unsigned int last_vdd;
static struct acpuclock_debug_dev *acpu_debug_dev;

static ssize_t debug_get_voltage(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	size_t bsize = 0;

	bsize = sprintf(debug_buffer, "%d %d\n", last_freq, last_vdd);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static ssize_t debug_set_voltage(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	unsigned int freq = 0;
	unsigned int vdd = 0;
	int ret = -EINVAL;

	sscanf(buf, "%d %d", &freq, &vdd);
	if (freq && vdd && acpu_debug_dev->update_freq_tbl)
		ret = acpu_debug_dev->update_freq_tbl(freq, vdd);

	if (!ret) {
		last_freq = freq;
		last_vdd = vdd;
	}

	return ret;
}

static ssize_t debug_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	int attr = (int)file->private_data;
	unsigned long val;
	int ret;

	ret = strict_strtoul(buf, 10, &val);
	if (ret)
		return ret;

	switch (attr) {
	case ATTR_WFI_RAMP_DOWN:
		if (!acpu_debug_dev->set_wfi_ramp_down)
			return -EINVAL;
		acpu_debug_dev->set_wfi_ramp_down(val & 0x1);
		break;
	case ATTR_PWRC_RAMP_DOWN:
		if (!acpu_debug_dev->set_pwrc_ramp_down)
			return -EINVAL;
		acpu_debug_dev->set_pwrc_ramp_down(val & 0x1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int attr = (int)file->private_data;
	size_t bsize = 0;

	switch (attr) {
	case ATTR_CURRENT_VDD:
		if (!acpu_debug_dev->get_current_vdd)
			return -EINVAL;
		bsize = sprintf(debug_buffer, "%d\n",
			acpu_debug_dev->get_current_vdd());
		break;
	case ATTR_WFI_RAMP_DOWN:
		if (!acpu_debug_dev->get_wfi_ramp_down)
			return -EINVAL;
		bsize = sprintf(debug_buffer,
			"%d\n", acpu_debug_dev->get_wfi_ramp_down());
		break;
	case ATTR_PWRC_RAMP_DOWN:
		if (!acpu_debug_dev->get_pwrc_ramp_down)
			return -EINVAL;
		bsize = sprintf(debug_buffer,
			"%d\n", acpu_debug_dev->get_pwrc_ramp_down());
		break;
	default:
		return -EINVAL;
	}

	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_set_freq_vdd = {
	.read = debug_get_voltage,
	.write = debug_set_voltage,
};

static const struct file_operations debug_ops = {
	.open = debug_open,
	.read = debug_read,
	.write = debug_write,
};

static int __init acpuclock_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("acpuclock", 0);
	if (IS_ERR(dent))
		return -1;

	if (!acpu_debug_dev)
		return -1;
	debugfs_create_file("set_freq_vdd", 0200, dent,
		NULL, &debug_set_freq_vdd);
	debugfs_create_file("current_vdd", 0400, dent,
		(void *)ATTR_CURRENT_VDD, &debug_ops);
	debugfs_create_file("wfi_ramp_down", 0600, dent,
		(void *)ATTR_WFI_RAMP_DOWN, &debug_ops);
	debugfs_create_file("pwrc_ramp_down", 0600, dent,
		(void *)ATTR_PWRC_RAMP_DOWN, &debug_ops);
	return 0;
}

late_initcall(acpuclock_debugfs_init);

int register_acpuclock_debug_dev(struct acpuclock_debug_dev *apcu_dev)
{
	if (acpu_debug_dev) {
		pr_warning("WARN: more than one acpuclock_debug_dev registered\n");
		return -1;
	}
	acpu_debug_dev = apcu_dev;
	return 0;
}

#endif
