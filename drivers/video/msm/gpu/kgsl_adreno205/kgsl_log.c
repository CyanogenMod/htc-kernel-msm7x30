/* Copyright (c) 2002,2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/debugfs.h>
#include "kgsl_log.h"
#include "kgsl_device.h"
#include "kgsl.h"

/*default log levels is error for everything*/
#define KGSL_LOG_LEVEL_DEFAULT 3
#define KGSL_LOG_LEVEL_MAX     7
unsigned int kgsl_drv_log = KGSL_LOG_LEVEL_DEFAULT;
unsigned int kgsl_cmd_log = KGSL_LOG_LEVEL_DEFAULT;
unsigned int kgsl_ctxt_log = KGSL_LOG_LEVEL_DEFAULT;
unsigned int kgsl_mem_log = KGSL_LOG_LEVEL_DEFAULT;

#ifdef CONFIG_MSM_KGSL_MMU
unsigned int kgsl_cache_enable;
#endif

#ifdef CONFIG_DEBUG_FS
static int kgsl_log_set(unsigned int *log_val, void *data, u64 val)
{
	*log_val = min((unsigned int)val, (unsigned int)KGSL_LOG_LEVEL_MAX);
	return 0;
}

static int kgsl_drv_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_drv_log, data, val);
}

static int kgsl_drv_log_get(void *data, u64 *val)
{
	*val = kgsl_drv_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_drv_log_fops, kgsl_drv_log_get,
			kgsl_drv_log_set, "%llu\n");

static int kgsl_cmd_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_cmd_log, data, val);
}

static int kgsl_cmd_log_get(void *data, u64 *val)
{
	*val = kgsl_cmd_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_cmd_log_fops, kgsl_cmd_log_get,
			kgsl_cmd_log_set, "%llu\n");

static int kgsl_ctxt_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_ctxt_log, data, val);
}

static int kgsl_ctxt_log_get(void *data, u64 *val)
{
	*val = kgsl_ctxt_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_ctxt_log_fops, kgsl_ctxt_log_get,
			kgsl_ctxt_log_set, "%llu\n");

static int kgsl_mem_log_set(void *data, u64 val)
{
	return kgsl_log_set(&kgsl_mem_log, data, val);
}

static int kgsl_mem_log_get(void *data, u64 *val)
{
	*val = kgsl_mem_log;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_mem_log_fops, kgsl_mem_log_get,
			kgsl_mem_log_set, "%llu\n");

#ifdef CONFIG_MSM_KGSL_MMU
static int kgsl_cache_enable_set(void *data, u64 val)
{
	kgsl_cache_enable = (val != 0);
	return 0;
}

static int kgsl_cache_enable_get(void *data, u64 *val)
{
	*val = kgsl_cache_enable;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(kgsl_cache_enable_fops, kgsl_cache_enable_get,
			kgsl_cache_enable_set, "%llu\n");
#endif /*CONFIG_MSM_KGSL_MMU*/

#endif /* CONFIG_DEBUG_FS */

int kgsl_debug_init(void)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent;
	dent = debugfs_create_dir("kgsl", 0);
	if (IS_ERR(dent))
		return 0;

	debugfs_create_file("log_level_cmd", 0644, dent, 0,
				&kgsl_cmd_log_fops);
	debugfs_create_file("log_level_ctxt", 0644, dent, 0,
				&kgsl_ctxt_log_fops);
	debugfs_create_file("log_level_drv", 0644, dent, 0,
				&kgsl_drv_log_fops);
	debugfs_create_file("log_level_mem", 0644, dent, 0,
				&kgsl_mem_log_fops);

#ifdef CONFIG_MSM_KGSL_MMU
    debugfs_create_file("cache_enable", 0644, dent, 0,
				&kgsl_cache_enable_fops);
#endif

#endif /* CONFIG_DEBUG_FS */
	return 0;
}
