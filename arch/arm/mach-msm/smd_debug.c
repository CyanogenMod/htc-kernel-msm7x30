/* arch/arm/mach-msm/smd_debug.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/list.h>
#include <linux/io.h>
#include <linux/earlysuspend.h>
#include <linux/rtc.h>
#include <linux/suspend.h>

#include <mach/msm_iomap.h>

#include "smd_private.h"
#include "smd_debug.h"


enum {
	F_SCREEN_OFF = 0,
	F_SUSPEND,
	F_RESUME,
	F_SCREEN_ON,
};

struct smem_sleep_stat {
	uint32_t tcxo_time;
	uint32_t tcxo_cnt;
	uint32_t suspend_tcxo_time;
	uint32_t suspend_tcxo_cnt;
	uint32_t garbage_pkt_cnt;
	uint32_t zone_based_reg_cnt;
	uint32_t idle_hand_off_cnt;
	uint32_t mo_2g_probe_cnt;
	uint32_t mo_3g_probe_cnt;
	uint32_t garbage_cnt;
	uint32_t cell_swt_cnt;
	uint32_t reserved[3];
};

struct mem_sleep_stat_attr {
	struct attribute attr;
	ssize_t (*show)(struct device *dev, struct mem_sleep_stat_attr *, char *);
	ssize_t (*store)(struct device *dev, struct mem_sleep_stat_attr *, char *);
};

struct kobject *sleep_stat_kobj;

static ssize_t show_mem_sleep_stat_attr(struct device *dev,
						struct mem_sleep_stat_attr *attr,
						char *buf);

struct smem_negate_client {
	uint32_t htc_negate_tcxo_client[16];
	uint32_t tcxo_cnt_during_suspend;
	uint32_t htc_total_sleep_clients;
	uint32_t htc_insuff_time_count;
};

struct mutex mem_sleep_stat_lock;
static struct smem_sleep_stat *sleep_stat;
static struct smem_sleep_stat *get_smem_sleep_stat(void)
{
#if CONFIG_SMD_OFFSET_TCXO_STAT
	return (struct smem_sleep_stat *)
		(MSM_SHARED_RAM_BASE + CONFIG_SMD_OFFSET_TCXO_STAT);
#else
	return 0;
#endif
}

static void print_sleep_stat(int flag)
{
	struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	if (!sleep_stat)
		return;

	pr_info("sleep_stat.%d: %dms %d %dms %d - "
		"%d %d %d - %d %d - %d %d %d %d %d"
		"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
		flag, sleep_stat->tcxo_time, sleep_stat->tcxo_cnt,
		sleep_stat->suspend_tcxo_time, sleep_stat->suspend_tcxo_cnt,
		sleep_stat->garbage_pkt_cnt, sleep_stat->zone_based_reg_cnt,
		sleep_stat->idle_hand_off_cnt, sleep_stat->mo_2g_probe_cnt,
		sleep_stat->mo_3g_probe_cnt, sleep_stat->garbage_cnt,
		sleep_stat->cell_swt_cnt, sleep_stat->reserved[0],
		sleep_stat->reserved[1], sleep_stat->reserved[2],
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

}

static struct smem_negate_client *negate_client_stat;
static struct smem_negate_client *get_smem_negate_client_stat(void)
{
#if CONFIG_SMD_OFFSET_NEGATE_CLIENT_STAT
	return (struct smem_negate_client *)
		(MSM_SHARED_RAM_BASE + CONFIG_SMD_OFFSET_NEGATE_CLIENT_STAT);
#else
	return 0;
#endif
}

static void print_negate_client_stat(void)
{
	if (!negate_client_stat)
		return;

	pr_info("negate_client_stat: %d - %d %d %d %d - %d %d %d %d - %d %d %d %d - %d %d %d %d - %d %d\n",
		negate_client_stat->tcxo_cnt_during_suspend,
		negate_client_stat->htc_negate_tcxo_client[0],
		negate_client_stat->htc_negate_tcxo_client[1],
		negate_client_stat->htc_negate_tcxo_client[2],
		negate_client_stat->htc_negate_tcxo_client[3],
		negate_client_stat->htc_negate_tcxo_client[4],
		negate_client_stat->htc_negate_tcxo_client[5],
		negate_client_stat->htc_negate_tcxo_client[6],
		negate_client_stat->htc_negate_tcxo_client[7],
		negate_client_stat->htc_negate_tcxo_client[8],
		negate_client_stat->htc_negate_tcxo_client[9],
		negate_client_stat->htc_negate_tcxo_client[10],
		negate_client_stat->htc_negate_tcxo_client[11],
		negate_client_stat->htc_negate_tcxo_client[12],
		negate_client_stat->htc_negate_tcxo_client[13],
		negate_client_stat->htc_negate_tcxo_client[14],
		negate_client_stat->htc_negate_tcxo_client[15],
		negate_client_stat->htc_total_sleep_clients,
		negate_client_stat->htc_insuff_time_count);
}

static int sleep_stat_suspend_notifier(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	switch (event) {
	/* enter suspend */
	case PM_SUSPEND_PREPARE:
		print_sleep_stat(F_SUSPEND);
		return NOTIFY_OK;
	/* exit suspend */
	case PM_POST_SUSPEND:
		print_sleep_stat(F_RESUME);
		print_negate_client_stat();
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block sleep_stat_notif_block = {
	.notifier_call = sleep_stat_suspend_notifier,
};

static void sleep_stat_early_suspend(struct early_suspend *handler)
{
	print_sleep_stat(F_SCREEN_OFF);
}

static void sleep_stat_late_resume(struct early_suspend *handler)
{
	print_sleep_stat(F_SCREEN_ON);
}

static struct early_suspend sleep_stat_screen_hdl = {
	.suspend = sleep_stat_early_suspend,
	.resume = sleep_stat_late_resume,
};

#if defined(CONFIG_DEBUG_FS)

static char *chstate(unsigned n)
{
	switch (n) {
	case SMD_SS_CLOSED:
		return "CLOSED";
	case SMD_SS_OPENING:
		return "OPENING";
	case SMD_SS_OPENED:
		return "OPENED";
	case SMD_SS_FLUSHING:
		return "FLUSHING";
	case SMD_SS_CLOSING:
		return "CLOSING";
	case SMD_SS_RESET:
		return "RESET";
	case SMD_SS_RESET_OPENING:
		return "ROPENING";
	default:
		return "UNKNOWN";
	}
}


static int dump_ch(char *buf, int max, struct smd_channel *ch)
{
	volatile struct smd_half_channel *s = ch->send;
	volatile struct smd_half_channel *r = ch->recv;

	return scnprintf(
		buf, max,
		"ch%02d:"
		" %8s(%05d/%05d) %c%c%c%c%c%c%c <->"
		" %8s(%05d/%05d) %c%c%c%c%c%c%c '%s'\n", ch->n,
		chstate(s->state), s->tail, s->head,
		s->fDSR ? 'D' : 'd',
		s->fCTS ? 'C' : 'c',
		s->fCD ? 'C' : 'c',
		s->fRI ? 'I' : 'i',
		s->fHEAD ? 'W' : 'w',
		s->fTAIL ? 'R' : 'r',
		s->fSTATE ? 'S' : 's',
		chstate(r->state), r->tail, r->head,
		r->fDSR ? 'D' : 'd',
		r->fCTS ? 'R' : 'r',
		r->fCD ? 'C' : 'c',
		r->fRI ? 'I' : 'i',
		r->fHEAD ? 'W' : 'w',
		r->fTAIL ? 'R' : 'r',
		r->fSTATE ? 'S' : 's',
		ch->name
		);
}

static int debug_read_stat(char *buf, int max)
{
	char *msg;
	int i = 0;

	msg = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);

	if (raw_smsm_get_state(SMSM_STATE_MODEM) & SMSM_RESET)
		i += scnprintf(buf + i, max - i,
			       "smsm: ARM9 HAS CRASHED\n");

	i += scnprintf(buf + i, max - i, "smsm: a9: %08x a11: %08x\n",
		       raw_smsm_get_state(SMSM_STATE_MODEM),
		       raw_smsm_get_state(SMSM_STATE_APPS));
#ifdef CONFIG_ARCH_MSM_SCORPION
	i += scnprintf(buf + i, max - i, "smsm dem: apps: %08x modem: %08x "
		       "qdsp6: %08x power: %08x time: %08x\n",
		       raw_smsm_get_state(SMSM_STATE_APPS_DEM),
		       raw_smsm_get_state(SMSM_STATE_MODEM_DEM),
		       raw_smsm_get_state(SMSM_STATE_QDSP6_DEM),
		       raw_smsm_get_state(SMSM_STATE_POWER_MASTER_DEM),
		       raw_smsm_get_state(SMSM_STATE_TIME_MASTER_DEM));
#endif
	if (msg) {
		msg[SZ_DIAG_ERR_MSG - 1] = 0;
		i += scnprintf(buf + i, max - i, "diag: '%s'\n", msg);
	}

#if CONFIG_SMD_OFFSET_TCXO_STAT
	if (sleep_stat) {
		i += scnprintf(buf + i, max - i,
			"tcxo_time: 0x%x (%ds) tcxo_cnt: %d\n"
			"suspend_tcxo_time: 0x%x (%ds) suspend_tcxo_cnt: %d\n"
			"garbage_pkt_cnt: %d "
			"zone_based_reg_cnt: %d "
			"idle_hand_off_cnt: %d "
			"mo_2g_probe_cnt: %d "
			"mo_3g_probe_cnt: %d\n",
			sleep_stat->tcxo_time, sleep_stat->tcxo_time >> 15,
			sleep_stat->tcxo_cnt, sleep_stat->suspend_tcxo_time,
			sleep_stat->suspend_tcxo_time >> 15,
			sleep_stat->suspend_tcxo_cnt,
			sleep_stat->garbage_pkt_cnt,
			sleep_stat->zone_based_reg_cnt,
			sleep_stat->idle_hand_off_cnt,
			sleep_stat->mo_2g_probe_cnt,
			sleep_stat->mo_3g_probe_cnt);
	}
#endif

	return i;
}

static int debug_read_mem(char *buf, int max)
{
	unsigned n;
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	struct smem_heap_entry *toc = shared->heap_toc;
	int i = 0;

	i += scnprintf(buf + i, max - i,
		       "heap: init=%d free=%d remain=%d\n",
		       shared->heap_info.initialized,
		       shared->heap_info.free_offset,
		       shared->heap_info.heap_remaining);

	for (n = 0; n < SMEM_NUM_ITEMS; n++) {
		if (toc[n].allocated == 0)
			continue;
		i += scnprintf(buf + i, max - i,
			       "%04d: offset %08x size %08x\n",
			       n, toc[n].offset, toc[n].size);
	}
	return i;
}

static int debug_read_ch(char *buf, int max)
{
	struct smd_channel *ch;
	unsigned long flags;
	int i = 0;

	spin_lock_irqsave(&smd_lock, flags);
	list_for_each_entry(ch, &smd_ch_list_dsp, ch_list)
		i += dump_ch(buf + i, max - i, ch);
	list_for_each_entry(ch, &smd_ch_list_modem, ch_list)
		i += dump_ch(buf + i, max - i, ch);
	list_for_each_entry(ch, &smd_ch_closed_list, ch_list)
		i += dump_ch(buf + i, max - i, ch);
	spin_unlock_irqrestore(&smd_lock, flags);

	return i;
}

static int debug_read_version(char *buf, int max)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	unsigned version = shared->version[VERSION_MODEM];
	return sprintf(buf, "%d.%d\n", version >> 16, version & 0xffff);
}

static int debug_read_build_id(char *buf, int max)
{
	unsigned size;
	void *data;

	data = smem_item(SMEM_HW_SW_BUILD_ID, &size);
	if (!data)
		return 0;

	if (size >= max)
		size = max;
	memcpy(buf, data, size);

	return size;
}

static int debug_read_alloc_tbl(char *buf, int max)
{
	struct smd_alloc_elm *shared;
	int n, i = 0;

	shared = smem_find(ID_CH_ALLOC_TBL, sizeof(*shared) * 64);
	if (!shared) {
		pr_err("smd: cannot find allocation table\n");
		return 0;
	}

	for (n = 0; n < 64; n++) {
		if (shared[n].ref_count == 0)
			continue;
		i += scnprintf(buf + i, max - i,
			       "%03d: %-20s cid=%02d type=%03d "
			       "kind=%02d ref_count=%d\n",
			       n, shared[n].name, shared[n].cid,
			       shared[n].ctype & 0xff,
			       (shared[n].ctype >> 8) & 0xf,
			       shared[n].ref_count);
	}

	return i;
}

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
			 struct dentry *dent,
			 int (*fill)(char *buf, int max))
{
	debugfs_create_file(name, mode, dent, fill, &debug_ops);
}

enum {
	TCXO_TIME = 0,
	TCXO_CNT,
	SUSPEND_TCXO_TIME,
	SUSPEND_TCXO_CNT,
	GARBAGE_CNT,
	GARBAGE_PKT_CNT,
	CELL_SWT_CNT,
	ZONE_BASED_REG_CNT,
	IDLE_HAND_OFF_CNT,
	MO_2G_PROBE_CNT,
	MO_3G_PROBE_CNT,
};

#define SLEEP_STAT_ATTR(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },  \
	.show = show_mem_sleep_stat_attr,                  \
	.store = NULL,                              \
}

static struct mem_sleep_stat_attr mem_sleep_stat_attrs[] = {
	SLEEP_STAT_ATTR(tcxo_time),
	SLEEP_STAT_ATTR(tcxo_cnt),
	SLEEP_STAT_ATTR(suspend_tcxo_time),
	SLEEP_STAT_ATTR(suspend_tcxo_cnt),
	SLEEP_STAT_ATTR(garbage_cnt),
	SLEEP_STAT_ATTR(garbage_pkt_cnt),
	SLEEP_STAT_ATTR(cell_swt_cnt),
	SLEEP_STAT_ATTR(zone_based_reg_cnt),
	SLEEP_STAT_ATTR(idle_hand_off_cnt),
	SLEEP_STAT_ATTR(mo_2g_probe_cnt),
	SLEEP_STAT_ATTR(mo_3g_probe_cnt),
};

/**
 * show_mem_sleep_stat_attr - current MEM Sleep status attributes
 */
static ssize_t show_mem_sleep_stat_attr(struct device *dev,
                      struct mem_sleep_stat_attr *attr,
                      char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - mem_sleep_stat_attrs;

	if (!sleep_stat) {
		pr_err("%s: sleep_stat is NULL", __func__);
		return sprintf(buf, "%d\n", 0);
	}

	pr_info("%s: mem_sleep_stat_attr: %s", __func__, attr->attr.name);

	mutex_lock(&mem_sleep_stat_lock);
	switch (off) {
	case TCXO_TIME:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->tcxo_time);
		break;
	case TCXO_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->tcxo_cnt);
		break;
	case SUSPEND_TCXO_TIME:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->suspend_tcxo_time);
		break;
	case SUSPEND_TCXO_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->suspend_tcxo_cnt);
		break;
	case GARBAGE_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->garbage_cnt);
		break;
	case GARBAGE_PKT_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->garbage_pkt_cnt);
		break;
	case CELL_SWT_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->cell_swt_cnt);
		break;
	case ZONE_BASED_REG_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->zone_based_reg_cnt);
		break;
	case IDLE_HAND_OFF_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->idle_hand_off_cnt);
		break;
	case MO_2G_PROBE_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->mo_2g_probe_cnt);
		break;
	case MO_3G_PROBE_CNT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
						sleep_stat->mo_3g_probe_cnt);
		break;
	default:
		i = -EINVAL;
	}
	mutex_unlock(&mem_sleep_stat_lock);

	if (i < 0)
		pr_err("%s: attribute is not supported: %d", __func__, off);

	return i;
}

static int __init smd_debugfs_init(void)
{
	struct dentry *dent;
	int ret;
	int i;

	dent = debugfs_create_dir("smd", 0);
	if (IS_ERR(dent))
		return -1;

	debug_create("ch", 0444, dent, debug_read_ch);
	debug_create("stat", 0444, dent, debug_read_stat);
	debug_create("mem", 0444, dent, debug_read_mem);
	debug_create("version", 0444, dent, debug_read_version);
	debug_create("tbl", 0444, dent, debug_read_alloc_tbl);
	debug_create("build", 0444, dent, debug_read_build_id);
#if CONFIG_SMD_OFFSET_TCXO_STAT
	sleep_stat = get_smem_sleep_stat();
	negate_client_stat = get_smem_negate_client_stat();
	register_early_suspend(&sleep_stat_screen_hdl);
	register_pm_notifier(&sleep_stat_notif_block);

	mutex_init(&mem_sleep_stat_lock);
	sleep_stat_kobj = kobject_create_and_add("systemlog", NULL);
	if (sleep_stat_kobj == NULL) {
		pr_err("smd_debugfs_init: create sleep_stat_kobj failed\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(mem_sleep_stat_attrs); i++) {
		ret = sysfs_create_file(sleep_stat_kobj, &mem_sleep_stat_attrs[i].attr);
		if (ret)
			pr_err("%s: sysfs_create_file for attr %d failed\n", __func__, i);
	}
#else
	pr_info("No sleep statistics\n");
#endif
	return 0;
}

late_initcall(smd_debugfs_init);
#endif


#define MAX_NUM_SLEEP_CLIENTS		64
#define MAX_SLEEP_NAME_LEN		8

#define NUM_GPIO_INT_REGISTERS		6
#define GPIO_SMEM_NUM_GROUPS		2
#define GPIO_SMEM_MAX_PC_INTERRUPTS	8

struct tramp_gpio_save {
	unsigned int enable;
	unsigned int detect;
	unsigned int polarity;
};

struct tramp_gpio_smem {
	uint16_t num_fired[GPIO_SMEM_NUM_GROUPS];
	uint16_t fired[GPIO_SMEM_NUM_GROUPS][GPIO_SMEM_MAX_PC_INTERRUPTS];
	uint32_t enabled[NUM_GPIO_INT_REGISTERS];
	uint32_t detection[NUM_GPIO_INT_REGISTERS];
	uint32_t polarity[NUM_GPIO_INT_REGISTERS];
};


void smsm_print_sleep_info(unsigned wakeup_reason_only)
{
	unsigned long flags;
	uint32_t *ptr;
#if defined(CONFIG_MSM_N_WAY_SMD)
	struct msm_dem_slave_data *smd_int_info;
#else
	struct tramp_gpio_smem *gpio;
	struct smsm_interrupt_info *int_info;
#endif

	spin_lock_irqsave(&smem_lock, flags);

	if (!wakeup_reason_only) {
		ptr = smem_alloc(SMEM_SMSM_SLEEP_DELAY, sizeof(*ptr));
		if (ptr)
			pr_info("SMEM_SMSM_SLEEP_DELAY: %x\n", *ptr);

		ptr = smem_alloc(SMEM_SMSM_LIMIT_SLEEP, sizeof(*ptr));
		if (ptr)
			pr_info("SMEM_SMSM_LIMIT_SLEEP: %x\n", *ptr);

		ptr = smem_alloc(SMEM_SLEEP_POWER_COLLAPSE_DISABLED, sizeof(*ptr));
		if (ptr)
			pr_info("SMEM_SLEEP_POWER_COLLAPSE_DISABLED: %x\n", *ptr);
	}
#if !defined(CONFIG_MSM_N_WAY_SMD)
	int_info = smem_alloc(SMEM_SMSM_INT_INFO, sizeof(*int_info));
	if (int_info)
		pr_info("SMEM_SMSM_INT_INFO %x %x %x\n",
			int_info->interrupt_mask,
			int_info->pending_interrupts,
			int_info->wakeup_reason);

	gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*gpio));
	if (gpio) {
		int i;
		if (!wakeup_reason_only) {
			for (i = 0; i < NUM_GPIO_INT_REGISTERS; i++)
				pr_info("SMEM_GPIO_INT: %d: e %x d %x p %x\n",
					i, gpio->enabled[i], gpio->detection[i],
					gpio->polarity[i]);
		}
		for (i = 0; i < GPIO_SMEM_NUM_GROUPS; i++)
			pr_info("SMEM_GPIO_INT: %d: f %d: %d %d...\n",
				i, gpio->num_fired[i], gpio->fired[i][0],
				gpio->fired[i][1]);
	}
#else
	smd_int_info = smem_find(SMEM_APPS_DEM_SLAVE_DATA, sizeof(*smd_int_info));
	if (smd_int_info) {
		pr_info("SMEM_APPS_DEM_SLAVE_DATA: %ds %x %x %x %x %x %x %x %s %x\n",
			smd_int_info->sleep_time / 32768,
			smd_int_info->interrupt_mask,
			smd_int_info->resources_used,
			smd_int_info->reserved1,
			smd_int_info->wakeup_reason,
			smd_int_info->pending_interrupts,
			smd_int_info->rpc_prog,
			smd_int_info->rpc_proc,
			smd_int_info->smd_port_name,
			smd_int_info->reserved2);
	}
#endif
	spin_unlock_irqrestore(&smem_lock, flags);
}

