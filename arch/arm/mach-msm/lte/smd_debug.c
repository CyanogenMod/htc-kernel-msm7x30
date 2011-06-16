/* arch/arm/mach-msm/smd_debug.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/ctype.h>
#include <linux/earlysuspend.h>
#include <linux/rtc.h>
#include <linux/suspend.h>

#include <mach/msm_iomap.h>

#include "smd_private.h"

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
	uint32_t reserved[5];
};

struct smem_negate_client {
	uint32_t htc_try_to_tcxo_cnt_during_suspend;
	uint32_t htc_insuff_time_count;
	uint32_t reserved;
	uint32_t htc_total_sleep_clients;
	uint32_t htc_negate_tcxo_client[25];
};

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
		sleep_stat->mo_3g_probe_cnt, sleep_stat->reserved[0],
		sleep_stat->reserved[1], sleep_stat->reserved[2],
		sleep_stat->reserved[3], sleep_stat->reserved[4],
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

	pr_info("negate_client_stat: %d %d %d - %d %d %d %d %d - %d %d %d %d %d - %d %d %d %d %d - %d %d %d %d %d - %d %d %d %d %d\n",
		negate_client_stat->htc_try_to_tcxo_cnt_during_suspend,
		negate_client_stat->htc_insuff_time_count,
		negate_client_stat->htc_total_sleep_clients,
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
		negate_client_stat->htc_negate_tcxo_client[16],
		negate_client_stat->htc_negate_tcxo_client[17],
		negate_client_stat->htc_negate_tcxo_client[18],
		negate_client_stat->htc_negate_tcxo_client[19],
		negate_client_stat->htc_negate_tcxo_client[20],
		negate_client_stat->htc_negate_tcxo_client[21],
		negate_client_stat->htc_negate_tcxo_client[22],
		negate_client_stat->htc_negate_tcxo_client[23],
		negate_client_stat->htc_negate_tcxo_client[24]);
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

static int debug_f3(char *buf, int max)
{
	char *x;
	int size;
	int i = 0, j = 0;
	unsigned cols = 0;
	char str[4*sizeof(unsigned)+1] = {0};

	i += scnprintf(buf + i, max - i,
		       "Printing to log\n");

	x = smem_get_entry(SMEM_ERR_F3_TRACE_LOG, &size);
	if (x != 0) {
		pr_info("smem: F3 TRACE LOG\n");
		while (size > 0) {
			if (size >= sizeof(unsigned)) {
				pr_info("%08x", *((unsigned *) x));
				for (j = 0; j < sizeof(unsigned); ++j)
					if (isprint(*(x+j)))
						str[cols*sizeof(unsigned) + j]
							= *(x+j);
					else
						str[cols*sizeof(unsigned) + j]
							= '-';
				x += sizeof(unsigned);
				size -= sizeof(unsigned);
			} else {
				while (size-- > 0)
					pr_info("%02x", (unsigned) *x++);
				break;
			}
			if (cols == 3) {
				cols = 0;
				str[4*sizeof(unsigned)] = 0;
				pr_info(" %s\n", str);
				str[0] = 0;
			} else {
				cols++;
				pr_info(" ");
			}
		}
		pr_info("\n");
	}

	return max;
}

static int debug_diag(char *buf, int max)
{
	int i = 0;

	i += scnprintf(buf + i, max - i,
		       "Printing to log\n");
	smd_diag();

	return i;
}

static int debug_modem_err_f3(char *buf, int max)
{
	char *x;
	int size;
	int i = 0, j = 0;
	unsigned cols = 0;
	char str[4*sizeof(unsigned)+1] = {0};

	x = smem_get_entry(SMEM_ERR_F3_TRACE_LOG, &size);
	if (x != 0) {
		pr_info("smem: F3 TRACE LOG\n");
		while (size > 0 && max - i) {
			if (size >= sizeof(unsigned)) {
				i += scnprintf(buf + i, max - i, "%08x",
					       *((unsigned *) x));
				for (j = 0; j < sizeof(unsigned); ++j)
					if (isprint(*(x+j)))
						str[cols*sizeof(unsigned) + j]
							= *(x+j);
					else
						str[cols*sizeof(unsigned) + j]
							= '-';
				x += sizeof(unsigned);
				size -= sizeof(unsigned);
			} else {
				while (size-- > 0 && max - i)
					i += scnprintf(buf + i, max - i,
						       "%02x",
						       (unsigned) *x++);
				break;
			}
			if (cols == 3) {
				cols = 0;
				str[4*sizeof(unsigned)] = 0;
				i += scnprintf(buf + i, max - i, " %s\n",
					       str);
				str[0] = 0;
			} else {
				cols++;
				i += scnprintf(buf + i, max - i, " ");
			}
		}
		i += scnprintf(buf + i, max - i, "\n");
	}

	return i;
}

static int debug_modem_err(char *buf, int max)
{
	char *x;
	int size;
	int i = 0;

	x = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);
	if (x != 0) {
		x[SZ_DIAG_ERR_MSG - 1] = 0;
		i += scnprintf(buf + i, max - i,
			       "smem: DIAG '%s'\n", x);
	}

	x = smem_get_entry(SMEM_ERR_CRASH_LOG, &size);
	if (x != 0) {
		x[size - 1] = 0;
		i += scnprintf(buf + i, max - i,
			       "smem: CRASH LOG\n'%s'\n", x);
	}
	i += scnprintf(buf + i, max - i, "\n");

	return i;
}

static int debug_read_diag_msg(char *buf, int max)
{
	char *msg;
	int i = 0;

	msg = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);

	if (msg) {
		msg[SZ_DIAG_ERR_MSG - 1] = 0;
		i += scnprintf(buf + i, max - i, "diag: '%s'\n", msg);
	}
	return i;
}

static int dump_ch(char *buf, int max, int n,
		  struct smd_half_channel *s,
		   struct smd_half_channel *r,
		   unsigned size)
{
	return scnprintf(
		buf, max,
		"ch%02d:"
		" %8s(%04d/%04d) %c%c%c%c%c%c%c <->"
		" %8s(%04d/%04d) %c%c%c%c%c%c%c : %5x\n", n,
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
		size
		);
}

static int debug_read_smsm_state(char *buf, int max)
{
	uint32_t *smsm;
	int n, i = 0;

	smsm = smem_find(ID_SHARED_STATE,
			 SMSM_NUM_ENTRIES * sizeof(uint32_t));

	if (smsm)
		for (n = 0; n < SMSM_NUM_ENTRIES; n++)
			i += scnprintf(buf + i, max - i, "entry %d: 0x%08x\n",
				       n, smsm[n]);

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

static int debug_read_ch_v1(char *buf, int max)
{
	void *shared;
	int n, i = 0;

	for (n = 0; n < SMD_CHANNELS; n++) {
		shared = smem_find(ID_SMD_CHANNELS + n,
				   2 * (sizeof(struct smd_half_channel) +
					SMD_BUF_SIZE));

		if (shared == 0)
			continue;
		i += dump_ch(buf + i, max - i, n, shared,
			     (shared + sizeof(struct smd_half_channel) +
			      SMD_BUF_SIZE), SMD_BUF_SIZE);
	}

	return i;
}

static int debug_read_ch_v2(char *buf, int max)
{
	void *shared, *buffer;
	unsigned buffer_sz;
	int n, i = 0;

	for (n = 0; n < SMD_CHANNELS; n++) {
		shared = smem_find(ID_SMD_CHANNELS + n,
				   2 * sizeof(struct smd_half_channel));

		if (shared == 0)
			continue;

		buffer = smem_get_entry(SMEM_SMD_FIFO_BASE_ID + n, &buffer_sz);

		if (buffer == 0)
			continue;

		i += dump_ch(buf + i, max - i, n, shared,
			     (shared + sizeof(struct smd_half_channel)),
			     buffer_sz / 2);
	}

	return i;
}

static int debug_read_ch(char *buf, int max)
{
	uint32_t *smd_ver;

	smd_ver = smem_alloc(SMEM_VERSION_SMD, 32 * sizeof(uint32_t));

	if (smd_ver && (((smd_ver[VERSION_MODEM] >> 16) >= 1) ||
			((smd_ver[VERSION_QDSP6] >> 16) >= 1) ||
			((smd_ver[VERSION_DSPS] >> 16) >= 1)))
		return debug_read_ch_v2(buf, max);
	else
		return debug_read_ch_v1(buf, max);
}

static int debug_read_smem_version(char *buf, int max)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	uint32_t n, version, i = 0;

	for (n = 0; n < 32; n++) {
		version = shared->version[n];
		i += scnprintf(buf + i, max - i,
			       "entry %d: smem = %d  proc_comm = %d\n", n,
			       version >> 16,
			       version & 0xffff);
	}

	return i;
}

/* NNV: revist, it may not be smd version */
static int debug_read_smd_version(char *buf, int max)
{
	uint32_t *smd_ver;
	uint32_t n, version, i = 0;

	smd_ver = smem_alloc(SMEM_VERSION_SMD, 32 * sizeof(uint32_t));

	if (smd_ver)
		for (n = 0; n < 32; n++) {
			version = smd_ver[n];
			i += scnprintf(buf + i, max - i,
				       "entry %d: %d.%d\n", n,
				       version >> 16,
				       version & 0xffff);
		}

	return i;
}

static int debug_read_build_id(char *buf, int max)
{
	unsigned size;
	void *data;

	data = smem_get_entry(SMEM_HW_SW_BUILD_ID, &size);
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

	shared = smem_find(ID_CH_ALLOC_TBL, sizeof(struct smd_alloc_elm[64]));

	if (!shared)
		return 0;

	for (n = 0; n < 64; n++) {
		i += scnprintf(buf + i, max - i,
				"name=%s cid=%d ch type=%d "
				"xfer type=%d ref_count=%d\n",
				shared[n].name,
				shared[n].cid,
				SMD_CHANNEL_TYPE(shared[n].type),
				SMD_XFER_TYPE(shared[n].type),
				shared[n].ref_count);
	}

	return i;
}

static int debug_read_intr_mask(char *buf, int max)
{
	uint32_t *smsm;
	int m, n, i = 0;

	smsm = smem_alloc(SMEM_SMSM_CPU_INTR_MASK,
			  SMSM_NUM_ENTRIES * SMSM_NUM_HOSTS * sizeof(uint32_t));

	if (smsm)
		for (m = 0; m < SMSM_NUM_ENTRIES; m++) {
			i += scnprintf(buf + i, max - i, "entry %d:", m);
			for (n = 0; n < SMSM_NUM_HOSTS; n++)
				i += scnprintf(buf + i, max - i,
					       "   host %d: 0x%08x",
					       n, smsm[m * SMSM_NUM_HOSTS + n]);
			i += scnprintf(buf + i, max - i, "\n");
		}

	return i;
}

static int debug_read_intr_mux(char *buf, int max)
{
	uint32_t *smsm;
	int n, i = 0;

	smsm = smem_alloc(SMEM_SMD_SMSM_INTR_MUX,
			  SMSM_NUM_INTR_MUX * sizeof(uint32_t));

	if (smsm)
		for (n = 0; n < SMSM_NUM_INTR_MUX; n++)
			i += scnprintf(buf + i, max - i, "entry %d: %d\n",
				       n, smsm[n]);

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

static int __init smd_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("smd", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debug_create("ch", 0444, dent, debug_read_ch);
	debug_create("diag", 0444, dent, debug_read_diag_msg);
	debug_create("mem", 0444, dent, debug_read_mem);
	debug_create("version", 0444, dent, debug_read_smd_version);
	debug_create("tbl", 0444, dent, debug_read_alloc_tbl);
	debug_create("modem_err", 0444, dent, debug_modem_err);
	debug_create("modem_err_f3", 0444, dent, debug_modem_err_f3);
	debug_create("print_diag", 0444, dent, debug_diag);
	debug_create("print_f3", 0444, dent, debug_f3);

	/* NNV: this is google only stuff */
	debug_create("build", 0444, dent, debug_read_build_id);

#if CONFIG_SMD_OFFSET_TCXO_STAT
	sleep_stat = get_smem_sleep_stat();
	negate_client_stat = get_smem_negate_client_stat();
	register_early_suspend(&sleep_stat_screen_hdl);
	register_pm_notifier(&sleep_stat_notif_block);
#else
	pr_info("No sleep statistics\n");
#endif
	return 0;
}

static int __init smsm_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("smsm", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debug_create("state", 0444, dent, debug_read_smsm_state);
	debug_create("intr_mask", 0444, dent, debug_read_intr_mask);
	debug_create("intr_mux", 0444, dent, debug_read_intr_mux);
	debug_create("version", 0444, dent, debug_read_smem_version);

	return 0;
}

late_initcall(smd_debugfs_init);
late_initcall(smsm_debugfs_init);
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

#if 0	/* Changed by Andy for HTC pm.c */
/*
 * Print debug information on shared memory sleep variables
 */
void smsm_print_sleep_info(uint32_t sleep_delay, uint32_t sleep_limit,
	uint32_t irq_mask, uint32_t wakeup_reason, uint32_t pending_irqs)
{
	unsigned long flags;
	uint32_t *ptr;
	struct tramp_gpio_smem *gpio;

	spin_lock_irqsave(&smem_lock, flags);

	pr_info("SMEM_SMSM_SLEEP_DELAY: %x\n", sleep_delay);
	pr_info("SMEM_SMSM_LIMIT_SLEEP: %x\n", sleep_limit);

	ptr = smem_alloc(SMEM_SLEEP_POWER_COLLAPSE_DISABLED, sizeof(*ptr));
	if (ptr)
		pr_info("SMEM_SLEEP_POWER_COLLAPSE_DISABLED: %x\n", *ptr);
	else
		pr_info("SMEM_SLEEP_POWER_COLLAPSE_DISABLED: missing\n");

	pr_info("SMEM_SMSM_INT_INFO %x %x %x\n",
		irq_mask, pending_irqs, wakeup_reason);

	gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*gpio));
	if (gpio) {
		int i;
		for (i = 0; i < NUM_GPIO_INT_REGISTERS; i++)
			pr_info("SMEM_GPIO_INT: %d: e %x d %x p %x\n",
				i, gpio->enabled[i], gpio->detection[i],
				gpio->polarity[i]);

		for (i = 0; i < GPIO_SMEM_NUM_GROUPS; i++)
			pr_info("SMEM_GPIO_INT: %d: f %d: %d %d...\n",
				i, gpio->num_fired[i], gpio->fired[i][0],
				gpio->fired[i][1]);
	} else
		pr_info("SMEM_GPIO_INT: missing\n");

	spin_unlock_irqrestore(&smem_lock, flags);
}
#else
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
#endif
