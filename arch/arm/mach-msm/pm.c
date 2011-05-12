/* arch/arm/mach-msm/pm.c
 *
 * MSM Power Management Routines
 *
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/earlysuspend.h>
#include <linux/pm_qos_params.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <asm/io.h>
#ifdef CONFIG_VFP
#include <asm/vfp.h>
#endif

#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include "smd_private.h"
#include "smd_rpcrouter.h"
#include "acpuclock.h"
#include "proc_comm.h"
#include "clock.h"
#include "spm.h"
#include "pm.h"
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

enum {
	MSM_PM_DEBUG_SUSPEND = 1U << 0,
	MSM_PM_DEBUG_POWER_COLLAPSE = 1U << 1,
	MSM_PM_DEBUG_STATE = 1U << 2,
	MSM_PM_DEBUG_CLOCK = 1U << 3,
	MSM_PM_DEBUG_RESET_VECTOR = 1U << 4,
	MSM_PM_DEBUG_SMSM_STATE = 1U << 5,
	MSM_PM_DEBUG_IDLE = 1U << 6,
	MSM_PM_DEBUG_CLOCK_VOTE = 1U << 7,
	MSM_PM_DEBUG_WAKEUP_REASON = 1U << 8,
};
static int msm_pm_debug_mask = MSM_PM_DEBUG_CLOCK_VOTE | MSM_PM_DEBUG_WAKEUP_REASON;
module_param_named(debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static int msm_pm_sleep_mode = CONFIG_MSM7X00A_SLEEP_MODE;
module_param_named(sleep_mode, msm_pm_sleep_mode, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int msm_pm_idle_sleep_mode = CONFIG_MSM7X00A_IDLE_SLEEP_MODE;
module_param_named(idle_sleep_mode, msm_pm_idle_sleep_mode, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int msm_pm_idle_sleep_min_time = CONFIG_MSM7X00A_IDLE_SLEEP_MIN_TIME;
module_param_named(idle_sleep_min_time, msm_pm_idle_sleep_min_time, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int msm_pm_idle_spin_time = CONFIG_MSM7X00A_IDLE_SPIN_TIME;
module_param_named(idle_spin_time, msm_pm_idle_spin_time, int, S_IRUGO | S_IWUSR | S_IWGRP);

static int idle_sleep_mode_def = CONFIG_MSM7X00A_IDLE_SLEEP_MODE;
static int param_set_idle_sleep_mode_def(const char *val,
						struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	msm_pm_idle_sleep_mode = idle_sleep_mode_def;
	return ret;
}
module_param_call(idle_sleep_mode_def, param_set_idle_sleep_mode_def,
	param_get_int, &idle_sleep_mode_def, S_IWUSR | S_IRUGO);

#ifdef CONFIG_HTC_OFFMODE_ALARM
static int offalarm_size = 10;
static unsigned int offalarm[10];
module_param_array_named(offalarm, offalarm, uint, &offalarm_size,
			S_IRUGO | S_IWUSR);
#endif

#if defined(CONFIG_ARCH_MSM7X30)
#define A11S_CLK_SLEEP_EN (MSM_GCC_BASE + 0x020)
#define A11S_PWRDOWN      (MSM_ACC_BASE + 0x01c)
#define A11S_SECOP        (MSM_TCSR_BASE + 0x038)
#else
#define A11S_CLK_SLEEP_EN (MSM_CSR_BASE + 0x11c)
#define A11S_PWRDOWN (MSM_CSR_BASE + 0x440)
#define A11S_STANDBY_CTL (MSM_CSR_BASE + 0x108)
#define A11RAMBACKBIAS (MSM_CSR_BASE + 0x508)
#endif

#if defined(CONFIG_MSM_N_WAY_SMD)
#define DEM_MASTER_BITS_PER_CPU             6

/* Power Master State Bits - Per CPU */
#define DEM_MASTER_SMSM_RUN \
        (0x01UL << (DEM_MASTER_BITS_PER_CPU * SMSM_STATE_APPS))
#define DEM_MASTER_SMSM_RSA \
        (0x02UL << (DEM_MASTER_BITS_PER_CPU * SMSM_STATE_APPS))
#define DEM_MASTER_SMSM_PWRC_EARLY_EXIT \
        (0x04UL << (DEM_MASTER_BITS_PER_CPU * SMSM_STATE_APPS))
#define DEM_MASTER_SMSM_SLEEP_EXIT \
        (0x08UL << (DEM_MASTER_BITS_PER_CPU * SMSM_STATE_APPS))
#define DEM_MASTER_SMSM_READY \
        (0x10UL << (DEM_MASTER_BITS_PER_CPU * SMSM_STATE_APPS))
#define DEM_MASTER_SMSM_SLEEP \
        (0x20UL << (DEM_MASTER_BITS_PER_CPU * SMSM_STATE_APPS))

/* Power Slave State Bits */
#define DEM_SLAVE_SMSM_RUN                  (0x0001)
#define DEM_SLAVE_SMSM_PWRC                 (0x0002)
#define DEM_SLAVE_SMSM_PWRC_DELAY           (0x0004)
#define DEM_SLAVE_SMSM_PWRC_EARLY_EXIT      (0x0008)
#define DEM_SLAVE_SMSM_WFPI                 (0x0010)
#define DEM_SLAVE_SMSM_SLEEP                (0x0020)
#define DEM_SLAVE_SMSM_SLEEP_EXIT           (0x0040)
#define DEM_SLAVE_SMSM_MSGS_REDUCED         (0x0080)
#define DEM_SLAVE_SMSM_RESET                (0x0100)
#define DEM_SLAVE_SMSM_PWRC_SUSPEND         (0x0200)

#define PM_SMSM_WRITE_STATE	SMSM_STATE_APPS_DEM
#define PM_SMSM_READ_STATE	SMSM_STATE_POWER_MASTER_DEM

#define PM_SMSM_WRITE_RUN	DEM_SLAVE_SMSM_RUN
#define PM_SMSM_READ_RUN	DEM_MASTER_SMSM_RUN
#else
#define PM_SMSM_WRITE_STATE     SMSM_STATE_APPS
#define PM_SMSM_READ_STATE      SMSM_STATE_MODEM

#define PM_SMSM_WRITE_RUN       SMSM_RUN
#define PM_SMSM_READ_RUN        SMSM_RUN
#endif

int msm_pm_collapse(void);
int msm_arch_idle(void);
void msm_pm_collapse_exit(void);

int64_t msm_timer_enter_idle(void);
void msm_timer_exit_idle(int low_power);
int msm_irq_idle_sleep_allowed(void);
int msm_irq_pending(void);
int clks_allow_tcxo_locked_debug(void);
extern int board_mfg_mode(void);
extern char * board_get_mfg_sleep_gpio_table(void);
extern void gpio_set_diag_gpio_table(unsigned long * dwMFG_gpio_table);
extern void wait_rmt_final_call_back(int timeout);

#ifdef CONFIG_AXI_SCREEN_POLICY
static int axi_rate;
static int sleep_axi_rate;
static struct clk *axi_clk;
#endif
static uint32_t *msm_pm_reset_vector;

static uint32_t msm_pm_max_sleep_time;
static struct msm_pm_platform_data msm_pm_mode_default[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,
	/*
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].supported = 1,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].latency = 8594,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].latency = 500,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].residency = 6000,
	*/
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};
static struct msm_pm_platform_data *msm_pm_modes = msm_pm_mode_default;

#ifdef CONFIG_MSM_IDLE_STATS
enum msm_pm_time_stats_id {
	MSM_PM_STAT_REQUESTED_IDLE,
	MSM_PM_STAT_IDLE_SPIN,
	MSM_PM_STAT_IDLE_WFI,
	MSM_PM_STAT_IDLE_SLEEP,
	MSM_PM_STAT_IDLE_FAILED_SLEEP,
	MSM_PM_STAT_NOT_IDLE,
	MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_FAILED_STANDALONE_POWER_COLLAPSE,
	MSM_PM_STAT_COUNT
};

static struct msm_pm_time_stats {
	const char *name;
	int bucket[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t min_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t max_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int count;
	int64_t total_time;
} msm_pm_stats[MSM_PM_STAT_COUNT] = {
	[MSM_PM_STAT_REQUESTED_IDLE].name = "idle-request",
	[MSM_PM_STAT_IDLE_SPIN].name = "idle-spin",
	[MSM_PM_STAT_IDLE_WFI].name = "idle-wfi",
	[MSM_PM_STAT_IDLE_SLEEP].name = "idle-sleep",
	[MSM_PM_STAT_IDLE_FAILED_SLEEP].name = "idle-failed-sleep",
	[MSM_PM_STAT_NOT_IDLE].name = "not-idle",
	[MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE].name =
		"idle-standalone-power-collapse",
	[MSM_PM_STAT_IDLE_FAILED_STANDALONE_POWER_COLLAPSE].name =
		"idle-failed-standalone-power-collapse",
};

void __init msm_pm_set_platform_data(struct msm_pm_platform_data *data)
{
	msm_pm_modes = data;
}

static void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t)
{
	int i;
	int64_t bt;
	msm_pm_stats[id].total_time += t;
	msm_pm_stats[id].count++;
	bt = t;
	do_div(bt, CONFIG_MSM_IDLE_STATS_FIRST_BUCKET);
	if (bt < 1ULL << (CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT *
				(CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1)))
		i = DIV_ROUND_UP(fls((uint32_t)bt),
					CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT);
	else
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;
	msm_pm_stats[id].bucket[i]++;
	if (t < msm_pm_stats[id].min_time[i] || !msm_pm_stats[id].max_time[i])
		msm_pm_stats[id].min_time[i] = t;
	if (t > msm_pm_stats[id].max_time[i])
		msm_pm_stats[id].max_time[i] = t;
}
#endif

static int
msm_pm_wait_state(uint32_t wait_all_set, uint32_t wait_all_clear,
                  uint32_t wait_any_set, uint32_t wait_any_clear)
{
	int i;
	uint32_t state;

	for (i = 0; i < 100000; i++) {
		state = smsm_get_state(PM_SMSM_READ_STATE);
		if (((wait_all_set || wait_all_clear) && 
		     !(~state & wait_all_set) && !(state & wait_all_clear)) ||
		    (state & wait_any_set) || (~state & wait_any_clear))
			return 0;
		udelay(1);
	}
	pr_err("msm_pm_wait_state(%x, %x, %x, %x) failed %x\n",	wait_all_set,
		wait_all_clear, wait_any_set, wait_any_clear, state);
	return -ETIMEDOUT;
}

/*
 * For speeding up boot time:
 * During booting up, disable entering arch_idle() by disable_hlt()
 * Enable it after booting up BOOT_LOCK_TIMEOUT sec.
 */
#define BOOT_LOCK_TIMEOUT_NORMAL      (60 * HZ)
#define BOOT_LOCK_TIMEOUT_SHORT      (10 * HZ)
static void do_expire_boot_lock(struct work_struct *work)
{
	enable_hlt();
	pr_info("Release 'boot-time' no_halt_lock\n");
}
static DECLARE_DELAYED_WORK(work_expire_boot_lock, do_expire_boot_lock);

/*
 * Configure hardware registers in preparation for Apps power down.
 */
static void
msm_pm_enter_prep_hw(void)
{
#if defined(CONFIG_ARCH_MSM7X30)
	writel(1, A11S_PWRDOWN);
	writel(4, A11S_SECOP);
#elif defined(CONFIG_ARCH_MSM7X27)
	writel(0x1f, A11S_CLK_SLEEP_EN);
	writel(1, A11S_PWRDOWN);
	writel(0, A11S_STANDBY_CTL);
#elif defined(CONFIG_ARCH_QSD8X50)
	writel(0x1f, A11S_CLK_SLEEP_EN);
	writel(1, A11S_PWRDOWN);
	writel(0, A11S_STANDBY_CTL);
#else /* 7X00/7X25 */
	writel(0x1f, A11S_CLK_SLEEP_EN);
	writel(1, A11S_PWRDOWN);
	writel(0, A11S_STANDBY_CTL);
	writel(0, A11RAMBACKBIAS);
#endif
}

/*
 * Clear hardware registers after Apps powers up.
 */
static void
msm_pm_exit_restore_hw(void)
{
#if defined(CONFIG_ARCH_MSM7X30)
	writel(0, A11S_SECOP);
	writel(0, A11S_PWRDOWN);
	msm_spm_reinit();
#else
	writel(0, A11S_CLK_SLEEP_EN);
	writel(0, A11S_PWRDOWN);
#endif
}

#ifdef CONFIG_MSM_FIQ_SUPPORT
void msm_fiq_exit_sleep(void);
#else
static inline void msm_fiq_exit_sleep(void) { }
#endif


static int msm_sleep(int sleep_mode, uint32_t sleep_delay, int from_idle)
{
	uint32_t saved_vector[2];
	int collapsed;
	void msm_irq_enter_sleep1(bool arm9_wake, int from_idle);
	int msm_irq_enter_sleep2(bool arm9_wake, int from_idle);
	void msm_irq_exit_sleep1(void);
	void msm_irq_exit_sleep2(void);
	void msm_irq_exit_sleep3(void);
	void msm_gpio_enter_sleep(int from_idle);
	void msm_gpio_exit_sleep(void);
	void smd_sleep_exit(void);
	uint32_t enter_state;
	uint32_t enter_wait_set = 0;
	uint32_t enter_wait_clear = 0;
	uint32_t exit_state;
	uint32_t exit_wait_clear = 0;
	uint32_t exit_wait_any_set = 0;
	unsigned long pm_saved_acpu_clk_rate = 0;
	int ret;
	int rv = -EINTR;
	bool invalid_inital_state = false;

	if (board_mfg_mode() == 4) /*power test mode*/
		gpio_set_diag_gpio_table(
			(unsigned long *)board_get_mfg_sleep_gpio_table());

	if (msm_pm_debug_mask & MSM_PM_DEBUG_SUSPEND)
		printk(KERN_INFO "msm_sleep(): mode %d delay %u idle %d\n",
		       sleep_mode, sleep_delay, from_idle);
#if defined(CONFIG_MSM_N_WAY_SMD)
	switch (sleep_mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		enter_state = DEM_SLAVE_SMSM_PWRC;
		enter_wait_set = DEM_MASTER_SMSM_RSA;
		exit_state = DEM_SLAVE_SMSM_WFPI;
		exit_wait_any_set =
			DEM_MASTER_SMSM_RUN | DEM_MASTER_SMSM_PWRC_EARLY_EXIT;
		break;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND:
		enter_state = DEM_SLAVE_SMSM_PWRC_SUSPEND;
		enter_wait_set = DEM_MASTER_SMSM_RSA;
		exit_state = DEM_SLAVE_SMSM_WFPI;
		exit_wait_any_set =
			DEM_MASTER_SMSM_RUN | DEM_MASTER_SMSM_PWRC_EARLY_EXIT;
		break;
	case MSM_PM_SLEEP_MODE_APPS_SLEEP:
		enter_state = DEM_SLAVE_SMSM_SLEEP;
		enter_wait_set = DEM_MASTER_SMSM_SLEEP;
		exit_state = DEM_SLAVE_SMSM_SLEEP_EXIT;
		exit_wait_any_set = DEM_MASTER_SMSM_SLEEP_EXIT;
		break;
	default:
		enter_state = 0;
		exit_state = 0;
	}
#else
	switch (sleep_mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		enter_state = SMSM_PWRC;
		enter_wait_set = SMSM_RSA;
		exit_state = SMSM_WFPI;
		exit_wait_clear = SMSM_RSA;
		break;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND:
		enter_state = SMSM_PWRC_SUSPEND;
		enter_wait_set = SMSM_RSA;
		exit_state = SMSM_WFPI;
		exit_wait_clear = SMSM_RSA;
		break;
	case MSM_PM_SLEEP_MODE_APPS_SLEEP:
		enter_state = SMSM_SLEEP;
		exit_state = SMSM_SLEEPEXIT;
		exit_wait_any_set = SMSM_SLEEPEXIT;
		break;
	default:
		enter_state = 0;
		exit_state = 0;
	}
#endif
	clk_enter_sleep(from_idle);
	msm_irq_enter_sleep1(!!enter_state, from_idle);
	msm_gpio_enter_sleep(from_idle);

	if (enter_state) {
		/* Make sure last sleep request did not end with a timeout */
		ret = msm_pm_wait_state(PM_SMSM_READ_RUN, 0, 0, 0);
		if (ret) {
			printk(KERN_ERR "msm_sleep(): invalid inital state\n");
			invalid_inital_state = true;
		}

		if (sleep_delay == 0 && sleep_mode >= MSM_PM_SLEEP_MODE_APPS_SLEEP)
			sleep_delay = 192000*5; /* APPS_SLEEP does not allow infinite timeout */
		ret = smsm_set_sleep_duration(sleep_delay);
		if (ret) {
			printk(KERN_ERR "msm_sleep(): smsm_set_sleep_duration %x failed\n", enter_state);
			enter_state = 0;
			exit_state = 0;
		}
		if ((!from_idle && (msm_pm_debug_mask & MSM_PM_DEBUG_CLOCK_VOTE)) ||
			(from_idle && (msm_pm_debug_mask & MSM_PM_DEBUG_IDLE)))
			clks_allow_tcxo_locked_debug();

		ret = smsm_change_state(PM_SMSM_WRITE_STATE, PM_SMSM_WRITE_RUN, enter_state);
		if (ret) {
			printk(KERN_ERR "msm_sleep(): smsm_change_state %x failed\n", enter_state);
			enter_state = 0;
			exit_state = 0;
		}
		ret = msm_pm_wait_state(enter_wait_set, enter_wait_clear, 0, 0);
		if (ret || invalid_inital_state) {
			printk(KERN_INFO "msm_sleep(): msm_pm_wait_state failed, %x\n", smsm_get_state(PM_SMSM_READ_STATE));
			goto enter_failed;
		}
	}
	if (msm_irq_enter_sleep2(!!enter_state, from_idle))
		goto enter_failed;

	if (enter_state) {
		msm_pm_enter_prep_hw();
		if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
			printk(KERN_INFO "msm_sleep(): enter "
			       "A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, "
			       "smsm_get_state %x\n", readl(A11S_CLK_SLEEP_EN),
			       readl(A11S_PWRDOWN), smsm_get_state(PM_SMSM_READ_STATE));
	}

	if (sleep_mode <= MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT) {
		pm_saved_acpu_clk_rate = acpuclk_power_collapse(from_idle);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_CLOCK)
			printk(KERN_INFO "msm_sleep(): %ld enter power collapse"
			       "\n", pm_saved_acpu_clk_rate);
		if (pm_saved_acpu_clk_rate == 0)
			goto ramp_down_failed;

#ifdef CONFIG_AXI_SCREEN_POLICY
		/* Drop AXI request when the screen is on */
		if (axi_rate)
			clk_set_rate(axi_clk, sleep_axi_rate);
#endif
	}

	if (sleep_mode < MSM_PM_SLEEP_MODE_APPS_SLEEP) {
		if (msm_pm_debug_mask & MSM_PM_DEBUG_SMSM_STATE)
			smsm_print_sleep_info(0);
		saved_vector[0] = msm_pm_reset_vector[0];
		saved_vector[1] = msm_pm_reset_vector[1];
		msm_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
		msm_pm_reset_vector[1] = virt_to_phys(msm_pm_collapse_exit);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_RESET_VECTOR)
			printk(KERN_INFO "msm_sleep(): vector %x %x -> "
			       "%x %x\n", saved_vector[0], saved_vector[1],
			       msm_pm_reset_vector[0], msm_pm_reset_vector[1]);
#ifdef CONFIG_VFP
		if (from_idle)
			vfp_flush_context();
#endif

#ifdef CONFIG_CACHE_L2X0
		l2x0_suspend();
#endif
		if (!from_idle) printk(KERN_INFO "[R] suspend end\n");
		/* reset idle sleep mode when suspend. */
		if (!from_idle) msm_pm_idle_sleep_mode = idle_sleep_mode_def;
		collapsed = msm_pm_collapse();
		if (!from_idle) printk(KERN_INFO "[R] resume start\n");
#ifdef CONFIG_CACHE_L2X0
		l2x0_resume(collapsed);
#endif
		msm_pm_reset_vector[0] = saved_vector[0];
		msm_pm_reset_vector[1] = saved_vector[1];
		if (collapsed) {
#ifdef CONFIG_VFP
			if (from_idle)
				vfp_reinit();
#endif
			cpu_init();
			__asm__("cpsie   a");
			msm_fiq_exit_sleep();
			local_fiq_enable();
			rv = 0;
		}
		if (msm_pm_debug_mask & MSM_PM_DEBUG_POWER_COLLAPSE)
			printk(KERN_INFO "msm_pm_collapse(): returned %d\n",
			       collapsed);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_SMSM_STATE)
			smsm_print_sleep_info(0);
	} else {
		msm_arch_idle();
		rv = 0;
	}

	if (sleep_mode <= MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT) {
		if (msm_pm_debug_mask & MSM_PM_DEBUG_CLOCK)
			printk(KERN_INFO "msm_sleep(): exit power collapse %ld"
			       "\n", pm_saved_acpu_clk_rate);
		if (acpuclk_set_rate(pm_saved_acpu_clk_rate,
				from_idle ? SETRATE_PC_IDLE : SETRATE_PC) < 0)
			printk(KERN_ERR "msm_sleep(): clk_set_rate %ld "
			       "failed\n", pm_saved_acpu_clk_rate);

#ifdef CONFIG_AXI_SCREEN_POLICY
		/* Restore axi rate if needed */
		if (axi_rate)
			clk_set_rate(axi_clk, axi_rate);
#endif
	}
	if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
		printk(KERN_INFO "msm_sleep(): exit A11S_CLK_SLEEP_EN %x, "
		       "A11S_PWRDOWN %x, smsm_get_state %x\n",
		       readl(A11S_CLK_SLEEP_EN), readl(A11S_PWRDOWN),
		       smsm_get_state(PM_SMSM_READ_STATE));
ramp_down_failed:
	msm_irq_exit_sleep1();
enter_failed:
	if (enter_state) {
		msm_pm_exit_restore_hw();
		smsm_change_state(PM_SMSM_WRITE_STATE, enter_state, exit_state);
		msm_pm_wait_state(0, exit_wait_clear, exit_wait_any_set, 0);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
			printk(KERN_INFO "msm_sleep(): sleep exit "
			       "A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, "
			       "smsm_get_state %x\n", readl(A11S_CLK_SLEEP_EN),
			       readl(A11S_PWRDOWN), smsm_get_state(PM_SMSM_READ_STATE));
		if (msm_pm_debug_mask & MSM_PM_DEBUG_SMSM_STATE)
			smsm_print_sleep_info(0);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_WAKEUP_REASON && !from_idle)
			smsm_print_sleep_info(1);
	}
	msm_irq_exit_sleep2();
	if (enter_state) {
		smsm_change_state(PM_SMSM_WRITE_STATE, exit_state, PM_SMSM_WRITE_RUN);
		msm_pm_wait_state(PM_SMSM_READ_RUN, 0, 0, 0);
		if (msm_pm_debug_mask & MSM_PM_DEBUG_STATE)
			printk(KERN_INFO "msm_sleep(): sleep exit "
			       "A11S_CLK_SLEEP_EN %x, A11S_PWRDOWN %x, "
			       "smsm_get_state %x\n", readl(A11S_CLK_SLEEP_EN),
			       readl(A11S_PWRDOWN), smsm_get_state(PM_SMSM_READ_STATE));
	}
	msm_irq_exit_sleep3();
	msm_gpio_exit_sleep();
	smd_sleep_exit();
	clk_exit_sleep();
	return rv;
}

#if defined(CONFIG_ARCH_MSM7X30)
/*
 * Power collapse the Apps processor without involving Modem.
 *
 * Return value:
 *      0: success
 */
static int msm_pm_power_collapse_standalone(void)
{
	uint32_t saved_vector[2];
	int collapsed = 0;
	int ret;

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_POWER_COLLAPSE, false);
	WARN_ON(ret);
	if (ret)
		return ret;

#ifdef CONFIG_VFP
		vfp_flush_context();
#endif

#ifdef CONFIG_CACHE_L2X0
		l2x0_suspend();
#endif

	saved_vector[0] = msm_pm_reset_vector[0];
	saved_vector[1] = msm_pm_reset_vector[1];
	msm_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
	msm_pm_reset_vector[1] = virt_to_phys(msm_pm_collapse_exit);

	collapsed = msm_pm_collapse();

#ifdef CONFIG_CACHE_L2X0
	l2x0_resume(collapsed);
#endif
	msm_pm_reset_vector[0] = saved_vector[0];
	msm_pm_reset_vector[1] = saved_vector[1];
	if (collapsed) {
#ifdef CONFIG_VFP
		vfp_reinit();
#endif
		cpu_init();
		__asm__("cpsie   a");
		msm_fiq_exit_sleep();
		local_fiq_enable();
	}

	ret = msm_spm_set_low_power_mode(MSM_SPM_MODE_CLOCK_GATING, false);
	WARN_ON(ret);

	return 0;
}
#else
static int msm_pm_power_collapse_standalone(void)
{
	return -ENOSYS;
}
#endif

static int msm_pm_idle_spin(void)
{
	int spin;
	spin = msm_pm_idle_spin_time >> 10;
	while (spin-- > 0) {
		if (msm_irq_pending()) {
			return -1;
		}
		udelay(1);
	}
	return 0;
}

void arch_idle(void)
{
	bool allow[MSM_PM_SLEEP_MODE_NR];
	int64_t sleep_time;
	int low_power = 0;
	int ret;
	int i;
#ifdef CONFIG_MSM_IDLE_STATS
	int64_t t1;
	static int64_t t2;
	int exit_stat;
#endif

	if (msm_pm_reset_vector == NULL)
		return;

	sleep_time = msm_timer_enter_idle();
#ifdef CONFIG_MSM_IDLE_STATS
	t1 = ktime_to_ns(ktime_get());
	msm_pm_add_stat(MSM_PM_STAT_NOT_IDLE, t1 - t2);
	msm_pm_add_stat(MSM_PM_STAT_REQUESTED_IDLE, sleep_time);
#endif

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++)
		allow[i] = true;

	switch (msm_pm_idle_sleep_mode) {
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		allow[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] =
			false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] = false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
		allow[MSM_PM_SLEEP_MODE_APPS_SLEEP] = false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_APPS_SLEEP:
		/*allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = false;*/
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND:
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		break;
	default:
		pr_err("idle sleep mode is invalid: %d\n", msm_pm_idle_sleep_mode);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_SPIN;
#endif
		goto abort_idle;
	}

	if ((sleep_time < msm_pm_idle_sleep_min_time) ||
#ifdef CONFIG_HAS_WAKELOCK
		has_wake_lock(WAKE_LOCK_IDLE) ||
#endif
		!msm_irq_idle_sleep_allowed()) {
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = false;
		/*allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = false;*/
		allow[MSM_PM_SLEEP_MODE_APPS_SLEEP] = false;
	}

	for (i = 0; i < MSM_PM_SLEEP_MODE_NR; i++) {
		struct msm_pm_platform_data *mode = &msm_pm_modes[i];
		if (!mode->supported || !mode->idle_enabled ||
			/*mode->latency >= latency_qos ||*/
			mode->residency * 1000ULL >= sleep_time)
			allow[i] = false;
	}

	if (msm_pm_debug_mask & MSM_PM_DEBUG_IDLE) {
		pr_info("arch_idle: sleep_time %llu, wl %ld irq %d,"
			" allow %d %d %d %d %d %d)\n",
			sleep_time, has_wake_lock(WAKE_LOCK_IDLE),
			msm_irq_idle_sleep_allowed(),
			allow[0], allow[1], allow[2], allow[3],
			allow[4], allow[5]);
	}

	/* Enter one idle sleep mode. */
	if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] ||
		allow[MSM_PM_SLEEP_MODE_APPS_SLEEP]) {
		if (msm_pm_idle_spin() < 0) {
#ifdef CONFIG_MSM_IDLE_STATS
			exit_stat = MSM_PM_STAT_IDLE_SPIN;
#endif
			goto abort_idle;
		}

		low_power = 1;
		do_div(sleep_time, NSEC_PER_SEC / 32768);
		if (sleep_time > 0x6DDD000) {
			printk("sleep_time too big %lld\n", sleep_time);
			sleep_time = 0x6DDD000;
		}
		ret = msm_sleep(msm_pm_idle_sleep_mode, sleep_time, 1);
#ifdef CONFIG_MSM_IDLE_STATS
		if (ret)
			exit_stat = MSM_PM_STAT_IDLE_FAILED_SLEEP;
		else
			exit_stat = MSM_PM_STAT_IDLE_SLEEP;
#endif
	} else if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE]) {
		ret = msm_pm_power_collapse_standalone();
#ifdef CONFIG_MSM_IDLE_STATS
		if (ret)
			exit_stat = MSM_PM_STAT_IDLE_FAILED_STANDALONE_POWER_COLLAPSE;
		else
			exit_stat = MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE;
#endif
	} else if (allow[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT]) {
		unsigned long saved_rate;
		/* only spin while trying wfi ramp down */
		if (acpuclk_get_wfi_rate() && msm_pm_idle_spin() < 0) {
#ifdef CONFIG_MSM_IDLE_STATS
			exit_stat = MSM_PM_STAT_IDLE_SPIN;
#endif
			goto abort_idle;
		}
		saved_rate = acpuclk_wait_for_irq();

		if (saved_rate && msm_pm_debug_mask & MSM_PM_DEBUG_CLOCK)
			printk(KERN_DEBUG "arch_idle: clk %ld -> swfi\n",
				saved_rate);

		/*
		 * If there is a wfi speed specified and we failed to ramp,
		 * do not go into wfi.
		 */
		if (acpuclk_get_wfi_rate() && !saved_rate)
			while (!msm_irq_pending())
				udelay(1);
		else
			msm_arch_idle();

		if (msm_pm_debug_mask & MSM_PM_DEBUG_CLOCK)
			printk(KERN_DEBUG "msm_sleep: clk swfi -> %ld\n",
				saved_rate);
		if (saved_rate
		    && acpuclk_set_rate(saved_rate, SETRATE_SWFI) < 0)
			printk(KERN_ERR "msm_sleep(): clk_set_rate %ld "
			       "failed\n", saved_rate);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_WFI;
#endif
	} else {
		msm_arch_idle();
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_WFI;
#endif
	}

abort_idle:
	msm_timer_exit_idle(low_power);
#ifdef CONFIG_MSM_IDLE_STATS
	t2 = ktime_to_ns(ktime_get());
	msm_pm_add_stat(exit_stat, t2 - t1);
#endif
}

static int msm_pm_enter(suspend_state_t state)
{
	msm_sleep(msm_pm_sleep_mode, msm_pm_max_sleep_time, 0);
	return 0;
}

static struct platform_suspend_ops msm_pm_ops = {
	.enter		= msm_pm_enter,
	.valid		= suspend_valid_only_mem,
};

static uint32_t restart_reason = 0x776655AA;

static int msm_wakeup_after;	/* default, no wakeup by alarm */
static int msm_power_wakeup_after(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	printk(KERN_INFO "+msm_power_wakeup_after, ret=%d\r\n", ret);

	if (!ret) {
		printk(KERN_INFO "msm_wakeup_after=%d\r\n", msm_wakeup_after);
		if (msm_wakeup_after < 0)
			msm_wakeup_after = 0; /* invalid, no wakeup */
	}
	printk(KERN_INFO "-msm_power_wakeup_after, msm_wakeup_after=%d\r\n", msm_wakeup_after);
	return ret;
}

module_param_call(wakeup_after, msm_power_wakeup_after, param_get_int,
		  &msm_wakeup_after, S_IWUSR | S_IRUGO);

#ifdef CONFIG_HTC_OFFMODE_ALARM
static int set_offmode_alarm(void)
{
	struct timespec rtc_now;
	int next_alarm_interval;
	int i;

	getnstimeofday(&rtc_now);
	for (i = 0; i < offalarm_size; i++) {
		next_alarm_interval = (offalarm[i] - rtc_now.tv_sec) * 1000;
		if (next_alarm_interval > 0) {
			if (msm_wakeup_after == 0)
				msm_wakeup_after = next_alarm_interval;
			else if (next_alarm_interval < msm_wakeup_after)
				msm_wakeup_after = next_alarm_interval;
		}
	}

	return 0;
}
#endif

static void msm_pm_power_off(void)
{
	printk(KERN_INFO "msm_pm_power_off:wakeup after %d\r\n", msm_wakeup_after);

#ifdef CONFIG_HTC_OFFMODE_ALARM
	set_offmode_alarm();
#endif
	if (msm_wakeup_after)
		msm_proc_comm(PCOM_SET_RTC_ALARM, &msm_wakeup_after, 0);

	msm_proc_comm(PCOM_POWER_DOWN, 0, 0);

#if defined(CONFIG_MSM_RMT_STORAGE_SERVER) || defined(CONFIG_MSM_RMT_STORAGE_CLIENT)
		printk(KERN_INFO "from %s\r\n", __func__);
		wait_rmt_final_call_back(10);
		printk(KERN_INFO "back %s\r\n", __func__);
#endif
	for (;;) ;
}

static bool console_flushed;

void msm_pm_flush_console(void)
{
	if (console_flushed)
		return;
	console_flushed = true;

	printk("\n");
	printk(KERN_EMERG "Restarting %s\n", linux_banner);
	if (!try_acquire_console_sem()) {
		release_console_sem();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (try_acquire_console_sem())
		printk(KERN_EMERG "msm_restart: Console was locked! Busting\n");
	else
		printk(KERN_EMERG "msm_restart: Console was locked!\n");
	release_console_sem();
}

static void msm_pm_restart(char str, const char *cmd)
{
	msm_pm_flush_console();

	/*  always reboot device through proc comm */
	if (restart_reason == 0x6f656d99)
		msm_proc_comm(PCOM_RESET_CHIP_IMM, &restart_reason, 0);
	else
		msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);

#if defined(CONFIG_MSM_RMT_STORAGE_SERVER) || defined(CONFIG_MSM_RMT_STORAGE_CLIENT)
	printk(KERN_INFO "from %s\r\n", __func__);
	wait_rmt_final_call_back(10);
	printk(KERN_INFO "back %s\r\n", __func__);
	/* wait 2 seconds to let radio reset device after the final EFS sync*/
	mdelay(2000);
#else
	/* In case Radio is dead, reset device after notify Radio 5 seconds */
	mdelay(5000);
#endif

	/* hard reboot if possible */
	if (msm_hw_reset_hook) {
		printk(KERN_INFO "%s : Do HW_RESET by APP not by RADIO\r\n", __func__);
		msm_hw_reset_hook();
	}

	for (;;) ;
}

static int msm_reboot_call(struct notifier_block *this, unsigned long code, void *_cmd)
{
	if((code == SYS_RESTART) && _cmd) {
		char *cmd = _cmd;
		if (!strcmp(cmd, "bootloader")) {
			restart_reason = 0x77665500;
		} else if (!strcmp(cmd, "recovery")) {
			restart_reason = 0x77665502;
		} else if (!strcmp(cmd, "eraseflash")) {
			restart_reason = 0x776655EF;
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned code = simple_strtoul(cmd + 4, 0, 16) & 0xff;
			restart_reason = 0x6f656d00 | code;
		} else if (!strcmp(cmd, "force-hard")) {
			restart_reason = 0x776655AA;
		} else {
			restart_reason = 0x77665501;
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block msm_reboot_notifier =
{
	.notifier_call = msm_reboot_call,
};

#ifdef CONFIG_MSM_IDLE_STATS
static int msm_pm_read_proc(char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
	int len = 0;
	int i, j;
	char *p = page;

	for (i = 0; i < ARRAY_SIZE(msm_pm_stats); i++) {
		int64_t bucket_time;
		int64_t s;
		uint32_t ns;
		s = msm_pm_stats[i].total_time;
		ns = do_div(s, NSEC_PER_SEC);
		p += sprintf(p,
			"%s:\n"
			"  count: %7d\n"
			"  total_time: %lld.%09u\n",
			msm_pm_stats[i].name,
			msm_pm_stats[i].count,
			s, ns);
		bucket_time = CONFIG_MSM_IDLE_STATS_FIRST_BUCKET;
		for (j = 0; j < CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1; j++) {
			s = bucket_time;
			ns = do_div(s, NSEC_PER_SEC);
			p += sprintf(p, "   <%2lld.%09u: %7d (%lld-%lld)\n",
				s, ns, msm_pm_stats[i].bucket[j],
				msm_pm_stats[i].min_time[j],
				msm_pm_stats[i].max_time[j]);
			bucket_time <<= CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT;
		}
		p += sprintf(p, "  >=%2lld.%09u: %7d (%lld-%lld)\n",
			s, ns, msm_pm_stats[i].bucket[j],
			msm_pm_stats[i].min_time[j],
			msm_pm_stats[i].max_time[j]);
	}
	*start = page + off;

	len = p - page;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len  : count;
}
#endif

void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns)
{
	int64_t max_sleep_time_bs = max_sleep_time_ns;

	/* Convert from ns -> BS units */
	do_div(max_sleep_time_bs, NSEC_PER_SEC / 32768);

	if (max_sleep_time_bs > 0x6DDD000)
		msm_pm_max_sleep_time = (uint32_t) 0x6DDD000;
	else
		msm_pm_max_sleep_time = (uint32_t) max_sleep_time_bs;

	if (msm_pm_debug_mask & MSM_PM_DEBUG_SUSPEND)
		printk("%s: Requested %lldns (%lldbs), Giving %ubs\n",
		       __func__, max_sleep_time_ns, 
		       max_sleep_time_bs, 
		       msm_pm_max_sleep_time);
}
EXPORT_SYMBOL(msm_pm_set_max_sleep_time);

#ifdef CONFIG_EARLYSUSPEND
#ifdef CONFIG_AXI_SCREEN_POLICY
/* axi 128 screen on, 61mhz screen off */
static void axi_early_suspend(struct early_suspend *handler)
{
	axi_rate = 0;
	clk_set_rate(axi_clk, axi_rate);
}

static void axi_late_resume(struct early_suspend *handler)
{
	axi_rate = 128000000;
	sleep_axi_rate = 117000000;
	clk_set_rate(axi_clk, axi_rate);
}

static struct early_suspend axi_screen_suspend = {
	.suspend = axi_early_suspend,
	.resume = axi_late_resume,
};
#endif
#endif

#ifdef CONFIG_AXI_SCREEN_POLICY
static void __init msm_pm_axi_init(void)
{
#ifdef CONFIG_EARLYSUSPEND
	axi_clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(axi_clk)) {
		int result = PTR_ERR(axi_clk);
		pr_err("clk_get(ebi1_clk) returned %d\n", result);
		return;
	}
	axi_rate = 128000000;
	sleep_axi_rate = 117000000;
	clk_set_rate(axi_clk, axi_rate);
	register_early_suspend(&axi_screen_suspend);
#else
	axi_rate = 0;
#endif
}
#endif

static void __init boot_lock_nohalt(void)
{
	int nohalt_timeout;

	/* normal/factory2/recovery */
	switch (board_mfg_mode()) {
	case 0: /* normal */
	case 1: /* factory2 */
	case 2: /* recovery */
		nohalt_timeout = BOOT_LOCK_TIMEOUT_NORMAL;
		break;
	case 3: /* charge */
	case 4: /* power_test */
	case 5: /* offmode_charge */
	default:
		nohalt_timeout = BOOT_LOCK_TIMEOUT_SHORT;
		break;
	}
	disable_hlt();
	schedule_delayed_work(&work_expire_boot_lock, nohalt_timeout);
	pr_info("Acquire 'boot-time' no_halt_lock %ds\n", nohalt_timeout / HZ);
}

static int __init msm_pm_init(void)
{
	pm_power_off = msm_pm_power_off;
	arm_pm_restart = msm_pm_restart;
	msm_pm_max_sleep_time = 0;
#if defined(CONFIG_ARCH_MSM_SCORPION)
#ifdef CONFIG_AXI_SCREEN_POLICY
	msm_pm_axi_init();
#endif
#endif

	register_reboot_notifier(&msm_reboot_notifier);
	msm_pm_reset_vector = ioremap(0x0, PAGE_SIZE);

	if (msm_pm_reset_vector == NULL) {
		printk(KERN_ERR "msm_pm_init: failed to map reset vector\n");
		return -ENODEV;
	}

	suspend_set_ops(&msm_pm_ops);

#ifdef CONFIG_MSM_IDLE_STATS
	create_proc_read_entry("msm_pm_stats", S_IRUGO,
				NULL, msm_pm_read_proc, NULL);
#endif

	boot_lock_nohalt();
	return 0;
}

__initcall(msm_pm_init);
