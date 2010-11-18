/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Shared memory logging implementation.
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/remote_spinlock.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/string.h>

#include <mach/msm_iomap.h>
#include <mach/smem_log.h>

#include "smd_private.h"

#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define D_DUMP_BUFFER(prestr, cnt, buf) \
do { \
	int i; \
	printk(KERN_ERR "%s", prestr); \
	for (i = 0; i < cnt; i++) \
		printk(KERN_ERR "%.2x", buf[i]); \
	printk(KERN_ERR "\n"); \
} while (0)
#else
#define D_DUMP_BUFFER(prestr, cnt, buf)
#endif

#ifdef DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

#define TIMESTAMP_ADDR (MSM_CSR_BASE + 0x04)

struct smem_log_item {
	uint32_t identifier;
	uint32_t timetick;
	uint32_t data1;
	uint32_t data2;
	uint32_t data3;
};

#define SMEM_LOG_NUM_ENTRIES 2000
#define SMEM_LOG_EVENTS_SIZE (sizeof(struct smem_log_item) * \
			      SMEM_LOG_NUM_ENTRIES)

#define SMEM_LOG_NUM_STATIC_ENTRIES 150
#define SMEM_STATIC_LOG_EVENTS_SIZE (sizeof(struct smem_log_item) * \
				     SMEM_LOG_NUM_STATIC_ENTRIES)

#define SMEM_LOG_NUM_POWER_ENTRIES 2000
#define SMEM_POWER_LOG_EVENTS_SIZE (sizeof(struct smem_log_item) * \
			      SMEM_LOG_NUM_POWER_ENTRIES)

#if defined(CONFIG_ARCH_MSM7X30)
#define SMEM_SPINLOCK_SMEM_LOG "S:2"
#define SMEM_SPINLOCK_STATIC_LOG "S:5"
#else
#define SMEM_SPINLOCK_SMEM_LOG 2
#define SMEM_SPINLOCK_STATIC_LOG 5
#endif
/* POWER shares with SMEM_SPINLOCK_SMEM_LOG */

static remote_spinlock_t remote_spinlock;
static remote_spinlock_t remote_spinlock_static;

struct smem_log_inst {
	int which_log;
	struct smem_log_item __iomem *events;
	uint32_t __iomem *idx;
	int num;
	remote_spinlock_t *remote_spinlock;
};

enum smem_logs {
	GEN = 0,
	STA,
	POW,
	NUM
};

static struct smem_log_inst inst[NUM];

#if defined(CONFIG_DEBUG_FS)

#define HSIZE 13

struct sym {
	uint32_t val;
	char *str;
	struct hlist_node node;
};

struct sym id_syms[] = {
	{ SMEM_LOG_PROC_ID_MODEM, "MODM" },
	{ SMEM_LOG_PROC_ID_Q6, "QDSP" },
	{ SMEM_LOG_PROC_ID_APPS, "APPS" },
};

struct sym base_syms[] = {
	{ SMEM_LOG_ONCRPC_EVENT_BASE, "ONCRPC" },
	{ SMEM_LOG_SMEM_EVENT_BASE, "SMEM" },
	{ SMEM_LOG_TMC_EVENT_BASE, "TMC" },
	{ SMEM_LOG_TIMETICK_EVENT_BASE, "TIMETICK" },
	{ SMEM_LOG_DEM_EVENT_BASE, "DEM" },
	{ SMEM_LOG_ERROR_EVENT_BASE, "ERROR" },
	{ SMEM_LOG_DCVS_EVENT_BASE, "DCVS" },
	{ SMEM_LOG_SLEEP_EVENT_BASE, "SLEEP" },
	{ SMEM_LOG_RPC_ROUTER_EVENT_BASE, "ROUTER" },
};

struct sym event_syms[] = {
#if defined(CONFIG_MSM_N_WAY_SMSM)
	{ DEM_SMSM_ISR, "SMSM_ISR" },
	{ DEM_STATE_CHANGE, "STATE_CHANGE" },
	{ DEM_STATE_MACHINE_ENTER, "STATE_MACHINE_ENTER" },
	{ DEM_ENTER_SLEEP, "ENTER_SLEEP" },
	{ DEM_END_SLEEP, "END_SLEEP" },
	{ DEM_SETUP_SLEEP, "SETUP_SLEEP" },
	{ DEM_SETUP_POWER_COLLAPSE, "SETUP_POWER_COLLAPSE" },
	{ DEM_SETUP_SUSPEND, "SETUP_SUSPEND" },
	{ DEM_EARLY_EXIT, "EARLY_EXIT" },
	{ DEM_WAKEUP_REASON, "WAKEUP_REASON" },
	{ DEM_DETECT_WAKEUP, "DETECT_WAKEUP" },
	{ DEM_DETECT_RESET, "DETECT_RESET" },
	{ DEM_DETECT_SLEEPEXIT, "DETECT_SLEEPEXIT" },
	{ DEM_DETECT_RUN, "DETECT_RUN" },
	{ DEM_APPS_SWFI, "APPS_SWFI" },
	{ DEM_SEND_WAKEUP, "SEND_WAKEUP" },
	{ DEM_ASSERT_OKTS, "ASSERT_OKTS" },
	{ DEM_NEGATE_OKTS, "NEGATE_OKTS" },
	{ DEM_PROC_COMM_CMD, "PROC_COMM_CMD" },
	{ DEM_REMOVE_PROC_PWR, "REMOVE_PROC_PWR" },
	{ DEM_RESTORE_PROC_PWR, "RESTORE_PROC_PWR" },
	{ DEM_SMI_CLK_DISABLED, "SMI_CLK_DISABLED" },
	{ DEM_SMI_CLK_ENABLED, "SMI_CLK_ENABLED" },
	{ DEM_MAO_INTS, "MAO_INTS" },
	{ DEM_APPS_WAKEUP_INT, "APPS_WAKEUP_INT" },
	{ DEM_PROC_WAKEUP, "PROC_WAKEUP" },
	{ DEM_PROC_POWERUP, "PROC_POWERUP" },
	{ DEM_TIMER_EXPIRED, "TIMER_EXPIRED" },
	{ DEM_SEND_BATTERY_INFO, "SEND_BATTERY_INFO" },
	{ DEM_REMOTE_PWR_CB, "REMOTE_PWR_CB" },
	{ DEM_TIME_SYNC_START, "TIME_SYNC_START" },
	{ DEM_TIME_SYNC_SEND_VALUE, "TIME_SYNC_SEND_VALUE" },
	{ DEM_TIME_SYNC_DONE, "TIME_SYNC_DONE" },
	{ DEM_TIME_SYNC_REQUEST, "TIME_SYNC_REQUEST" },
	{ DEM_TIME_SYNC_POLL, "TIME_SYNC_POLL" },
	{ DEM_TIME_SYNC_INIT, "TIME_SYNC_INIT" },
	{ DEM_INIT, "INIT" },
#else

	{ DEM_NO_SLEEP, "NO_SLEEP" },
	{ DEM_INSUF_TIME, "INSUF_TIME" },
	{ DEMAPPS_ENTER_SLEEP, "APPS_ENTER_SLEEP" },
	{ DEMAPPS_DETECT_WAKEUP, "APPS_DETECT_WAKEUP" },
	{ DEMAPPS_END_APPS_TCXO, "APPS_END_APPS_TCXO" },
	{ DEMAPPS_ENTER_SLEEPEXIT, "APPS_ENTER_SLEEPEXIT" },
	{ DEMAPPS_END_APPS_SLEEP, "APPS_END_APPS_SLEEP" },
	{ DEMAPPS_SETUP_APPS_PWRCLPS, "APPS_SETUP_APPS_PWRCLPS" },
	{ DEMAPPS_PWRCLPS_EARLY_EXIT, "APPS_PWRCLPS_EARLY_EXIT" },
	{ DEMMOD_SEND_WAKEUP, "MOD_SEND_WAKEUP" },
	{ DEMMOD_NO_APPS_VOTE, "MOD_NO_APPS_VOTE" },
	{ DEMMOD_NO_TCXO_SLEEP, "MOD_NO_TCXO_SLEEP" },
	{ DEMMOD_BT_CLOCK, "MOD_BT_CLOCK" },
	{ DEMMOD_UART_CLOCK, "MOD_UART_CLOCK" },
	{ DEMMOD_OKTS, "MOD_OKTS" },
	{ DEM_SLEEP_INFO, "SLEEP_INFO" },
	{ DEMMOD_TCXO_END, "MOD_TCXO_END" },
	{ DEMMOD_END_SLEEP_SIG, "MOD_END_SLEEP_SIG" },
	{ DEMMOD_SETUP_APPSSLEEP, "MOD_SETUP_APPSSLEEP" },
	{ DEMMOD_ENTER_TCXO, "MOD_ENTER_TCXO" },
	{ DEMMOD_WAKE_APPS, "MOD_WAKE_APPS" },
	{ DEMMOD_POWER_COLLAPSE_APPS, "MOD_POWER_COLLAPSE_APPS" },
	{ DEMMOD_RESTORE_APPS_PWR, "MOD_RESTORE_APPS_PWR" },
	{ DEMAPPS_ASSERT_OKTS, "APPS_ASSERT_OKTS" },
	{ DEMAPPS_RESTART_START_TIMER, "APPS_RESTART_START_TIMER" },
	{ DEMAPPS_ENTER_RUN, "APPS_ENTER_RUN" },
	{ DEMMOD_MAO_INTS, "MOD_MAO_INTS" },
	{ DEMMOD_POWERUP_APPS_CALLED, "MOD_POWERUP_APPS_CALLED" },
	{ DEMMOD_PC_TIMER_EXPIRED, "MOD_PC_TIMER_EXPIRED" },
	{ DEM_DETECT_SLEEPEXIT, "_DETECT_SLEEPEXIT" },
	{ DEM_DETECT_RUN, "DETECT_RUN" },
	{ DEM_SET_APPS_TIMER, "SET_APPS_TIMER" },
	{ DEM_NEGATE_OKTS, "NEGATE_OKTS" },
	{ DEMMOD_APPS_WAKEUP_INT, "MOD_APPS_WAKEUP_INT" },
	{ DEMMOD_APPS_SWFI, "MOD_APPS_SWFI" },
	{ DEM_SEND_BATTERY_INFO, "SEND_BATTERY_INFO" },
	{ DEM_SMI_CLK_DISABLED, "SMI_CLK_DISABLED" },
	{ DEM_SMI_CLK_ENABLED, "SMI_CLK_ENABLED" },
	{ DEMAPPS_SETUP_APPS_SUSPEND, "APPS_SETUP_APPS_SUSPEND" },
	{ DEM_RPC_EARLY_EXIT, "RPC_EARLY_EXIT" },
	{ DEMAPPS_WAKEUP_REASON, "APPS_WAKEUP_REASON" },
	{ DEM_INIT, "INIT" },
#endif
	{ DEMMOD_UMTS_BASE, "MOD_UMTS_BASE" },
	{ DEMMOD_GL1_GO_TO_SLEEP, "GL1_GO_TO_SLEEP" },
	{ DEMMOD_GL1_SLEEP_START, "GL1_SLEEP_START" },
	{ DEMMOD_GL1_AFTER_GSM_CLK_ON, "GL1_AFTER_GSM_CLK_ON" },
	{ DEMMOD_GL1_BEFORE_RF_ON, "GL1_BEFORE_RF_ON" },
	{ DEMMOD_GL1_AFTER_RF_ON, "GL1_AFTER_RF_ON" },
	{ DEMMOD_GL1_FRAME_TICK, "GL1_FRAME_TICK" },
	{ DEMMOD_GL1_WCDMA_START, "GL1_WCDMA_START" },
	{ DEMMOD_GL1_WCDMA_ENDING, "GL1_WCDMA_ENDING" },
	{ DEMMOD_UMTS_NOT_OKTS, "UMTS_NOT_OKTS" },
	{ DEMMOD_UMTS_START_TCXO_SHUTDOWN, "UMTS_START_TCXO_SHUTDOWN" },
	{ DEMMOD_UMTS_END_TCXO_SHUTDOWN, "UMTS_END_TCXO_SHUTDOWN" },
	{ DEMMOD_UMTS_START_ARM_HALT, "UMTS_START_ARM_HALT" },
	{ DEMMOD_UMTS_END_ARM_HALT, "UMTS_END_ARM_HALT" },
	{ DEMMOD_UMTS_NEXT_WAKEUP_SCLK, "UMTS_NEXT_WAKEUP_SCLK" },
	{ TIME_REMOTE_LOG_EVENT_START, "START" },
	{ TIME_REMOTE_LOG_EVENT_GOTO_WAIT,
	  "GOTO_WAIT" },
	{ TIME_REMOTE_LOG_EVENT_GOTO_INIT,
	  "GOTO_INIT" },
	{ ERR_ERROR_FATAL, "ERR_ERROR_FATAL" },
	{ ERR_ERROR_FATAL_TASK, "ERR_ERROR_FATAL_TASK" },
	{ DCVSAPPS_LOG_IDLE, "DCVSAPPS_LOG_IDLE" },
	{ DCVSAPPS_LOG_ERR, "DCVSAPPS_LOG_ERR" },
	{ DCVSAPPS_LOG_CHG, "DCVSAPPS_LOG_CHG" },
	{ DCVSAPPS_LOG_REG, "DCVSAPPS_LOG_REG" },
	{ DCVSAPPS_LOG_DEREG, "DCVSAPPS_LOG_DEREG" },
	{ SMEM_LOG_EVENT_CB, "CB" },
	{ SMEM_LOG_EVENT_START, "START" },
	{ SMEM_LOG_EVENT_INIT, "INIT" },
	{ SMEM_LOG_EVENT_RUNNING, "RUNNING" },
	{ SMEM_LOG_EVENT_STOP, "STOP" },
	{ SMEM_LOG_EVENT_RESTART, "RESTART" },
	{ SMEM_LOG_EVENT_SS, "SS" },
	{ SMEM_LOG_EVENT_READ, "READ" },
	{ SMEM_LOG_EVENT_WRITE, "WRITE" },
	{ SMEM_LOG_EVENT_SIGS1, "SIGS1" },
	{ SMEM_LOG_EVENT_SIGS2, "SIGS2" },
	{ SMEM_LOG_EVENT_WRITE_DM, "WRITE_DM" },
	{ SMEM_LOG_EVENT_READ_DM, "READ_DM" },
	{ SMEM_LOG_EVENT_SKIP_DM, "SKIP_DM" },
	{ SMEM_LOG_EVENT_STOP_DM, "STOP_DM" },
	{ SMEM_LOG_EVENT_ISR, "ISR" },
	{ SMEM_LOG_EVENT_TASK, "TASK" },
	{ SMEM_LOG_EVENT_RS, "RS" },
	{ ONCRPC_LOG_EVENT_SMD_WAIT, "SMD_WAIT" },
	{ ONCRPC_LOG_EVENT_RPC_WAIT, "RPC_WAIT" },
	{ ONCRPC_LOG_EVENT_RPC_BOTH_WAIT, "RPC_BOTH_WAIT" },
	{ ONCRPC_LOG_EVENT_RPC_INIT, "RPC_INIT" },
	{ ONCRPC_LOG_EVENT_RUNNING, "RUNNING" },
	{ ONCRPC_LOG_EVENT_APIS_INITED, "APIS_INITED" },
	{ ONCRPC_LOG_EVENT_AMSS_RESET, "AMSS_RESET" },
	{ ONCRPC_LOG_EVENT_SMD_RESET, "SMD_RESET" },
	{ ONCRPC_LOG_EVENT_ONCRPC_RESET, "ONCRPC_RESET" },
	{ ONCRPC_LOG_EVENT_CB, "CB" },
	{ ONCRPC_LOG_EVENT_STD_CALL, "STD_CALL" },
	{ ONCRPC_LOG_EVENT_STD_REPLY, "STD_REPLY" },
	{ ONCRPC_LOG_EVENT_STD_CALL_ASYNC, "STD_CALL_ASYNC" },
	{ NO_SLEEP_OLD, "NO_SLEEP_OLD" },
	{ INSUF_TIME, "INSUF_TIME" },
	{ MOD_UART_CLOCK, "MOD_UART_CLOCK" },
	{ SLEEP_INFO, "SLEEP_INFO" },
	{ MOD_TCXO_END, "MOD_TCXO_END" },
	{ MOD_ENTER_TCXO, "MOD_ENTER_TCXO" },
	{ NO_SLEEP_NEW, "NO_SLEEP_NEW" },
	{ RPC_ROUTER_LOG_EVENT_UNKNOWN, "UNKNOWN" },
	{ RPC_ROUTER_LOG_EVENT_MSG_READ, "MSG_READ" },
	{ RPC_ROUTER_LOG_EVENT_MSG_WRITTEN, "MSG_WRITTEN" },
	{ RPC_ROUTER_LOG_EVENT_MSG_CFM_REQ, "MSG_CFM_REQ" },
	{ RPC_ROUTER_LOG_EVENT_MSG_CFM_SNT, "MSG_CFM_SNT" },
	{ RPC_ROUTER_LOG_EVENT_MID_READ, "MID_READ" },
	{ RPC_ROUTER_LOG_EVENT_MID_WRITTEN, "MID_WRITTEN" },
	{ RPC_ROUTER_LOG_EVENT_MID_CFM_REQ, "MID_CFM_REQ" },

};

struct sym oncrpc_syms[] = {
	{ 0x30000000, "CM" },
	{ 0x30000001, "DB" },
	{ 0x30000002, "SND" },
	{ 0x30000003, "WMS" },
	{ 0x30000004, "PDSM" },
	{ 0x30000005, "MISC_MODEM_APIS" },
	{ 0x30000006, "MISC_APPS_APIS" },
	{ 0x30000007, "JOYST" },
	{ 0x30000008, "VJOY" },
	{ 0x30000009, "JOYSTC" },
	{ 0x3000000a, "ADSPRTOSATOM" },
	{ 0x3000000b, "ADSPRTOSMTOA" },
	{ 0x3000000c, "I2C" },
	{ 0x3000000d, "TIME_REMOTE" },
	{ 0x3000000e, "NV" },
	{ 0x3000000f, "CLKRGM_SEC" },
	{ 0x30000010, "RDEVMAP" },
	{ 0x30000011, "FS_RAPI" },
	{ 0x30000012, "PBMLIB" },
	{ 0x30000013, "AUDMGR" },
	{ 0x30000014, "MVS" },
	{ 0x30000015, "DOG_KEEPALIVE" },
	{ 0x30000016, "GSDI_EXP" },
	{ 0x30000017, "AUTH" },
	{ 0x30000018, "NVRUIMI" },
	{ 0x30000019, "MMGSDILIB" },
	{ 0x3000001a, "CHARGER" },
	{ 0x3000001b, "UIM" },
	{ 0x3000001C, "ONCRPCTEST" },
	{ 0x3000001d, "PDSM_ATL" },
	{ 0x3000001e, "FS_XMOUNT" },
	{ 0x3000001f, "SECUTIL " },
	{ 0x30000020, "MCCMEID" },
	{ 0x30000021, "PM_STROBE_FLASH" },
	{ 0x30000022, "DS707_EXTIF" },
	{ 0x30000023, "SMD BRIDGE_MODEM" },
	{ 0x30000024, "SMD PORT_MGR" },
	{ 0x30000025, "BUS_PERF" },
	{ 0x30000026, "BUS_MON" },
	{ 0x30000027, "MC" },
	{ 0x30000028, "MCCAP" },
	{ 0x30000029, "MCCDMA" },
	{ 0x3000002a, "MCCDS" },
	{ 0x3000002b, "MCCSCH" },
	{ 0x3000002c, "MCCSRID" },
	{ 0x3000002d, "SNM" },
	{ 0x3000002e, "MCCSYOBJ" },
	{ 0x3000002f, "DS707_APIS" },
	{ 0x30000030, "DS_MP_SHIM_APPS_ASYNC" },
	{ 0x30000031, "DSRLP_APIS" },
	{ 0x30000032, "RLP_APIS" },
	{ 0x30000033, "DS_MP_SHIM_MODEM" },
	{ 0x30000034, "DSHDR_APIS" },
	{ 0x30000035, "DSHDR_MDM_APIS" },
	{ 0x30000036, "DS_MP_SHIM_APPS" },
	{ 0x30000037, "HDRMC_APIS" },
	{ 0x30000038, "SMD_BRIDGE_MTOA" },
	{ 0x30000039, "SMD_BRIDGE_ATOM" },
	{ 0x3000003a, "DPMAPP_OTG" },
	{ 0x3000003b, "DIAG" },
	{ 0x3000003c, "GSTK_EXP" },
	{ 0x3000003d, "DSBC_MDM_APIS" },
	{ 0x3000003e, "HDRMRLP_MDM_APIS" },
	{ 0x3000003f, "HDRMRLP_APPS_APIS" },
	{ 0x30000040, "HDRMC_MRLP_APIS" },
	{ 0x30000041, "PDCOMM_APP_API" },
	{ 0x30000042, "DSAT_APIS" },
	{ 0x30000043, "MISC_RF_APIS" },
	{ 0x30000044, "CMIPAPP" },
	{ 0x30000045, "DSMP_UMTS_MODEM_APIS" },
	{ 0x30000046, "DSMP_UMTS_APPS_APIS" },
	{ 0x30000047, "DSUCSDMPSHIM" },
	{ 0x30000048, "TIME_REMOTE_ATOM" },
	{ 0x3000004a, "SD" },
	{ 0x3000004b, "MMOC" },
	{ 0x3000004c, "WLAN_ADP_FTM" },
	{ 0x3000004d, "WLAN_CP_CM" },
	{ 0x3000004e, "FTM_WLAN" },
	{ 0x3000004f, "SDCC_CPRM" },
	{ 0x30000050, "CPRMINTERFACE" },
	{ 0x30000051, "DATA_ON_MODEM_MTOA_APIS" },
	{ 0x30000052, "DATA_ON_APPS_ATOM_APIS" },
	{ 0x30000053, "MISC_MODEM_APIS_NONWINMOB" },
	{ 0x30000054, "MISC_APPS_APIS_NONWINMOB" },
	{ 0x30000055, "PMEM_REMOTE" },
	{ 0x30000056, "TCXOMGR" },
	{ 0x30000057, "DSUCSDAPPIF_APIS" },
	{ 0x30000058, "BT" },
	{ 0x30000059, "PD_COMMS_API" },
	{ 0x3000005a, "PD_COMMS_CLIENT_API" },
	{ 0x3000005b, "PDAPI" },
	{ 0x3000005c, "LSA_SUPL_DSM" },
	{ 0x3000005d, "TIME_REMOTE_MTOA" },
	{ 0x3000005e, "FTM_BT" },
	{ 0X3000005f, "DSUCSDAPPIF_APIS" },
	{ 0X30000060, "PMAPP_GEN" },
	{ 0X30000061, "PM_LIB" },
	{ 0X30000062, "KEYPAD" },
	{ 0X30000063, "HSU_APP_APIS" },
	{ 0X30000064, "HSU_MDM_APIS" },
	{ 0X30000065, "ADIE_ADC_REMOTE_ATOM " },
	{ 0X30000066, "TLMM_REMOTE_ATOM" },
	{ 0X30000067, "UI_CALLCTRL" },
	{ 0X30000068, "UIUTILS" },
	{ 0X30000069, "PRL" },
	{ 0X3000006a, "HW" },
	{ 0X3000006b, "OEM_RAPI" },
	{ 0X3000006c, "WMSPM" },
	{ 0X3000006d, "BTPF" },
	{ 0X3000006e, "CLKRGM_SYNC_EVENT" },
	{ 0X3000006f, "USB_APPS_RPC" },
	{ 0X30000070, "USB_MODEM_RPC" },
	{ 0X30000071, "ADC" },
	{ 0X30000072, "CAMERAREMOTED" },
	{ 0X30000073, "SECAPIREMOTED" },
	{ 0X30000074, "DSATAPI" },
	{ 0X30000075, "CLKCTL_RPC" },
	{ 0X30000076, "BREWAPPCOORD" },
	{ 0X30000077, "ALTENVSHELL" },
	{ 0X30000078, "WLAN_TRP_UTILS" },
	{ 0X30000079, "GPIO_RPC" },
	{ 0X3000007a, "PING_RPC" },
	{ 0X3000007b, "DSC_DCM_API" },
	{ 0X3000007c, "L1_DS" },
	{ 0X3000007d, "QCHATPK_APIS" },
	{ 0X3000007e, "GPS_API" },
	{ 0X3000007f, "OSS_RRCASN_REMOTE" },
	{ 0X30000080, "PMAPP_OTG_REMOTE" },
	{ 0X30000081, "PING_MDM_RPC" },
	{ 0X30000082, "PING_KERNEL_RPC" },
	{ 0X30000083, "TIMETICK" },
	{ 0X30000084, "WM_BTHCI_FTM " },
	{ 0X30000085, "WM_BT_PF" },
	{ 0X30000086, "IPA_IPC_APIS" },
	{ 0X30000087, "UKCC_IPC_APIS" },
	{ 0X30000088, "CMIPSMS " },
	{ 0X30000089, "VBATT_REMOTE" },
	{ 0X3000008a, "MFPAL" },
	{ 0X3000008b, "DSUMTSPDPREG" },
	{ 0X3000fe00, "RESTART_DAEMON NUMBER 0" },
	{ 0X3000fe01, "RESTART_DAEMON NUMBER 1" },
	{ 0X3000feff, "RESTART_DAEMON NUMBER 255" },
	{ 0X3000fffe, "BACKWARDS_COMPATIBILITY_IN_RPC_CLNT_LOOKUP" },
	{ 0X3000ffff, "RPC_ROUTER_SERVER_PROGRAM" },
	{ 0x31000000, "CM CB" },
	{ 0x31000001, "DB CB" },
	{ 0x31000002, "SND CB" },
	{ 0x31000003, "WMS CB" },
	{ 0x31000004, "PDSM CB" },
	{ 0x31000005, "MISC_MODEM_APIS CB" },
	{ 0x31000006, "MISC_APPS_APIS CB" },
	{ 0x31000007, "JOYST CB" },
	{ 0x31000008, "VJOY CB" },
	{ 0x31000009, "JOYSTC CB" },
	{ 0x3100000a, "ADSPRTOSATOM CB" },
	{ 0x3100000b, "ADSPRTOSMTOA CB" },
	{ 0x3100000c, "I2C CB" },
	{ 0x3100000d, "TIME_REMOTE CB" },
	{ 0x3100000e, "NV CB" },
	{ 0x3100000f, "CLKRGM_SEC CB" },
	{ 0x31000010, "RDEVMAP CB" },
	{ 0x31000011, "FS_RAPI CB" },
	{ 0x31000012, "PBMLIB CB" },
	{ 0x31000013, "AUDMGR CB" },
	{ 0x31000014, "MVS CB" },
	{ 0x31000015, "DOG_KEEPALIVE CB" },
	{ 0x31000016, "GSDI_EXP CB" },
	{ 0x31000017, "AUTH CB" },
	{ 0x31000018, "NVRUIMI CB" },
	{ 0x31000019, "MMGSDILIB CB" },
	{ 0x3100001a, "CHARGER CB" },
	{ 0x3100001b, "UIM CB" },
	{ 0x3100001C, "ONCRPCTEST CB" },
	{ 0x3100001d, "PDSM_ATL CB" },
	{ 0x3100001e, "FS_XMOUNT CB" },
	{ 0x3100001f, "SECUTIL CB" },
	{ 0x31000020, "MCCMEID" },
	{ 0x31000021, "PM_STROBE_FLASH CB" },
	{ 0x31000022, "DS707_EXTIF CB" },
	{ 0x31000023, "SMD BRIDGE_MODEM CB" },
	{ 0x31000024, "SMD PORT_MGR CB" },
	{ 0x31000025, "BUS_PERF CB" },
	{ 0x31000026, "BUS_MON CB" },
	{ 0x31000027, "MC CB" },
	{ 0x31000028, "MCCAP CB" },
	{ 0x31000029, "MCCDMA CB" },
	{ 0x3100002a, "MCCDS CB" },
	{ 0x3100002b, "MCCSCH CB" },
	{ 0x3100002c, "MCCSRID CB" },
	{ 0x3100002d, "SNM CB" },
	{ 0x3100002e, "MCCSYOBJ CB" },
	{ 0x3100002f, "DS707_APIS CB" },
	{ 0x31000030, "DS_MP_SHIM_APPS_ASYNC CB" },
	{ 0x31000031, "DSRLP_APIS CB" },
	{ 0x31000032, "RLP_APIS CB" },
	{ 0x31000033, "DS_MP_SHIM_MODEM CB" },
	{ 0x31000034, "DSHDR_APIS CB" },
	{ 0x31000035, "DSHDR_MDM_APIS CB" },
	{ 0x31000036, "DS_MP_SHIM_APPS CB" },
	{ 0x31000037, "HDRMC_APIS CB" },
	{ 0x31000038, "SMD_BRIDGE_MTOA CB" },
	{ 0x31000039, "SMD_BRIDGE_ATOM CB" },
	{ 0x3100003a, "DPMAPP_OTG CB" },
	{ 0x3100003b, "DIAG CB" },
	{ 0x3100003c, "GSTK_EXP CB" },
	{ 0x3100003d, "DSBC_MDM_APIS CB" },
	{ 0x3100003e, "HDRMRLP_MDM_APIS CB" },
	{ 0x3100003f, "HDRMRLP_APPS_APIS CB" },
	{ 0x31000040, "HDRMC_MRLP_APIS CB" },
	{ 0x31000041, "PDCOMM_APP_API CB" },
	{ 0x31000042, "DSAT_APIS CB" },
	{ 0x31000043, "MISC_RF_APIS CB" },
	{ 0x31000044, "CMIPAPP CB" },
	{ 0x31000045, "DSMP_UMTS_MODEM_APIS CB" },
	{ 0x31000046, "DSMP_UMTS_APPS_APIS CB" },
	{ 0x31000047, "DSUCSDMPSHIM CB" },
	{ 0x31000048, "TIME_REMOTE_ATOM CB" },
	{ 0x3100004a, "SD CB" },
	{ 0x3100004b, "MMOC CB" },
	{ 0x3100004c, "WLAN_ADP_FTM CB" },
	{ 0x3100004d, "WLAN_CP_CM CB" },
	{ 0x3100004e, "FTM_WLAN CB" },
	{ 0x3100004f, "SDCC_CPRM CB" },
	{ 0x31000050, "CPRMINTERFACE CB" },
	{ 0x31000051, "DATA_ON_MODEM_MTOA_APIS CB" },
	{ 0x31000052, "DATA_ON_APPS_ATOM_APIS CB" },
	{ 0x31000053, "MISC_APIS_NONWINMOB CB" },
	{ 0x31000054, "MISC_APPS_APIS_NONWINMOB CB" },
	{ 0x31000055, "PMEM_REMOTE CB" },
	{ 0x31000056, "TCXOMGR CB" },
	{ 0x31000057, "DSUCSDAPPIF_APIS CB" },
	{ 0x31000058, "BT CB" },
	{ 0x31000059, "PD_COMMS_API CB" },
	{ 0x3100005a, "PD_COMMS_CLIENT_API CB" },
	{ 0x3100005b, "PDAPI CB" },
	{ 0x3100005c, "LSA_SUPL_DSM CB" },
	{ 0x3100005d, "TIME_REMOTE_MTOA CB" },
	{ 0x3100005e, "FTM_BT CB" },
	{ 0X3100005f, "DSUCSDAPPIF_APIS CB" },
	{ 0X31000060, "PMAPP_GEN CB" },
	{ 0X31000061, "PM_LIB CB" },
	{ 0X31000062, "KEYPAD CB" },
	{ 0X31000063, "HSU_APP_APIS CB" },
	{ 0X31000064, "HSU_MDM_APIS CB" },
	{ 0X31000065, "ADIE_ADC_REMOTE_ATOM CB" },
	{ 0X31000066, "TLMM_REMOTE_ATOM CB" },
	{ 0X31000067, "UI_CALLCTRL CB" },
	{ 0X31000068, "UIUTILS CB" },
	{ 0X31000069, "PRL CB" },
	{ 0X3100006a, "HW CB" },
	{ 0X3100006b, "OEM_RAPI CB" },
	{ 0X3100006c, "WMSPM CB" },
	{ 0X3100006d, "BTPF CB" },
	{ 0X3100006e, "CLKRGM_SYNC_EVENT CB" },
	{ 0X3100006f, "USB_APPS_RPC CB" },
	{ 0X31000070, "USB_MODEM_RPC CB" },
	{ 0X31000071, "ADC CB" },
	{ 0X31000072, "CAMERAREMOTED CB" },
	{ 0X31000073, "SECAPIREMOTED CB" },
	{ 0X31000074, "DSATAPI CB" },
	{ 0X31000075, "CLKCTL_RPC CB" },
	{ 0X31000076, "BREWAPPCOORD CB" },
	{ 0X31000077, "ALTENVSHELL CB" },
	{ 0X31000078, "WLAN_TRP_UTILS CB" },
	{ 0X31000079, "GPIO_RPC CB" },
	{ 0X3100007a, "PING_RPC CB" },
	{ 0X3100007b, "DSC_DCM_API CB" },
	{ 0X3100007c, "L1_DS CB" },
	{ 0X3100007d, "QCHATPK_APIS CB" },
	{ 0X3100007e, "GPS_API CB" },
	{ 0X3100007f, "OSS_RRCASN_REMOTE CB" },
	{ 0X31000080, "PMAPP_OTG_REMOTE CB" },
	{ 0X31000081, "PING_MDM_RPC CB" },
	{ 0X31000082, "PING_KERNEL_RPC CB" },
	{ 0X31000083, "TIMETICK CB" },
	{ 0X31000084, "WM_BTHCI_FTM CB" },
	{ 0X31000085, "WM_BT_PF CB" },
	{ 0X31000086, "IPA_IPC_APIS CB" },
	{ 0X31000087, "UKCC_IPC_APIS CB" },
	{ 0X31000088, "CMIPSMS CB" },
	{ 0X31000089, "VBATT_REMOTE CB" },
	{ 0X3100008a, "MFPAL CB" },
	{ 0X3100008b, "DSUMTSPDPREG CB" },
	{ 0X3100fe00, "RESTART_DAEMON NUMBER 0 CB" },
	{ 0X3100fe01, "RESTART_DAEMON NUMBER 1 CB" },
	{ 0X3100feff, "RESTART_DAEMON NUMBER 255 CB" },
	{ 0X3100fffe, "BACKWARDS_COMPATIBILITY_IN_RPC_CLNT_LOOKUP CB" },
	{ 0X3100ffff, "RPC_ROUTER_SERVER_PROGRAM CB" },
};

struct sym wakeup_syms[] = {
	{ 0x00000040, "OTHER" },
	{ 0x00000020, "RESET" },
	{ 0x00000010, "ALARM" },
	{ 0x00000008, "TIMER" },
	{ 0x00000004, "GPIO" },
	{ 0x00000002, "INT" },
	{ 0x00000001, "RPC" },
	{ 0x00000000, "NONE" },
};

struct sym wakeup_int_syms[] = {
	{ 0, "MDDI_EXT" },
	{ 1, "MDDI_PRI" },
	{ 2, "MDDI_CLIENT"},
	{ 3, "USB_OTG" },
	{ 4, "I2CC" },
	{ 5, "SDC1_0" },
	{ 6, "SDC1_1" },
	{ 7, "SDC2_0" },
	{ 8, "SDC2_1" },
	{ 9, "ADSP_A9A11" },
	{ 10, "UART1" },
	{ 11, "UART2" },
	{ 12, "UART3" },
	{ 13, "DP_RX_DATA" },
	{ 14, "DP_RX_DATA2" },
	{ 15, "DP_RX_DATA3" },
	{ 16, "DM_UART" },
	{ 17, "DM_DP_RX_DATA" },
	{ 18, "KEYSENSE" },
	{ 19, "HSSD" },
	{ 20, "NAND_WR_ER_DONE" },
	{ 21, "NAND_OP_DONE" },
	{ 22, "TCHSCRN1" },
	{ 23, "TCHSCRN2" },
	{ 24, "TCHSCRN_SSBI" },
	{ 25, "USB_HS" },
	{ 26, "UART2_DM_RX" },
	{ 27, "UART2_DM" },
	{ 28, "SDC4_1" },
	{ 29, "SDC4_0" },
	{ 30, "SDC3_1" },
	{ 31, "SDC3_0" },
};

struct sym smsm_syms[] = {
	{ 0x80000000, "UN" },
	{ 0x7F000000, "ERR" },
	{ 0x00800000, "SMLP" },
	{ 0x00400000, "ADWN" },
	{ 0x00200000, "PWRS" },
	{ 0x00100000, "DWLD" },
	{ 0x00080000, "SRBT" },
	{ 0x00040000, "SDWN" },
	{ 0x00020000, "ARBT" },
	{ 0x00010000, "REL" },
	{ 0x00008000, "SLE" },
	{ 0x00004000, "SLP" },
	{ 0x00002000, "WFPI" },
	{ 0x00001000, "EEX" },
	{ 0x00000800, "TIN" },
	{ 0x00000400, "TWT" },
	{ 0x00000200, "PWRC" },
	{ 0x00000100, "RUN" },
	{ 0x00000080, "SA" },
	{ 0x00000040, "RES" },
	{ 0x00000020, "RIN" },
	{ 0x00000010, "RWT" },
	{ 0x00000008, "SIN" },
	{ 0x00000004, "SWT" },
	{ 0x00000002, "OE" },
	{ 0x00000001, "I" },
};

/* never reorder */
struct sym voter_d2_syms[] = {
	{ 0x00000001, NULL },
	{ 0x00000002, NULL },
	{ 0x00000004, NULL },
	{ 0x00000008, NULL },
	{ 0x00000010, NULL },
	{ 0x00000020, NULL },
	{ 0x00000040, NULL },
	{ 0x00000080, NULL },
	{ 0x00000100, NULL },
	{ 0x00000200, NULL },
	{ 0x00000400, NULL },
	{ 0x00000800, NULL },
	{ 0x00001000, NULL },
	{ 0x00002000, NULL },
	{ 0x00004000, NULL },
	{ 0x00008000, NULL },
	{ 0x00010000, NULL },
	{ 0x00020000, NULL },
	{ 0x00040000, NULL },
	{ 0x00080000, NULL },
	{ 0x00100000, NULL },
	{ 0x00200000, NULL },
	{ 0x00400000, NULL },
	{ 0x00800000, NULL },
	{ 0x01000000, NULL },
	{ 0x02000000, NULL },
	{ 0x04000000, NULL },
	{ 0x08000000, NULL },
	{ 0x10000000, NULL },
	{ 0x20000000, NULL },
	{ 0x40000000, NULL },
	{ 0x80000000, NULL },
};

/* never reorder */
struct sym voter_d3_syms[] = {
	{ 0x00000001, NULL },
	{ 0x00000002, NULL },
	{ 0x00000004, NULL },
	{ 0x00000008, NULL },
	{ 0x00000010, NULL },
	{ 0x00000020, NULL },
	{ 0x00000040, NULL },
	{ 0x00000080, NULL },
	{ 0x00000100, NULL },
	{ 0x00000200, NULL },
	{ 0x00000400, NULL },
	{ 0x00000800, NULL },
	{ 0x00001000, NULL },
	{ 0x00002000, NULL },
	{ 0x00004000, NULL },
	{ 0x00008000, NULL },
	{ 0x00010000, NULL },
	{ 0x00020000, NULL },
	{ 0x00040000, NULL },
	{ 0x00080000, NULL },
	{ 0x00100000, NULL },
	{ 0x00200000, NULL },
	{ 0x00400000, NULL },
	{ 0x00800000, NULL },
	{ 0x01000000, NULL },
	{ 0x02000000, NULL },
	{ 0x04000000, NULL },
	{ 0x08000000, NULL },
	{ 0x10000000, NULL },
	{ 0x20000000, NULL },
	{ 0x40000000, NULL },
	{ 0x80000000, NULL },
};

struct sym dem_state_master_syms[] = {
	{ 0, "INIT" },
	{ 1, "RUN" },
	{ 2, "SLEEP_WAIT" },
	{ 3, "SLEEP_CONFIRMED" },
	{ 4, "SLEEP_EXIT" },
	{ 5, "RSA" },
	{ 6, "EARLY_EXIT" },
	{ 7, "RSA_DELAYED" },
	{ 8, "RSA_CHECK_INTS" },
	{ 9, "RSA_CONFIRMED" },
	{ 10, "RSA_WAKING" },
	{ 11, "RSA_RESTORE" },
	{ 12, "RESET" },
};

struct sym dem_state_slave_syms[] = {
	{ 0, "INIT" },
	{ 1, "RUN" },
	{ 2, "SLEEP_WAIT" },
	{ 3, "SLEEP_EXIT" },
	{ 4, "SLEEP_RUN_PENDING" },
	{ 5, "POWER_COLLAPSE" },
	{ 6, "CHECK_INTERRUPTS" },
	{ 7, "SWFI" },
	{ 8, "WFPI" },
	{ 9, "EARLY_EXIT" },
	{ 10, "RESET_RECOVER" },
	{ 11, "RESET_ACKNOWLEDGE" },
	{ 12, "ERROR" },
};

struct sym smsm_entry_type_syms[] = {
	{ 0, "SMSM_APPS_STATE" },
	{ 1, "SMSM_MODEM_STATE" },
	{ 2, "SMSM_Q6_STATE" },
	{ 3, "SMSM_APPS_DEM" },
	{ 4, "SMSM_MODEM_DEM" },
	{ 5, "SMSM_Q6_DEM" },
	{ 6, "SMSM_POWER_MASTER_DEM" },
	{ 7, "SMSM_TIME_MASTER_DEM" },
};

struct sym smsm_state_syms[] = {
	{ 0x00000001, "INIT" },
	{ 0x00000002, "OSENTERED" },
	{ 0x00000004, "SMDWAIT" },
	{ 0x00000008, "SMDINIT" },
	{ 0x00000010, "RPCWAIT" },
	{ 0x00000020, "RPCINIT" },
	{ 0x00000040, "RESET" },
	{ 0x00000080, "RSA" },
	{ 0x00000100, "RUN" },
	{ 0x00000200, "PWRC" },
	{ 0x00000400, "TIMEWAIT" },
	{ 0x00000800, "TIMEINIT" },
	{ 0x00001000, "PWRC_EARLY_EXIT" },
	{ 0x00002000, "WFPI" },
	{ 0x00004000, "SLEEP" },
	{ 0x00008000, "SLEEPEXIT" },
	{ 0x00010000, "OEMSBL_RELEASE" },
	{ 0x00020000, "APPS_REBOOT" },
	{ 0x00040000, "SYSTEM_POWER_DOWN" },
	{ 0x00080000, "SYSTEM_REBOOT" },
	{ 0x00100000, "SYSTEM_DOWNLOAD" },
	{ 0x00200000, "PWRC_SUSPEND" },
	{ 0x00400000, "APPS_SHUTDOWN" },
	{ 0x00800000, "SMD_LOOPBACK" },
	{ 0x01000000, "RUN_QUIET" },
	{ 0x02000000, "MODEM_WAIT" },
	{ 0x04000000, "MODEM_BREAK" },
	{ 0x08000000, "MODEM_CONTINUE" },
	{ 0x80000000, "UNKNOWN" },
};

#define ID_SYM 0
#define BASE_SYM 1
#define EVENT_SYM 2
#define ONCRPC_SYM 3
#define WAKEUP_SYM 4
#define WAKEUP_INT_SYM 5
#define SMSM_SYM 6
#define VOTER_D2_SYM 7
#define VOTER_D3_SYM 8
#define DEM_STATE_MASTER_SYM 9
#define DEM_STATE_SLAVE_SYM 10
#define SMSM_ENTRY_TYPE_SYM 11
#define SMSM_STATE_SYM 12

static struct sym_tbl {
	struct sym *data;
	int size;
	struct hlist_head hlist[HSIZE];
} tbl[] = {
	{ id_syms, ARRAY_SIZE(id_syms) },
	{ base_syms, ARRAY_SIZE(base_syms) },
	{ event_syms, ARRAY_SIZE(event_syms) },
	{ oncrpc_syms, ARRAY_SIZE(oncrpc_syms) },
	{ wakeup_syms, ARRAY_SIZE(wakeup_syms) },
	{ wakeup_int_syms, ARRAY_SIZE(wakeup_int_syms) },
	{ smsm_syms, ARRAY_SIZE(smsm_syms) },
	{ voter_d2_syms, ARRAY_SIZE(voter_d2_syms) },
	{ voter_d3_syms, ARRAY_SIZE(voter_d3_syms) },
	{ dem_state_master_syms, ARRAY_SIZE(dem_state_master_syms) },
	{ dem_state_slave_syms, ARRAY_SIZE(dem_state_slave_syms) },
	{ smsm_entry_type_syms, ARRAY_SIZE(smsm_entry_type_syms) },
	{ smsm_state_syms, ARRAY_SIZE(smsm_state_syms) },
};

static void find_voters(void)
{
	void *x, *next;
	unsigned size;
	int i = 0, j = 0;

	x = smem_item(SMEM_SLEEP_STATIC, &size);
	next = x;
	while (next && (next < (x + size)) &&
	       ((i + j) < (ARRAY_SIZE(voter_d3_syms) +
			   ARRAY_SIZE(voter_d2_syms)))) {

		if (i < ARRAY_SIZE(voter_d3_syms)) {
			voter_d3_syms[i].str = (char *) next;
			i++;
		} else if (i >= ARRAY_SIZE(voter_d3_syms) &&
			   j < ARRAY_SIZE(voter_d2_syms)) {
			voter_d2_syms[j].str = (char *) next;
			j++;
		}

		next += 9;
	}
}

#define hash(val) (val % HSIZE)

static void init_syms(void)
{
	int i;
	int j;

	for (i = 0; i < ARRAY_SIZE(tbl); ++i)
		for (j = 0; j < HSIZE; ++j)
			INIT_HLIST_HEAD(&tbl[i].hlist[j]);

	for (i = 0; i < ARRAY_SIZE(tbl); ++i)
		for (j = 0; j < tbl[i].size; ++j) {
			INIT_HLIST_NODE(&tbl[i].data[j].node);
			hlist_add_head(&tbl[i].data[j].node,
				       &tbl[i].hlist[hash(tbl[i].data[j].val)]);
		}
}

static char *find_sym(uint32_t id, uint32_t val)
{
	struct hlist_node *n;
	struct sym *s;

	hlist_for_each(n, &tbl[id].hlist[hash(val)]) {
		s = hlist_entry(n, struct sym, node);
		if (s->val == val)
			return s->str;
	}

	return 0;
}

#else
static void init_syms(void) {}
#endif

static inline unsigned int read_timestamp(void)
{
	unsigned int tick = 0;

	do {
		tick = readl(TIMESTAMP_ADDR);
	} while (tick != readl(TIMESTAMP_ADDR));

	return tick;
}

static void smem_log_event_from_user(struct smem_log_inst *inst,
				     const char __user *buf, int size, int num)
{
	uint32_t idx;
	uint32_t next_idx;
	unsigned long flags;
	uint32_t identifier = 0;
	uint32_t timetick = 0;
	int first = 1;
	int ret;

	remote_spin_lock_irqsave(inst->remote_spinlock, flags);

	while (num--) {
		idx = *inst->idx;

		if (idx < inst->num) {
			ret = copy_from_user(&inst->events[idx],
					     buf, size);
			if (ret) {
				printk("ERROR %s:%i tried to write "
				       "%i got ret %i",
				       __func__, __LINE__,
				       size, size - ret);
				goto out;
			}

			if (first) {
				identifier =
					inst->events[idx].
					identifier;
				timetick = read_timestamp();
				first = 0;
			} else {
				identifier |= SMEM_LOG_CONT;
			}
			inst->events[idx].identifier =
				identifier;
			inst->events[idx].timetick =
				timetick;
		}

		next_idx = idx + 1;
		if (next_idx >= inst->num)
			next_idx = 0;
		*inst->idx = next_idx;

		buf += sizeof(struct smem_log_item);
	}

 out:
	remote_spin_unlock_irqrestore(inst->remote_spinlock, flags);
}

static void _smem_log_event(
	struct smem_log_item __iomem *events,
	uint32_t __iomem *_idx,
	remote_spinlock_t *lock,
	int num,
	uint32_t id, uint32_t data1, uint32_t data2,
	uint32_t data3)
{
	struct smem_log_item item;
	uint32_t idx;
	uint32_t next_idx;
	unsigned long flags;

	item.timetick = read_timestamp();
	item.identifier = id;
	item.data1 = data1;
	item.data2 = data2;
	item.data3 = data3;

	remote_spin_lock_irqsave(lock, flags);

	idx = *_idx;

	if (idx < num) {
		memcpy(&events[idx],
		       &item, sizeof(item));
	}

	next_idx = idx + 1;
	if (next_idx >= num)
		next_idx = 0;
	*_idx = next_idx;

	remote_spin_unlock_irqrestore(lock, flags);
}

static void _smem_log_event6(
	struct smem_log_item __iomem *events,
	uint32_t __iomem *_idx,
	remote_spinlock_t *lock,
	int num,
	uint32_t id, uint32_t data1, uint32_t data2,
	uint32_t data3, uint32_t data4, uint32_t data5,
	uint32_t data6)
{
	struct smem_log_item item[2];
	uint32_t idx;
	uint32_t next_idx;
	unsigned long flags;

	item[0].timetick = read_timestamp();
	item[0].identifier = id;
	item[0].data1 = data1;
	item[0].data2 = data2;
	item[0].data3 = data3;
	item[1].identifier = item[0].identifier;
	item[1].timetick = item[0].timetick;
	item[1].data1 = data4;
	item[1].data2 = data5;
	item[1].data3 = data6;

	remote_spin_lock_irqsave(lock, flags);

	idx = *_idx;

	if (idx < (num-1)) {
		memcpy(&events[idx],
		       &item, sizeof(item));
	}

	next_idx = idx + 2;
	if (next_idx >= num)
		next_idx = 0;
	*_idx = next_idx;

	remote_spin_unlock_irqrestore(lock, flags);
}

void smem_log_event(uint32_t id, uint32_t data1, uint32_t data2,
		    uint32_t data3)
{
	_smem_log_event(inst[GEN].events, inst[GEN].idx,
			inst[GEN].remote_spinlock, SMEM_LOG_NUM_ENTRIES,
			id, data1, data2, data3);
}

void smem_log_event6(uint32_t id, uint32_t data1, uint32_t data2,
		     uint32_t data3, uint32_t data4, uint32_t data5,
		     uint32_t data6)
{
	_smem_log_event6(inst[GEN].events, inst[GEN].idx,
			 inst[GEN].remote_spinlock, SMEM_LOG_NUM_ENTRIES,
			 id, data1, data2, data3, data4, data5, data6);
}

void smem_log_event_to_static(uint32_t id, uint32_t data1, uint32_t data2,
		    uint32_t data3)
{
	_smem_log_event(inst[STA].events, inst[STA].idx,
			inst[STA].remote_spinlock, SMEM_LOG_NUM_STATIC_ENTRIES,
			id, data1, data2, data3);
}

void smem_log_event6_to_static(uint32_t id, uint32_t data1, uint32_t data2,
		     uint32_t data3, uint32_t data4, uint32_t data5,
		     uint32_t data6)
{
	_smem_log_event6(inst[STA].events, inst[STA].idx,
			 inst[STA].remote_spinlock, SMEM_LOG_NUM_STATIC_ENTRIES,
			 id, data1, data2, data3, data4, data5, data6);
}

static int _smem_log_init(void)
{
	inst[GEN].which_log = GEN;
	inst[GEN].events =
		(struct smem_log_item *)smem_alloc(SMEM_SMEM_LOG_EVENTS,
						  SMEM_LOG_EVENTS_SIZE);
	inst[GEN].idx = (uint32_t *)smem_alloc(SMEM_SMEM_LOG_IDX,
					     sizeof(uint32_t));
	if (!inst[GEN].events || !inst[GEN].idx) {
		pr_err("%s: no log or log_idx allocated, "
		       "smem_log disabled\n", __func__);
	}
	inst[GEN].num = SMEM_LOG_NUM_ENTRIES;
	inst[GEN].remote_spinlock = &remote_spinlock;

	inst[STA].which_log = STA;
	inst[STA].events =
		(struct smem_log_item *)
		smem_alloc(SMEM_SMEM_STATIC_LOG_EVENTS,
			   SMEM_STATIC_LOG_EVENTS_SIZE);
	inst[STA].idx = (uint32_t *)smem_alloc(SMEM_SMEM_STATIC_LOG_IDX,
						     sizeof(uint32_t));
	if (!inst[STA].events || !inst[STA].idx) {
		pr_err("%s: no static log or log_idx "
		       "allocated, smem_log disabled\n", __func__);
	}
	inst[STA].num = SMEM_LOG_NUM_STATIC_ENTRIES;
	inst[STA].remote_spinlock = &remote_spinlock_static;

	inst[POW].which_log = POW;
#ifdef CONFIG_MSM_N_WAY_SMD

	inst[POW].events =
		(struct smem_log_item *)
		smem_alloc(SMEM_SMEM_LOG_POWER_EVENTS,
			   SMEM_POWER_LOG_EVENTS_SIZE);
	inst[POW].idx = (uint32_t *)smem_alloc(SMEM_SMEM_LOG_POWER_IDX,
						     sizeof(uint32_t));
#else
	inst[POW].events = NULL;
	inst[POW].idx = NULL;
#endif
	if (!inst[POW].events || !inst[POW].idx) {
		pr_err("%s: no power log or log_idx "
		       "allocated, smem_log disabled\n", __func__);
	}
	inst[POW].num = SMEM_LOG_NUM_POWER_ENTRIES;
	inst[POW].remote_spinlock = &remote_spinlock;

	remote_spin_lock_init(&remote_spinlock,
			      SMEM_SPINLOCK_SMEM_LOG);
	remote_spin_lock_init(&remote_spinlock_static,
			      SMEM_SPINLOCK_STATIC_LOG);

	init_syms();

	return 0;
}

static ssize_t smem_log_read_bin(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	int idx;
	int orig_idx;
	unsigned long flags;
	int ret;
	int tot_bytes = 0;
	struct smem_log_inst *inst;

	inst = fp->private_data;

	remote_spin_lock_irqsave(inst->remote_spinlock, flags);

	orig_idx = *inst->idx;
	idx = orig_idx;

	while (1) {
		idx--;
		if (idx < 0)
			idx = inst->num - 1;
		if (idx == orig_idx) {
			ret = tot_bytes;
			break;
		}

		if ((tot_bytes + sizeof(struct smem_log_item)) > count) {
			ret = tot_bytes;
			break;
		}

		ret = copy_to_user(buf, &inst[GEN].events[idx],
				   sizeof(struct smem_log_item));
		if (ret) {
			ret = -EIO;
			break;
		}

		tot_bytes += sizeof(struct smem_log_item);

		buf += sizeof(struct smem_log_item);
	}

	remote_spin_unlock_irqrestore(inst->remote_spinlock, flags);

	return ret;
}

static ssize_t smem_log_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	char loc_buf[128];
	int i;
	int idx;
	int orig_idx;
	unsigned long flags;
	int ret;
	int tot_bytes = 0;
	struct smem_log_inst *inst;

	inst = fp->private_data;

	remote_spin_lock_irqsave(inst->remote_spinlock, flags);

	orig_idx = *inst->idx;
	idx = orig_idx;

	while (1) {
		idx--;
		if (idx < 0)
			idx = inst->num - 1;
		if (idx == orig_idx) {
			ret = tot_bytes;
			break;
		}

		i = scnprintf(loc_buf, 128,
			      "0x%x 0x%x 0x%x 0x%x 0x%x\n",
			      inst->events[idx].identifier,
			      inst->events[idx].timetick,
			      inst->events[idx].data1,
			      inst->events[idx].data2,
			      inst->events[idx].data3);
		if (i == 0) {
			ret = -EIO;
			break;
		}

		if ((tot_bytes + i) > count) {
			ret = tot_bytes;
			break;
		}

		tot_bytes += i;

		ret = copy_to_user(buf, loc_buf, i);
		if (ret) {
			ret = -EIO;
			break;
		}

		buf += i;
	}

	remote_spin_unlock_irqrestore(inst->remote_spinlock, flags);

	return ret;
}

static ssize_t smem_log_write_bin(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	if (count < sizeof(struct smem_log_item))
		return -EINVAL;

	smem_log_event_from_user(fp->private_data, buf,
				 sizeof(struct smem_log_item),
				 count / sizeof(struct smem_log_item));

	return count;
}

static ssize_t smem_log_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	int ret;
	const char delimiters[] = " ,;";
	char locbuf[256] = {0};
	uint32_t val[10];
	int vals = 0;
	char *token;
	char *running;
	struct smem_log_inst *inst;
	unsigned long res;

	inst = fp->private_data;

	if (count < 0) {
		printk(KERN_ERR "ERROR: %s passed neg count = %i\n",
		       __func__, count);
		return -EINVAL;
	}

	count = count > 255 ? 255 : count;

	locbuf[count] = '\0';

	ret = copy_from_user(locbuf, buf, count);
	if (ret != 0) {
		printk(KERN_ERR "ERROR: %s could not copy %i bytes\n",
		       __func__, ret);
		return -EINVAL;
	}

	D(KERN_ERR "%s: ", __func__);
	D_DUMP_BUFFER("We got", len, locbuf);

	running = locbuf;

	token = strsep(&running, delimiters);
	while (token && vals < ARRAY_SIZE(val)) {
		if (*token != '\0') {
			D(KERN_ERR "%s: ", __func__);
			D_DUMP_BUFFER("", strlen(token), token);
			ret = strict_strtoul(token, 0, &res);
			if (ret) {
				printk(KERN_ERR "ERROR: %s:%i got bad char "
				       "at strict_strtoul\n",
				       __func__, __LINE__-4);
				return -EINVAL;
			}
			val[vals++] = res;
		}
		token = strsep(&running, delimiters);
	}

	if (vals > 5) {
		if (inst->which_log == GEN)
			smem_log_event6(val[0], val[2], val[3], val[4],
					val[7], val[8], val[9]);
		else if (inst->which_log == STA)
			smem_log_event6_to_static(val[0],
						  val[2], val[3], val[4],
						  val[7], val[8], val[9]);
		else
			return -1;
	} else {
		if (inst->which_log == GEN)
			smem_log_event(val[0], val[2], val[3], val[4]);
		else if (inst->which_log == STA)
			smem_log_event_to_static(val[0],
						 val[2], val[3], val[4]);
		else
			return -1;
	}

	return count;
}

static int smem_log_open(struct inode *ip, struct file *fp)
{
	fp->private_data = &inst[GEN];

	return 0;
}


static int smem_log_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static int smem_log_ioctl(struct inode *ip, struct file *fp,
			  unsigned int cmd, unsigned long arg);

static const struct file_operations smem_log_fops = {
	.owner = THIS_MODULE,
	.read = smem_log_read,
	.write = smem_log_write,
	.open = smem_log_open,
	.release = smem_log_release,
	.ioctl = smem_log_ioctl,
};

static const struct file_operations smem_log_bin_fops = {
	.owner = THIS_MODULE,
	.read = smem_log_read_bin,
	.write = smem_log_write_bin,
	.open = smem_log_open,
	.release = smem_log_release,
	.ioctl = smem_log_ioctl,
};

static int smem_log_ioctl(struct inode *ip, struct file *fp,
			  unsigned int cmd, unsigned long arg)
{
	struct smem_log_inst *inst;

	inst = fp->private_data;

	switch (cmd) {
	default:
		return -ENOTTY;

	case SMIOC_SETMODE:
		if (arg == SMIOC_TEXT) {
			D("%s set text mode\n", __func__);
			fp->f_op = &smem_log_fops;
		} else if (arg == SMIOC_BINARY) {
			D("%s set bin mode\n", __func__);
			fp->f_op = &smem_log_bin_fops;
		} else {
			return -EINVAL;
		}
		break;
	case SMIOC_SETLOG:
		if (arg == SMIOC_LOG)
			fp->private_data = &inst[GEN];
		else if (arg == SMIOC_STATIC_LOG)
			fp->private_data = &inst[STA];
		else
			return -EINVAL;
		break;
	}

	return 0;
}

static struct miscdevice smem_log_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smem_log",
	.fops = &smem_log_fops,
};

#if defined(CONFIG_DEBUG_FS)

static int _debug_dump(int log, char *buf, int max)
{
	unsigned int idx;
	int orig_idx;
	unsigned long flags;
	int i = 0;

	if (!inst[log].events)
		return 0;

	remote_spin_lock_irqsave(inst[log].remote_spinlock, flags);

	orig_idx = *inst[log].idx;
	idx = orig_idx;

	while (1) {
		idx++;
		if (idx > inst[log].num - 1)
			idx = 0;
		if (idx == orig_idx)
			break;

		if (idx < inst[log].num) {
			if (!inst[log].events[idx].identifier)
				continue;

			i += scnprintf(buf + i, max - i,
			       "%08x %08x %08x %08x %08x\n",
			       inst[log].events[idx].identifier,
			       inst[log].events[idx].timetick,
			       inst[log].events[idx].data1,
			       inst[log].events[idx].data2,
			       inst[log].events[idx].data3);
		}
	}

	remote_spin_unlock_irqrestore(inst[log].remote_spinlock, flags);

	return i;
}

static int _debug_dump_sym(int log, char *buf, int max)
{
	unsigned int idx;
	int orig_idx;
	unsigned long flags;
	int i = 0;

	char *proc;
	char *sub;
	char *id;
	char *sym = NULL;

	uint32_t data[3];

	uint32_t proc_val = 0;
	uint32_t sub_val = 0;
	uint32_t id_val = 0;
	uint32_t id_only_val = 0;
	uint32_t data1 = 0;
	uint32_t data2 = 0;
	uint32_t data3 = 0;

	int k;

	if (!inst[log].events)
		return 0;

	find_voters(); /* need to call each time in case voters come and go */

	i += scnprintf(buf + i, max - i, "Voters:\n");
	for (k = 0; k < ARRAY_SIZE(voter_d3_syms); ++k)
		if (voter_d3_syms[k].str)
			i += scnprintf(buf + i, max - i, "%s ",
				       voter_d3_syms[k].str);
	for (k = 0; k < ARRAY_SIZE(voter_d2_syms); ++k)
		if (voter_d2_syms[k].str)
			i += scnprintf(buf + i, max - i, "%s ",
				       voter_d2_syms[k].str);
	i += scnprintf(buf + i, max - i, "\n");

	remote_spin_lock_irqsave(inst[log].remote_spinlock, flags);

	orig_idx = *inst[log].idx;
	idx = orig_idx;

	while (1) {
		idx++;
		if (idx > inst[log].num - 1)
			idx = 0;
		if (idx == orig_idx) {
			i += scnprintf(buf + i, max - i, "\n");
			break;
		}
		if (idx < inst[log].num) {
			if (!inst[log].events[idx].identifier)
				continue;

			proc_val = PROC & inst[log].events[idx].identifier;
			sub_val = SUB & inst[log].events[idx].identifier;
			id_val = (SUB | ID) & inst[log].events[idx].identifier;
			id_only_val = ID & inst[log].events[idx].identifier;
			data1 = inst[log].events[idx].data1;
			data2 = inst[log].events[idx].data2;
			data3 = inst[log].events[idx].data3;

			if (!(proc_val & SMEM_LOG_CONT)) {
				i += scnprintf(buf + i, max - i, "\n");

				proc = find_sym(ID_SYM, proc_val);

				if (proc)
					i += scnprintf(buf + i, max - i,
						       "%4s: ",
						       proc);
				else
					i += scnprintf(buf + i, max - i,
						       "%04x: ",
						       PROC &
						       inst[log].events[idx].
						       identifier);

				i += scnprintf(buf + i, max - i,
					       "%10u ",
					       inst[log].events[idx].timetick);

				sub = find_sym(BASE_SYM, sub_val);

				if (sub)
					i += scnprintf(buf + i, max - i,
						       "%9s: ",
						       sub);
				else
					i += scnprintf(buf + i, max - i,
						       "%08x: ",
						       sub_val);

				id = find_sym(EVENT_SYM, id_val);

				if (id)
					i += scnprintf(buf + i, max - i,
						       "%11s: ",
						       id);
				else
					i += scnprintf(buf + i, max - i,
						       "%08x: ",
						       id_only_val);
			}

			if ((proc_val & SMEM_LOG_CONT) &&
			    (id_val == ONCRPC_LOG_EVENT_STD_CALL ||
			     id_val == ONCRPC_LOG_EVENT_STD_REPLY)) {
				data[0] = data1;
				data[1] = data2;
				data[2] = data3;
				i += scnprintf(buf + i, max - i,
					       " %.16s",
					       (char *) data);
			} else if (proc_val & SMEM_LOG_CONT) {
				i += scnprintf(buf + i, max - i,
					       " %08x %08x %08x",
					       data1,
					       data2,
					       data3);
			} else if (id_val == ONCRPC_LOG_EVENT_STD_CALL) {
				sym = find_sym(ONCRPC_SYM, data2);

				if (sym)
					i += scnprintf(buf + i, max - i,
						       "xid:%4i %8s proc:%3i",
						       data1,
						       sym,
						       data3);
				else
					i += scnprintf(buf + i, max - i,
						       "xid:%4i %08x proc:%3i",
						       data1,
						       data2,
						       data3);
#if defined(CONFIG_MSM_N_WAY_SMSM)
			} else if (id_val == DEM_STATE_CHANGE) {
				if (data1 == 1) {
					i += scnprintf(buf + i,
						       max - i,
						       "MASTER: ");
					sym = find_sym(DEM_STATE_MASTER_SYM,
						       data2);
				} else if (data1 == 0) {
					i += scnprintf(buf + i,
						       max - i,
						       " SLAVE: ");
					sym = find_sym(DEM_STATE_SLAVE_SYM,
						       data2);
				} else {
					i += scnprintf(buf + i,
						       max - i,
						       "%x: ",
						       data1);
					sym = NULL;
				}
				if (sym)
					i += scnprintf(buf + i,
						       max - i,
						       "from:%s ",
						       sym);
				else
					i += scnprintf(buf + i,
						       max - i,
						       "from:0x%x ",
						       data2);

				if (data1 == 1)
					sym = find_sym(DEM_STATE_MASTER_SYM,
						       data3);
				else if (data1 == 0)
					sym = find_sym(DEM_STATE_SLAVE_SYM,
						       data3);
				else
					sym = NULL;
				if (sym)
					i += scnprintf(buf + i,
						       max - i,
						       "to:%s ",
						       sym);
				else
					i += scnprintf(buf + i,
						       max - i,
						       "to:0x%x ",
						       data3);

			} else if (id_val == DEM_STATE_MACHINE_ENTER) {
				i += scnprintf(buf + i,
					       max - i,
					       "swfi:%i timer:%i manexit:%i",
					       data1, data2, data3);

			} else if (id_val == DEM_TIME_SYNC_REQUEST ||
				   id_val == DEM_TIME_SYNC_POLL ||
				   id_val == DEM_TIME_SYNC_INIT) {
				sym = find_sym(SMSM_ENTRY_TYPE_SYM,
					       data1);
				if (sym)
					i += scnprintf(buf + i,
						       max - i,
						       "hostid:%s",
						       sym);
				else
					i += scnprintf(buf + i,
						       max - i,
						       "hostid:%x",
						       data1);

			} else if (id_val == DEM_TIME_SYNC_START ||
				   id_val == DEM_TIME_SYNC_SEND_VALUE) {
				unsigned mask = 0x1;
				unsigned tmp = 0;
				if (id_val == DEM_TIME_SYNC_START)
					i += scnprintf(buf + i,
						       max - i,
						       "req:");
				else
					i += scnprintf(buf + i,
						       max - i,
						       "pol:");
				while (mask) {
					if (mask & data1) {
						sym = find_sym(
							SMSM_ENTRY_TYPE_SYM,
							tmp);
						if (sym)
							i += scnprintf(buf + i,
								       max - i,
								       "%s ",
								       sym);
						else
							i += scnprintf(buf + i,
								       max - i,
								       "%i ",
								       tmp);
					}
					mask <<= 1;
					tmp++;
				}
				if (id_val == DEM_TIME_SYNC_SEND_VALUE)
					i += scnprintf(buf + i,
						       max - i,
						       "tick:%x",
						       data2);
			} else if (id_val == DEM_SMSM_ISR) {
				unsigned vals[] = {data2, data3};
				unsigned j;
				unsigned mask;
				unsigned tmp;
				unsigned once;
				sym = find_sym(SMSM_ENTRY_TYPE_SYM,
					       data1);
				if (sym)
					i += scnprintf(buf + i,
						       max - i,
						       "%s ",
						       sym);
				else
					i += scnprintf(buf + i,
						       max - i,
						       "%x ",
						       data1);

				for (j = 0; j < ARRAY_SIZE(vals); ++j) {
					i += scnprintf(buf + i, max - i, "[");
					mask = 0x80000000;
					once = 0;
					while (mask) {
						tmp = vals[j] & mask;
						mask >>= 1;
						if (!tmp)
							continue;
						sym = find_sym(SMSM_STATE_SYM,
							       tmp);

						if (once)
							i += scnprintf(buf + i,
								       max - i,
								       " ");
						if (sym)
							i += scnprintf(buf + i,
								       max - i,
								       "%s",
								       sym);
						else
							i += scnprintf(buf + i,
								       max - i,
								       "0x%x",
								       tmp);
						once = 1;
					}
					i += scnprintf(buf + i, max - i, "] ");
				}
#else
			} else if (id_val == DEMAPPS_WAKEUP_REASON) {
				unsigned mask = 0x80000000;
				unsigned tmp = 0;
				while (mask) {
					tmp = data1 & mask;
					mask >>= 1;
					if (!tmp)
						continue;
					sym = find_sym(WAKEUP_SYM, tmp);
					if (sym)
						i += scnprintf(buf + i,
							       max - i,
							       "%s ",
							       sym);
					else
						i += scnprintf(buf + i,
							       max - i,
							       "%08x ",
							       tmp);
				}
				i += scnprintf(buf + i, max - i,
					       "%08x %08x",
					       data2,
					       data3);
			} else if (id_val == DEMMOD_APPS_WAKEUP_INT) {
				sym = find_sym(WAKEUP_INT_SYM, data1);

				if (sym)
					i += scnprintf(buf + i, max - i,
						       "%s %08x %08x",
						       sym,
						       data2,
						       data3);
				else
					i += scnprintf(buf + i, max - i,
						       "%08x %08x %08x",
						       data1,
						       data2,
						       data3);
			} else if (id_val == DEM_NO_SLEEP ||
				   id_val == NO_SLEEP_NEW) {
				unsigned vals[] = {data3, data2};
				unsigned j;
				unsigned mask;
				unsigned tmp;
				unsigned once;
				i += scnprintf(buf + i, max - i, "%08x ",
					       data1);
				i += scnprintf(buf + i, max - i, "[");
				once = 0;
				for (j = 0; j < ARRAY_SIZE(vals); ++j) {
					mask = 0x00000001;
					while (mask) {
						tmp = vals[j] & mask;
						mask <<= 1;
						if (!tmp)
							continue;
						if (j == 0)
							sym = find_sym(
								VOTER_D3_SYM,
								tmp);
						else
							sym = find_sym(
								VOTER_D2_SYM,
								tmp);

						if (once)
							i += scnprintf(buf + i,
								       max - i,
								       " ");
						if (sym)
							i += scnprintf(buf + i,
								       max - i,
								       "%s",
								       sym);
						else
							i += scnprintf(buf + i,
								       max - i,
								       "%08x",
								       tmp);
						once = 1;
					}
				}
				i += scnprintf(buf + i, max - i, "] ");
#endif
			} else if (id_val == SMEM_LOG_EVENT_CB) {
				unsigned vals[] = {data2, data3};
				unsigned j;
				unsigned mask;
				unsigned tmp;
				unsigned once;
				i += scnprintf(buf + i, max - i, "%08x ",
					       data1);
				for (j = 0; j < ARRAY_SIZE(vals); ++j) {
					i += scnprintf(buf + i, max - i, "[");
					mask = 0x80000000;
					once = 0;
					while (mask) {
						tmp = vals[j] & mask;
						mask >>= 1;
						if (!tmp)
							continue;
						sym = find_sym(SMSM_SYM, tmp);

						if (once)
							i += scnprintf(buf + i,
								       max - i,
								       " ");
						if (sym)
							i += scnprintf(buf + i,
								       max - i,
								       "%s",
								       sym);
						else
							i += scnprintf(buf + i,
								       max - i,
								       "%08x",
								       tmp);
						once = 1;
					}
					i += scnprintf(buf + i, max - i, "] ");
				}
			} else {
				i += scnprintf(buf + i, max - i,
					       "%08x %08x %08x",
					       data1,
					       data2,
					       data3);
			}
		}
	}

	remote_spin_unlock_irqrestore(inst[log].remote_spinlock, flags);

	return i;
}

static int debug_dump(char *buf, int max)
{
	return _debug_dump(GEN, buf, max);
}

static int debug_dump_sym(char *buf, int max)
{
	return _debug_dump_sym(GEN, buf, max);
}

static int debug_dump_static(char *buf, int max)
{
	return _debug_dump(STA, buf, max);
}

static int debug_dump_static_sym(char *buf, int max)
{
	return _debug_dump_sym(STA, buf, max);
}

static int debug_dump_power(char *buf, int max)
{
	return _debug_dump(POW, buf, max);
}

static int debug_dump_power_sym(char *buf, int max)
{
	return _debug_dump_sym(POW, buf, max);
}

#define SMEM_LOG_ITEM_PRINT_SIZE 160

#define EVENTS_PRINT_SIZE \
(SMEM_LOG_ITEM_PRINT_SIZE * SMEM_LOG_NUM_ENTRIES)

static char debug_buffer[EVENTS_PRINT_SIZE];

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, EVENTS_PRINT_SIZE);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer,
				       bsize);
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

static void smem_log_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("smem_log", 0);
	if (IS_ERR(dent))
		return;

	debug_create("dump", 0444, dent, debug_dump);
	debug_create("dump_sym", 0444, dent, debug_dump_sym);
	debug_create("dump_static", 0444, dent, debug_dump_static);
	debug_create("dump_static_sym", 0444, dent, debug_dump_static_sym);
	debug_create("dump_power", 0444, dent, debug_dump_power);
	debug_create("dump_power_sym", 0444, dent, debug_dump_power_sym);
}
#else
static void smem_log_debugfs_init(void) {}
#endif

static int __init smem_log_init(void)
{
	int ret;

	ret = _smem_log_init();
	if (ret < 0)
		return ret;

	smem_log_debugfs_init();

	return misc_register(&smem_log_dev);
}


module_init(smem_log_init);

MODULE_DESCRIPTION("smem log");
MODULE_LICENSE("GPL v2");
