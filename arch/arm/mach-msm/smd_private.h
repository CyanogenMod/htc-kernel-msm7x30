/* arch/arm/mach-msm/smd_private.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
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

#if defined(CONFIG_ARCH_MSM7X30_LTE)
#include "7x30-lte/smd_private.h"
#elif defined(CONFIG_ARCH_MSM8X60)
#include "smd_private-8x60.h"
#endif

#ifndef _ARCH_ARM_MACH_MSM_MSM_SMD_PRIVATE_H_
#define _ARCH_ARM_MACH_MSM_MSM_SMD_PRIVATE_H_

struct smem_heap_info {
	unsigned initialized;
	unsigned free_offset;
	unsigned heap_remaining;
	unsigned reserved;
};

struct smem_heap_entry {
	unsigned allocated;
	unsigned offset;
	unsigned size;
	unsigned reserved;
};

struct smem_proc_comm {
	unsigned command;
	unsigned status;
	unsigned data1;
	unsigned data2;
};

#define PC_APPS  0
#define PC_MODEM 1

#define VERSION_SMD       0
#define VERSION_QDSP6     4
#define VERSION_APPS_SBL  6
#define VERSION_MODEM_SBL 7
#define VERSION_APPS      8
#define VERSION_MODEM     9

struct smem_shared {
	struct smem_proc_comm proc_comm[4];
	unsigned version[32];
	struct smem_heap_info heap_info;
	struct smem_heap_entry heap_toc[512];
};

#define SMSM_V1_SIZE		(sizeof(unsigned) * 8)
#define SMSM_V2_SIZE		(sizeof(unsigned) * 4)

#if defined(CONFIG_MSM_N_WAY_SMD)
#define DEM_MAX_PORT_NAME_LEN (20)
struct msm_dem_slave_data {
	uint32_t sleep_time;
	uint32_t interrupt_mask;
	uint32_t resources_used;
	uint32_t reserved1;

	uint32_t wakeup_reason;
	uint32_t pending_interrupts;
	uint32_t rpc_prog;
	uint32_t rpc_proc;
	char     smd_port_name[DEM_MAX_PORT_NAME_LEN];
	uint32_t reserved2;
};
#else
#define SMSM_MAX_PORT_NAME_LEN    20
struct smsm_interrupt_info {
	uint32_t interrupt_mask;
	uint32_t pending_interrupts;
	uint32_t wakeup_reason;
	uint32_t aArm_rpc_prog;
	uint32_t aArm_rpc_proc;
	char aArm_smd_port_name[SMSM_MAX_PORT_NAME_LEN];
	/* If the wakeup reason is GPIO then send the gpio info */
	uint32_t aArm_gpio_info;
	/*uint32_t interrupt_mask;
	uint32_t pending_interrupts;
	uint32_t wakeup_reason;*/
};
#endif

#if defined(CONFIG_MSM_N_WAY_SMSM)
enum {
	SMSM_APPS_STATE,
	SMSM_MODEM_STATE,
	SMSM_Q6_STATE,
	SMSM_APPS_DEM,
	SMSM_MODEM_DEM,
	SMSM_Q6_DEM,
	SMSM_POWER_MASTER_DEM,
	SMSM_TIME_MASTER_DEM,
	SMSM_NUM_ENTRIES,
};
#else
enum {
	SMSM_APPS_STATE = 1,
	SMSM_MODEM_STATE = 3,
	SMSM_NUM_ENTRIES,
};
#endif

enum {
	SMSM_APPS,
	SMSM_MODEM,
	SMSM_Q6,
	SMSM_NUM_HOSTS,
};

#define SZ_DIAG_ERR_MSG 0xC8
#define ID_DIAG_ERR_MSG SMEM_DIAG_ERR_MESSAGE
#define ID_SMD_CHANNELS SMEM_SMD_BASE_ID
#define ID_SHARED_STATE SMEM_SMSM_SHARED_STATE
#define ID_CH_ALLOC_TBL SMEM_CHANNEL_ALLOC_TBL

#define SMSM_INIT		0x00000001
#define SMSM_OSENTERED		0x00000002
#define SMSM_SMDWAIT		0x00000004
#define SMSM_SMDINIT		0x00000008
#define SMSM_RPCWAIT		0x00000010
#define SMSM_RPCINIT		0x00000020
#define SMSM_RESET		0x00000040
#define SMSM_RSA		0x00000080
#define SMSM_RUN		0x00000100
#define SMSM_PWRC		0x00000200
#define SMSM_TIMEWAIT		0x00000400
#define SMSM_TIMEINIT		0x00000800
#define SMSM_PWRC_EARLY_EXIT	0x00001000
#define SMSM_WFPI		0x00002000
#define SMSM_SLEEP		0x00004000
#define SMSM_SLEEPEXIT		0x00008000
#define SMSM_OEMSBL_RELEASE	0x00010000
#define SMSM_APPS_REBOOT	0x00020000
#define SMSM_SYSTEM_POWER_DOWN	0x00040000
#define SMSM_SYSTEM_REBOOT	0x00080000
#define SMSM_SYSTEM_DOWNLOAD	0x00100000
#define SMSM_PWRC_SUSPEND	0x00200000
#define SMSM_APPS_SHUTDOWN	0x00400000
#define SMSM_SMD_LOOPBACK	0x00800000
#define SMSM_RUN_QUIET		0x01000000
#define SMSM_MODEM_WAIT		0x02000000
#define SMSM_MODEM_BREAK	0x04000000
#define SMSM_MODEM_CONTINUE	0x08000000
#define SMSM_UNKNOWN		0x80000000

#define SMSM_WKUP_REASON_RPC	0x00000001
#define SMSM_WKUP_REASON_INT	0x00000002
#define SMSM_WKUP_REASON_GPIO	0x00000004
#define SMSM_WKUP_REASON_TIMER	0x00000008
#define SMSM_WKUP_REASON_ALARM	0x00000010
#define SMSM_WKUP_REASON_RESET	0x00000020

#if defined(CONFIG_MSM_N_WAY_SMD)
enum smsm_state_item {
	SMSM_STATE_APPS,
	SMSM_STATE_MODEM,
	SMSM_STATE_HEXAGON,
	SMSM_STATE_APPS_DEM,
	SMSM_STATE_MODEM_DEM,
	SMSM_STATE_QDSP6_DEM,
	SMSM_STATE_POWER_MASTER_DEM,
	SMSM_STATE_TIME_MASTER_DEM,
	SMSM_STATE_COUNT,
};
#else
enum smsm_state_item {
	SMSM_STATE_APPS = 1,
	SMSM_STATE_MODEM = 3,
	SMSM_STATE_COUNT,
};
#endif

void *smem_alloc(unsigned id, unsigned size);
int smsm_change_state(enum smsm_state_item item, uint32_t clear_mask, uint32_t set_mask);
uint32_t smsm_get_state(enum smsm_state_item item);
int smsm_set_sleep_duration(uint32_t delay);
int smsm_set_sleep_limit(uint32_t sleep_limit);
void smsm_print_sleep_info(unsigned wakeup_reason_only);

#define SMEM_NUM_SMD_CHANNELS        64

typedef enum {
	/* fixed items */
	SMEM_PROC_COMM = 0,
	SMEM_HEAP_INFO,
	SMEM_ALLOCATION_TABLE,
	SMEM_VERSION_INFO,
	SMEM_HW_RESET_DETECT,
	SMEM_AARM_WARM_BOOT,
	SMEM_DIAG_ERR_MESSAGE,
	SMEM_SPINLOCK_ARRAY,
	SMEM_MEMORY_BARRIER_LOCATION,

	/* dynamic items */
	SMEM_AARM_PARTITION_TABLE,
	SMEM_AARM_BAD_BLOCK_TABLE,
	SMEM_RESERVE_BAD_BLOCKS,
	SMEM_WM_UUID,
	SMEM_CHANNEL_ALLOC_TBL,
	SMEM_SMD_BASE_ID,
	SMEM_SMEM_LOG_IDX = SMEM_SMD_BASE_ID + SMEM_NUM_SMD_CHANNELS,
	SMEM_SMEM_LOG_EVENTS,
	SMEM_SMEM_STATIC_LOG_IDX,
	SMEM_SMEM_STATIC_LOG_EVENTS,
	SMEM_SMEM_SLOW_CLOCK_SYNC,
	SMEM_SMEM_SLOW_CLOCK_VALUE,
	SMEM_BIO_LED_BUF,
	SMEM_SMSM_SHARED_STATE,
	SMEM_SMSM_INT_INFO,
	SMEM_SMSM_SLEEP_DELAY,
	SMEM_SMSM_LIMIT_SLEEP,
	SMEM_SLEEP_POWER_COLLAPSE_DISABLED,
	SMEM_KEYPAD_KEYS_PRESSED,
	SMEM_KEYPAD_STATE_UPDATED,
	SMEM_KEYPAD_STATE_IDX,
	SMEM_GPIO_INT,
	SMEM_MDDI_LCD_IDX,
	SMEM_MDDI_HOST_DRIVER_STATE,
	SMEM_MDDI_LCD_DISP_STATE,
	SMEM_LCD_CUR_PANEL,
	SMEM_MARM_BOOT_SEGMENT_INFO,
	SMEM_AARM_BOOT_SEGMENT_INFO,
	SMEM_SLEEP_STATIC,
	SMEM_SCORPION_FREQUENCY,
	SMEM_SMD_PROFILES,
	SMEM_TSSC_BUSY,
	SMEM_HS_SUSPEND_FILTER_INFO,
	SMEM_BATT_INFO,
	SMEM_APPS_BOOT_MODE,
	SMEM_VERSION_FIRST,
	SMEM_VERSION_LAST = SMEM_VERSION_FIRST + 24,
	SMEM_OSS_RRCASN1_BUF1,
	SMEM_OSS_RRCASN1_BUF2,
	SMEM_ID_VENDOR0,
	SMEM_ID_VENDOR1,
	SMEM_ID_VENDOR2,
	SMEM_HW_SW_BUILD_ID,
#if defined(CONFIG_MSM_N_WAY_SMD)
	SMEM_SMD_BLOCK_PORT_BASE_ID,
	SMEM_SMD_BLOCK_PORT_PROC0_HEAP = SMEM_SMD_BLOCK_PORT_BASE_ID + SMEM_NUM_SMD_CHANNELS,
	SMEM_SMD_BLOCK_PORT_PROC1_HEAP = SMEM_SMD_BLOCK_PORT_PROC0_HEAP + SMEM_NUM_SMD_CHANNELS,
	SMEM_I2C_MUTEX = SMEM_SMD_BLOCK_PORT_PROC1_HEAP + SMEM_NUM_SMD_CHANNELS,
	SMEM_SCLK_CONVERSION,
	SMEM_SMD_SMSM_INTR_MUX,
	SMEM_SMSM_CPU_INTR_MASK,
	SMEM_APPS_DEM_SLAVE_DATA,
	SMEM_QDSP6_DEM_SLAVE_DATA,
	SMEM_CLKREGIM_BSP,
	SMEM_CLKREGIM_SOURCES,
	SMEM_SMD_FIFO_BASE_ID,
	SMEM_USABLE_RAM_PARTITION_TABLE = SMEM_SMD_FIFO_BASE_ID + SMEM_NUM_SMD_CHANNELS,
	SMEM_POWER_ON_STATUS_INFO,
	SMEM_DAL_AREA,
	SMEM_SMEM_LOG_POWER_IDX,
	SMEM_SMEM_LOG_POWER_WRAP,
	SMEM_SMEM_LOG_POWER_EVENTS,
	SMEM_ERR_CRASH_LOG,
	SMEM_ERR_F3_TRACE_LOG,
	SMEM_SMD_BRIDGE_ALLOC_TABLE,
	SMEM_SMDLITE_TABLE,
	SMEM_SD_IMG_UPGRADE_STATUS,
	SMEM_SEFS_INFO,
#else
	SMEM_SMD_FIFO_BASE_ID,
	SMEM_SMEM_LAST = SMEM_SMD_FIFO_BASE_ID + SMEM_NUM_SMD_CHANNELS,
#endif

	SMEM_NUM_ITEMS,
} smem_mem_type;


#define SMD_SS_CLOSED		0x00000000
#define SMD_SS_OPENING		0x00000001
#define SMD_SS_OPENED		0x00000002
#define SMD_SS_FLUSHING		0x00000003
#define SMD_SS_CLOSING		0x00000004
#define SMD_SS_RESET		0x00000005
#define SMD_SS_RESET_OPENING	0x00000006

#define SMD_BUF_SIZE		8192
#define SMD_CHANNELS		64

#define SMD_HEADER_SIZE		20

#define SMD_TYPE_MASK		0x0FF
#define SMD_TYPE_APPS_MODEM	0x000
#define SMD_TYPE_APPS_DSP	0x001
#define SMD_TYPE_MODEM_DSP	0x002

#define SMD_KIND_MASK		0xF00
#define SMD_KIND_UNKNOWN	0x000
#define SMD_KIND_STREAM		0x100
#define SMD_KIND_PACKET		0x200

void *smem_find(unsigned id, unsigned size);
void *smem_item(unsigned id, unsigned *size);
uint32_t raw_smsm_get_state(enum smsm_state_item item);

#endif
