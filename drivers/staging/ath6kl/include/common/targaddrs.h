//------------------------------------------------------------------------------
// Copyright (c) 2010 Atheros Corporation.  All rights reserved.
//
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//
//
//
// Author(s): ="Atheros"
//------------------------------------------------------------------------------

#ifndef __TARGADDRS_H__
#define __TARGADDRS_H__

#if defined(AR6002)
#include "AR6002/addrs.h"
#endif

/*
 * AR6K option bits, to enable/disable various features.
 * By default, all option bits are 0.
 * These bits can be set in LOCAL_SCRATCH register 0.
 */
#define AR6K_OPTION_BMI_DISABLE      0x01 /* Disable BMI comm with Host */
#define AR6K_OPTION_SERIAL_ENABLE    0x02 /* Enable serial port msgs */
#define AR6K_OPTION_WDT_DISABLE      0x04 /* WatchDog Timer override */
#define AR6K_OPTION_SLEEP_DISABLE    0x08 /* Disable system sleep */
#define AR6K_OPTION_STOP_BOOT        0x10 /* Stop boot processes (for ATE) */
#define AR6K_OPTION_ENABLE_NOANI     0x20 /* Operate without ANI */
#define AR6K_OPTION_DSET_DISABLE     0x40 /* Ignore DataSets */
#define AR6K_OPTION_IGNORE_FLASH     0x80 /* Ignore flash during bootup */

/*
 * xxx_HOST_INTEREST_ADDRESS is the address in Target RAM of the
 * host_interest structure.  It must match the address of the _host_interest
 * symbol (see linker script).
 *
 * Host Interest is shared between Host and Target in order to coordinate
 * between the two, and is intended to remain constant (with additions only
 * at the end) across software releases.
 *
 * All addresses are available here so that it's possible to
 * write a single binary that works with all Target Types.
 * May be used in assembler code as well as C.
 */
#define AR6002_HOST_INTEREST_ADDRESS    0x00500400
#define AR6003_HOST_INTEREST_ADDRESS    0x00540600


#define HOST_INTEREST_MAX_SIZE          0x100

#if !defined(__ASSEMBLER__)
struct register_dump_s;
struct dbglog_hdr_s;

/*
 * These are items that the Host may need to access
 * via BMI or via the Diagnostic Window. The position
 * of items in this structure must remain constant
 * across firmware revisions!
 *
 * Types for each item must be fixed size across
 * target and host platforms.
 *
 * More items may be added at the end.
 */
PREPACK struct host_interest_s {
    /*
     * Pointer to application-defined area, if any.
     * Set by Target application during startup.
     */
    u32 hi_app_host_interest;                      /* 0x00 */

    /* Pointer to register dump area, valid after Target crash. */
    u32 hi_failure_state;                          /* 0x04 */

    /* Pointer to debug logging header */
    u32 hi_dbglog_hdr;                             /* 0x08 */

    u32 hi_unused1;                       /* 0x0c */

    /*
     * General-purpose flag bits, similar to AR6000_OPTION_* flags.
     * Can be used by application rather than by OS.
     */
    u32 hi_option_flag;                            /* 0x10 */

    /*
     * Boolean that determines whether or not to
     * display messages on the serial port.
     */
    u32 hi_serial_enable;                          /* 0x14 */

    /* Start address of DataSet index, if any */
    u32 hi_dset_list_head;                         /* 0x18 */

    /* Override Target application start address */
    u32 hi_app_start;                              /* 0x1c */

    /* Clock and voltage tuning */
    u32 hi_skip_clock_init;                        /* 0x20 */
    u32 hi_core_clock_setting;                     /* 0x24 */
    u32 hi_cpu_clock_setting;                      /* 0x28 */
    u32 hi_system_sleep_setting;                   /* 0x2c */
    u32 hi_xtal_control_setting;                   /* 0x30 */
    u32 hi_pll_ctrl_setting_24ghz;                 /* 0x34 */
    u32 hi_pll_ctrl_setting_5ghz;                  /* 0x38 */
    u32 hi_ref_voltage_trim_setting;               /* 0x3c */
    u32 hi_clock_info;                             /* 0x40 */

    /*
     * Flash configuration overrides, used only
     * when firmware is not executing from flash.
     * (When using flash, modify the global variables
     * with equivalent names.)
     */
    u32 hi_bank0_addr_value;                       /* 0x44 */
    u32 hi_bank0_read_value;                       /* 0x48 */
    u32 hi_bank0_write_value;                      /* 0x4c */
    u32 hi_bank0_config_value;                     /* 0x50 */

    /* Pointer to Board Data  */
    u32 hi_board_data;                             /* 0x54 */
    u32 hi_board_data_initialized;                 /* 0x58 */

    u32 hi_dset_RAM_index_table;                   /* 0x5c */

    u32 hi_desired_baud_rate;                      /* 0x60 */
    u32 hi_dbglog_config;                          /* 0x64 */
    u32 hi_end_RAM_reserve_sz;                     /* 0x68 */
    u32 hi_mbox_io_block_sz;                       /* 0x6c */

    u32 hi_num_bpatch_streams;                     /* 0x70 -- unused */
    u32 hi_mbox_isr_yield_limit;                   /* 0x74 */

    u32 hi_refclk_hz;                              /* 0x78 */
    u32 hi_ext_clk_detected;                       /* 0x7c */
    u32 hi_dbg_uart_txpin;                         /* 0x80 */
    u32 hi_dbg_uart_rxpin;                         /* 0x84 */
    u32 hi_hci_uart_baud;                          /* 0x88 */
    u32 hi_hci_uart_pin_assignments;               /* 0x8C */
        /* NOTE: byte [0] = tx pin, [1] = rx pin, [2] = rts pin, [3] = cts pin */
    u32 hi_hci_uart_baud_scale_val;                /* 0x90 */
    u32 hi_hci_uart_baud_step_val;                 /* 0x94 */

    u32 hi_allocram_start;                         /* 0x98 */
    u32 hi_allocram_sz;                            /* 0x9c */
    u32 hi_hci_bridge_flags;                       /* 0xa0 */
    u32 hi_hci_uart_support_pins;                  /* 0xa4 */
        /* NOTE: byte [0] = RESET pin (bit 7 is polarity), bytes[1]..bytes[3] are for future use */
    u32 hi_hci_uart_pwr_mgmt_params;               /* 0xa8 */
	/*
	 * 0xa8   - [1]: 0 = UART FC active low, 1 = UART FC active high
	 *      [31:16]: wakeup timeout in ms
	 */

	/* Pointer to extended board data */
	u32 hi_board_ext_data;                /* 0xac */
	u32 hi_board_ext_data_config;         /* 0xb0 */

	/*
         * Bit [0]  :   valid
         * Bit[31:16:   size
	*/
	/*
	 * hi_reset_flag is used to do some stuff when target reset.
	 * such as restore app_start after warm reset or
	 * preserve host Interest area, or preserve ROM data, literals etc.
	 */
	u32 hi_reset_flag;                            /* 0xb4 */
	/* indicate hi_reset_flag is valid */
	u32 hi_reset_flag_valid;                      /* 0xb8 */
	u32 hi_hci_uart_pwr_mgmt_params_ext;           /* 0xbc */
	/*
	 * 0xbc - [31:0]: idle timeout in ms
	 */
	/* ACS flags */
	u32 hi_acs_flags;                              /* 0xc0 */
	u32 hi_console_flags;                          /* 0xc4 */
	u32 hi_nvram_state;                            /* 0xc8 */
	u32 hi_option_flag2;                           /* 0xcc */

	/* If non-zero, override values sent to Host in WMI_READY event. */
	u32 hi_sw_version_override;                    /* 0xd0 */
	u32 hi_abi_version_override;                   /* 0xd4 */

	/*
	 * Percentage of high priority RX traffic to total expected RX traffic -
	 * applicable only to ar6004
	 */
	u32 hi_hp_rx_traffic_ratio;                    /* 0xd8 */

	/* test applications flags */
	u32 hi_test_apps_related    ;                  /* 0xdc */
	/* location of test script */
	u32 hi_ota_testscript;                         /* 0xe0 */
	/* location of CAL data */
	u32 hi_cal_data;                               /* 0xe4 */
	/* Number of packet log buffers */
        u32 hi_pktlog_num_buffers;                     /* 0xe8 */

} POSTPACK;

/* Bits defined in hi_option_flag */
#define HI_OPTION_TIMER_WAR       0x01 /* Enable timer workaround */
#define HI_OPTION_BMI_CRED_LIMIT  0x02 /* Limit BMI command credits */
#define HI_OPTION_RELAY_DOT11_HDR 0x04 /* Relay Dot11 hdr to/from host */
/* MAC addr method 0-locally administred 1-globally unique addrs */
#define HI_OPTION_MAC_ADDR_METHOD   0x08
#define HI_OPTION_FW_BRIDGE         0x10 /* Firmware Bridging */
#define HI_OPTION_ENABLE_PROFILE    0x20 /* Enable CPU profiling */
#define HI_OPTION_DISABLE_DBGLOG    0x40 /* Disable debug logging */
#define HI_OPTION_SKIP_ERA_TRACKING 0x80 /* Skip Era Tracking */
#define HI_OPTION_PAPRD_DISABLE     0x100 /* Disable PAPRD (debug) */
#define HI_OPTION_NUM_DEV_LSB       0x200
#define HI_OPTION_NUM_DEV_MSB       0x800
#define HI_OPTION_DEV_MODE_LSB      0x1000
#define HI_OPTION_DEV_MODE_MSB      0x8000000
/* Disable LowFreq Timer Stabilization */
#define HI_OPTION_NO_LFT_STBL       0x10000000
#define HI_OPTION_SKIP_REG_SCAN     0x20000000 /* Skip regulatory scan */
/* Do regulatory scan during init beforesending WMI ready event to host */
#define HI_OPTION_INIT_REG_SCAN     0x40000000
#define HI_OPTION_SKIP_MEMMAP       0x80000000 /* REV6: Do not adjust memory
                                                map */

/* hi_option_flag2 options */
#define HI_OPTION_OFFLOAD_AMSDU     0x01
#define HI_OPTION_DFS_SUPPORT       0x02 /* Enable DFS support */

#define HI_OPTION_MAC_ADDR_METHOD_SHIFT 3

/* 2 bits of hi_option_flag are used to represent 3 modes */
#define HI_OPTION_FW_MODE_IBSS    0x0 /* IBSS Mode */
#define HI_OPTION_FW_MODE_BSS_STA 0x1 /* STA Mode */
#define HI_OPTION_FW_MODE_AP      0x2 /* AP Mode */

/* 2 bits of hi_option flag are usedto represent 4 submodes */
#define HI_OPTION_FW_SUBMODE_NONE    0x0  /* Normal mode */
#define HI_OPTION_FW_SUBMODE_P2PDEV  0x1  /* p2p device mode */
#define HI_OPTION_FW_SUBMODE_P2PCLIENT 0x2 /* p2p client mode */
#define HI_OPTION_FW_SUBMODE_P2PGO   0x3 /* p2p go mode */

/* Num dev Mask */
#define HI_OPTION_NUM_DEV_MASK    0x7
#define HI_OPTION_NUM_DEV_SHIFT   0x9

/* firmware bridging */
#define HI_OPTION_FW_BRIDGE_SHIFT 0x04

/* Fw Mode/SubMode Mask
|------------------------------------------------------------------------------|
|   SUB   |   SUB   |   SUB   |  SUB    |         |         |         |
| MODE[3] | MODE[2] | MODE[1] | MODE[0] | MODE[3] | MODE[2] | MODE[1] | MODE[0|
|   (2)   |   (2)   |   (2)   |   (2)   |   (2)   |   (2)   |   (2)   |   (2)
|------------------------------------------------------------------------------|
*/
#define HI_OPTION_FW_MODE_BITS         0x2
#define HI_OPTION_FW_MODE_MASK         0x3
#define HI_OPTION_FW_MODE_SHIFT        0xC
#define HI_OPTION_ALL_FW_MODE_MASK     0xFF

#define HI_OPTION_FW_SUBMODE_BITS      0x2
#define HI_OPTION_FW_SUBMODE_MASK      0x3
#define HI_OPTION_FW_SUBMODE_SHIFT     0x14
#define HI_OPTION_ALL_FW_SUBMODE_MASK  0xFF00
#define HI_OPTION_ALL_FW_SUBMODE_SHIFT 0x8

/* hi_reset_flag */

/* preserve App Start address */
#define HI_RESET_FLAG_PRESERVE_APP_START         0x01
/* preserve host interest */
#define HI_RESET_FLAG_PRESERVE_HOST_INTEREST     0x02
#define HI_RESET_FLAG_PRESERVE_ROMDATA           0x04  /* preserve ROM data */
#define HI_RESET_FLAG_PRESERVE_NVRAM_STATE       0x08
#define HI_RESET_FLAG_PRESERVE_BOOT_INFO         0x10

#define HI_RESET_FLAG_IS_VALID  0x12345678 /* indicate the reset flag is
valid */

#define ON_RESET_FLAGS_VALID() \
	(HOST_INTEREST->hi_reset_flag_valid == HI_RESET_FLAG_IS_VALID)

#define RESET_FLAGS_VALIDATE()  \
	(HOST_INTEREST->hi_reset_flag_valid = HI_RESET_FLAG_IS_VALID)

#define RESET_FLAGS_INVALIDATE() \
	(HOST_INTEREST->hi_reset_flag_valid = 0)

#define ON_RESET_PRESERVE_APP_START() \
        (HOST_INTEREST->hi_reset_flag & HI_RESET_FLAG_PRESERVE_APP_START)

#define ON_RESET_PRESERVE_NVRAM_STATE() \
        (HOST_INTEREST->hi_reset_flag & HI_RESET_FLAG_PRESERVE_NVRAM_STATE)

#define ON_RESET_PRESERVE_HOST_INTEREST() \
        (HOST_INTEREST->hi_reset_flag & HI_RESET_FLAG_PRESERVE_HOST_INTEREST)

#define ON_RESET_PRESERVE_ROMDATA() \
        (HOST_INTEREST->hi_reset_flag & HI_RESET_FLAG_PRESERVE_ROMDATA)

#define ON_RESET_PRESERVE_BOOT_INFO() \
        (HOST_INTEREST->hi_reset_flag & HI_RESET_FLAG_PRESERVE_BOOT_INFO)

#define HI_ACS_FLAGS_ENABLED        (1 << 0)    /* ACS is enabled */
#define HI_ACS_FLAGS_USE_WWAN       (1 << 1)    /* Use physical WWAN device */
#define HI_ACS_FLAGS_TEST_VAP       (1 << 2)    /* Use test VAP */

/* CONSOLE FLAGS
 *
 * Bit Range  Meaning
 * ---------  --------------------------------
 *   2..0     UART ID (0 = Default)
 *    3       Baud Select (0 = 9600, 1 = 115200)
 *   30..4    Reserved
 *    31      Enable Console
 *
 */

#define HI_CONSOLE_FLAGS_ENABLE       (1 << 31)
#define HI_CONSOLE_FLAGS_UART_MASK    (0x7)
#define HI_CONSOLE_FLAGS_UART_SHIFT   0
#define HI_CONSOLE_FLAGS_BAUD_SELECT  (1 << 3)

/*
 * Intended for use by Host software, this macro returns the Target RAM
 * address of any item in the host_interest structure.
 * Example: target_addr = AR6002_HOST_INTEREST_ITEM_ADDRESS(hi_board_data);
 */
#define AR6002_HOST_INTEREST_ITEM_ADDRESS(item) \
    (u32)((unsigned long)&((((struct host_interest_s *)(AR6002_HOST_INTEREST_ADDRESS))->item)))

#define AR6003_HOST_INTEREST_ITEM_ADDRESS(item) \
    (u32)((unsigned long)&((((struct host_interest_s *)(AR6003_HOST_INTEREST_ADDRESS))->item)))

#define AR6004_HOST_INTEREST_ITEM_ADDRESS(item) \
        ((u32)&((((struct host_interest_s *)(AR6004_HOST_INTEREST_ADDRESS))->item)))


#define HOST_INTEREST_DBGLOG_IS_ENABLED() \
        (!(HOST_INTEREST->hi_option_flag & HI_OPTION_DISABLE_DBGLOG))

#define HOST_INTEREST_PKTLOG_IS_ENABLED() \
        ((HOST_INTEREST->hi_pktlog_num_buffers))


#define HOST_INTEREST_PROFILE_IS_ENABLED() \
        (HOST_INTEREST->hi_option_flag & HI_OPTION_ENABLE_PROFILE)

#define LF_TIMER_STABILIZATION_IS_ENABLED() \
        (!(HOST_INTEREST->hi_option_flag & HI_OPTION_NO_LFT_STBL))

#define IS_AMSDU_OFFLAOD_ENABLED() \
        ((HOST_INTEREST->hi_option_flag2 & HI_OPTION_OFFLOAD_AMSDU))

#define HOST_INTEREST_DFS_IS_ENABLED() \
        ((HOST_INTEREST->hi_option_flag2 & HI_OPTION_DFS_SUPPORT))

/* Convert a Target virtual address into a Target physical address */
#define AR6002_VTOP(vaddr) ((vaddr) & 0x001fffff)
#define AR6003_VTOP(vaddr) ((vaddr) & 0x001fffff)
#define TARG_VTOP(TargetType, vaddr) \
        (((TargetType) == TARGET_TYPE_AR6002) ? AR6002_VTOP(vaddr) : AR6003_VTOP(vaddr))

#define AR6003_REV2_APP_START_OVERRIDE          0x944C00
#define AR6003_REV2_APP_LOAD_ADDRESS            0x543180
#define AR6003_REV2_BOARD_EXT_DATA_ADDRESS      0x57E500
#define AR6003_REV2_DATASET_PATCH_ADDRESS       0x57e884
#define AR6003_REV2_RAM_RESERVE_SIZE            6912

#define AR6003_REV3_APP_START_OVERRIDE          0x945d00
#define AR6003_REV3_APP_LOAD_ADDRESS            0x545000
#define AR6003_REV3_BOARD_EXT_DATA_ADDRESS      0x542330
#define AR6003_REV3_DATASET_PATCH_ADDRESS       0x57FF74
#define AR6003_REV3_RAM_RESERVE_SIZE            512

#define AR6003_BOARD_EXT_DATA_ADDRESS           0x57E600

/* # of u32 entries in targregs, used by DIAG_FETCH_TARG_REGS */
#define AR6003_FETCH_TARG_REGS_COUNT 64

#endif /* !__ASSEMBLER__ */

#endif /* __TARGADDRS_H__ */
