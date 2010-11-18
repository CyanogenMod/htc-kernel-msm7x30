/* arch/arm/mach-msm/include/mach/board.h
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

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <asm/setup.h>

/* platform device data structures */
struct msm_acpu_clock_platform_data {
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long mpll_khz;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
};

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
	uint32_t camifpadphy;
	uint32_t camifpadsz;
	uint32_t csiphy;
	uint32_t csisz;
	uint32_t csiirq;
};

struct msm_camera_device_platform_data {
	void (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	struct msm_camera_io_ext ioext;
};
enum msm_camera_csi_data_format {
	CSI_8BIT,
	CSI_10BIT,
	CSI_12BIT,
};
struct msm_camera_csi_params {
	enum msm_camera_csi_data_format data_format;
	uint8_t lane_cnt;
	uint8_t lane_assign;
	uint8_t settle_cnt;
	uint8_t dpcm_scheme;
};

struct msm_camera_legacy_device_platform_data {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
	struct msm_camsensor_device_platform_data *sensor_info;
};

struct msm_i2c_platform_data {
	int clk_freq;
#ifdef CONFIG_ARCH_MSM7X30 /* TODO.606 need camera team help to check */
	uint32_t rmutex;
	const char *rsl_id;
#else
	uint32_t *rmutex;
	int rsl_id;
#endif
	uint32_t pm_lat;
	int pri_clk;
	int pri_dat;
	int aux_clk;
	int aux_dat;
	const char *clk;
	const char *pclk;
	void (*msm_i2c_config_gpio)(int iface, int config_type);
};

#define MSM_CAMERA_FLASH_NONE 0
#define MSM_CAMERA_FLASH_LED  1
#define MSM_CAMERA_FLASH_SRC_PMIC (0x00000001<<0)
#define MSM_CAMERA_FLASH_SRC_PWM  (0x00000001<<1)

struct msm_camera_sensor_flash_pmic {
	uint32_t low_current;
	uint32_t high_current;
};

struct msm_camera_sensor_flash_pwm {
	uint32_t freq;
	uint32_t max_load;
	uint32_t low_load;
	uint32_t high_load;
	uint32_t channel;
};

struct msm_camera_sensor_flash_src {
	int flash_sr_type;

	union {
		struct msm_camera_sensor_flash_pmic pmic_src;
		struct msm_camera_sensor_flash_pwm pwm_src;
	} _fsrc;
};

struct msm_camera_sensor_flash_data {
	int flash_type;
	struct msm_camera_sensor_flash_src *flash_src;
};

struct camera_flash_cfg {
	int num_flash_levels;
	int (*camera_flash)(int level);
	uint16_t low_temp_limit;
	uint16_t low_cap_limit;
	uint8_t postpone_led_mode;
};

struct msm_camera_sensor_info {
	const char *sensor_name;
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void(*camera_clk_switch)(void);
	/*power*/
	char *camera_analog_pwd;
	char *camera_io_pwd;
	char *camera_vcm_pwd;
	char *camera_digital_pwd;
	int analog_pwd1_gpio;
	int (*camera_power_on)(void);
	int (*camera_power_off)(void);
	int (*camera_main_get_probe)(void);
	void (*camera_main_set_probe)(int);
	int mclk;
	int flash_type; /* for back support */
	uint8_t led_high_enabled;
	int need_suspend;
	struct msm_camera_device_platform_data *pdata;
	struct resource *resource;
	uint8_t num_resources;
	uint32_t waked_up;
	wait_queue_head_t event_wait;
	uint32_t kpi_sensor_start;
	uint32_t kpi_sensor_end;
	struct camera_flash_cfg* flash_cfg;
	int csi_if;
	struct msm_camera_csi_params csi_params;
	int sensor_lc_disable; /* for sensor lens correction support */
	uint8_t (*preview_skip_frame)(void);
};
struct clk;

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

enum {
	BOOTMODE_NORMAL = 	0x0,
	BOOTMODE_FACTORY = 	0x1,
	BOOTMODE_RECOVERY = 	0x2,
	BOOTMODE_CHARGE	= 	0x3,
	BOOTMODE_POWERTEST = 	0x4,
	BOOTMODE_OFFMODE_CHARGING = 0x5,
};

#define MSM_MAX_DEC_CNT 14
/* 7k target ADSP information */
/* Bit 23:0, for codec identification like mp3, wav etc *
 * Bit 27:24, for mode identification like tunnel, non tunnel*
 * bit 31:28, for operation support like DM, DMA */
enum msm_adspdec_concurrency {
	MSM_ADSP_CODEC_WAV = 0,
	MSM_ADSP_CODEC_ADPCM = 1,
	MSM_ADSP_CODEC_MP3 = 2,
	MSM_ADSP_CODEC_REALAUDIO = 3,
	MSM_ADSP_CODEC_WMA = 4,
	MSM_ADSP_CODEC_AAC = 5,
	MSM_ADSP_CODEC_RESERVED = 6,
	MSM_ADSP_CODEC_MIDI = 7,
	MSM_ADSP_CODEC_YADPCM = 8,
	MSM_ADSP_CODEC_QCELP = 9,
	MSM_ADSP_CODEC_AMRNB = 10,
	MSM_ADSP_CODEC_AMRWB = 11,
	MSM_ADSP_CODEC_EVRC = 12,
	MSM_ADSP_CODEC_WMAPRO = 13,
	MSM_ADSP_MODE_TUNNEL = 24,
	MSM_ADSP_MODE_NONTUNNEL = 25,
	MSM_ADSP_MODE_LP = 26,
	MSM_ADSP_OP_DMA = 28,
	MSM_ADSP_OP_DM = 29,
};

struct msm_adspdec_info {
	const char *module_name;
	unsigned module_queueid;
	int module_decid; /* objid */
	unsigned nr_codec_support;
};

/* Carries information about number codec
 * supported if same codec or different codecs
 */
struct dec_instance_table {
	uint8_t max_instances_same_dec;
	uint8_t max_instances_diff_dec;
};

struct msm_adspdec_database {
	unsigned num_dec;
	unsigned num_concurrency_support;
	unsigned int *dec_concurrency_table; /* Bit masked entry to *
					      *	represents codec, mode etc */
	struct msm_adspdec_info  *dec_info_list;
	struct dec_instance_table *dec_instance_list;
};

/* common init routines for use by arch/arm/mach-msm/board-*.c */
void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_init_irq(void);
void __init msm_clock_init(void);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);

#if defined(CONFIG_MSM_RMT_STORAGE_SERVER)
struct shared_ramfs_entry {
	uint32_t client_id;   	/* Client id to uniquely identify a client */
	uint32_t base_addr;	/* Base address of shared RAMFS memory */
	uint32_t size;		/* Size of the shared RAMFS memory */
	uint32_t server_status;	/* This will be initialized to 1 when
				   remote storage RPC server is available */
};
struct shared_ramfs_table {
	uint32_t magic_id;  	/* Identify RAMFS details in SMEM */
	uint32_t version;	/* Version of shared_ramfs_table */
	uint32_t entries;	/* Total number of valid entries   */
	struct shared_ramfs_entry ramfs_entry[3];	/* List all entries */
};

int __init rmt_storage_add_ramfs(void);
#endif

#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K)
void msm_hsusb_set_vbus_state(int online);
/* START: add USB connected notify function */
struct t_usb_status_notifier{
	struct list_head notifier_link;
	const char *name;
	void (*func)(int online);
};
	int usb_register_notifier(struct t_usb_status_notifier *);
	static LIST_HEAD(g_lh_usb_notifier_list);
/* END: add USB connected notify function */
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif

int __init parse_tag_skuid(const struct tag *tags);
int __init parse_tag_engineerid(const struct tag *tags);
int __init parse_tag_memsize(const struct tag *tags);
int board_mfg_mode(void);
void __init msm_snddev_init(void);
void msm_snddev_poweramp_on(void);
void msm_snddev_poweramp_off(void);
void msm_snddev_hsed_pamp_on(void);
void msm_snddev_hsed_pamp_off(void);
void msm_snddev_tx_route_config(void);
void msm_snddev_tx_route_deconfig(void);

extern int emmc_partition_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data);

#endif
