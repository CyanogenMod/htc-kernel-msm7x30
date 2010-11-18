/* include/asm/mach-msm/atmega_microp.h
 *
 * Copyright (C) 2009 HTC Corporation.
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


#ifndef _LINUX_ATMEGA_MICROP_H
#define _LINUX_ATMEGA_MICROP_H

#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>


#define MICROP_I2C_NAME "atmega-microp"

#define MICROP_FUNCTION_LSENSOR		1
#define MICROP_FUNCTION_REMOTEKEY	2
#define MICROP_FUNCTION_LCD_BL		3
#define MICROP_FUNCTION_RMK_VALUE	4
#define MICROP_FUNCTION_INTR		11
#define MICROP_FUNCTION_GSENSOR		12
#define MICROP_FUNCTION_LED		13
#define MICROP_FUNCTION_HPIN		14
#define MICROP_FUNCTION_RESET_INT	15
#define MICROP_FUNCTION_SIM_CARD	16
#define MICROP_FUNCTION_SDCARD		17
#define MICROP_FUNCTION_OJ		18
#define MICROP_FUNCTION_P		19

#define HEADSET_NO_MIC			0
#define HEADSET_MIC			1
#define HEADSET_METRICO			2

#define LED_RGB					(1 << 0)
#define LED_JOGBALL				(1 << 1)
#define LED_GPO					(1 << 2)
#define LED_PWM					(1 << 3)
#define LED_WIMAX				(1 << 4)
#define LED_MOBEAM				(1 << 5)

#define SPI_GSENSOR				(1 << 0)
#define SPI_LCM					(1 << 1)
#define SPI_OJ					(1 << 2)

#define LS_PWR_ON				(1 << 0)
#define PS_PWR_ON				(1 << 1)

#define ALS_BACKLIGHT				(1 << 0)
#define ALS_VKEY_LED				(1 << 1)

#define CMD_83_DIFF				(1 << 0)
#define CMD_25_DIFF				(1 << 1)

#define ALS_CALIBRATED				0x6DA5

#define MICROP_I2C_WCMD_MISC			0x20
#define MICROP_I2C_WCMD_SPI_EN			0x21
#define MICROP_I2C_WCMD_LCM_BL_MANU_CTL		0x22
#define MICROP_I2C_WCMD_AUTO_BL_CTL		0x23
#define MICROP_I2C_RCMD_SPI_BL_STATUS		0x24
#define MICROP_I2C_WCMD_LED_PWM			0x25
#define MICROP_I2C_WCMD_BL_EN			0x26
#define MICROP_I2C_RCMD_VERSION			0x30
#define MICROP_I2C_WCMD_ADC_TABLE		0x42
#define MICROP_I2C_WCMD_LED_MODE		0x53
#define MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME	0x54
#define MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME	0x55
#define MICROP_I2C_RCMD_LED_REMAIN_TIME		0x56
#define MICROP_I2C_RCMD_BLUE_LED_REMAIN_TIME	0x57
#define MICROP_I2C_RCMD_LED_STATUS		0x58
#define MICROP_I2C_WCMD_JOGBALL_LED_MODE	0x5A
#define MICROP_I2C_WCMD_JOGBALL_LED_PWM_SET	0x5C
#define MICROP_I2C_WCMD_READ_ADC_VALUE_REQ	0x60
#define MICROP_I2C_RCMD_ADC_VALUE		0x62
#define MICROP_I2C_WCMD_REMOTEKEY_TABLE		0x63
#define MICROP_I2C_WCMD_LCM_BURST		0x6A
#define MICROP_I2C_WCMD_LCM_BURST_EN		0x6B
#define MICROP_I2C_WCMD_LCM_REGISTER		0x70
#define MICROP_I2C_WCMD_GSENSOR_REG		0x73
#define MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ	0x74
#define MICROP_I2C_RCMD_GSENSOR_REG_DATA	0x75
#define MICROP_I2C_WCMD_GSENSOR_DATA_REQ	0x76
#define MICROP_I2C_RCMD_GSENSOR_X_DATA		0x77
#define MICROP_I2C_RCMD_GSENSOR_Y_DATA		0x78
#define MICROP_I2C_RCMD_GSENSOR_Z_DATA		0x79
#define MICROP_I2C_RCMD_GSENSOR_DATA		0x7A
#define MICROP_I2C_WCMD_OJ_REG			0x7B
#define MICROP_I2C_WCMD_OJ_REG_DATA_REQ		0x7C
#define MICROP_I2C_RCMD_OJ_REG_DATA		0x7D
#define MICROP_I2C_WCMD_OJ_POS_DATA_REQ		0x7E
#define MICROP_I2C_RCMD_OJ_POS_DATA		0x7F
#define MICROP_I2C_WCMD_GPI_INT_CTL_EN		0x80
#define MICROP_I2C_WCMD_GPI_INT_CTL_DIS		0x81
#define MICROP_I2C_RCMD_GPI_INT_STATUS		0x82
#define MICROP_I2C_RCMD_GPIO_STATUS		0x83
#define MICROP_I2C_WCMD_GPI_INT_STATUS_CLR	0x84
#define MICROP_I2C_RCMD_GPI_INT_SETTING		0x85
#define MICROP_I2C_RCMD_REMOTE_KEYCODE		0x87
#define MICROP_I2C_WCMD_REMOTE_KEY_DEBN_TIME	0x88
#define MICROP_I2C_WCMD_REMOTE_PLUG_DEBN_TIME	0x89
#define MICROP_I2C_WCMD_SIMCARD_DEBN_TIME	0x8A
#define MICROP_I2C_WCMD_GPO_LED_STATUS_EN	0x90
#define MICROP_I2C_WCMD_GPO_LED_STATUS_DIS	0x91
#define MICROP_I2C_WCMD_OJ_INT_STATUS		0xA8
#define MICROP_I2C_RCMD_MOBEAM_STATUS		0xB1
#define MICROP_I2C_WCMD_MOBEAM_DL		0xB2
#define MICROP_I2C_WCMD_MOBEAM_SEND		0xB3

struct microp_function_config {
	const char 	*name;
	uint8_t		category;
	uint8_t		init_value;
	uint8_t		channel;
	uint8_t		fade_time;
	uint32_t	sub_categ;
	uint16_t	levels[10];
	uint16_t	dutys[10];
	uint16_t	int_pin;
	uint16_t	golden_adc;
	uint8_t		mask_r[3];
	uint8_t		mask_w[3];
	uint32_t	ls_gpio_on;
	int (*ls_power)(int, uint8_t);
};

struct microp_i2c_platform_data {
	struct microp_function_config	*microp_function;
	struct platform_device *microp_devices;
	int			num_devices;
	int			num_functions;
	uint32_t		gpio_reset;
	uint32_t		microp_ls_on;
	void 			*dev_id;
	uint8_t			microp_mic_status;
	uint8_t			function_node[20];
	uint32_t		cmd_diff;
	uint32_t		spi_devices;
	uint32_t		spi_devices_init;
};

struct microp_led_config {
	const char *name;
	uint32_t type;
	uint8_t init_value;
	uint8_t fade_time;
	uint16_t led_pin;
	uint8_t mask_w[3];
};

struct microp_led_platform_data {
	struct microp_led_config *led_config;
	int num_leds;
};

struct microp_int_pin {
	uint16_t int_gsensor;
	uint16_t int_lsensor;
	uint16_t int_reset;
	uint16_t int_simcard;
	uint16_t int_hpin;
	uint16_t int_remotekey;
	uint16_t int_sdcard;
	uint16_t int_oj;
	uint16_t int_psensor;
};

struct microp_gpio_status {
	uint32_t hpin;
	uint32_t sdcard;
	uint32_t psensor;
};

struct microp_function_node {
	uint8_t lsensor;
	uint8_t psensor;
};

struct microp_led_data {
	struct led_classdev ldev;
	struct microp_led_config *led_config;
	struct mutex led_data_mutex;
	spinlock_t brightness_lock;
	enum led_brightness brightness;
	uint8_t mode;
	uint8_t blink;
};

struct microp_i2c_client_data {
	struct mutex microp_adc_mutex;
	struct mutex microp_i2c_rw_mutex;
	uint16_t version;
	struct workqueue_struct *microp_queue;
	struct work_struct microp_intr_work;
	struct delayed_work ls_on_work;
	struct delayed_work hpin_enable_intr_work;
	struct delayed_work hpin_debounce_work;
	struct early_suspend early_suspend;
	struct microp_int_pin int_pin;
	struct microp_gpio_status gpio;
	struct microp_function_node fnode;
	struct wake_lock hpin_wake_lock;

	atomic_t microp_is_suspend;
	atomic_t als_intr_enabled;
	atomic_t als_intr_enable_flag;
	int headset_is_in;
	int sdcard_is_in;
	uint32_t spi_devices_vote;
	uint32_t pwr_devices_vote;
	uint32_t als_func;
	struct hrtimer gen_irq_timer;
	uint16_t intr_status;
};

struct lightsensor_platform_data{
	struct i2c_client *client;
	struct microp_function_config	*config;
	int irq;
	int old_intr_cmd;
};

struct microp_ops {
	int (*init_microp_func)(struct i2c_client *);
	int (*als_pwr_enable)(int pwr_device, uint8_t en);
	int (*als_intr_enable)(struct i2c_client *,
			uint32_t als_func, uint8_t en);
	void (*als_level_change)(struct i2c_client *, uint8_t *data);
	void (*headset_enable)(int en);
	void (*spi_enable)(int en);
	void (*notifier_func)(struct i2c_client *, struct microp_led_data *);
	void (*led_gpio_set)(struct microp_led_data *);
};

int microp_i2c_read(uint8_t addr, uint8_t *data, int length);
int microp_i2c_write(uint8_t addr, uint8_t *data, int length);
int microp_function_check(struct i2c_client *client, uint8_t category);
int microp_read_gpio_status(uint8_t *data);
int microp_write_interrupt(struct i2c_client *client,
		uint16_t interrupt, uint8_t enable);
void microp_get_als_kvalue(int i);
int microp_spi_vote_enable(int spi_device, uint8_t enable);
void microp_register_ops(struct microp_ops *ops);

int microp_read_adc(uint8_t *data);
void microp_mobeam_enable(int enable);

#endif /* _LINUX_ATMEGA_MICROP_H */
