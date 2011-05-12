/* include/linux/leds-pm8058.h
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

#ifndef _LINUX_LEDS_PM8058_H
#define _LINUX_LEDS_PM8058_H

#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/workqueue.h>
#include <linux/android_alarm.h>

#define	PM_PWM_CLK_1KHZ		0
#define	PM_PWM_CLK_32KHZ	1
#define	PM_PWM_CLK_19P2MHZ	2

#define	PM_PWM_PREDIVIDE_2	0
#define	PM_PWM_PREDIVIDE_3	1
#define	PM_PWM_PREDIVIDE_5	2
#define	PM_PWM_PREDIVIDE_6	3

#define PM8058_LED_RGB			(1 << 0)
#define PM8058_LED_PWM			(1 << 1)
#define PM8058_LED_CURRENT		(1 << 2)
#define PM8058_LED_DRVX			(1 << 3)

#define PM8058_LED_LTU_EN			(1 << 0)
#define PM8058_LED_FADE_EN			(1 << 1)
#define PM8058_LED_BLINK_EN			(1 << 2)
#define PM8058_LED_DYNAMIC_BRIGHTNESS_EN	(1 << 3)

struct pm8058_led_config {
	const char *name;
	uint32_t type;
	int bank;
	int flags;
	int pwm_size;
	int clk;
	int pre_div;
	int pre_div_exp;
	int pwm_value;
/* for LUT */
	int period_us;
	int start_index;
	int duites_size;
	int duty_time_ms;
	int lut_flag;
	int out_current;
};

struct pm8058_led_platform_data {
	struct pm8058_led_config *led_config;
	int num_leds;
	int duties[64];
};

struct pm8058_led_data {
	struct led_classdev ldev;
	struct pm8058_led_config *led_config;
	enum led_brightness brightness;
	struct pwm_device *pwm_led;
	struct alarm led_alarm;
	struct work_struct led_work;
	struct delayed_work led_delayed_work;
	int bank;
	int flags;
	int pwm_size;
	int clk;
	int pre_div;
	int pre_div_exp;
	int pwm_value;
/* for LUT */
	int period_us;
	int start_index;
	int duites_size;
	int duty_time_ms;
	int lut_flag;
	int out_current;
};

#endif /* _LINUX_LEDS_PM8058_H */
