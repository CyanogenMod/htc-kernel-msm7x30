/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LEDS_PM8XXX_H__
#define __LEDS_PM8XXX_H__


#define PM8XXX_LEDS_DEV_NAME	"pm8xxx-led"

#include <linux/android_alarm.h>
#include <linux/leds.h>

enum pm8xxx_blink_type {
	BLINK_STOP = -1,
	BLINK_UNCHANGE = 0,
	BLINK_64MS_PER_2SEC,
	BLINK_64MS_ON_310MS_PER_2SEC,
	BLINK_64MS_ON_2SEC_PER_2SEC,
	BLINK_1SEC_PER_2SEC,
};

/**
 * enum pm8xxx_leds - PMIC8XXX supported led ids
 * @PM8XXX_ID_LED_KB_LIGHT - keyboard backlight led
 * @PM8XXX_ID_LED_0 - First low current led
 * @PM8XXX_ID_LED_1 - Second low current led
 * @PM8XXX_ID_LED_2 - Third low current led
 * @PM8XXX_ID_FLASH_LED_0 - First flash led
 * @PM8XXX_ID_FLASH_LED_0 - Second flash led
 */
enum pm8xxx_leds {
	PM8XXX_ID_GPIO24 = 0,
	PM8XXX_ID_GPIO25,
	PM8XXX_ID_GPIO26,
	PM8XXX_ID_LED_KB_LIGHT,
	PM8XXX_ID_LED_2,
	PM8XXX_ID_LED_1,
	PM8XXX_ID_LED_0,
	PM8XXX_ID_FLASH_LED_0,
	PM8XXX_ID_FLASH_LED_1,
};

/**
 * pm8xxx_led_modes - Operating modes of LEDs
 */
enum pm8xxx_led_modes {
	PM8XXX_LED_MODE_MANUAL = 0,
	PM8XXX_LED_MODE_PWM1,
	PM8XXX_LED_MODE_PWM2,
	PM8XXX_LED_MODE_PWM3,
	PM8XXX_LED_MODE_DTEST1,
	PM8XXX_LED_MODE_DTEST2,
	PM8XXX_LED_MODE_DTEST3,
	PM8XXX_LED_MODE_DTEST4
};

int pm8xxx_led_config(enum pm8xxx_leds led_id,
		enum pm8xxx_led_modes led_mode, int max_current);

#define LED_BLINK_FUNCTION	(1 << 0)
#define LED_BRETH_FUNCTION	(1 << 1)

struct pm8xxx_led_configure {
	const char	*name;
	int		flags;
	int 		period_us;
	int 		start_index;
	int 		duites_size;
	int 		duty_time_ms;
	int 		lut_flag;
	int		out_current;
	int		function_flags;
	int		duties[64];
};

struct pm8xxx_led_platform_data {
	int				num_leds;
	struct pm8xxx_led_configure	*leds;
};

struct pm8xxx_led_data {
	struct led_classdev		cdev;
	struct pwm_device 		*pwm_led;
	int				id;
	int				bank;
	int				function_flags;
	int 				period_us;
	int 				duty_time_ms;
	int 				start_index;
	int 				duites_size;
	int 				lut_flag;
	int				out_current;
	int 				*duties;
	u8				reg;
	struct device			*dev;
	struct delayed_work		blink_delayed_work;
	struct delayed_work 		fade_delayed_work;
	struct work_struct 		led_work;
	struct alarm 			led_alarm;
};
#endif /* __LEDS_PM8XXX_H__ */
