/*
 * LED Core
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/leds.h>
#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND

extern int queue_brightness_change(struct led_classdev *led_cdev,
       enum led_brightness value);

struct deferred_brightness_change {
       struct work_struct brightness_change_work;
       struct led_classdev *led_cdev;
       enum led_brightness value;
};

#endif

static inline void led_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	if (value > led_cdev->max_brightness)
		value = led_cdev->max_brightness;
	led_cdev->brightness = value;

	if (value > led_cdev->offset)
		value -= led_cdev->offset;

	if (!(led_cdev->flags & LED_SUSPENDED)) {
		/* [FIXME -- workaround] backlight: go for workqueue, others: direct call */
		if (!strcmp(led_cdev->name, "lcd-backlight")) {
#ifdef CONFIG_HAS_EARLYSUSPEND
			if (queue_brightness_change(led_cdev, value) != 0)
#endif
			led_cdev->brightness_set(led_cdev, value);
		} else
			led_cdev->brightness_set(led_cdev, value);
	}
}

static inline int led_get_brightness(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

#ifdef CONFIG_LEDS_TRIGGERS
void led_trigger_set_default(struct led_classdev *led_cdev);
void led_trigger_set(struct led_classdev *led_cdev,
			struct led_trigger *trigger);
void led_trigger_remove(struct led_classdev *led_cdev);

static inline void *led_get_trigger_data(struct led_classdev *led_cdev)
{
	return led_cdev->trigger_data;
}

#else
#define led_trigger_set_default(x) do {} while (0)
#define led_trigger_set(x, y) do {} while (0)
#define led_trigger_remove(x) do {} while (0)
#define led_get_trigger_data(x) (NULL)
#endif

ssize_t led_trigger_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t led_trigger_show(struct device *dev, struct device_attribute *attr,
			char *buf);

#endif	/* __LEDS_H_INCLUDED */
