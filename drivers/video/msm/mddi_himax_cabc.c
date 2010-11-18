/*
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/msm_fb.h>
#include <linux/wakelock.h>
#include <mach/htc_battery.h>
#include <mach/htc_pwrsink.h>
#include <mach/msm_panel.h>
#include "mddi_client_eid.h"

enum bc_mode {
	BC_OFF = 0,
	BC_MANUAL,
	BC_AUTO,
	BC_MERGED,
	BC_UNDEF,
};

enum cabc_mode {
	CABC_OFF = 0,
	CABC_UI,
	CABC_STILL,
	CABC_MOVING,
	CABC_UNDEF,
};

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

#define DEBUG 0

static int g_brightness;
static char *str_bc_mode[] = {"BC_OFF", "BC_MANUAL", "BC_AUTO", "BC_MERGED"};
static char *str_cabc_mode[] = {"CABC_OFF", "CABC_USER_INTERFACE",
				"CABC_STILL_IMAGE", "CABC_MOVING"};

static ssize_t
himax_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
himax_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count);

struct cabc {
	struct cabc_config *cabc_config;
	struct led_classdev lcd_backlight;	/* user */
	struct mutex lock;
	struct mutex data_lock;
	struct work_struct lcd_changed_work;	/* lcd backlight */
	struct work_struct set_auto_work;
	struct wake_lock wakelock;
	struct workqueue_struct *cabc_queue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	unsigned long status;
	enum cabc_mode mode_cabc;
	enum bc_mode mode_bc;
};

enum {
	LS_STATE,
	GATE_ON,
	ENFORCE_ON,
	SUSPEND,
	AUTO_SETTING,
	LS_SWITCH,
};

#define to_cabc(p, m) container_of(p, struct cabc, m)

static inline
struct msm_mddi_client_data *cabc_get_client(struct cabc* cabc)
{
	struct cabc_config *data = cabc->cabc_config;
	return data->client;
}

static inline int cabc_shrink(struct cabc *cabc, int brightness)
{
	struct cabc_config *config = cabc->cabc_config;

	if (config->shrink) {
		if (brightness <= 102) {
			if (brightness <= 30)
				brightness = 8;
			else
				brightness = 123 * (brightness - 30) / 72 + 8;
		} else
			brightness = 124 * (brightness - 102) / 153 + 131;
	}
	return brightness;
}

static int
himax_change_cabcmode(struct msm_mddi_client_data *client_data,
		int mode, u8 dimming)
{
	pr_debug("+%s, mode=%d, dimming=%d\n", __func__, mode, dimming);
	return 0;
}

static int
__set_brightness(struct cabc *cabc, int brightness, u8 dimming)
{
	struct msm_mddi_client_data *client_data = cabc_get_client(cabc);

	int shrink_br;
	/* no need to check brightness > LED_FULL, the led class
	 * already does */
	printk(KERN_INFO "brightness = %d, %s ls-(%s)\n",
		brightness, str_bc_mode[cabc->mode_bc],
		(test_bit(LS_STATE, &cabc->status) ? "on" : "off"));

	mutex_lock(&cabc->data_lock);
	if(cabc->cabc_config->shrink && cabc->cabc_config->shrink_br)
		shrink_br = cabc->cabc_config->shrink_br(brightness);
	else
		shrink_br = cabc_shrink(cabc, brightness);
        client_data->remote_write(client_data, (u8)shrink_br, 0x94);
	g_brightness = brightness;
	mutex_unlock(&cabc->data_lock);
	return 0;
}

#if DEBUG
static void cabc_dump(unsigned long status)
{
	if (test_bit(LS_STATE, &status))
		B("LS ");
	if (test_bit(GATE_ON, &status))
		B("GATE ");
	if (test_bit(ENFORCE_ON, &status))
		B("EN_ON ");
	if (test_bit(SUSPEND, &status))
		B("SUSPEND ");
	if (test_bit(AUTO_SETTING, &status))
		B("AUTO_SETTING ");
	if (test_bit(LS_SWITCH, &status))
		B("SWITCH ");
	B("\n");
}
#endif

static void cabc_lcd_work(struct work_struct *work)
{
	struct cabc *cabc = to_cabc(work, lcd_changed_work);
	struct led_classdev *led_cdev = &cabc->lcd_backlight;
	char event_string[30];
	char *envp[] = { event_string, NULL };

	/* check again, if we are doing early_suspend, but the check
	 * already passed. */
	if ((test_bit(SUSPEND, &cabc->status) == 0) &&
	    (test_bit(ENFORCE_ON, &cabc->status) == 0)) {
		if(led_cdev->brightness >= 0 && led_cdev->brightness <= 255)
			__set_brightness(cabc, led_cdev->brightness, 1U << 3);

		sprintf(event_string, "CABC_BRIGHTNESS=%d",
			led_cdev->brightness);
		kobject_uevent_env(&led_cdev->dev->kobj, KOBJ_CHANGE, envp);
	}

	wake_unlock(&cabc->wakelock);
}

static void
himax_set_brightness(struct led_classdev *led_cdev,
		       enum led_brightness brightness)
{
	struct cabc *cabc = to_cabc(led_cdev, lcd_backlight);

	B(KERN_DEBUG "%s\n", __func__);

	if ((test_bit(SUSPEND, &cabc->status))  ||
	    (test_bit(ENFORCE_ON, &cabc->status)))
		return;

	wake_lock(&cabc->wakelock);
	queue_work(cabc->cabc_queue, &cabc->lcd_changed_work);
	wake_unlock(&cabc->wakelock);
}

static enum led_brightness
himax_get_brightness(struct led_classdev *led_cdev)
{
	B(KERN_DEBUG "%s\n", __func__);

	return led_cdev->brightness;
}

/*
 * for consistent with android UI,
 * 1: turn on, LABC
 * 0: turn off, Manual
 * */
static void cabc_auto_work(struct work_struct *work)
{
	struct cabc *cabc = to_cabc(work, set_auto_work);
	int on;

	on = test_bit(LS_SWITCH, &cabc->status);
	/* update on success */
	mutex_lock(&cabc->data_lock);
	if (on) {
		cabc->mode_bc = BC_MERGED;
		cabc->mode_cabc = CABC_MOVING;
		set_bit(LS_STATE, &cabc->status);
		mutex_unlock(&cabc->data_lock);
	} else {
		clear_bit(LS_STATE, &cabc->status);
		cabc->mode_bc = BC_MANUAL;
		cabc->mode_cabc = CABC_OFF;
		mutex_unlock(&cabc->data_lock);
	}
	clear_bit(AUTO_SETTING, &cabc->status);
}

static int
cabc_bl_handle(struct platform_device *pdev, int brightness)
{
        struct cabc *cabc = platform_get_drvdata(pdev);
        struct led_classdev *lcd_cdev;

        if (unlikely(cabc == NULL)) {
                printk(KERN_ERR "%s: do not have cabc data\n", __func__);
                return -ENOENT;
        }

        printk(KERN_DEBUG "turn %s backlight.\n",
                        brightness == LED_FULL ? "on" : "off");

        lcd_cdev = &cabc->lcd_backlight;
        wake_lock(&cabc->wakelock);

        if (brightness != LED_FULL) {
		/* enter screen off*/
        } else {
		__set_brightness(cabc, (g_brightness >= 0 && g_brightness <= LED_FULL)?
			g_brightness : LED_FULL, 0);
        }

        wake_unlock(&cabc->wakelock);
        return 0;
}

static int
himax_auto_backlight(struct cabc *cabc, int on)
{
	B(KERN_DEBUG "%s: %s\n", __func__, on ? "ON" : "OFF");

	if (test_bit(LS_STATE, &cabc->status) == on)
		return 0;

	if (on)
		set_bit(LS_SWITCH, &cabc->status);
	else
		clear_bit(LS_SWITCH, &cabc->status);

	if (test_bit(AUTO_SETTING, &cabc->status) == 0) {
		set_bit(AUTO_SETTING, &cabc->status);

		/* if we are not in suspend mode, we can
		 * do it right away */
		if (test_bit(SUSPEND, &cabc->status) == 0)
			cabc_auto_work(&cabc->set_auto_work);
	}

	return 0;
}

#define CABC_ATTR(name) __ATTR(name, 0644, himax_show, himax_store)

enum {
	CABC_MODE = 0,
	BC_MODE,
	LIGHT_SENSOR,
	AUTO_BACKLIGHT,
};

static struct device_attribute cabc_attrs[] = {
	CABC_ATTR(cabc_mode),
	CABC_ATTR(bc_mode),
	CABC_ATTR(light_sensor),
	CABC_ATTR(auto),
};

static ssize_t
himax_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - cabc_attrs;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct cabc *cabc = to_cabc(led_cdev, lcd_backlight);

	mutex_lock(&cabc->lock);
	switch (off) {
	case CABC_MODE:
		i += scnprintf(buf + i, PAGE_SIZE - 1, "%s\n",
				str_cabc_mode[cabc->mode_cabc]);
		break;
	case BC_MODE:
		i += scnprintf(buf + i, PAGE_SIZE - 1, "%s\n",
				str_bc_mode[cabc->mode_bc]);
		break;
	case AUTO_BACKLIGHT:
		i += scnprintf(buf + i, PAGE_SIZE - 1, "%d\n",
				test_bit(LS_STATE, &cabc->status));
		break;
	default:
		i = -EINVAL;
		break;
	}
	mutex_unlock(&cabc->lock);
	return i;
}

static ssize_t
himax_store(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	int rc;
	unsigned long res;
	const ptrdiff_t off = attr - cabc_attrs;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct cabc *cabc = to_cabc(led_cdev, lcd_backlight);
	struct msm_mddi_client_data *client_data = cabc_get_client(cabc);

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		printk(KERN_ERR "invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	mutex_lock(&cabc->lock);
	switch (off) {
	case CABC_MODE:
		if (res >= CABC_OFF && res < CABC_UNDEF) {
			cabc->mode_cabc = res;
			cabc->cabc_config->change_cabcmode(client_data, res, 0x25);
		}
		break;
	case BC_MODE:
		if (res >= BC_OFF && res < BC_UNDEF)
			cabc->mode_bc = res;
		break;
	case AUTO_BACKLIGHT:
		if (himax_auto_backlight(cabc, !!res))
			count = -EIO;
		break;
	default:
		count = -EINVAL;
		break;
	}
	mutex_unlock(&cabc->lock);
err_out:
	return count;
}


static int himax_cabc_probe(struct platform_device *pdev)
{
	int i, err;
	struct cabc *cabc;
	struct cabc_config *data;

	B(KERN_DEBUG "%s\n", __func__);
	cabc = kzalloc(sizeof(struct cabc), GFP_KERNEL);
	if (!cabc)
		return -ENOMEM;
	platform_set_drvdata(pdev, cabc);

	data = pdev->dev.platform_data;
	if (data == NULL || !data->client) {
		printk(KERN_ERR "No CABC config data\n");
		err = -EINVAL;
		goto err_client;
	}
	g_brightness = LED_FULL;
	cabc->cabc_config = data;
	cabc->cabc_config->bl_handle = cabc_bl_handle;
	if (!cabc->cabc_config->change_cabcmode)
		cabc->cabc_config->change_cabcmode = himax_change_cabcmode;

	INIT_WORK(&cabc->lcd_changed_work, cabc_lcd_work);
	INIT_WORK(&cabc->set_auto_work, cabc_auto_work);
	mutex_init(&cabc->lock);
	mutex_init(&cabc->data_lock);
	cabc->cabc_queue = create_singlethread_workqueue("cabc_work_q");
	wake_lock_init(&cabc->wakelock, WAKE_LOCK_IDLE, "cabc_present");


	cabc->lcd_backlight.name = "lcd-backlight";
	cabc->lcd_backlight.brightness_set = himax_set_brightness;
	cabc->lcd_backlight.brightness_get = himax_get_brightness;
	cabc->lcd_backlight.brightness = LED_FULL;
	err = led_classdev_register(&pdev->dev, &cabc->lcd_backlight);
	if (err)
		goto err_register_lcd_bl;

	for (i = 0; i < ARRAY_SIZE(cabc_attrs); i++) {
		err = device_create_file(cabc->lcd_backlight.dev,
					&cabc_attrs[i]);
		if (err)
			goto err_out;
	}

	return 0;

err_out:
	while (i--)
		device_remove_file(&pdev->dev, &cabc_attrs[i]);

err_register_lcd_bl:
	led_classdev_unregister(&cabc->lcd_backlight);
err_client:
	kfree(cabc);
	return err;
}

static struct platform_driver himax_cabc_driver = {
	.probe = himax_cabc_probe,
	.driver = { .name = "himax_cabc" },
};

static int __init himax_cabc_init(void)
{
	return platform_driver_register(&himax_cabc_driver);
}

module_init(himax_cabc_init);
