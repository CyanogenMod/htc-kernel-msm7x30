/*
 * Copyright (C) 2008 HTC Corporation.
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
#include <linux/slab.h>
#include <mach/msm_fb.h>
#include <linux/wakelock.h>
#include <mach/htc_battery.h>
#include <mach/htc_pwrsink.h>
#include <mach/msm_panel.h>
#include "mddi_client_eid.h"
#include <mach/debug_display.h>

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
#define DEFAULT_BRIGHTNESS 100
static char *str_bc_mode[] = {"BC_OFF", "BC_MANUAL", "BC_AUTO", "BC_MERGED"};
static char *str_cabc_mode[] = {"CABC_OFF", "CABC_USER_INTERFACE",
				"CABC_STILL_IMAGE", "CABC_MOVING"};

static ssize_t
samsung_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
samsung_store(struct device *dev, struct device_attribute *attr,
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
	GATE_ON,
	SUSPEND,
	AUTO_SETTING,
};

#define to_cabc(p, m) container_of(p, struct cabc, m)

struct complete_data {
	int done;
};

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

static void
samsung_send_cmd(struct msm_mddi_client_data *client_data, unsigned cmd,
	     u8 *prm, int size, uint8_t attrs, ...)
{
	int i;
	u8 tmp;
	va_list attr_list;

	if (size <= 0)
		return;

	prm[0] = attrs;

	va_start(attr_list, attrs);

	for (i = 1; i < size; i++) {
		tmp = (u8)va_arg(attr_list, int);
		prm[i] = tmp;
	}

	va_end(attr_list);
#if DEBUG
	PR_DISP_DEBUG("0x%x ", cmd);
	for (i = 0; i < size; i++)
		PR_DISP_WARN("0x%x ", prm[i]);
	PR_DISP_WARN("\n");
#endif
	if (client_data)
		client_data->remote_write_vals(client_data, prm, cmd, size);
}

static int
samsung_change_cabcmode(struct msm_mddi_client_data *client_data,
		int mode, u8 dimming)
{
	u8 prm[20];

	PR_DISP_DEBUG("+%s, mode=%d, dimming=%d\n", __func__, mode, dimming);

	samsung_send_cmd(client_data, MIECTL1, prm, 4, 0x5a, 0x5a, 0x30, 0x00);
	samsung_send_cmd(client_data, MIECTL2, prm, 8, 0xe0, 0x07, 0x7c, 0x01,
			0x3f, 0x00, 0x00, 0x00);
	samsung_send_cmd(client_data, MIECTL3, prm, 4, 0x7f, dimming,
			0x00, 0x00);
	samsung_send_cmd(client_data, WRCABC, prm, 4, (u8)mode, 0x00, 0x00,
			0x00);
	samsung_send_cmd(client_data, DCON, prm, 4, 0x07, 0x00, 0x00, 0x00);

	return 0;
}

static int
__set_brightness(struct cabc *cabc, int brightness, u8 dimming)
{
	struct msm_mddi_client_data *client_data = cabc_get_client(cabc);
	u8 prm[20];
	unsigned percent;
	int shrink_br;

	/* no need to check brightness > LED_FULL, the led class
	 * already does */
	//PR_DISP_INFO("_set_brightness = %d\n", brightness);

	mutex_lock(&cabc->data_lock);
	if(cabc->cabc_config->shrink && cabc->cabc_config->shrink_br)
		shrink_br = cabc->cabc_config->shrink_br(brightness);
	else
		shrink_br = cabc_shrink(cabc, brightness);

	percent = (shrink_br * 100) / 255;
	htc_pwrsink_set(PWRSINK_BACKLIGHT, percent);

	samsung_send_cmd(client_data, BCMODE, prm, 4,
			(u8)cabc->mode_bc, 0x00, 0x00, 0x00);
	samsung_send_cmd(client_data, WRDISBV, prm, 4,
			(u8)shrink_br, 0x00, 0x00, 0x00);
	samsung_send_cmd(client_data, WRCABC, prm, 4,
			(u8)cabc->mode_cabc, 0x00, 0x00, 0x00);
	samsung_send_cmd(client_data, WRCTRLD, prm, 4,
			0x24 | (dimming), 0x00, 0x00, 0x00);
	samsung_send_cmd(client_data, DCON, prm, 4, 0x07, 0x00, 0x00, 0x00);

	mutex_unlock(&cabc->data_lock);
	return 0;
}

/*
 * Disable HW dimming when resume, enables after resume
 * */
static void __turn_on_backlight(struct cabc *cabc, u8 brightness)
{
	enum bc_mode bc_tmp;

	mutex_lock(&cabc->data_lock);
	bc_tmp = cabc->mode_bc;
	cabc->mode_bc = BC_MANUAL;
	mutex_unlock(&cabc->data_lock);

	cabc->cabc_config->change_cabcmode(cabc->cabc_config->client,
		CABC_OFF, 0x25);
	__set_brightness(cabc, brightness, 0);

	mutex_lock(&cabc->data_lock);
	cabc->mode_bc = bc_tmp;
	mutex_unlock(&cabc->data_lock);
}

#if DEBUG
static void cabc_dump(unsigned long status)
{
	if (test_bit(GATE_ON, &status))
		B("GATE ");
	if (test_bit(SUSPEND, &status))
		B("SUSPEND ");
	if (test_bit(AUTO_SETTING, &status))
		B("AUTO_SETTING ");
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
	if (test_bit(SUSPEND, &cabc->status) == 0) {
		__set_brightness(cabc, led_cdev->brightness, 1U << 3);
		sprintf(event_string, "CABC_BRIGHTNESS=%d",
			led_cdev->brightness);
		kobject_uevent_env(&led_cdev->dev->kobj, KOBJ_CHANGE, envp);
	}

	wake_unlock(&cabc->wakelock);
}

static void
samsung_set_brightness(struct led_classdev *led_cdev,
		       enum led_brightness brightness)
{
	struct cabc *cabc = to_cabc(led_cdev, lcd_backlight);

	B(KERN_DEBUG "%s\n", __func__);

	if (test_bit(SUSPEND, &cabc->status))
		return;


	wake_lock(&cabc->wakelock);

	if (test_bit(GATE_ON, &cabc->status))
		queue_work(cabc->cabc_queue, &cabc->lcd_changed_work);
	else
		wake_unlock(&cabc->wakelock);
}

static enum led_brightness
samsung_get_brightness(struct led_classdev *led_cdev)
{
	B(KERN_DEBUG "%s enter\n", __func__);
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
	struct msm_mddi_client_data *client_data = cabc_get_client(cabc);
	int on;

	on = test_bit(AUTO_SETTING, &cabc->status);
	/* update on success */
	mutex_lock(&cabc->data_lock);
	if (on) {
		cabc->mode_bc = BC_MERGED;
		cabc->mode_cabc = CABC_MOVING;
		mutex_unlock(&cabc->data_lock);
	} else {
		cabc->mode_bc = BC_MANUAL;
		cabc->mode_cabc = CABC_OFF;
		mutex_unlock(&cabc->data_lock);
	}
	cabc->cabc_config->change_cabcmode(client_data, cabc->mode_cabc, 0x25);
}

static int
cabc_bl_handle(struct platform_device *pdev, int brightness)
{
	struct cabc *cabc = platform_get_drvdata(pdev);
	struct led_classdev *lcd_cdev;
	int resume_br;

	if (unlikely(cabc == NULL)) {
		PR_DISP_ERR("%s: do not have cabc data\n", __func__);
		return -ENOENT;
	}

	PR_DISP_DEBUG("turn %s backlight.\n",
			brightness == LED_FULL ? "on" : "off");

	lcd_cdev = &cabc->lcd_backlight;
	wake_lock(&cabc->wakelock);

	if (brightness != LED_FULL) {
		clear_bit(GATE_ON, &cabc->status);
	} else {
		set_bit(GATE_ON, &cabc->status);
		resume_br = (cabc->cabc_config->default_br) ? cabc->cabc_config->default_br
			: DEFAULT_BRIGHTNESS;

		__turn_on_backlight(cabc, test_bit(AUTO_SETTING, &cabc->status) ?
			resume_br : lcd_cdev->brightness);
	}

	wake_unlock(&cabc->wakelock);
	return 0;
}

static int
samsung_auto_backlight(struct cabc *cabc, int on)
{
	B(KERN_DEBUG "%s: %s\n", __func__, on ? "ON" : "OFF");

	if (on)
		set_bit(AUTO_SETTING, &cabc->status);
	else
		clear_bit(AUTO_SETTING, &cabc->status);

	if (test_bit(SUSPEND, &cabc->status) == 0)
		cabc_auto_work(&cabc->set_auto_work);



	return 0;
}

#define CABC_ATTR(name) __ATTR(name, 0644, samsung_show, samsung_store)

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
samsung_show(struct device *dev, struct device_attribute *attr, char *buf)
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
			test_bit(AUTO_SETTING, &cabc->status));
		break;
	default:
		i = -EINVAL;
		break;
	}
	mutex_unlock(&cabc->lock);
	return i;
}

static ssize_t
samsung_store(struct device *dev, struct device_attribute *attr,
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
		PR_DISP_ERR("invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	mutex_lock(&cabc->lock);
	switch (off) {
	case CABC_MODE:
		if (res < CABC_UNDEF) {
			cabc->mode_cabc = res;
			cabc->cabc_config->change_cabcmode(client_data, res, 0x25);
		}
		break;
	case BC_MODE:
		if (res < BC_UNDEF)
			cabc->mode_bc = res;
		break;
	case AUTO_BACKLIGHT:
		if (samsung_auto_backlight(cabc, !!res))
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

#define LED_DEV(ptr, member, _name)				\
{								\
	(ptr)->member.name = #_name;				\
	(ptr)->member.brightness_set = _name##_set_brightness;	\
	(ptr)->member.brightness_get = _name##_get_brightness;	\
}

static void
samsung_cabc_suspend(struct early_suspend *h)
{
	struct cabc *cabc = to_cabc(h, early_suspend);

	B(KERN_DEBUG "%s\n", __func__);

	if (1) {
		set_bit(SUSPEND, &cabc->status);
		cancel_work_sync(&cabc->set_auto_work);
		flush_workqueue(cabc->cabc_queue);
	} else {
		__set_brightness(cabc, DEFAULT_BRIGHTNESS, 0);
	}
}

static void
samsung_cabc_resume(struct early_suspend *h)
{
	struct cabc *cabc = to_cabc(h, early_suspend);

	B(KERN_DEBUG "%s\n", __func__);

	clear_bit(SUSPEND, &cabc->status);

	if (test_bit(AUTO_SETTING, &cabc->status))
		queue_work(cabc->cabc_queue, &cabc->set_auto_work);
}

static int samsung_cabc_probe(struct platform_device *pdev)
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
		PR_DISP_ERR("No CABC config data\n");
		err = -EINVAL;
		goto err_client;
	}

	cabc->cabc_config = data;
	cabc->cabc_config->bl_handle = cabc_bl_handle;
	if (!cabc->cabc_config->change_cabcmode)
		cabc->cabc_config->change_cabcmode = samsung_change_cabcmode;

	INIT_WORK(&cabc->lcd_changed_work, cabc_lcd_work);
	INIT_WORK(&cabc->set_auto_work, cabc_auto_work);
	mutex_init(&cabc->lock);
	mutex_init(&cabc->data_lock);
	cabc->cabc_queue = create_singlethread_workqueue("cabc_work_q");
	wake_lock_init(&cabc->wakelock, WAKE_LOCK_IDLE, "cabc_present");

	cabc->lcd_backlight.name = "lcd-backlight";
	cabc->lcd_backlight.brightness_set = samsung_set_brightness;
	cabc->lcd_backlight.brightness_get = samsung_get_brightness;
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

	/* default setting */
	cabc->mode_cabc = CABC_OFF;
	cabc->cabc_config->change_cabcmode(cabc->cabc_config->client,
		cabc->mode_cabc, 0x25);
	cabc->mode_bc = BC_MANUAL;
	samsung_set_brightness(&cabc->lcd_backlight, 255);

#ifdef CONFIG_HAS_EARLYSUSPEND
	cabc->early_suspend.suspend = samsung_cabc_suspend;
	cabc->early_suspend.resume = samsung_cabc_resume;
	cabc->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&cabc->early_suspend);
#endif
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

static struct platform_driver samsung_cabc_driver = {
	.probe = samsung_cabc_probe,
	.driver = { .name = "samsung_cabc" },
};

static int __init samsung_cabc_init(void)
{
	return platform_driver_register(&samsung_cabc_driver);
}

module_init(samsung_cabc_init);
