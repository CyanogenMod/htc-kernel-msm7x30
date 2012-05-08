/*
 * arch/arm/mach-msm/msm_flashlight.c - The flashlight driver
 * Copyright (C) 2009  HTC Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/htc_flashlight.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>


#define FLT_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[FLT]" fmt, ##__VA_ARGS__)
#define FLT_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[FLT]" fmt, ##__VA_ARGS__)
#define FLT_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[FLT][ERR]" fmt, ##__VA_ARGS__)

struct flashlight_struct {
	struct led_classdev fl_lcdev;
	struct early_suspend early_suspend_flashlight;
	struct hrtimer timer;
	spinlock_t spin_lock;
	uint32_t gpio_torch;
	uint32_t gpio_flash;
	uint32_t flash_sw_timeout_ms;
	enum flashlight_mode_flags mode_status;
	unsigned long spinlock_flags;
	uint8_t led_count;
};
static struct flashlight_struct *this_fl_str;



static void flashlight_hw_command(uint8_t addr, uint8_t data)
{
	uint8_t loop_i, loop_j;
	const uint8_t fl_addr_to_rising_count[4] = { 17, 18, 19, 20 };
	uint8_t loop_tmp;
	if (!this_fl_str->gpio_torch && !this_fl_str->gpio_flash) {
		FLT_ERR_LOG("%s: not setup GPIO??? torch: %d, flash: %d\n",
					__func__, this_fl_str->gpio_torch,
						this_fl_str->gpio_flash);
		return;
	}
	for (loop_j = 0; loop_j < 2; loop_j++) {
		if (!loop_j)
			loop_tmp = fl_addr_to_rising_count[addr];
		else
			loop_tmp = data;
		for (loop_i = 0; loop_i < loop_tmp; loop_i++) {
			gpio_direction_output(this_fl_str->gpio_torch, 0);
			udelay(2);
			gpio_direction_output(this_fl_str->gpio_torch, 1);
			udelay(2);
		}
		udelay(500);
	}
}

static void flashlight_turn_off(void)
{
	if (this_fl_str->mode_status == FL_MODE_OFF)
		return;

	gpio_direction_output(this_fl_str->gpio_flash, 0);
	gpio_direction_output(this_fl_str->gpio_torch, 0);
	this_fl_str->mode_status = FL_MODE_OFF;
	this_fl_str->fl_lcdev.brightness = LED_OFF;
}

static enum hrtimer_restart flashlight_hrtimer_func(struct hrtimer *timer)
{
	struct flashlight_struct *fl_str = container_of(timer,
			struct flashlight_struct, timer);
	FLT_INFO_LOG("%s: turn off flash mode\n", __func__);
	spin_lock_irqsave(&fl_str->spin_lock, fl_str->spinlock_flags);
	flashlight_turn_off();
	spin_unlock_irqrestore(&fl_str->spin_lock, fl_str->spinlock_flags);
	return HRTIMER_NORESTART;
}

int aat1271_flashlight_control(int mode)
{
	int ret = 0;
	uint32_t flash_ns = ktime_to_ns(ktime_get());

	if (this_fl_str->mode_status == mode) {
		FLT_INFO_LOG("%s: mode is same: %d\n",
							"aat1271_flashlight", mode);

		if (!hrtimer_active(&this_fl_str->timer) &&
			this_fl_str->mode_status == FL_MODE_OFF) {
			FLT_INFO_LOG("flashlight hasn't been enable or" \
				" has already reset to 0 due to timeout\n");
			return ret;
		} else
			return -EINVAL;
	}

	spin_lock_irqsave(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	if (this_fl_str->mode_status == FL_MODE_FLASH) {
		hrtimer_cancel(&this_fl_str->timer);
		flashlight_turn_off();
	}
	switch (mode) {
	case FL_MODE_OFF:
		flashlight_turn_off();
	break;
	case FL_MODE_TORCH:
		if (this_fl_str->led_count == 2)
			flashlight_hw_command(3, 4);
		else
			flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 6);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_TORCH;
		this_fl_str->fl_lcdev.brightness = LED_HALF;
	break;
	case FL_MODE_TORCH_LED_A:
		flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 15);
		flashlight_hw_command(2, 3);
		this_fl_str->mode_status = FL_MODE_TORCH_LED_A;
		this_fl_str->fl_lcdev.brightness = 1;
	break;
	case FL_MODE_TORCH_LED_B:
		flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 15);
		flashlight_hw_command(2, 2);
		this_fl_str->mode_status = FL_MODE_TORCH_LED_B;
		this_fl_str->fl_lcdev.brightness = 2;
	break;
	case FL_MODE_FLASH:
		flashlight_hw_command(2, 4);
		gpio_direction_output(this_fl_str->gpio_flash, 1);
		this_fl_str->mode_status = FL_MODE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_FULL;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;
	case FL_MODE_PRE_FLASH:
		flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 6);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_PRE_FLASH;
		this_fl_str->fl_lcdev.brightness = LED_HALF + 1;
	break;
	case FL_MODE_TORCH_LEVEL_1:
		if (this_fl_str->led_count == 2)
			flashlight_hw_command(3, 4);
		else
			flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 15);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_1;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 2;
	break;
	case FL_MODE_TORCH_LEVEL_2:
		if (this_fl_str->led_count == 2)
			flashlight_hw_command(3, 4);
		else
			flashlight_hw_command(3, 3);
		flashlight_hw_command(0, 10);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_TORCH_LEVEL_2;
		this_fl_str->fl_lcdev.brightness = LED_HALF - 1;
	break;
	case FL_MODE_CAMERA_EFFECT_PRE_FLASH:
		flashlight_hw_command(3, 12);
		flashlight_hw_command(0, 1);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_CAMERA_EFFECT_PRE_FLASH;
		this_fl_str->fl_lcdev.brightness = 3;
	break;
	case FL_MODE_CAMERA_EFFECT_FLASH:
		flashlight_hw_command(3, 1);
		flashlight_hw_command(0, 1);
		flashlight_hw_command(2, 4);
		this_fl_str->mode_status = FL_MODE_CAMERA_EFFECT_FLASH;
		this_fl_str->fl_lcdev.brightness = 4;
		hrtimer_start(&this_fl_str->timer,
			ktime_set(this_fl_str->flash_sw_timeout_ms / 1000,
				(this_fl_str->flash_sw_timeout_ms % 1000) *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
	break;

	default:
		FLT_ERR_LOG("%s: unknown flash_light flags: %d\n",
							__func__, mode);
		ret = -EINVAL;
	break;
	}

	FLT_INFO_LOG("%s: mode: %d, %u\n", "aat1271_flashlight", mode,
		flash_ns/(1000*1000));

	spin_unlock_irqrestore(&this_fl_str->spin_lock,
						this_fl_str->spinlock_flags);
	return ret;
}

static void fl_lcdev_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness brightness)
{
	struct flashlight_struct *fl_str;
	enum flashlight_mode_flags mode;

	fl_str = container_of(led_cdev, struct flashlight_struct, fl_lcdev);
	if (brightness > 0 && brightness <= LED_HALF) {
		/* Torch mode */
		if (brightness == (LED_HALF - 2))
			mode = FL_MODE_TORCH_LEVEL_1;
		else if (brightness == (LED_HALF - 1))
			mode = FL_MODE_TORCH_LEVEL_2;
		else if (brightness == 1 && fl_str->led_count == 2)
			mode = FL_MODE_TORCH_LED_A;
		else if (brightness == 2 && fl_str->led_count == 2)
			mode = FL_MODE_TORCH_LED_B;
		else if (brightness == 3 && fl_str->led_count == 2)
			mode = FL_MODE_CAMERA_EFFECT_PRE_FLASH;
		else if (brightness == 4 && fl_str->led_count == 2)
			mode = FL_MODE_CAMERA_EFFECT_FLASH;
		else
			mode = FL_MODE_TORCH;
	} else if (brightness > LED_HALF && brightness <= LED_FULL) {
		/* Flashlight mode */
		if (brightness == (LED_HALF + 1))
			mode = FL_MODE_PRE_FLASH; /* pre-flash mode */
		else
			mode = FL_MODE_FLASH;
	} else
		/* off and else */
		mode = FL_MODE_OFF;


		aat1271_flashlight_control(mode);

	return;
}

static void flashlight_early_suspend(struct early_suspend *handler)
{
	struct flashlight_struct *fl_str = container_of(handler,
			struct flashlight_struct, early_suspend_flashlight);
	if (fl_str != NULL && fl_str->mode_status) {
		if (fl_str->mode_status == FL_MODE_FLASH)
			hrtimer_cancel(&fl_str->timer);
		spin_lock_irqsave(&fl_str->spin_lock, fl_str->spinlock_flags);
		flashlight_turn_off();
		spin_unlock_irqrestore(&fl_str->spin_lock,
						fl_str->spinlock_flags);
	}
}

static void flashlight_late_resume(struct early_suspend *handler)
{
	/*
	struct flashlight_struct *fl_str = container_of(handler,
			struct flashlight_struct, early_suspend_flashlight);
	*/
}

static int flashlight_setup_gpio(struct flashlight_platform_data *flashlight,
					struct flashlight_struct *fl_str)
{
	int ret = 0;
	if (flashlight->gpio_init)
		flashlight->gpio_init();
	if (flashlight->torch) {
		ret = gpio_request(flashlight->torch, "fl_torch");
		if (ret < 0) {
			FLT_ERR_LOG("%s: gpio_request(torch) failed\n", __func__);
			return ret;
		}
		fl_str->gpio_torch = flashlight->torch;
	}

	if (flashlight->flash) {
		ret = gpio_request(flashlight->flash, "fl_flash");
		if (ret < 0) {
			FLT_ERR_LOG("%s: gpio_request(flash) failed\n", __func__);
			return ret;
		}
		fl_str->gpio_flash = flashlight->flash;
	}

	return ret;
}

static int flashlight_free_gpio(struct flashlight_platform_data *flashlight,
					struct flashlight_struct *fl_str)
{
	int ret = 0;
	if (fl_str->gpio_torch) {
		gpio_free(flashlight->torch);
		fl_str->gpio_torch = 0;
	}

	if (fl_str->gpio_flash) {
		gpio_free(flashlight->flash);
		fl_str->gpio_flash = 0;
	}
	return ret;
}

static int flashlight_probe(struct platform_device *pdev)
{
	struct flashlight_platform_data *flashlight = pdev->dev.platform_data;
	struct flashlight_struct *fl_str;
	int err = 0;

	fl_str = kzalloc(sizeof(struct flashlight_struct), GFP_KERNEL);
	if (!fl_str) {
		FLT_ERR_LOG("%s: kzalloc fail !!!\n", __func__);
		return -ENOMEM;
	}
	err = flashlight_setup_gpio(flashlight, fl_str);
	if (err < 0) {
		FLT_ERR_LOG("%s: setup GPIO fail !!!\n", __func__);
		goto fail_free_mem;
	}
	spin_lock_init(&fl_str->spin_lock);
	fl_str->fl_lcdev.name = FLASHLIGHT_NAME;
	fl_str->fl_lcdev.brightness_set = fl_lcdev_brightness_set;
	fl_str->fl_lcdev.brightness = 0;
	if (flashlight->flash_duration_ms)
		fl_str->flash_sw_timeout_ms = flashlight->flash_duration_ms;
	else /* load default value */
		fl_str->flash_sw_timeout_ms = 600;
	err = led_classdev_register(&pdev->dev, &fl_str->fl_lcdev);
	if (err < 0) {
		FLT_ERR_LOG("%s: failed on led_classdev_register\n", __func__);
		goto fail_free_gpio;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	fl_str->early_suspend_flashlight.suspend = flashlight_early_suspend;
	fl_str->early_suspend_flashlight.resume = flashlight_late_resume;
	register_early_suspend(&fl_str->early_suspend_flashlight);
#endif
	hrtimer_init(&fl_str->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fl_str->timer.function = flashlight_hrtimer_func;
	fl_str->led_count = (flashlight->led_count) ? flashlight->led_count : 1;
	FLT_INFO_LOG("%s: led_count = %d\n", __func__, fl_str->led_count);

	this_fl_str = fl_str;
	FLT_INFO_LOG("%s: The Flashlight Driver is ready\n", __func__);
	return 0;

fail_free_gpio:
	flashlight_free_gpio(flashlight, fl_str);
fail_free_mem:
	kfree(fl_str);
	return err;
}

static int flashlight_remove(struct platform_device *pdev)
{
	struct flashlight_platform_data *flashlight = pdev->dev.platform_data;

	flashlight_turn_off();
	hrtimer_cancel(&this_fl_str->timer);
	unregister_early_suspend(&this_fl_str->early_suspend_flashlight);
	led_classdev_unregister(&this_fl_str->fl_lcdev);
	flashlight_free_gpio(flashlight, this_fl_str);

	kfree(this_fl_str);
	return 0;
}

static struct platform_driver flashlight_driver = {
	.probe		= flashlight_probe,
	.remove		= flashlight_remove,
	.driver		= {
		.name		= "FLASHLIGHT_AAT1271",
		.owner		= THIS_MODULE,
	},
};

static int __init flashlight_init(void)
{
	return platform_driver_register(&flashlight_driver);
}

static void __exit flashlight_exit(void)
{
	platform_driver_unregister(&flashlight_driver);
}

module_init(flashlight_init);
module_exit(flashlight_exit);

MODULE_DESCRIPTION("flash light driver");
MODULE_LICENSE("GPL");

