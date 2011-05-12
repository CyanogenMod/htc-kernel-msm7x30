/* drivers/input/misc/gpio_matrix.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#ifndef CONFIG_ARCH_MSM8X60
#include <linux/hrtimer.h>
#endif
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
#include <asm/mach-types.h>
#include <linux/curcial_oj.h>
#endif

#ifdef CONFIG_ARCH_MSM8X60
static struct workqueue_struct *km_queue;
#endif

static int kp_use_irq;
int get_kp_irq_mode(void)
{
	KEY_LOGD("%s: kp_use_irq=%d\n", __func__, kp_use_irq);
	return kp_use_irq;
}
EXPORT_SYMBOL_GPL(get_kp_irq_mode);

struct gpio_kp {
	struct gpio_event_input_devs *input_devs;
	struct gpio_event_matrix_info *keypad_info;
#ifndef CONFIG_ARCH_MSM8X60
	struct hrtimer timer;
#else
	struct work_struct work;
#endif
	struct wake_lock wake_lock;
	int current_output;
	unsigned int use_irq:1;
	unsigned int key_state_changed:1;
	unsigned int last_key_state_changed:1;
	unsigned int some_keys_pressed:2;
	unsigned int disabled_irq:1;
	unsigned long keys_pressed[0];
};

static void clear_phantom_key(struct gpio_kp *kp, int out, int in)
{
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	int key_index = out * mi->ninputs + in;
	unsigned short keyentry = mi->keymap[key_index];
	unsigned short keycode = keyentry & MATRIX_KEY_MASK;
	unsigned short dev = keyentry >> MATRIX_CODE_BITS;

	if (!test_bit(keycode, kp->input_devs->dev[dev]->key)) {
		if (mi->flags & GPIOKPF_PRINT_PHANTOM_KEYS)
			KEY_LOGI("gpiomatrix: phantom key %d, %d-%d (%d-%d) "
				"cleared\n", keycode, out, in,
				mi->output_gpios[out], mi->input_gpios[in]);
		__clear_bit(key_index, kp->keys_pressed);
	} else {
		if (mi->flags & GPIOKPF_PRINT_PHANTOM_KEYS)
			KEY_LOGI("gpiomatrix: phantom key %d, %d-%d (%d-%d) "
				"not cleared\n", keycode, out, in,
				mi->output_gpios[out], mi->input_gpios[in]);
	}
}

static int restore_keys_for_input(struct gpio_kp *kp, int out, int in)
{
	int rv = 0;
	int key_index;

	key_index = out * kp->keypad_info->ninputs + in;
	while (out < kp->keypad_info->noutputs) {
		if (test_bit(key_index, kp->keys_pressed)) {
			rv = 1;
			clear_phantom_key(kp, out, in);
		}
		key_index += kp->keypad_info->ninputs;
		out++;
	}
	return rv;
}

static void remove_phantom_keys(struct gpio_kp *kp)
{
	int out, in, inp;
	int key_index;

	if (kp->some_keys_pressed < 3)
		return;

	for (out = 0; out < kp->keypad_info->noutputs; out++) {
		inp = -1;
		key_index = out * kp->keypad_info->ninputs;
		for (in = 0; in < kp->keypad_info->ninputs; in++, key_index++) {
			if (test_bit(key_index, kp->keys_pressed)) {
				if (inp == -1) {
					inp = in;
					continue;
				}
				if (inp >= 0) {
					if (!restore_keys_for_input(kp, out + 1,
									inp))
						break;
					clear_phantom_key(kp, out, inp);
					inp = -2;
				}
				restore_keys_for_input(kp, out, in);
			}
		}
	}
}

static void report_key(struct gpio_kp *kp, int key_index, int out, int in)
{
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	int pressed = test_bit(key_index, kp->keys_pressed);
	unsigned short keyentry = mi->keymap[key_index];
	unsigned short keycode = keyentry & MATRIX_KEY_MASK;
	unsigned short dev = keyentry >> MATRIX_CODE_BITS;
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	static unsigned need_send_spec_key = 1;
#endif

	if (pressed != test_bit(keycode, kp->input_devs->dev[dev]->key)) {
		if (keycode == KEY_RESERVED) {
			KEY_LOGI("gpiomatrix: unmapped key, %d-%d "
					"(%d-%d) changed to %d\n",
					out, in, mi->output_gpios[out],
					mi->input_gpios[in], pressed);
		} else {
			KEY_LOGI("gpiomatrix: key %d, %d-%d (%d-%d) "
					"changed to %d\n", keycode,
					out, in, mi->output_gpios[out],
					mi->input_gpios[in], pressed);
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
			if (mi->info.oj_btn && keycode == BTN_MOUSE)
				;
			else
#endif
				input_report_key(kp->input_devs->dev[dev],
							keycode, pressed);
		}
	}
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	if (mi->info.oj_btn && keycode == BTN_MOUSE) {
		if (need_send_spec_key == pressed) {
			curcial_oj_send_key(keycode, pressed);
			need_send_spec_key = !pressed;
			KEY_LOGI("%s: send OJ action key, pressed: %d\n",
				__func__, pressed);
		}
	}
#endif
}

#ifndef CONFIG_ARCH_MSM8X60
static enum hrtimer_restart gpio_keypad_timer_func(struct hrtimer *timer)
#else
static void gpio_keypad_timer_func(struct work_struct *work)
#endif
{
	int out, in;
	int key_index;
	int gpio;
#ifndef CONFIG_ARCH_MSM8X60
	struct gpio_kp *kp = container_of(timer, struct gpio_kp, timer);
#else
	struct gpio_kp *kp = container_of(work, struct gpio_kp, work);
#endif
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	unsigned gpio_keypad_flags = mi->flags;
	unsigned polarity = !!(gpio_keypad_flags & GPIOKPF_ACTIVE_HIGH);

	out = kp->current_output;
	if (out == mi->noutputs) {
		out = 0;
		kp->last_key_state_changed = kp->key_state_changed;
		kp->key_state_changed = 0;
		kp->some_keys_pressed = 0;
	} else {
		key_index = out * mi->ninputs;
		for (in = 0; in < mi->ninputs; in++, key_index++) {
			gpio = mi->input_gpios[in];
			if (gpio_get_value(gpio) ^ !polarity) {
				if (kp->some_keys_pressed < 3)
					kp->some_keys_pressed++;
				kp->key_state_changed |= !__test_and_set_bit(
						key_index, kp->keys_pressed);
			} else
				kp->key_state_changed |= __test_and_clear_bit(
						key_index, kp->keys_pressed);
		}
		gpio = mi->output_gpios[out];
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(gpio, !polarity);
		else
			gpio_direction_input(gpio);
		out++;
	}
	kp->current_output = out;
	if (out < mi->noutputs) {
		gpio = mi->output_gpios[out];
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(gpio, polarity);
		else
			gpio_direction_output(gpio, polarity);
#ifndef CONFIG_ARCH_MSM8X60
		hrtimer_start(timer, mi->settle_time, HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;
#else
		queue_work(km_queue, &kp->work);
		return;
#endif
	}
	if (gpio_keypad_flags & GPIOKPF_DEBOUNCE) {
		if (kp->key_state_changed) {
#ifndef CONFIG_ARCH_MSM8X60
			hrtimer_start(&kp->timer, mi->debounce_delay,
				      HRTIMER_MODE_REL);
			return HRTIMER_NORESTART;
#else
			queue_work(km_queue, &kp->work);
			return;
#endif
		}
		kp->key_state_changed = kp->last_key_state_changed;
	}
	if (kp->key_state_changed) {
		if (gpio_keypad_flags & GPIOKPF_REMOVE_SOME_PHANTOM_KEYS)
			remove_phantom_keys(kp);
		key_index = 0;
		for (out = 0; out < mi->noutputs; out++)
			for (in = 0; in < mi->ninputs; in++, key_index++)
				report_key(kp, key_index, out, in);
	}
	if (!kp->use_irq || kp->some_keys_pressed) {
#ifndef CONFIG_ARCH_MSM8X60
		hrtimer_start(timer, mi->poll_time, HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;
#else
		queue_work(km_queue, &kp->work);
		return;
#endif
	}

	/* No keys are pressed, reenable interrupt */
	for (out = 0; out < mi->noutputs; out++) {
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(mi->output_gpios[out], polarity);
		else
			gpio_direction_output(mi->output_gpios[out], polarity);
	}
	for (in = 0; in < mi->ninputs; in++)
		enable_irq(gpio_to_irq(mi->input_gpios[in]));
	wake_unlock(&kp->wake_lock);
#ifndef CONFIG_ARCH_MSM8X60
	return HRTIMER_NORESTART;
#else
	return;
#endif
}

static irqreturn_t gpio_keypad_irq_handler(int irq_in, void *dev_id)
{
	int i;
	struct gpio_kp *kp = dev_id;
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	unsigned gpio_keypad_flags = mi->flags;

	if (!kp->use_irq) {
		/* ignore interrupt while registering the handler */
		kp->disabled_irq = 1;
		disable_irq_nosync(irq_in);
		return IRQ_HANDLED;
	}

	for (i = 0; i < mi->ninputs; i++)
		disable_irq_nosync(gpio_to_irq(mi->input_gpios[i]));
	for (i = 0; i < mi->noutputs; i++) {
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(mi->output_gpios[i],
				!(gpio_keypad_flags & GPIOKPF_ACTIVE_HIGH));
		else
			gpio_direction_input(mi->output_gpios[i]);
	}
	wake_lock(&kp->wake_lock);
#ifndef CONFIG_ARCH_MSM8X60
	hrtimer_start(&kp->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
#else
	queue_work(km_queue, &kp->work);
#endif
	return IRQ_HANDLED;
}

static int gpio_keypad_request_irqs(struct gpio_kp *kp)
{
	int i;
	int err;
	unsigned int irq;
	unsigned long request_flags;
	struct gpio_event_matrix_info *mi = kp->keypad_info;

	switch (mi->flags & (GPIOKPF_ACTIVE_HIGH|GPIOKPF_LEVEL_TRIGGERED_IRQ)) {
	default:
		request_flags = IRQF_TRIGGER_FALLING;
		break;
	case GPIOKPF_ACTIVE_HIGH:
		request_flags = IRQF_TRIGGER_RISING;
		break;
	case GPIOKPF_LEVEL_TRIGGERED_IRQ:
		request_flags = IRQF_TRIGGER_LOW;
		break;
	case GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_ACTIVE_HIGH:
		request_flags = IRQF_TRIGGER_HIGH;
		break;
	}

	for (i = 0; i < mi->ninputs; i++) {
		err = irq = gpio_to_irq(mi->input_gpios[i]);
		if (err < 0)
			goto err_gpio_get_irq_num_failed;
#ifndef CONFIG_ARCH_MSM8X60
		err = request_irq(irq, gpio_keypad_irq_handler, request_flags,
				  "gpio_kp", kp);
		if (err) {
			KEY_LOGE("gpiomatrix: request_irq failed for input %d, "
				"irq %d\n", mi->input_gpios[i], irq);
			goto err_request_irq_failed;
		}
#else
		err = request_any_context_irq(irq, gpio_keypad_irq_handler, request_flags,
				  "gpio_kp", kp);
		if (err < 0) {
			KEY_LOGE("gpiomatrix: request_irq failed for input %d, "
				"irq %d, err %d\n", mi->input_gpios[i], irq, err);
			goto err_request_irq_failed;
		}
#endif
		err = set_irq_wake(irq, 1);
		if (err) {
			KEY_LOGE("gpiomatrix: set_irq_wake failed for input %d, "
				"irq %d\n", mi->input_gpios[i], irq);
		}
		disable_irq(irq);
		if (kp->disabled_irq) {
			kp->disabled_irq = 0;
			enable_irq(irq);
		}
	}
	return 0;

	for (i = mi->noutputs - 1; i >= 0; i--) {
		free_irq(gpio_to_irq(mi->input_gpios[i]), kp);
err_request_irq_failed:
err_gpio_get_irq_num_failed:
		;
	}
	return err;
}

int gpio_event_matrix_func(struct gpio_event_input_devs *input_devs,
	struct gpio_event_info *info, void **data, int func)
{
	int i;
	int err;
	int key_count;
	int phone_call_status;
	int fm_radio_status;
	int irq;
	static int irq_status = 1;
	struct gpio_kp *kp;
	struct gpio_event_matrix_info *mi;

	mi = container_of(info, struct gpio_event_matrix_info, info);
	if (func == GPIO_EVENT_FUNC_SUSPEND || func == GPIO_EVENT_FUNC_RESUME) {
		/* TODO: disable scanning */
		if (mi->detect_phone_status == 0) {
			if (func == GPIO_EVENT_FUNC_SUSPEND)
				irq_status = 0;
			else
				irq_status = 1;
		} else {
			phone_call_status = gpio_event_get_phone_call_status() & 0x01;
			fm_radio_status = gpio_event_get_fm_radio_status() & 0x01;
			KEY_LOGI("%s: mi->ninputs: %d, func&0x01 = %d, phone_call_status=%d, fm_radio_status=%d\n", __func__, mi->ninputs, func & 0x01, phone_call_status, fm_radio_status);

			if (irq_status != ((func & 0x01) | phone_call_status | fm_radio_status)) {
				irq_status = ((func & 0x01) | phone_call_status | fm_radio_status);
				KEY_LOGI("%s: irq_status %d \n", __func__, irq_status);
			} else {
				KEY_LOGI("%s: irq_status %d, did not change\n", __func__, irq_status);
				return 0;
			}
		}

		for (i = 0; i < mi->ninputs; i++) {
			irq = gpio_to_irq(mi->input_gpios[i]);
			err = set_irq_wake(irq, irq_status);
			if (err)
				KEY_LOGE("gpiomatrix: set_irq_wake failed ,irq_status %d ,for input irq %d,%d\n",  irq_status, i, irq);
			else
				KEY_LOGD("%s: set ok,irq_status %d, irq %d = %d\n", __func__, irq_status, i, irq);
		}
		return 0;
	}

	if (func == GPIO_EVENT_FUNC_INIT) {
		if (mi->keymap == NULL ||
		   mi->input_gpios == NULL ||
		   mi->output_gpios == NULL) {
			err = -ENODEV;
			KEY_LOGE("gpiomatrix: Incomplete pdata\n");
			goto err_invalid_platform_data;
		}
		key_count = mi->ninputs * mi->noutputs;

		*data = kp = kzalloc(sizeof(*kp) + sizeof(kp->keys_pressed[0]) *
				     BITS_TO_LONGS(key_count), GFP_KERNEL);
		if (kp == NULL) {
			err = -ENOMEM;
			KEY_LOGE("gpiomatrix: Failed to allocate private data\n");
			goto err_kp_alloc_failed;
		}
		kp->input_devs = input_devs;
		kp->keypad_info = mi;
		for (i = 0; i < key_count; i++) {
			unsigned short keyentry = mi->keymap[i];
			unsigned short keycode = keyentry & MATRIX_KEY_MASK;
			unsigned short dev = keyentry >> MATRIX_CODE_BITS;
			if (dev >= input_devs->count) {
				KEY_LOGE("gpiomatrix: bad device index %d >= "
					"%d for key code %d\n",
					dev, input_devs->count, keycode);
				err = -EINVAL;
				goto err_bad_keymap;
			}
			if (keycode && keycode <= KEY_MAX)
				input_set_capability(input_devs->dev[dev],
							EV_KEY, keycode);
		}

#ifndef CONFIG_ARCH_MSM8X60
		if (mi->setup_ninputs_gpio)
			mi->setup_ninputs_gpio();
#else
		if (mi->setup_matrix_gpio)
			mi->setup_matrix_gpio();
#endif

		for (i = 0; i < mi->noutputs; i++) {
			err = gpio_request(mi->output_gpios[i], "gpio_kp_out");
			if (err) {
				KEY_LOGE("gpiomatrix: gpio_request failed for "
					"output %d\n", mi->output_gpios[i]);
				goto err_request_output_gpio_failed;
			}
			if (gpio_cansleep(mi->output_gpios[i])) {
				KEY_LOGE("gpiomatrix: unsupported output gpio %d,"
					" can sleep\n", mi->output_gpios[i]);
				err = -EINVAL;
				goto err_output_gpio_configure_failed;
			}
			if (mi->flags & GPIOKPF_DRIVE_INACTIVE)
				err = gpio_direction_output(mi->output_gpios[i],
					!(mi->flags & GPIOKPF_ACTIVE_HIGH));
			else
				err = gpio_direction_input(mi->output_gpios[i]);
			if (err) {
				KEY_LOGE("gpiomatrix: gpio_configure failed for "
					"output %d\n", mi->output_gpios[i]);
				goto err_output_gpio_configure_failed;
			}
		}
		for (i = 0; i < mi->ninputs; i++) {
			err = gpio_request(mi->input_gpios[i], "gpio_kp_in");
			if (err) {
				KEY_LOGE("gpiomatrix: gpio_request failed for "
					"input %d\n", mi->input_gpios[i]);
				goto err_request_input_gpio_failed;
			}
			err = gpio_direction_input(mi->input_gpios[i]);
			if (err) {
				KEY_LOGE("gpiomatrix: gpio_direction_input failed"
					" for input %d\n", mi->input_gpios[i]);
				goto err_gpio_direction_input_failed;
			}
		}
		kp->current_output = mi->noutputs;
		kp->key_state_changed = 1;

#ifndef CONFIG_ARCH_MSM8X60
		hrtimer_init(&kp->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		kp->timer.function = gpio_keypad_timer_func;
#else
		km_queue = create_singlethread_workqueue("km_queue");
		INIT_WORK(&kp->work, gpio_keypad_timer_func);
#endif
		wake_lock_init(&kp->wake_lock, WAKE_LOCK_SUSPEND, "gpio_kp");
		err = gpio_keypad_request_irqs(kp);
		kp->use_irq = err == 0;
#ifndef CONFIG_ARCH_MSM8X60
		kp_use_irq = kp->use_irq;
#endif

		KEY_LOGI("GPIO Matrix Keypad Driver: Start keypad matrix for "
			"%s%s in %s mode\n", input_devs->dev[0]->name,
			(input_devs->count > 1) ? "..." : "",
			kp->use_irq ? "interrupt" : "polling");

		if (kp->use_irq)
			wake_lock(&kp->wake_lock);
#ifndef CONFIG_ARCH_MSM8X60
		hrtimer_start(&kp->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
#else
		queue_work(km_queue, &kp->work);
#endif
		return 0;
	}

	err = 0;
	kp = *data;

	if (kp->use_irq)
		for (i = mi->noutputs - 1; i >= 0; i--)
			free_irq(gpio_to_irq(mi->input_gpios[i]), kp);

#ifndef CONFIG_ARCH_MSM8X60
	hrtimer_cancel(&kp->timer);
#else
	cancel_work_sync(&kp->work);
#endif
	wake_lock_destroy(&kp->wake_lock);
	for (i = mi->noutputs - 1; i >= 0; i--) {
err_gpio_direction_input_failed:
		gpio_free(mi->input_gpios[i]);
err_request_input_gpio_failed:
		;
	}
	for (i = mi->noutputs - 1; i >= 0; i--) {
err_output_gpio_configure_failed:
		gpio_free(mi->output_gpios[i]);
err_request_output_gpio_failed:
		;
	}
err_bad_keymap:
	kfree(kp);
err_kp_alloc_failed:
err_invalid_platform_data:
	return err;
}
