/* drivers/input/misc/gpio_input.c
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
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/irq.h>


enum {
	DEBOUNCE_UNSTABLE     = BIT(0),	/* Got irq, while debouncing */
	DEBOUNCE_PRESSED      = BIT(1),
	DEBOUNCE_NOTPRESSED   = BIT(2),
	DEBOUNCE_WAIT_IRQ     = BIT(3),	/* Stable irq state */
	DEBOUNCE_POLL         = BIT(4),	/* Stable polling state */

	DEBOUNCE_UNKNOWN =
		DEBOUNCE_PRESSED | DEBOUNCE_NOTPRESSED,
};

struct gpio_key_state {
	struct gpio_switch_state *ds;
	uint8_t debounce;
};

struct gpio_switch_state {
	struct gpio_event_input_devs *input_devs;
	const struct gpio_event_switch_info *info;
	struct hrtimer timer;
	int use_irq;
	int irq_status;
	int debounce_count;
	spinlock_t irq_lock;
	struct wake_lock wake_lock;
	struct gpio_key_state key_state[0];
};

static enum hrtimer_restart gpio_event_switch_timer_func(struct hrtimer *timer)
{
	int i;
	int pressed = 0;
	int pin_state = 0;
	struct gpio_switch_state *ds =
		container_of(timer, struct gpio_switch_state, timer);
	unsigned gpio_flags = ds->info->flags;
	unsigned npolarity;
	int nkeys = ds->info->keymap_size;
	const struct gpio_event_direct_entry *key_entry;
	struct gpio_key_state *key_state;
	unsigned long irqflags;
	uint8_t debounce;
	bool sync_needed;

#if 0
	key_entry = kp->keys_info->keymap;
	key_state = kp->key_state;
	for (i = 0; i < nkeys; i++, key_entry++, key_state++)
		pr_info("gpio_read_detect_status %d %d\n", key_entry->gpio,
			gpio_read_detect_status(key_entry->gpio));
#endif
	key_entry = ds->info->keymap;
	key_state = ds->key_state;
	sync_needed = false;
	spin_lock_irqsave(&ds->irq_lock, irqflags);
	for (i = 0; i < nkeys; i++, key_entry++, key_state++) {
		debounce = key_state->debounce;
		if (debounce & DEBOUNCE_WAIT_IRQ)
			continue;
		if (key_state->debounce & DEBOUNCE_UNSTABLE) {
			debounce = key_state->debounce = DEBOUNCE_UNKNOWN;
			enable_irq(gpio_to_irq(key_entry->gpio));
			pr_info("[J] gpio_switch_scan_keys: key %x-%x, %d "
				"(%d) continue debounce\n",
				ds->info->type, key_entry->code,
				i, key_entry->gpio);
		}
		npolarity = !(gpio_flags & GPIOEDF_ACTIVE_HIGH);

		pin_state = gpio_get_value(key_entry->gpio);
		if (ds->use_irq)
			irq_set_irq_type(gpio_to_irq(key_entry->gpio), (pin_state ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
		pressed = pin_state ^ npolarity;

		if (debounce & DEBOUNCE_POLL) {
			if (pressed == !(debounce & DEBOUNCE_PRESSED)) {
				ds->debounce_count++;
				key_state->debounce = DEBOUNCE_UNKNOWN;
				if (gpio_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
					pr_info("[J] gpio_switch_scan_keys: key %x-"
						"%x, %d (%d) start debounce\n",
						ds->info->type, key_entry->code,
						i, key_entry->gpio);
			}
			continue;
		}
		if (pressed && (debounce & DEBOUNCE_NOTPRESSED)) {
			if (gpio_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
				pr_info("[J] gpio_switch_scan_keys: key %x-%x, %d "
					"(%d) debounce pressed 1\n",
					ds->info->type, key_entry->code,
					i, key_entry->gpio);
			key_state->debounce = DEBOUNCE_PRESSED;
			continue;
		}
		if (!pressed && (debounce & DEBOUNCE_PRESSED)) {
			if (gpio_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
				pr_info("[J] gpio_switch_scan_keys: key %x-%x, %d "
					"(%d) debounce pressed 0\n",
					ds->info->type, key_entry->code,
					i, key_entry->gpio);
			key_state->debounce = DEBOUNCE_NOTPRESSED;
			continue;
		}
		/* key is stable */
		ds->debounce_count--;
		if (ds->use_irq)
			key_state->debounce |= DEBOUNCE_WAIT_IRQ;
		else
			key_state->debounce |= DEBOUNCE_POLL;
		if (gpio_flags & GPIOEDF_PRINT_KEYS)
			pr_info("[J] gpio_switch_scan_keys: key %x-%x, %d (%d) "
				"changed to %d\n", ds->info->type,
				key_entry->code, i, key_entry->gpio, pressed);
		input_event(ds->input_devs->dev[key_entry->dev], ds->info->type,
			    key_entry->code, pressed);
		sync_needed = true;
	}
	if (sync_needed) {
		for (i = 0; i < ds->input_devs->count; i++)
			input_sync(ds->input_devs->dev[i]);
	}

#if 0
	key_entry = kp->keys_info->keymap;
	key_state = kp->key_state;
	for (i = 0; i < nkeys; i++, key_entry++, key_state++) {
		pr_info("gpio_read_detect_status %d %d\n", key_entry->gpio,
			gpio_read_detect_status(key_entry->gpio));
	}
#endif

	if (ds->debounce_count)
		hrtimer_start(timer, ds->info->debounce_time, HRTIMER_MODE_REL);
	else if (!ds->use_irq)
		hrtimer_start(timer, ds->info->poll_time, HRTIMER_MODE_REL);
	else
		wake_unlock(&ds->wake_lock);

	spin_unlock_irqrestore(&ds->irq_lock, irqflags);

	return HRTIMER_NORESTART;
}

void switch_reprort_keycode(struct gpio_key_state *ks)
{
	struct gpio_switch_state *ds = ks->ds;
	int keymap_index = ks - ds->key_state;
	const struct gpio_event_direct_entry *key_entry;
	int pressed;
	int pin_state;

	key_entry = &ds->info->keymap[keymap_index];

	pin_state = gpio_get_value(key_entry->gpio);
	pressed = pin_state ^ !(ds->info->flags & GPIOEDF_ACTIVE_HIGH);
	irq_set_irq_type(gpio_to_irq(key_entry->gpio),
		pin_state ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (ds->info->flags & GPIOEDF_PRINT_KEYS)
		KEY_LOGI("switch_reprort_keycode: key %d-%d, %d "
			"(%d) changed to %d\n",
			ds->info->type, key_entry->code, keymap_index,
			key_entry->gpio, pressed);

	input_event(ds->input_devs->dev[key_entry->dev],
		ds->info->type, key_entry->code, pressed);

	if (key_entry->code == SW_LID) {
		if (ds->info->set_qty_irq)
			ds->info->set_qty_irq(pressed);
	}

	enable_irq(gpio_to_irq(key_entry->gpio));
}

static irqreturn_t gpio_event_switch_irq_handler(int irq, void *dev_id)
{
	struct gpio_key_state *ks = dev_id;
	struct gpio_switch_state *ds = ks->ds;
	int keymap_index = ks - ds->key_state;
	const struct gpio_event_direct_entry *key_entry;
	unsigned long irqflags;
	int pressed;
	pr_info("[J] %s, irq=%d, use_irq=%d\n", __func__, irq, ds->use_irq);

	if (!ds->use_irq)
		return IRQ_HANDLED;

	key_entry = &ds->info->keymap[keymap_index];

	if (ds->info->debounce_time.tv64) {
		spin_lock_irqsave(&ds->irq_lock, irqflags);
		if (ks->debounce & DEBOUNCE_WAIT_IRQ) {
			ks->debounce = DEBOUNCE_UNKNOWN;
			if (ds->debounce_count++ == 0) {
				wake_lock(&ds->wake_lock);
				hrtimer_start(
					&ds->timer, ds->info->debounce_time,
					HRTIMER_MODE_REL);
			}
			if (ds->info->flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
				pr_info("gpio_event_switch_irq_handler: "
					"key %x-%x, %d (%d) start debounce\n",
					ds->info->type, key_entry->code,
					keymap_index, key_entry->gpio);
		} else {
	pr_info("[J] %s, disable_irq_nosync(irq:%d), DEBOUNCE_UNSTABLE\n", __func__, irq);
			disable_irq_nosync(irq);
			ks->debounce = DEBOUNCE_UNSTABLE;
		}
		spin_unlock_irqrestore(&ds->irq_lock, irqflags);
	} else {
		pressed = gpio_get_value(key_entry->gpio) ^
			!(ds->info->flags & GPIOEDF_ACTIVE_HIGH);
		if (ds->info->flags & GPIOEDF_PRINT_KEYS)
			pr_info("gpio_event_input_irq_handler: key %x-%x, %d "
				"(%d) changed to %d\n",
				ds->info->type, key_entry->code, keymap_index,
				key_entry->gpio, pressed);
		input_event(ds->input_devs->dev[key_entry->dev], ds->info->type,
			    key_entry->code, pressed);
		input_sync(ds->input_devs->dev[key_entry->dev]);
	}
	return IRQ_HANDLED;
}

static int gpio_event_switch_request_irqs(struct gpio_switch_state *ds)
{
	int i;
	int err;
	unsigned int irq;
	unsigned long req_flags = 0;

	for (i = 0; i < ds->info->keymap_size; i++) {
		err = irq = gpio_to_irq(ds->info->keymap[i].gpio);
		if (err < 0)
			goto err_gpio_get_irq_num_failed;
		req_flags = gpio_get_value(ds->info->keymap[i].gpio) ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
		err = request_any_context_irq(irq, gpio_event_switch_irq_handler,
				  req_flags, "gpio_switchs", &ds->key_state[i]);
		if (err < 0) {
			pr_err("[J] gpio_event_switch_irq_handler: request_irq "
				"failed for input %d, irq %d, err %d\n",
				ds->info->keymap[i].gpio, irq, err);
			goto err_request_irq_failed;
		}
		if (!ds->info->keymap[i].not_wakeup_src)
			enable_irq_wake(irq);
	}
	return 0;

	for (i = ds->info->keymap_size - 1; i >= 0; i--) {
		free_irq(gpio_to_irq(ds->info->keymap[i].gpio),
			 &ds->key_state[i]);
err_request_irq_failed:
err_gpio_get_irq_num_failed:
		;
	}
	return err;
}

int gpio_event_switch_func(struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info, void **data, int func)
{
	int ret;
	int i;
	unsigned long irqflags;
	struct gpio_event_switch_info *di;
	struct gpio_switch_state *ds = *data;

	di = container_of(info, struct gpio_event_switch_info, info);

	if (func == GPIO_EVENT_FUNC_SUSPEND) {
		if (ds->use_irq)
			for (i = 0; i < di->keymap_size; i++)
				disable_irq(gpio_to_irq(di->keymap[i].gpio));
		hrtimer_cancel(&ds->timer);
		return 0;
	}
	if (func == GPIO_EVENT_FUNC_RESUME) {
		spin_lock_irqsave(&ds->irq_lock, irqflags);
		if (ds->use_irq)
			for (i = 0; i < di->keymap_size; i++)
				enable_irq(gpio_to_irq(di->keymap[i].gpio));
		hrtimer_start(&ds->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&ds->irq_lock, irqflags);
		return 0;
	}

	if (func == GPIO_EVENT_FUNC_INIT) {
		if (ktime_to_ns(di->poll_time) <= 0)
			di->poll_time = ktime_set(0, 20 * NSEC_PER_MSEC);

		*data = ds = kzalloc(sizeof(*ds) + sizeof(ds->key_state[0]) *
					di->keymap_size, GFP_KERNEL);
		if (ds == NULL) {
			ret = -ENOMEM;
			pr_err("[J] gpio_event_switch_func: "
				"Failed to allocate private data\n");
			goto err_ds_alloc_failed;
		}
		ds->debounce_count = di->keymap_size;
		ds->input_devs = input_devs;
		ds->info = di;
		wake_lock_init(&ds->wake_lock, WAKE_LOCK_SUSPEND, "gpio_switch");
		spin_lock_init(&ds->irq_lock);

		for (i = 0; i < di->keymap_size; i++) {
			int dev = di->keymap[i].dev;
			if (dev >= input_devs->count) {
				pr_err("[J] gpio_event_switch_func: bad device "
					"index %d >= %d for key code %d\n",
					dev, input_devs->count,
					di->keymap[i].code);
				ret = -EINVAL;
				goto err_bad_keymap;
			}
			input_set_capability(input_devs->dev[dev], di->type,
					     di->keymap[i].code);
			ds->key_state[i].ds = ds;
			ds->key_state[i].debounce = DEBOUNCE_UNKNOWN;
		}

		for (i = 0; i < di->keymap_size; i++) {
			ret = gpio_request(di->keymap[i].gpio, "gpio_switch");
			if (ret) {
				pr_err("[J] gpio_event_switch_func: gpio_request "
					"failed for %d\n", di->keymap[i].gpio);
				goto err_gpio_request_failed;
			}
			ret = gpio_direction_input(di->keymap[i].gpio);
			if (ret) {
				pr_err("[J] gpio_event_switch_func: "
					"gpio_direction_input failed for %d\n",
					di->keymap[i].gpio);
				goto err_gpio_configure_failed;
			}
		}

		if (di->setup_switch_gpio)
			di->setup_switch_gpio();

		ret = gpio_event_switch_request_irqs(ds);

		spin_lock_irqsave(&ds->irq_lock, irqflags);
		ds->use_irq = ret == 0;

		pr_info("[J] GPIO Switch Driver: Start gpio switchs for %s%s in %s "
			"mode\n", input_devs->dev[0]->name,
			(input_devs->count > 1) ? "..." : "",
			ret == 0 ? "interrupt" : "polling");

		hrtimer_init(&ds->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ds->timer.function = gpio_event_switch_timer_func;
		hrtimer_start(&ds->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&ds->irq_lock, irqflags);
		return 0;
	}

	ret = 0;
	spin_lock_irqsave(&ds->irq_lock, irqflags);
	hrtimer_cancel(&ds->timer);
	if (ds->use_irq) {
		for (i = di->keymap_size - 1; i >= 0; i--) {
			free_irq(gpio_to_irq(di->keymap[i].gpio),
				 &ds->key_state[i]);
		}
	}
	spin_unlock_irqrestore(&ds->irq_lock, irqflags);

	for (i = di->keymap_size - 1; i >= 0; i--) {
err_gpio_configure_failed:
		gpio_free(di->keymap[i].gpio);
err_gpio_request_failed:
		;
	}
err_bad_keymap:
	wake_lock_destroy(&ds->wake_lock);
	kfree(ds);
err_ds_alloc_failed:
	return ret;
}
