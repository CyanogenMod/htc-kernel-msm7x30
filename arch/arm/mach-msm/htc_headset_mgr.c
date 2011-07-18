/*
 *
 * /arch/arm/mach-msm/htc_headset_mgr.c
 *
 * HTC headset manager driver.
 *
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#include <mach/htc_headset_mgr.h>

#define DRIVER_NAME "HS_MGR"

static struct workqueue_struct *detect_wq;
static void insert_detect_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(insert_detect_work, insert_detect_work_func);
static void remove_detect_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(remove_detect_work, remove_detect_work_func);
static void mic_detect_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(mic_detect_work, mic_detect_work_func);

static struct workqueue_struct *button_wq;
static void button_35mm_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_35mm_work, button_35mm_work_func);

static struct workqueue_struct *debug_wq;
static void debug_work_func(struct work_struct *work);
static DECLARE_WORK(debug_work, debug_work_func);

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len);

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_RPC_SERVER_PROG,
	.vers		= HS_RPC_SERVER_VERS,
	.rpc_call	= hs_mgr_rpc_call,
};

struct button_work {
	struct delayed_work key_work;
	int key_code;
};

static struct htc_headset_mgr_info *hi;
static struct hs_notifier_func hs_mgr_notifier;

static void init_next_driver(void)
{
	int i = hi->driver_init_seq;

	if (!hi->pdata.headset_devices_num)
		return;

	if (i < hi->pdata.headset_devices_num) {
		hi->driver_init_seq++;
		platform_device_register(hi->pdata.headset_devices[i]);
	}
}

int hs_debug_log_state(void)
{
	return (hi->debug_flag & DEBUG_FLAG_LOG) ? 1 : 0;
}

void hs_notify_driver_ready(char *name)
{
	HS_LOG("%s ready", name);
	init_next_driver();
}

void hs_notify_hpin_irq(void)
{
	hi->hpin_jiffies = jiffies;
	HS_LOG("HPIN IRQ");
}

struct class *hs_get_attribute_class(void)
{
	return hi->htc_accessory_class;
}

int hs_hpin_stable(void)
{
	unsigned long last_hpin_jiffies = 0;
	unsigned long unstable_jiffies = 1.2 * HZ;

	HS_DBG();

	last_hpin_jiffies = hi->hpin_jiffies;

	if (time_before_eq(jiffies, last_hpin_jiffies + unstable_jiffies))
		return 0;

	return 1;
}

int headset_notifier_register(struct headset_notifier *notifier)
{
	if (!notifier->func) {
		HS_LOG("NULL register function");
		return 0;
	}

	switch (notifier->id) {
	case HEADSET_REG_HPIN_GPIO:
		HS_LOG("Register HPIN_GPIO notifier");
		hs_mgr_notifier.hpin_gpio = notifier->func;
		break;
	case HEADSET_REG_REMOTE_ADC:
		HS_LOG("Register REMOTE_ADC notifier");
		hs_mgr_notifier.remote_adc = notifier->func;
		break;
	case HEADSET_REG_REMOTE_KEYCODE:
		HS_LOG("Register REMOTE_KEYCODE notifier");
		hs_mgr_notifier.remote_keycode = notifier->func;
		break;
	case HEADSET_REG_RPC_KEY:
		HS_LOG("Register RPC_KEY notifier");
		hs_mgr_notifier.rpc_key = notifier->func;
		break;
	case HEADSET_REG_MIC_STATUS:
		HS_LOG("Register MIC_STATUS notifier");
		hs_mgr_notifier.mic_status = notifier->func;
		if (hi) {
			cancel_delayed_work_sync(&mic_detect_work);
			hi->mic_detect_counter = HS_DEF_MIC_DETECT_COUNT;
			queue_delayed_work(detect_wq, &mic_detect_work,
					   HS_JIFFIES_MIC_DETECT);
		}
		break;
	case HEADSET_REG_MIC_BIAS:
		HS_LOG("Register MIC_BIAS notifier");
		hs_mgr_notifier.mic_bias_enable = notifier->func;
		break;
	case HEADSET_REG_MIC_SELECT:
		HS_LOG("Register MIC_SELECT notifier");
		hs_mgr_notifier.mic_select = notifier->func;
		break;
	case HEADSET_REG_KEY_INT_ENABLE:
		HS_LOG("Register KEY_INT_ENABLE notifier");
		hs_mgr_notifier.key_int_enable = notifier->func;
		break;
	case HEADSET_REG_KEY_ENABLE:
		HS_LOG("Register KEY_ENABLE notifier");
		hs_mgr_notifier.key_enable = notifier->func;
		break;
	default:
		HS_LOG("Unknown register ID");
		return 0;
	}

	return 1;
}

static int hs_mgr_rpc_call(struct msm_rpc_server *server,
			    struct rpc_request_hdr *req, unsigned len)
{
	struct hs_rpc_server_args_key *args_key;

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	switch (req->procedure) {
	case HS_RPC_SERVER_PROC_NULL:
		HS_LOG("RPC_SERVER_NULL");
		break;
	case HS_RPC_SERVER_PROC_KEY:
		args_key = (struct hs_rpc_server_args_key *)(req + 1);
		args_key->adc = be32_to_cpu(args_key->adc);
		HS_LOG("RPC_SERVER_KEY ADC = %u (0x%X)",
			args_key->adc, args_key->adc);
		if (hs_mgr_notifier.rpc_key)
			hs_mgr_notifier.rpc_key(args_key->adc);
		else
			HS_LOG("RPC_KEY notify function doesn't exist");
		break;
	default:
		HS_LOG("Unknown RPC procedure");
		return -EINVAL;
	}

	return 0;
}

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

void button_pressed(int type)
{
	HS_LOG_TIME("button_pressed %d", type);
	atomic_set(&hi->btn_state, type);
	input_report_key(hi->input, type, 1);
	input_sync(hi->input);
}

void button_released(int type)
{
	HS_LOG_TIME("button_released %d", type);
	atomic_set(&hi->btn_state, 0);
	input_report_key(hi->input, type, 0);
	input_sync(hi->input);
}

void headset_button_event(int is_press, int type)
{
	HS_DBG();

	if (!hs_hpin_stable()) {
		HS_LOG("The HPIN is unstable, SKIP THE BUTTON EVENT.");
		return;
	}

	if (!is_press)
		button_released(type);
	else if (!atomic_read(&hi->btn_state))
		button_pressed(type);
}

void hs_set_mic_select(int state)
{
	HS_DBG();

	if (hs_mgr_notifier.mic_select)
		hs_mgr_notifier.mic_select(state);
}

static void set_35mm_hw_state(int state)
{
	HS_DBG();

	if (hi->mic_bias_state != state && hs_mgr_notifier.mic_bias_enable) {
		hs_mgr_notifier.mic_bias_enable(state);
		hi->mic_bias_state = state;
		if (state) /* Wait for MIC bias stable */
			msleep(HS_DELAY_MIC_BIAS);
	}

	hs_set_mic_select(state);

	if (hs_mgr_notifier.key_enable)
		hs_mgr_notifier.key_enable(state);

	if (hs_mgr_notifier.key_int_enable)
		hs_mgr_notifier.key_int_enable(state);
}

static int tv_out_detect(void)
{
	int adc = 0;
	int mic = HEADSET_NO_MIC;

	HS_DBG();

	if (!hs_mgr_notifier.remote_adc)
		return HEADSET_NO_MIC;

	if (!hi->pdata.hptv_det_hp_gpio || !hi->pdata.hptv_det_tv_gpio)
		return HEADSET_NO_MIC;

	gpio_set_value(hi->pdata.hptv_det_hp_gpio, 0);
	gpio_set_value(hi->pdata.hptv_det_tv_gpio, 1);
	msleep(HS_DELAY_MIC_BIAS);

	hs_mgr_notifier.remote_adc(&adc);
	if (adc >= HS_DEF_HPTV_ADC_16_BIT_MIN &&
	    adc <= HS_DEF_HPTV_ADC_16_BIT_MAX)
	mic = HEADSET_TV_OUT;

	gpio_set_value(hi->pdata.hptv_det_hp_gpio, 1);
	gpio_set_value(hi->pdata.hptv_det_tv_gpio, 0);

	return mic;
}

#if 0
static void insert_h2w_35mm(int *state)
{
	int mic = HEADSET_NO_MIC;

	HS_LOG_TIME("Insert H2W 3.5mm headset");
	set_35mm_hw_state(1);

	if (hs_mgr_notifier.mic_status)
		mic = hs_mgr_notifier.mic_status();

	if (mic == HEADSET_NO_MIC) {
		*state |= BIT_HEADSET_NO_MIC;
		hi->h2w_35mm_status = HTC_35MM_NO_MIC;
		HS_LOG_TIME("H2W 3.5mm without microphone");
	} else {
		*state |= BIT_HEADSET;
		hi->h2w_35mm_status = HTC_35MM_MIC;
		HS_LOG_TIME("H2W 3.5mm with microphone");
	}
}

static void remove_h2w_35mm(void)
{
	HS_LOG_TIME("Remove H2W 3.5mm headset");

	set_35mm_hw_state(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->h2w_35mm_status = HTC_35MM_UNPLUG;
}
#endif /* #if 0 */

static void mic_detect_work_func(struct work_struct *work)
{
	int mic = HEADSET_NO_MIC;
	int old_state, new_state;

	wake_lock_timeout(&hi->hs_wake_lock, HS_MIC_DETECT_TIMEOUT);

	HS_DBG();

	if (!hs_mgr_notifier.mic_status) {
		HS_LOG("Failed to get MIC status");
		return;
	}

	mutex_lock(&hi->mutex_lock);

	mic = hs_mgr_notifier.mic_status();

	if (mic == HEADSET_UNKNOWN_MIC) {
		mutex_unlock(&hi->mutex_lock);
		if (hi->mic_detect_counter--)
			queue_delayed_work(detect_wq, &mic_detect_work,
					   HS_JIFFIES_MIC_DETECT);
		else
			HS_LOG("MIC polling timeout (UNKNOWN MIC status)");
		return;
	}

	old_state = switch_get_state(&hi->sdev);
	if (!(old_state & MASK_HEADSET)) {
		HS_LOG("Headset has been removed");
		mutex_unlock(&hi->mutex_lock);
		return;
	}

	new_state = old_state & ~MASK_HEADSET;
	new_state |= (mic == HEADSET_NO_MIC) ? BIT_HEADSET_NO_MIC : BIT_HEADSET;
	hi->ext_35mm_status = (mic == HEADSET_NO_MIC) ? HTC_35MM_NO_MIC :
			      HTC_35MM_MIC;

	if (new_state != old_state) {
		if (new_state & BIT_HEADSET)
			HS_LOG_TIME("Headset with MIC");
		else
			HS_LOG_TIME("Headset without MIC");
		switch_set_state(&hi->sdev, new_state);
	} else
		HS_LOG("MIC status has not changed");

	mutex_unlock(&hi->mutex_lock);
}

static void button_35mm_work_func(struct work_struct *work)
{
	int key;
	struct button_work *works;

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	works = container_of(work, struct button_work, key_work.work);
	hi->key_level_flag = works->key_code;

	if (!hi->is_ext_insert && !hi->h2w_35mm_status) {
		kfree(works);
		HS_LOG("3.5mm headset is plugged out, skip report key event");
		return;
	}

	if (hi->key_level_flag) {
		switch (hi->key_level_flag) {
		case 1:
			HS_LOG("3.5mm RC: Play Pressed");
			key = HS_MGR_KEYCODE_MEDIA;
			break;
		case 2:
			HS_LOG("3.5mm RC: BACKWARD Pressed");
			key = HS_MGR_KEYCODE_BACKWARD;
			break;
		case 3:
			HS_LOG("3.5mm RC: FORWARD Pressed");
			key = HS_MGR_KEYCODE_FORWARD;
			break;
		default:
			HS_LOG("3.5mm RC: WRONG Button Pressed");
			return;
		}
		headset_button_event(1, key);
	} else { /* key release */
		if (atomic_read(&hi->btn_state))
			headset_button_event(0, atomic_read(&hi->btn_state));
		else
			HS_LOG("3.5mm RC: WRONG Button Release");
	}

	kfree(works);
}

static void debug_work_func(struct work_struct *work)
{
	int flag = 0;
	int adc = -EINVAL;
	int hpin_gpio = -EINVAL;

	HS_DBG();

	while (hi->debug_flag & DEBUG_FLAG_ADC) {
		flag = hi->debug_flag;
		if (hs_mgr_notifier.hpin_gpio)
			hpin_gpio = hs_mgr_notifier.hpin_gpio();
		if (hs_mgr_notifier.remote_adc)
			hs_mgr_notifier.remote_adc(&adc);
		HS_LOG("Debug Flag %d, HP_DET %d, ADC %d", flag,
		       hpin_gpio, adc);
		msleep(HS_DELAY_SEC);
	}
}

static void enable_metrico_headset(int enable)
{
	HS_DBG();

	if (enable && !hi->metrico_status) {
#if 0
		enable_mos_test(1);
#endif
		hi->metrico_status = 1;
		HS_LOG("Enable metrico headset");
	}

	if (!enable && hi->metrico_status) {
#if 0
		enable_mos_test(0);
#endif
		hi->metrico_status = 0;
		HS_LOG("Disable metrico headset");
	}
}

static void remove_detect_work_func(struct work_struct *work)
{
	int state;

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	if (time_before_eq(jiffies, hi->insert_jiffies + HZ)) {
		HS_LOG("Waiting for HPIN stable");
		msleep(HS_DELAY_SEC - HS_DELAY_REMOVE);
	}

	if (hi->is_ext_insert) {
		HS_LOG("Headset has been inserted");
		return;
	}

	set_35mm_hw_state(0);

	if (hi->ext_35mm_status == HTC_35MM_TV_OUT && hi->pdata.hptv_sel_gpio)
		gpio_set_value(hi->pdata.hptv_sel_gpio, 0);

	if (hi->metrico_status)
		enable_metrico_headset(0);

	if (atomic_read(&hi->btn_state))
		button_released(atomic_read(&hi->btn_state));
	hi->ext_35mm_status = HTC_35MM_UNPLUG;

	mutex_lock(&hi->mutex_lock);

	state = switch_get_state(&hi->sdev);
	if (!(state & MASK_35MM_HEADSET)) {
		HS_LOG("Headset has been removed");
		mutex_unlock(&hi->mutex_lock);
		return;
	}

#if 0
	if (hi->cable_in1 && !gpio_get_value(hi->cable_in1)) {
		state &= ~BIT_35MM_HEADSET;
		switch_set_state(&hi->sdev, state);
		queue_delayed_work(detect_wq, &detect_h2w_work,
				   HS_DELAY_ZERO_JIFFIES);
	} else {
		state &= ~(MASK_35MM_HEADSET | MASK_FM_ATTRIBUTE);
		switch_set_state(&hi->sdev, state);
	}
#else
	state &= ~(MASK_35MM_HEADSET | MASK_FM_ATTRIBUTE);
	switch_set_state(&hi->sdev, state);
#endif

	HS_LOG_TIME("Remove 3.5mm headset");

	mutex_unlock(&hi->mutex_lock);
}

static void insert_detect_work_func(struct work_struct *work)
{
	int state;
	int mic = HEADSET_NO_MIC;

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);

	HS_DBG();

	if (!hi->is_ext_insert) {
		HS_LOG("Headset has been removed");
		return;
	}

	hi->insert_jiffies = jiffies;
	set_35mm_hw_state(1);

	mutex_lock(&hi->mutex_lock);

	if (hs_mgr_notifier.mic_status)
		mic = hs_mgr_notifier.mic_status();

	if (mic == HEADSET_NO_MIC)
		mic = tv_out_detect();

	if (mic == HEADSET_TV_OUT && hi->pdata.hptv_sel_gpio)
		gpio_set_value(hi->pdata.hptv_sel_gpio, 1);

	if (mic == HEADSET_METRICO && !hi->metrico_status)
		enable_metrico_headset(1);

	state = switch_get_state(&hi->sdev);
	state &= ~MASK_35MM_HEADSET;
	state |= BIT_35MM_HEADSET;

	switch (mic) {
	case HEADSET_TV_OUT:
		state |= BIT_TV_OUT;
		break;
	case HEADSET_METRICO:
	case HEADSET_MIC:
		state |= BIT_HEADSET;
		break;
	default:
		state |= BIT_HEADSET_NO_MIC;
	}

	if (mic == HEADSET_UNKNOWN_MIC) {
		hi->ext_35mm_status = HTC_35MM_UNKNOWN_MIC;
		HS_LOG_TIME("Insert 3.5mm headset with UNKNOWN MIC");
	} else if (mic == HEADSET_TV_OUT) {
		hi->ext_35mm_status = HTC_35MM_TV_OUT;
		HS_LOG_TIME("Insert 3.5mm TV OUT cable");
	} else if (state & BIT_HEADSET) {
		hi->ext_35mm_status = HTC_35MM_MIC;
		HS_LOG_TIME("Insert 3.5mm headset with MIC");
	} else {
		hi->ext_35mm_status = HTC_35MM_NO_MIC;
		HS_LOG_TIME("Insert 3.5mm headset without MIC");
	}
	switch_set_state(&hi->sdev, state);

	mutex_unlock(&hi->mutex_lock);

	if (mic == HEADSET_UNKNOWN_MIC) {
		HS_LOG("Start MIC status polling");
		cancel_delayed_work_sync(&mic_detect_work);
		hi->mic_detect_counter = HS_DEF_MIC_DETECT_COUNT;
		queue_delayed_work(detect_wq, &mic_detect_work,
				   HS_JIFFIES_MIC_DETECT);
	}
}

int hs_notify_plug_event(int insert)
{
	HS_DBG("Headset status %d", insert);

	mutex_lock(&hi->mutex_lock);
	hi->is_ext_insert = insert;
	mutex_unlock(&hi->mutex_lock);

	cancel_delayed_work_sync(&mic_detect_work);
	cancel_delayed_work_sync(&insert_detect_work);
	cancel_delayed_work_sync(&remove_detect_work);

	if (hi->is_ext_insert)
		queue_delayed_work(detect_wq, &insert_detect_work,
				   HS_JIFFIES_INSERT);
	else
		queue_delayed_work(detect_wq, &remove_detect_work,
				   HS_JIFFIES_REMOVE);

	return 1;
}

int hs_notify_key_event(int key_code)
{
	struct button_work *work;

	HS_DBG();

	if (hi->ext_35mm_status == HTC_35MM_UNKNOWN_MIC ||
	    hi->ext_35mm_status == HTC_35MM_NO_MIC ||
	    hi->h2w_35mm_status == HTC_35MM_NO_MIC) {
		HS_LOG("MIC re-detection");
		cancel_delayed_work_sync(&mic_detect_work);
		hi->mic_detect_counter = HS_DEF_MIC_DETECT_COUNT;
		queue_delayed_work(detect_wq, &mic_detect_work,
				   HS_JIFFIES_MIC_DETECT);
	} else if (!hs_hpin_stable()) {
		HS_LOG("The HPIN is unstable, SKIP THE BUTTON EVENT.");
		return 1;
	} else {
		work = kzalloc(sizeof(struct button_work), GFP_KERNEL);
		if (!work) {
			HS_ERR("Failed to allocate button memory");
			return 1;
		}
		work->key_code = key_code;
		INIT_DELAYED_WORK(&work->key_work, button_35mm_work_func);
		queue_delayed_work(button_wq, &work->key_work,
				   HS_JIFFIES_BUTTON);
	}

	return 1;
}

int hs_notify_key_irq(void)
{
	int adc = 0;
	int key_code = HS_MGR_KEY_INVALID;

	if (!hs_mgr_notifier.remote_adc || !hs_mgr_notifier.remote_keycode)
		return 1;

	hs_mgr_notifier.remote_adc(&adc);
	key_code = hs_mgr_notifier.remote_keycode(adc);
	hs_notify_key_event(key_code);

	return 1;
}

static void usb_headset_detect(int type)
{
	int state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);

	switch (type) {
	case USB_NO_HEADSET:
		HS_LOG_TIME("Remove USB_HEADSET");
		hi->usb_headset.type = USB_NO_HEADSET;
		hi->usb_headset.status = STATUS_DISCONNECTED;
		state &= ~MASK_USB_HEADSET;
		break;
	case USB_AUDIO_OUT:
		HS_LOG_TIME("Insert USB_AUDIO_OUT");
		hi->usb_headset.type = USB_AUDIO_OUT;
		hi->usb_headset.status = STATUS_CONNECTED_ENABLED;
		state |= BIT_USB_AUDIO_OUT;
		break;
	default:
		HS_LOG("Unknown headset type");
	}

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
}

void headset_ext_detect(int type)
{
	HS_DBG();

	switch (type) {
	case H2W_NO_HEADSET:
		/* Release Key */
	case H2W_HEADSET:
	case H2W_35MM_HEADSET:
	case H2W_REMOTE_CONTROL:
	case H2W_USB_CRADLE:
	case H2W_UART_DEBUG:
	case H2W_TVOUT:
		break;
	case USB_NO_HEADSET:
		/* Release Key */
	case USB_AUDIO_OUT:
		usb_headset_detect(type);
		break;
	default:
		HS_LOG("Unknown headset type");
	}
}

void headset_ext_button(int headset_type, int key_code, int press)
{
	HS_LOG("Headset %d, Key %d, Press %d", headset_type, key_code, press);
	headset_button_event(press, key_code);
}

int switch_send_event(unsigned int bit, int on)
{
	unsigned long state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(bit);

	if (on)
		state |= bit;

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static ssize_t tty_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	s += sprintf(s, "%d\n", hi->tty_enable_flag);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t tty_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_TTY_FULL | BIT_TTY_VCO | BIT_TTY_HCO);

	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		hi->tty_enable_flag = 1;
		switch_set_state(&hi->sdev, state | BIT_TTY_FULL);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Enable TTY FULL");
		return count;
	}
	if (count == (strlen("vco_enable") + 1) &&
	   strncmp(buf, "vco_enable", strlen("vco_enable")) == 0) {
		hi->tty_enable_flag = 2;
		switch_set_state(&hi->sdev, state | BIT_TTY_VCO);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Enable TTY VCO");
		return count;
	}
	if (count == (strlen("hco_enable") + 1) &&
	   strncmp(buf, "hco_enable", strlen("hco_enable")) == 0) {
		hi->tty_enable_flag = 3;
		switch_set_state(&hi->sdev, state | BIT_TTY_HCO);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Enable TTY HCO");
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->tty_enable_flag = 0;
		switch_set_state(&hi->sdev, state);
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Disable TTY");
		return count;
	}

	mutex_unlock(&hi->mutex_lock);
	HS_LOG("Invalid TTY argument");

	return -EINVAL;
}

static DEVICE_ACCESSORY_ATTR(tty, 0644, tty_flag_show, tty_flag_store);

static ssize_t fm_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	if (hi->fm_flag == 0)
		show_str = "disable";
	if (hi->fm_flag == 1)
		show_str = "fm_headset";
	if (hi->fm_flag == 2)
		show_str = "fm_speaker";

	s += sprintf(s, "%s\n", show_str);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t fm_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	HS_DBG();

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_FM_HEADSET | BIT_FM_SPEAKER);

	if (count == (strlen("fm_headset") + 1) &&
	   strncmp(buf, "fm_headset", strlen("fm_headset")) == 0) {
		hi->fm_flag = 1;
		state |= BIT_FM_HEADSET;
		HS_LOG("Enable FM HEADSET");
	} else if (count == (strlen("fm_speaker") + 1) &&
	   strncmp(buf, "fm_speaker", strlen("fm_speaker")) == 0) {
		hi->fm_flag = 2;
		state |= BIT_FM_SPEAKER;
		HS_LOG("Enable FM SPEAKER");
	} else if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->fm_flag = 0 ;
		HS_LOG("Disable FM");
	} else {
		mutex_unlock(&hi->mutex_lock);
		HS_LOG("Invalid FM argument");
		return -EINVAL;
	}

	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);

	return count;
}

static DEVICE_ACCESSORY_ATTR(fm, 0644, fm_flag_show, fm_flag_store);

static ssize_t debug_flag_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int flag = hi->debug_flag;
	int adc = -EINVAL;
	int hpin_gpio = -EINVAL;

	HS_DBG();

	if (hs_mgr_notifier.hpin_gpio)
		hpin_gpio = hs_mgr_notifier.hpin_gpio();
	if (hs_mgr_notifier.remote_adc)
		hs_mgr_notifier.remote_adc(&adc);

	return sprintf(buf, "Debug Flag %d, HP_DET %d, ADC %d\n", flag,
		       hpin_gpio, adc);
}

static ssize_t debug_flag_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long state = 0;

	HS_DBG();

	if (strncmp(buf, "enable", count - 1) == 0) {
		if (hi->debug_flag & DEBUG_FLAG_ADC) {
			HS_LOG("Debug work is already running");
			return count;
		}
		if (!debug_wq) {
			debug_wq = create_workqueue("debug");
			if (!debug_wq) {
				HS_LOG("Failed to create debug workqueue");
				return count;
			}
		}
		HS_LOG("Enable headset debug");
		mutex_lock(&hi->mutex_lock);
		hi->debug_flag |= DEBUG_FLAG_ADC;
		mutex_unlock(&hi->mutex_lock);
		queue_work(debug_wq, &debug_work);
	} else if (strncmp(buf, "disable", count - 1) == 0) {
		if (!(hi->debug_flag & DEBUG_FLAG_ADC)) {
			HS_LOG("Debug work has been stopped");
			return count;
		}
		HS_LOG("Disable headset debug");
		mutex_lock(&hi->mutex_lock);
		hi->debug_flag &= ~DEBUG_FLAG_ADC;
		mutex_unlock(&hi->mutex_lock);
		if (debug_wq) {
			flush_workqueue(debug_wq);
			destroy_workqueue(debug_wq);
			debug_wq = NULL;
		}
	} else if (strncmp(buf, "debug_log_enable", count - 1) == 0) {
		HS_LOG("Enable headset debug log");
		hi->debug_flag |= DEBUG_FLAG_LOG;
	} else if (strncmp(buf, "debug_log_disable", count - 1) == 0) {
		HS_LOG("Disable headset debug log");
		hi->debug_flag &= ~DEBUG_FLAG_LOG;
	} else if (strncmp(buf, "no_headset", count - 1) == 0) {
		HS_LOG("Headset simulation: no_headset");
		state = BIT_HEADSET | BIT_HEADSET_NO_MIC | BIT_35MM_HEADSET |
			BIT_TV_OUT | BIT_USB_AUDIO_OUT;
		switch_send_event(state, 0);
	} else if (strncmp(buf, "35mm_mic", count - 1) == 0) {
		HS_LOG("Headset simulation: 35mm_mic");
		state = BIT_HEADSET | BIT_35MM_HEADSET;
		switch_send_event(state, 1);
	} else if (strncmp(buf, "35mm_no_mic", count - 1) == 0) {
		HS_LOG("Headset simulation: 35mm_no_mic");
		state = BIT_HEADSET_NO_MIC | BIT_35MM_HEADSET;
		switch_send_event(state, 1);
	} else if (strncmp(buf, "35mm_tv_out", count - 1) == 0) {
		HS_LOG("Headset simulation: 35mm_tv_out");
		state = BIT_TV_OUT | BIT_35MM_HEADSET;
		switch_send_event(state, 1);
	} else if (strncmp(buf, "usb_audio", count - 1) == 0) {
		HS_LOG("Headset simulation: usb_audio");
		state = BIT_USB_AUDIO_OUT;
		switch_send_event(state, 1);
	} else {
		HS_LOG("Invalid parameter");
		return count;
	}

	return count;
}

static DEVICE_ACCESSORY_ATTR(debug, 0644, debug_flag_show, debug_flag_store);

static int register_attributes(void)
{
	int ret = 0;

	hi->htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(hi->htc_accessory_class)) {
		ret = PTR_ERR(hi->htc_accessory_class);
		hi->htc_accessory_class = NULL;
		goto err_create_class;
	}

	/* Register TTY attributes */
	hi->tty_dev = device_create(hi->htc_accessory_class,
				    NULL, 0, "%s", "tty");
	if (unlikely(IS_ERR(hi->tty_dev))) {
		ret = PTR_ERR(hi->tty_dev);
		hi->tty_dev = NULL;
		goto err_create_tty_device;
	}

	ret = device_create_file(hi->tty_dev, &dev_attr_tty);
	if (ret)
		goto err_create_tty_device_file;

	/* Register FM attributes */
	hi->fm_dev = device_create(hi->htc_accessory_class,
				   NULL, 0, "%s", "fm");
	if (unlikely(IS_ERR(hi->fm_dev))) {
		ret = PTR_ERR(hi->fm_dev);
		hi->fm_dev = NULL;
		goto err_create_fm_device;
	}

	ret = device_create_file(hi->fm_dev, &dev_attr_fm);
	if (ret)
		goto err_create_fm_device_file;

	/* Register debug attributes */
	hi->debug_dev = device_create(hi->htc_accessory_class,
				      NULL, 0, "%s", "debug");
	if (unlikely(IS_ERR(hi->debug_dev))) {
		ret = PTR_ERR(hi->debug_dev);
		hi->debug_dev = NULL;
		goto err_create_debug_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->debug_dev, &dev_attr_debug);
	if (ret)
		goto err_create_debug_device_file;

	return 0;

err_create_debug_device_file:
	device_unregister(hi->debug_dev);

err_create_debug_device:
	device_remove_file(hi->fm_dev, &dev_attr_fm);

err_create_fm_device_file:
	device_unregister(hi->fm_dev);

err_create_fm_device:
	device_remove_file(hi->tty_dev, &dev_attr_tty);

err_create_tty_device_file:
	device_unregister(hi->tty_dev);

err_create_tty_device:
	class_destroy(hi->htc_accessory_class);

err_create_class:

	return ret;
}

static void unregister_attributes(void)
{
	device_remove_file(hi->debug_dev, &dev_attr_debug);
	device_unregister(hi->debug_dev);
	device_remove_file(hi->fm_dev, &dev_attr_fm);
	device_unregister(hi->fm_dev);
	device_remove_file(hi->tty_dev, &dev_attr_tty);
	device_unregister(hi->tty_dev);
	class_destroy(hi->htc_accessory_class);
}

static void headset_mgr_init(void)
{
	if (hi->pdata.hptv_det_hp_gpio)
		gpio_set_value(hi->pdata.hptv_det_hp_gpio, 1);
	if (hi->pdata.hptv_det_tv_gpio)
		gpio_set_value(hi->pdata.hptv_det_tv_gpio, 0);
	if (hi->pdata.hptv_sel_gpio)
		gpio_set_value(hi->pdata.hptv_sel_gpio, 0);
}

static int htc_headset_mgr_probe(struct platform_device *pdev)
{
	int ret;

	struct htc_headset_mgr_platform_data *pdata = pdev->dev.platform_data;

	HS_LOG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_headset_mgr_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	hi->pdata.driver_flag = pdata->driver_flag;
	hi->pdata.headset_devices_num = pdata->headset_devices_num;
	hi->pdata.headset_devices = pdata->headset_devices;

	hi->pdata.hptv_det_hp_gpio = pdata->hptv_det_hp_gpio;
	hi->pdata.hptv_det_tv_gpio = pdata->hptv_det_tv_gpio;
	hi->pdata.hptv_sel_gpio = pdata->hptv_sel_gpio;

	hi->driver_init_seq = 0;

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	hi->hpin_jiffies = jiffies;
	hi->usb_headset.type = USB_NO_HEADSET;
	hi->usb_headset.status = STATUS_DISCONNECTED;

	hi->ext_35mm_status = HTC_35MM_UNPLUG;
	hi->h2w_35mm_status = HTC_35MM_UNPLUG;
	hi->is_ext_insert = 0;
	hi->mic_bias_state = 0;
	hi->mic_detect_counter = 0;
	hi->key_level_flag = -1;

	atomic_set(&hi->btn_state, 0);

	hi->tty_enable_flag = 0;
	hi->fm_flag = 0;
	hi->debug_flag = 0;

	mutex_init(&hi->mutex_lock);

	hi->sdev.name = "h2w";
	hi->sdev.print_name = h2w_print_name;

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	detect_wq = create_workqueue("detect");
	if (detect_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	button_wq = create_workqueue("button");
	if (button_wq  == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create button workqueue");
		goto err_create_button_work_queue;
	}

	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "h2w headset";
	set_bit(EV_SYN, hi->input->evbit);
	set_bit(EV_KEY, hi->input->evbit);
	set_bit(KEY_END, hi->input->keybit);
	set_bit(KEY_MUTE, hi->input->keybit);
	set_bit(KEY_VOLUMEDOWN, hi->input->keybit);
	set_bit(KEY_VOLUMEUP, hi->input->keybit);
	set_bit(KEY_NEXTSONG, hi->input->keybit);
	set_bit(KEY_PLAYPAUSE, hi->input->keybit);
	set_bit(KEY_PREVIOUSSONG, hi->input->keybit);
	set_bit(KEY_MEDIA, hi->input->keybit);
	set_bit(KEY_SEND, hi->input->keybit);

	ret = input_register_device(hi->input);
	if (ret < 0)
	goto err_register_input_dev;

	ret = register_attributes();
	if (ret)
		goto err_register_attributes;

	if (hi->pdata.driver_flag & DRIVER_HS_MGR_RPC_SERVER) {
		/* Create RPC server */
		ret = msm_rpc_create_server(&hs_rpc_server);
		if (ret < 0) {
			HS_ERR("Failed to create RPC server");
			goto err_create_rpc_server;
		}
		HS_LOG("Create RPC server successfully");
	}

	headset_mgr_init();
	hs_notify_driver_ready(DRIVER_NAME);

	HS_LOG("--------------------");

	return 0;

err_create_rpc_server:

err_register_attributes:
	input_unregister_device(hi->input);

err_register_input_dev:
	input_free_device(hi->input);

err_request_input_dev:
	destroy_workqueue(button_wq);

err_create_button_work_queue:
	destroy_workqueue(detect_wq);

err_create_detect_work_queue:
	switch_dev_unregister(&hi->sdev);

err_switch_dev_register:
	mutex_destroy(&hi->mutex_lock);
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_mgr_remove(struct platform_device *pdev)
{
#if 0
	if ((switch_get_state(&hi->sdev) & MASK_HEADSET) != 0)
		remove_headset();
#endif

	unregister_attributes();
	input_unregister_device(hi->input);
	destroy_workqueue(button_wq);
	destroy_workqueue(detect_wq);
	switch_dev_unregister(&hi->sdev);
	mutex_destroy(&hi->mutex_lock);
	wake_lock_destroy(&hi->hs_wake_lock);
	kfree(hi);

	return 0;
}

static struct platform_driver htc_headset_mgr_driver = {
	.probe		= htc_headset_mgr_probe,
	.remove		= htc_headset_mgr_remove,
	.driver		= {
		.name		= "HTC_HEADSET_MGR",
		.owner		= THIS_MODULE,
	},
};


static int __init htc_headset_mgr_init(void)
{
	return platform_driver_register(&htc_headset_mgr_driver);
}

static void __exit htc_headset_mgr_exit(void)
{
	platform_driver_unregister(&htc_headset_mgr_driver);
}

module_init(htc_headset_mgr_init);
module_exit(htc_headset_mgr_exit);

MODULE_DESCRIPTION("HTC headset manager driver");
MODULE_LICENSE("GPL");
