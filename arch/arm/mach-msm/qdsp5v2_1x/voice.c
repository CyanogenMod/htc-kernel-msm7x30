/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <mach/debug_audio_mm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/msm_audio.h>
#include <mach/qdsp5v2_1x/audio_dev_ctl.h>
#include <mach/dal.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <mach/qdsp5v2_1x/voice.h>
#include <mach/qdsp5v2_1x/audio_def.h>
#include <mach/debug_audio_mm.h>

struct voice_data {
	void *handle; /* DALRPC handle */
	void *cb_handle; /* DALRPC callback handle */
	int network; /* Network information */
	int dev_state;/*READY, CHANGE, REL_DONE,INIT*/
	int voc_state;/*INIT, CHANGE, RELEASE, ACQUIRE */
	struct mutex lock;
	int voc_event;
	int dev_event;
	atomic_t rel_start_flag;
	atomic_t acq_start_flag;
	struct task_struct *task;
	struct completion complete;
	wait_queue_head_t dev_wait;
	wait_queue_head_t voc_wait;
	uint32_t device_events;
	/* cache the values related to Rx and Tx */
	struct device_data dev_rx;
	struct device_data dev_tx;
	/* these default values are for all devices */
	uint32_t default_mute_val;
	uint32_t default_vol_val;
	uint32_t default_sample_val;
	/* call status */
	int v_call_status; /* Start or End */
};

static struct q5v2_hw_info def_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_gain[VOC_NB_INDEX] = 400,
		.min_gain[VOC_NB_INDEX] = -1600,
		.max_gain[VOC_WB_INDEX] = 400,
		.min_gain[VOC_WB_INDEX] = -1600,
	},
	[Q5V2_HW_HEADSET] = {
		.max_gain[VOC_NB_INDEX] = 900,
		.min_gain[VOC_NB_INDEX] = -1100,
		.max_gain[VOC_WB_INDEX] = 900,
		.min_gain[VOC_WB_INDEX] = -1100,
	},
	[Q5V2_HW_SPEAKER] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
	[Q5V2_HW_BT_SCO] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = -1500,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = -1500,
	},
	[Q5V2_HW_TTY] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
	},
	[Q5V2_HW_HS_SPKR] = {
		.max_gain[VOC_NB_INDEX] = -500,
		.min_gain[VOC_NB_INDEX] = -2000,
		.max_gain[VOC_WB_INDEX] = -500,
		.min_gain[VOC_WB_INDEX] = -2000,
	},
	[Q5V2_HW_USB_HS] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
	[Q5V2_HW_HAC] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
};

static struct voice_data voice;
static struct q5v2voice_ops default_voice_ops;
static struct q5v2voice_ops *voice_ops = &default_voice_ops;

static int voice_cmd_device_info(struct voice_data *);
static int voice_cmd_acquire_done(struct voice_data *);
static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data);

static int get_def_voice_volume(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info *info;
	int vol, maxv, minv;

	info = &def_audio_hw[hw];
	maxv = info->max_gain[network];
	minv = info->min_gain[network];
	vol = minv + ((maxv - minv) * level) / 100;
	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}
static int voice_cmd_change(void)
{

	struct voice_header hdr;
	struct voice_data *v = &voice;
	int err;

	hdr.id = CMD_DEVICE_CHANGE;
	hdr.data_len = 0;

	pr_aud_info("[voice] send CMD_DEVICE_CHANGE\n");

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
			sizeof(struct voice_header));

	if (err)
		MM_AUD_ERR("%s: failed, err %d\n", __func__, err);
	return err;
}

static void voice_auddev_cb_function(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data)
{
	struct voice_data *v = &voice;
	int rc = 0, mute = 0;

	MM_AUD_INFO("auddev_cb_function, evt_id = %d, dev_state = %d\n",
		evt_id, v->dev_state);

	if ((evt_id != AUDDEV_EVT_START_VOICE) ||
			(evt_id != AUDDEV_EVT_END_VOICE)) {
		if (evt_payload == NULL) {
			MM_AUD_ERR("%s: NULL payload\n", __func__);
			return;
		}
	}
	switch (evt_id) {
	case AUDDEV_EVT_START_VOICE:
		pr_aud_info("AUDDEV_EVT_START_VOICE\n");
		if ((v->dev_state == DEV_INIT) ||
				(v->dev_state == DEV_REL_DONE)) {
			v->v_call_status = VOICE_CALL_START;
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED)
				&& (v->dev_tx.enabled == VOICE_DEV_ENABLED)) {
				v->dev_state = DEV_READY;
				pr_aud_info("dev_state -> DEV_READY\n");
				wake_up(&v->dev_wait);
				if (v->voc_state == VOICE_CHANGE) {
					mutex_lock(&voice.lock);
					v->dev_event = DEV_CHANGE_READY;
					mutex_unlock(&voice.lock);
					complete(&v->complete);
				}
			}
		}
		break;
	case AUDDEV_EVT_DEV_CHG_VOICE:
		pr_aud_info("[dev ctrl] AUDDEV_EVT_DEV_CHG_VOICE\n");
		if (v->dev_state == DEV_READY) {
			v->dev_rx.enabled = VOICE_DEV_DISABLED;
			v->dev_tx.enabled = VOICE_DEV_DISABLED;
			v->dev_state = DEV_CHANGE;
			pr_aud_info("dev_state -> DEV_CHANGE\n");
			if (v->voc_state == VOICE_ACQUIRE) {
				msm_snddev_enable_sidetone(v->dev_rx.dev_id,
				0);
				/* send device change to modem */
				voice_cmd_change();
				/* block to wait for CHANGE_START */
				pr_aud_info("start waiting for "
					"voc_state -> VOICE_CHANGE\n");
				rc = wait_event_interruptible(
				v->voc_wait, (v->voc_state == VOICE_CHANGE)
				|| (atomic_read(&v->rel_start_flag) == 1));
				pr_aud_info("wait done, voc_state = %d\n", v->voc_state);
			} else {
				MM_AUD_ERR("Get AUDDEV_EVT_DEV_CHG_VOICE "
				       "at improper voc_state %d\n", v->voc_state);
				voice_cmd_change();
			}
		} else if ((v->dev_state == DEV_INIT) ||
				(v->dev_state == DEV_REL_DONE)) {
				v->dev_rx.enabled = VOICE_DEV_DISABLED;
				v->dev_tx.enabled = VOICE_DEV_DISABLED;
		} else {
			MM_AUD_ERR("Get AUDDEV_EVT_DEV_CHG_VOICE "
			       "at improper dev_state %d\n", v->dev_state);
			voice_cmd_change();
		}

		break;
	case AUDDEV_EVT_DEV_RDY:
		/* update the dev info */
		pr_aud_info("[dev ctrl] AUDDEV_EVT_DEV_RDY\n");
		if (evt_payload->voc_devinfo.dev_type == DIR_RX)
			v->dev_rx.vol_idx = evt_payload->voc_devinfo.vol_idx;

		if (v->dev_state == DEV_CHANGE) {
			if (evt_payload->voc_devinfo.dev_type == DIR_RX) {
				v->dev_rx.dev_acdb_id =
					evt_payload->voc_devinfo.acdb_dev_id;
				v->dev_rx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_rx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_rx.enabled = VOICE_DEV_ENABLED;
			} else {
				v->dev_tx.dev_acdb_id =
					evt_payload->voc_devinfo.acdb_dev_id;
				v->dev_tx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_tx.enabled = VOICE_DEV_ENABLED;
				v->dev_tx.dev_id =
				evt_payload->voc_devinfo.dev_id;
			}
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED) &&
				(v->dev_tx.enabled == VOICE_DEV_ENABLED)) {
				mutex_lock(&voice.lock);
				v->dev_event = DEV_CHANGE_READY;
				mutex_unlock(&voice.lock);
				complete(&v->complete);
				v->dev_state = DEV_READY;
				pr_aud_info("dev_state -> DEV_READY\n");
				wake_up(&v->dev_wait);
			}
		} else if ((v->dev_state == DEV_INIT) ||
			(v->dev_state == DEV_REL_DONE)) {
			if (evt_payload->voc_devinfo.dev_type == DIR_RX) {
				v->dev_rx.dev_acdb_id =
					evt_payload->voc_devinfo.acdb_dev_id;
				v->dev_rx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_rx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_rx.enabled = VOICE_DEV_ENABLED;
			} else {
				v->dev_tx.dev_acdb_id =
					evt_payload->voc_devinfo.acdb_dev_id;
				v->dev_tx.sample =
					evt_payload->voc_devinfo.dev_sample;
				v->dev_tx.dev_id =
				evt_payload->voc_devinfo.dev_id;
				v->dev_tx.enabled = VOICE_DEV_ENABLED;
			}
			if ((v->dev_rx.enabled == VOICE_DEV_ENABLED) &&
				(v->dev_tx.enabled == VOICE_DEV_ENABLED) &&
				(v->v_call_status == VOICE_CALL_START)) {
				v->dev_state = DEV_READY;
				pr_aud_info("dev_state -> DEV_READY\n");
				wake_up(&v->dev_wait);
				if (v->voc_state == VOICE_CHANGE) {
					mutex_lock(&voice.lock);
					v->dev_event = DEV_CHANGE_READY;
					mutex_unlock(&voice.lock);
					complete(&v->complete);
				}
			}
		} else {
			MM_AUD_ERR("Get AUDDEV_EVT_DEV_RDY "
			       "at improper dev_state %d\n", v->dev_state);
			voice_cmd_device_info(v);
		}

		break;
	case AUDDEV_EVT_DEVICE_VOL_MUTE_CHG:
		pr_aud_info("[dev ctrl] AUDDEV_EVT_DEVICE_VOL_MUTE_CHG\n");
		if (evt_payload->voc_devinfo.dev_type == DIR_TX)
			v->dev_tx.mute =
				evt_payload->voc_vm_info.dev_vm_val.mute;
		else {
			mute = (int)evt_payload->voc_vm_info.dev_vm_val.mute;
			pr_aud_info("%s, mute = %d\n", __func__, mute);
			if (mute == 1) { /*mute rx*/
				v->dev_rx.mute = evt_payload->
					voc_vm_info.dev_vm_val.mute;
			} else {
				v->dev_rx.mute = 0;
				v->dev_rx.volume = evt_payload->
					voc_vm_info.dev_vm_val.vol;
			}
		}
		/* send device info */
		voice_cmd_device_info(v);
		break;
	case AUDDEV_EVT_REL_PENDING:
		pr_aud_info("[dev ctrl] AUDDEV_EVT_REL_PENDING, dev_state %d\n",
			v->dev_state);
		/* recover the tx mute and rx volume to the default values */
		if (v->dev_state == DEV_READY) {
			if (atomic_read(&v->rel_start_flag)) {
				atomic_dec(&v->rel_start_flag);
				if (evt_payload->voc_devinfo.dev_type == DIR_RX)
					v->dev_rx.enabled = VOICE_DEV_DISABLED;
				else
					v->dev_tx.enabled = VOICE_DEV_DISABLED;
				v->dev_state = DEV_REL_DONE;
				pr_aud_info("dev_state -> DEV_REL_DONE\n");
				wake_up(&v->dev_wait);
			} else if ((v->voc_state == VOICE_RELEASE) ||
					(v->voc_state == VOICE_INIT)) {
				if (evt_payload->voc_devinfo.dev_type
							== DIR_RX) {
					v->dev_rx.enabled = VOICE_DEV_DISABLED;
				} else {
					v->dev_tx.enabled = VOICE_DEV_DISABLED;
				}
				v->dev_state = DEV_REL_DONE;
				pr_aud_info("dev_state -> DEV_REL_DONE\n");
				wake_up(&v->dev_wait);
			} else {
				/* send mute and default volume value to MCAD */
				v->dev_tx.mute = v->default_mute_val;
				v->dev_rx.volume = v->default_vol_val;
				voice_cmd_device_info(v);
				/* send device change to modem */
				voice_cmd_change();
				pr_aud_info("start waiting for "
					"voc_state -> VOICE_CHANGE\n");
				rc = wait_event_interruptible(
				v->voc_wait, (v->voc_state == VOICE_CHANGE)
				|| (atomic_read(&v->rel_start_flag) == 1));
				if (atomic_read(&v->rel_start_flag) == 1)
					atomic_dec(&v->rel_start_flag);
				/* clear Rx/Tx to Disable */
				if (evt_payload->voc_devinfo.dev_type == DIR_RX)
					v->dev_rx.enabled = VOICE_DEV_DISABLED;
				else
					v->dev_tx.enabled = VOICE_DEV_DISABLED;
				v->dev_state = DEV_REL_DONE;
				pr_aud_info("dev_state -> DEV_REL_DONE\n");
				wake_up(&v->dev_wait);
			}
		} else if ((v->dev_state == DEV_INIT) ||
				(v->dev_state == DEV_REL_DONE)) {
			if (evt_payload->voc_devinfo.dev_type == DIR_RX)
				v->dev_rx.enabled = VOICE_DEV_DISABLED;
			else
				v->dev_tx.enabled = VOICE_DEV_DISABLED;
		}
		break;
	case AUDDEV_EVT_END_VOICE:
		pr_aud_info("AUDDEV_EVT_END_VOICE\n");
		/* recover the tx mute and rx volume to the default values */
		v->dev_tx.mute = v->default_mute_val;
		v->dev_rx.volume = v->default_vol_val;

		if (v->dev_rx.enabled == VOICE_DEV_ENABLED)
			msm_snddev_enable_sidetone(v->dev_rx.dev_id, 0);

		if ((v->dev_state == DEV_READY) ||
			(v->dev_state == DEV_CHANGE)) {
			if (atomic_read(&v->rel_start_flag)) {
				atomic_dec(&v->rel_start_flag);
				v->v_call_status = VOICE_CALL_END;
				v->dev_state = DEV_REL_DONE;
				pr_aud_info("dev_state -> DEV_REL_DONE\n");
				wake_up(&v->dev_wait);
			} else if ((v->voc_state == VOICE_RELEASE) ||
					(v->voc_state == VOICE_INIT)) {
				v->v_call_status = VOICE_CALL_END;
				v->dev_state = DEV_REL_DONE;
				pr_aud_info("dev_state -> DEV_REL_DONE\n");
				wake_up(&v->dev_wait);
			} else {
				pr_aud_info("send voice_cmd_change at voc_state %d\n",
					v->voc_state);
				/* send mute and default volume value to MCAD */
				voice_cmd_device_info(v);
				/* send device change to modem */
				voice_cmd_change();
				/* block to wait for RELEASE_START
						or CHANGE_START */
				pr_aud_info("start waiting for "
					"voc_state -> VOICE_CHANGE\n");
				rc = wait_event_interruptible(
				v->voc_wait, (v->voc_state == VOICE_CHANGE)
				|| (atomic_read(&v->rel_start_flag) == 1));
				if (atomic_read(&v->rel_start_flag) == 1)
					atomic_dec(&v->rel_start_flag);
				/* set voice call to END state */
				v->v_call_status = VOICE_CALL_END;
				v->dev_state = DEV_REL_DONE;
				pr_aud_info("dev_state -> DEV_REL_DONE\n");
				wake_up(&v->dev_wait);
			}
		} else
			v->v_call_status = VOICE_CALL_END;
		break;
	case AUDDEV_EVT_FREQ_CHG:
		pr_aud_info("[dev ctrl] AUDDEV_EVT_FREQ_CHG\n");
		MM_DBG("Voice Driver got sample rate change Event\n");
		MM_DBG("sample rate %d\n", evt_payload->freq_info.sample_rate);
		MM_DBG("dev_type %d\n", evt_payload->freq_info.dev_type);
		MM_DBG("acdb_dev_id %d\n", evt_payload->freq_info.acdb_dev_id);
		if (v->dev_state == DEV_READY) {
			v->dev_tx.enabled = VOICE_DEV_DISABLED;
			v->dev_state = DEV_CHANGE;
			pr_aud_info("dev_state -> DEV_CHANGE\n");
			if (v->voc_state == VOICE_ACQUIRE) {
				/* send device change to modem */
				voice_cmd_change();
				/* block to wait for CHANGE_START */
				pr_aud_info("start waiting for "
					"voc_state -> VOICE_CHANGE\n");
				rc = wait_event_interruptible(
				v->voc_wait, (v->voc_state == VOICE_CHANGE)
				|| (atomic_read(&v->rel_start_flag) == 1));
			} else
				MM_AUD_ERR(" Voice is not at ACQUIRE state"
				       " (voc_state %d)\n", v->voc_state);
		} else if ((v->dev_state == DEV_INIT) ||
				(v->dev_state == DEV_REL_DONE)) {
				v->dev_tx.enabled = VOICE_DEV_DISABLED;
		} else
			MM_AUD_ERR("Get AUDDEV_EVT_FREQ_CHG "
			       "at improper dev_state %d\n", v->dev_state);

		break;
	default:
		MM_AUD_ERR("%s: unknown event %d\n", __func__, evt_id);
	}
	return;
}
EXPORT_SYMBOL(voice_auddev_cb_function);

static void remote_cb_function(void *context, u32 param,
				void *evt_buf, u32 len)
{
	struct voice_header *hdr;
	struct voice_data *v = context;

	hdr = (struct voice_header *)evt_buf;

	MM_AUD_INFO("%s() len = %d, id = %d\n", __func__, len, hdr->id);

	if (len <= 0) {
		MM_AUD_ERR("%s: invalid param length %d \n", __func__, len);
		return;
	}

	switch (hdr->id) {
	case EVENT_ACQUIRE_START:
		pr_aud_info("[radio] EVENT_ACQUIRE_START\n");
		atomic_inc(&v->acq_start_flag);
		wake_up(&v->dev_wait);
		mutex_lock(&voice.lock);
		v->voc_event = VOICE_ACQUIRE_START;
		v->network = ((struct voice_network *)evt_buf)->network_info;
		mutex_unlock(&voice.lock);
		complete(&v->complete);
		break;
	case EVENT_RELEASE_START:
		pr_aud_info("[radio] EVENT_RELEASE_START\n");
		/* If ACQUIRED come in before the RELEASE,
		* will only services the RELEASE */
		atomic_inc(&v->rel_start_flag);
		wake_up(&v->voc_wait);
		wake_up(&v->dev_wait);
		mutex_lock(&voice.lock);
		v->voc_event = VOICE_RELEASE_START;
		mutex_unlock(&voice.lock);
		complete(&v->complete);
		break;
	case EVENT_CHANGE_START:
		pr_aud_info("[radio] EVENT_CHANGE_START\n");
		mutex_lock(&voice.lock);
		v->voc_event = VOICE_CHANGE_START;
		mutex_unlock(&voice.lock);
		complete(&v->complete);
		break;
	case EVENT_NETWORK_RECONFIG:
		/* send network change to audio_dev,
		if sample rate is less than 16k,
		otherwise, send acquire done */
		pr_aud_info("[radio] EVENT_NETWORK_CONFIG\n");
		mutex_lock(&voice.lock);
		v->voc_event = VOICE_NETWORK_RECONFIG;
		v->network = ((struct voice_network *)evt_buf)->network_info;
		mutex_unlock(&voice.lock);
		complete(&v->complete);
		break;
	default:
		MM_AUD_ERR("%s: unknown event %d \n", __func__, hdr->id);
	}

}

static int voice_cmd_init(struct voice_data *v)
{

	struct voice_init cmd;
	int err;

	pr_aud_info("[voice] send CMD_ACQUIRE_INIT\n");

	cmd.hdr.id = CMD_VOICE_INIT;
	cmd.hdr.data_len = sizeof(struct voice_init) -
				sizeof(struct voice_header);
	cmd.cb_handle = v->cb_handle;

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &cmd,
			 sizeof(struct voice_init));

	if (err)
		MM_AUD_ERR("%s: failed, err %d\n", __func__, err);
	return err;
}

static int voice_cmd_acquire_done(struct voice_data *v)
{
	struct voice_header hdr;
	int err;

	hdr.id = CMD_ACQUIRE_DONE;
	hdr.data_len = 0;

	pr_aud_info("[voice] send CMD_ACQUIRE_DONE\n");

	/* Enable HW sidetone if device supports it  */
	msm_snddev_enable_sidetone(v->dev_rx.dev_id, 1);

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
			 sizeof(struct voice_header));

	if (err)
		MM_AUD_ERR("%s: failed, err %d\n", __func__, err);
	return err;
}

static int voice_cmd_release_done(struct voice_data *v)
{
	struct voice_header hdr;
	int err;

	hdr.id = CMD_RELEASE_DONE;
	hdr.data_len = 0;

	pr_aud_info("[voice] send CMD_RELEASE_DONE\n");

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &hdr,
			 sizeof(struct voice_header));

	if (err)
		MM_AUD_ERR("%s: failed, err %d\n", __func__, err);
	return err;
}

static int voice_cmd_device_info(struct voice_data *v)
{
	struct voice_device cmd;
	int err, vol;

	MM_AUD_INFO("%s(), tx_dev = %d, rx_dev = %d,"
		" tx_sample = %d, rx_sample = %d \n",
		__func__, v->dev_tx.dev_acdb_id, v->dev_rx.dev_acdb_id,
		v->dev_tx.sample, v->dev_rx.sample);

	pr_aud_info("[voice] send CMD_DEVICE_INFO "
		"(tx %d, rate %d) (rx %d, rate %d)\n",
		v->dev_tx.dev_acdb_id, v->dev_tx.sample,
		v->dev_rx.dev_acdb_id, v->dev_rx.sample);


	cmd.hdr.id = CMD_DEVICE_INFO;
	cmd.hdr.data_len = sizeof(struct voice_device) -
			sizeof(struct voice_header);
	cmd.tx_device = v->dev_tx.dev_acdb_id;
	cmd.rx_device = v->dev_rx.dev_acdb_id;
	if (v->network == NETWORK_WCDMA_WB) {
		if (voice_ops->get_rx_vol) {
			vol = voice_ops->get_rx_vol(v->dev_rx.vol_idx,
						VOC_WB_INDEX, v->dev_rx.volume);
		} else {
			vol = get_def_voice_volume(v->dev_rx.vol_idx,
						VOC_WB_INDEX, v->dev_rx.volume);
		}
	} else {
		if (voice_ops->get_rx_vol) {
			vol = voice_ops->get_rx_vol(v->dev_rx.vol_idx,
						VOC_NB_INDEX, v->dev_rx.volume);
		} else {
			vol = get_def_voice_volume(v->dev_rx.vol_idx,
						VOC_NB_INDEX, v->dev_rx.volume);
		}
	}
	cmd.rx_volume = (u32)vol; /* in mb */
	/*cmd.rx_mute = 0;*/
	cmd.rx_mute = v->dev_rx.mute;
	cmd.tx_mute = v->dev_tx.mute;
	cmd.rx_sample = v->dev_rx.sample/1000;
	cmd.tx_sample = v->dev_tx.sample/1000;

	pr_aud_info("rx dev_id = %d, tx_dev_id = %d,"
		"rx_vol = %d, tx_mute = %d, rx_mute = %d\n",
		v->dev_rx.dev_id, v->dev_tx.dev_id,
		cmd.rx_volume, v->dev_tx.mute, v->dev_rx.mute);
	MM_AUD_INFO("rx_vol = %d, tx_mute = %d\n", cmd.rx_volume, v->dev_tx.mute);

	err = dalrpc_fcn_5(VOICE_DALRPC_CMD, v->handle, &cmd,
			 sizeof(struct voice_device));

	if (err)
		MM_AUD_ERR("%s: failed, err %d\n", __func__, err);
	return err;
}
EXPORT_SYMBOL(voice_cmd_device_info);

void voice_change_sample_rate(struct voice_data *v)
{
	int freq = 8000;
	int rc = 0;

	MM_AUD_INFO("network = %d, vote freq = %d\n", v->network, freq);
	pr_aud_info("%s: network %d, freq %d\n", __func__, v->network, freq);
	if (freq != v->dev_tx.sample) {
		rc = msm_snddev_request_freq(&freq, 0,
				SNDDEV_CAP_TX, AUDDEV_CLNT_VOC);
		if (rc >= 0) {
			v->dev_tx.sample = freq;
			MM_AUD_INFO("%s: set freq %d success.\n", __func__, freq);
		} else
			MM_AUD_ERR("%s: set freq %d failed.\n\n", __func__, freq);
	}
}

static int voice_thread(void *data)
{
	struct voice_data *v = (struct voice_data *)data;
	int rc = 0;

	MM_AUD_INFO("voice_thread() start\n");

	while (!kthread_should_stop()) {
		wait_for_completion(&v->complete);
		init_completion(&v->complete);

		pr_aud_info("handle voice event %d, "
			"(voc_state %d, dev_event %d)\n",
			v->voc_event, v->voc_state, v->dev_event);

		if (v->dev_event != DEV_CHANGE_READY) {
			switch (v->voc_event) {
			case VOICE_ACQUIRE_START:
				/* check if dev_state = READY */
				/* if ready, send device_info and acquire_done */
				/* if not ready, block to wait the dev_state = READY */
				if ((v->voc_state == VOICE_INIT) ||
					(v->voc_state == VOICE_RELEASE)) {
					if (v->dev_state == DEV_READY) {
						voice_change_sample_rate(v);
						rc = voice_cmd_device_info(v);
						rc = voice_cmd_acquire_done(v);
						v->voc_state = VOICE_ACQUIRE;
                                                broadcast_event(AUDDEV_EVT_VOICE_STATE_CHG,
                                                        VOICE_STATE_INCALL, SESSION_IGNORE);
						pr_aud_info("voc_state -> VOICE_ACQUIRE\n");
					} else {
						pr_aud_info("start waiting for "
							"dev_state -> DEV_READY\n");
						rc = wait_event_interruptible(
							v->dev_wait,
							(v->dev_state == DEV_READY)
							|| (atomic_read(&v->rel_start_flag) == 1));
						if (atomic_read(&v->rel_start_flag)
							== 1) {
							v->voc_state = VOICE_RELEASE;
							pr_aud_info("voc_state -> VOICE_RELEASE\n");
							atomic_dec(&v->rel_start_flag);
							msm_snddev_withdraw_freq(0,
								SNDDEV_CAP_TX, AUDDEV_CLNT_VOC);
                                                        broadcast_event(AUDDEV_EVT_VOICE_STATE_CHG,
                                                                VOICE_STATE_OFFCALL, SESSION_IGNORE);
						} else {
							voice_change_sample_rate(v);
							rc = voice_cmd_device_info(v);
							rc = voice_cmd_acquire_done(v);
							v->voc_state = VOICE_ACQUIRE;
                                                        broadcast_event(AUDDEV_EVT_VOICE_STATE_CHG,
                                                                VOICE_STATE_INCALL, SESSION_IGNORE);
							pr_aud_info("voc_state -> VOICE_ACQUIRE\n");
						}
					}
				} else {
					pr_aud_err("Get VOICE_ACQUIRE_START "
					       "at wrong voc_state %d\n", v->voc_state);
					/* avoid vocoder state of modem side will be blocked
					when audo path has been changed before acquire start */
					voice_change_sample_rate(v);
					rc = voice_cmd_device_info(v);
					rc = voice_cmd_acquire_done(v);
				}

				if (atomic_read(&v->acq_start_flag))
					atomic_dec(&v->acq_start_flag);
				break;
			case VOICE_RELEASE_START:
				if ((v->dev_state == DEV_REL_DONE) ||
					(v->dev_state == DEV_INIT)) {
					v->voc_state = VOICE_RELEASE;
					pr_aud_info("voc_state -> VOICE_RELEASE\n");
					msm_snddev_withdraw_freq(0, SNDDEV_CAP_TX,
						AUDDEV_CLNT_VOC);
                                        broadcast_event(AUDDEV_EVT_VOICE_STATE_CHG,
                                                VOICE_STATE_OFFCALL,SESSION_IGNORE);
				} else {
					/* wait for the dev_state = RELEASE */
					pr_aud_info("start waiting for "
						"dev_state -> DEV_REL_DONE\n");
					rc = wait_event_interruptible(v->dev_wait,
						(v->dev_state == DEV_REL_DONE)
						|| (atomic_read(&v->acq_start_flag) == 1));
					if (atomic_read(&v->acq_start_flag) == 1)
						atomic_dec(&v->acq_start_flag);
					else
						rc = voice_cmd_release_done(v);
					v->voc_state = VOICE_RELEASE;
					pr_aud_info("voc_state -> VOICE_RELEASE\n");
					msm_snddev_withdraw_freq(0, SNDDEV_CAP_TX,
						AUDDEV_CLNT_VOC);
                                        broadcast_event(AUDDEV_EVT_VOICE_STATE_CHG,
                                                VOICE_STATE_OFFCALL,SESSION_IGNORE);
				}
				if (atomic_read(&v->rel_start_flag))
					atomic_dec(&v->rel_start_flag);
				break;
			case VOICE_CHANGE_START:
				if (v->voc_state == VOICE_ACQUIRE) {
					v->voc_state = VOICE_CHANGE;
					pr_aud_info("voc_state -> VOICE_CHANGE\n");
				} else
					MM_AUD_ERR("Get VOICE_CHANGE_START "
					       "at wrong voc_state %d\n", v->voc_state);

				if (v->dev_state == DEV_READY)
					voice_cmd_device_info(v);

				wake_up(&v->voc_wait);
				break;
			case VOICE_NETWORK_RECONFIG:
				if ((v->voc_state == VOICE_ACQUIRE)
					|| (v->voc_state == VOICE_CHANGE)) {
					voice_change_sample_rate(v);
					rc = voice_cmd_device_info(v);
					rc = voice_cmd_acquire_done(v);
					rc = voice_cmd_device_info(v);
				} else
					pr_aud_err("Get VOICE_NETWORK_RECONFIG "
					       "at wrong voc_state %d\n", v->voc_state);
				break;
			default:
				break;
			}
		}

		switch (v->dev_event) {
		case DEV_CHANGE_READY:
			pr_aud_info("Get DEV_CHANGE_READY at voc_state %d\n",
				v->voc_state);
			if (v->voc_state == VOICE_CHANGE) {
				msm_snddev_enable_sidetone(v->dev_rx.dev_id, 1);
				/* send device info to modem */
				voice_cmd_device_info(v);
				/* update voice state */
				v->voc_state = VOICE_ACQUIRE;
				pr_aud_info("voc_state -> VOICE_ACQUIRE\n");
                                broadcast_event(AUDDEV_EVT_VOICE_STATE_CHG,
                                        VOICE_STATE_INCALL, SESSION_IGNORE);
			} else {
				MM_AUD_ERR("Get DEV_CHANGE_READY "
					"at the wrong voc_state %d\n", v->voc_state);
				voice_cmd_device_info(v);
			}

			break;
		default:
			break;
		}
		mutex_lock(&voice.lock);
		v->dev_event = 0;
		mutex_unlock(&voice.lock);
	}
	return 0;
}

void htc_7x30_register_voice_ops(struct q5v2voice_ops *ops)
{
	voice_ops = ops;
}

static int __init voice_init(void)
{
	int rc;
	struct voice_data *v = &voice;
	MM_AUD_INFO("%s\n", __func__);

	mutex_init(&voice.lock);
	v->handle = NULL;
	v->cb_handle = NULL;

	/* set default value */
	v->default_mute_val = 1;  /* default is mute */
	v->default_vol_val = 100;
	v->default_sample_val = 8000;
	v->network = NETWORK_GSM;

	/* initialize dev_rx and dev_tx */
	memset(&v->dev_tx, 0, sizeof(struct device_data));
	memset(&v->dev_rx, 0, sizeof(struct device_data));
	v->dev_rx.volume = v->default_vol_val;
	v->dev_tx.mute = v->default_mute_val;

	v->dev_state = DEV_INIT;
	pr_aud_info("dev_state -> DEV_INIT\n");
	v->voc_state = VOICE_INIT;
	pr_aud_info("voc_state -> VOICE_INIT\n");
	atomic_set(&v->rel_start_flag, 0);
	atomic_set(&v->acq_start_flag, 0);
	v->dev_event = 0;
	v->voc_event = 0;
	init_completion(&voice.complete);
	init_waitqueue_head(&v->dev_wait);
	init_waitqueue_head(&v->voc_wait);

	 /* get device handle */
	rc = daldevice_attach(VOICE_DALRPC_DEVICEID,
				VOICE_DALRPC_PORT_NAME,
				VOICE_DALRPC_CPU,
				&v->handle);
	if (rc) {
		MM_AUD_ERR("%s: daldevice_attach failed, rc %d\n",
			__func__, rc);
		goto done;
	}

	/* Allocate the callback handle */
	v->cb_handle = dalrpc_alloc_cb(v->handle, remote_cb_function, v);
	if (v->cb_handle == NULL) {
		MM_AUD_ERR("%s: dalrpc_alloc_cb failed\n", __func__);
		goto err;
	}

	/* setup the callback */
	rc = voice_cmd_init(v);
	if (rc)
		goto err1;

	v->device_events = AUDDEV_EVT_DEV_CHG_VOICE |
			AUDDEV_EVT_DEV_RDY |
			AUDDEV_EVT_REL_PENDING |
			AUDDEV_EVT_START_VOICE |
			AUDDEV_EVT_END_VOICE |
			AUDDEV_EVT_DEVICE_VOL_MUTE_CHG |
			AUDDEV_EVT_FREQ_CHG;

	/* register callback to auddev */
	auddev_register_evt_listner(v->device_events, AUDDEV_CLNT_VOC,
				0, voice_auddev_cb_function, v);

	/* create and start thread */
	v->task = kthread_run(voice_thread, v, "voice");
	if (IS_ERR(v->task)) {
		rc = PTR_ERR(v->task);
		v->task = NULL;
	} else
		goto done;

err1:   dalrpc_dealloc_cb(v->handle, v->cb_handle);
err:
	daldevice_detach(v->handle);
	v->handle = NULL;
done:
	return rc;
}

late_initcall(voice_init);
