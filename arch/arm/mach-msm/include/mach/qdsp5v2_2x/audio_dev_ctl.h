/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __MACH_QDSP5_V2_SNDDEV_H
#define __MACH_QDSP5_V2_SNDDEV_H
#include <mach/qdsp5v2_2x/audio_def.h>

#define AUDIO_DEV_CTL_MAX_DEV 64
#define DIR_TX	2
#define DIR_RX	1

#define DEVICE_IGNORE	0xff
#define SESSION_IGNORE 0x00000000

#define VOICE_STATE_INVALID 0x0
#define VOICE_STATE_INCALL 0x1
#define VOICE_STATE_OFFCALL 0x2
#define MAX_COPP_NODE_SUPPORTED 6
#define MAX_AUDREC_SESSIONS 3

#define REAL_STEREO_CHANNEL_MODE	9

struct msm_snddev_info {
	const char *name;
	u32 capability;
	u32 copp_id;
	u32 acdb_id;
	u32 dev_volume;
	struct msm_snddev_ops {
		int (*open)(struct msm_snddev_info *);
		int (*close)(struct msm_snddev_info *);
		int (*set_freq)(struct msm_snddev_info *, u32);
		int (*enable_sidetone)(struct msm_snddev_info *, u32);
		int (*set_device_volume)(struct msm_snddev_info *, u32);
	} dev_ops;
	u8 opened;
	void *private_data;
	bool state;
	u32 sample_rate;
	u32 set_sample_rate;
	u32 sessions;
	u32 vol_idx;
};

struct msm_volume {
	int volume; /* Volume parameter, in % Scale */
	int pan;
};

extern struct msm_volume msm_vol_ctl;

void msm_snddev_register(struct msm_snddev_info *);
void msm_snddev_unregister(struct msm_snddev_info *);
int msm_snddev_devcount(void);
int msm_snddev_query(int dev_id);
unsigned short msm_snddev_route_dec(int popp_id);
unsigned short msm_snddev_route_enc(int enc_id);
int msm_snddev_set_dec(int popp_id, int copp_id, int set);
int msm_snddev_set_enc(int popp_id, int copp_id, int set);
int msm_snddev_is_set(int popp_id, int copp_id);
int msm_get_voc_route(u32 *rx_id, u32 *tx_id);
int msm_set_voc_route(struct msm_snddev_info *dev_info, int stream_type,
			int dev_id);
int msm_snddev_enable_sidetone(u32 dev_id, u32 enable);

struct msm_snddev_info *audio_dev_ctrl_find_dev(u32 dev_id);

void msm_release_voc_thread(void);

int snddev_voice_set_volume(int vol, int path);

struct auddev_evt_voc_devinfo {
	u32 dev_type;           /* Rx or Tx */
	u32 acdb_dev_id;        /* acdb id of device */
	u32 dev_sample;         /* Sample rate of device */
	s32 max_rx_vol[VOC_RX_VOL_ARRAY_NUM]; 	/* unit is mb (milibel),
						[0] is for NB, other for WB */
	s32 min_rx_vol[VOC_RX_VOL_ARRAY_NUM];	/* unit is mb */
	u32 dev_id;             /* registered device id */
	u32 vol_idx;
};

struct auddev_evt_audcal_info {
	u32 dev_id;
	u32 acdb_id;
	u32 sample_rate;
	u32 dev_type;
	u32 sessions;
};

union msm_vol_mute {
	int vol;
	bool mute;
};

struct auddev_evt_voc_mute_info {
	u32 dev_type;
	u32 acdb_dev_id;
	union msm_vol_mute dev_vm_val;
};

struct auddev_evt_freq_info {
	u32 dev_type;
	u32 acdb_dev_id;
	u32 sample_rate;
};

union auddev_evt_data {
	struct auddev_evt_voc_devinfo voc_devinfo;
	struct auddev_evt_voc_mute_info voc_vm_info;
	struct auddev_evt_freq_info freq_info;
	u32 routing_id;
	s32 session_vol;
	s32 voice_state;
	struct auddev_evt_audcal_info audcal_info;
};

struct message_header {
	uint32_t id;
	uint32_t data_len;
};

#define AUDDEV_EVT_DEV_CHG_VOICE	0x01 	/* device change event */
#define AUDDEV_EVT_DEV_RDY 		0x02 	/* device ready event */
#define AUDDEV_EVT_DEV_RLS 		0x04 	/* device released event */
#define AUDDEV_EVT_REL_PENDING		0x08 	/* device release pending */
#define AUDDEV_EVT_DEVICE_VOL_MUTE_CHG	0x10 	/* device volume changed */
#define AUDDEV_EVT_START_VOICE		0x20	/* voice call start */
#define AUDDEV_EVT_END_VOICE		0x40	/* voice call end */
#define AUDDEV_EVT_STREAM_VOL_CHG	0x80 	/* device volume changed */
#define AUDDEV_EVT_FREQ_CHG		0x100	/* Change in freq */
#define AUDDEV_EVT_VOICE_STATE_CHG	0x200   /* Change in voice state */
#define AUDDEV_EVT_DEVICE_INFO		0x400	/* routed device information */

#define AUDDEV_CLNT_VOC 		0x1	/* Vocoder clients */
#define AUDDEV_CLNT_DEC 		0x2	/* Decoder clients */
#define AUDDEV_CLNT_ENC 		0x3	/* Encoder clients */
#define AUDDEV_CLNT_AUDIOCAL 		0x4	/* AudioCalibration client */

#define AUDIO_DEV_CTL_MAX_LISTNER	20	/* Max Listeners Supported */

struct msm_snd_evt_listner {
	uint32_t evt_id;
	uint32_t clnt_type;
	uint32_t clnt_id;
	void *private_data;
	void (*auddev_evt_listener)(u32 evt_id,
		union auddev_evt_data *evt_payload,
		void *private_data);
	struct msm_snd_evt_listner *cb_next;
	struct msm_snd_evt_listner *cb_prev;
};

struct event_listner {
	struct msm_snd_evt_listner *cb;
	u32 num_listner;
	int state; /* Call state */ /* TODO remove this if not req*/
};

extern struct event_listner event;
int auddev_register_evt_listner(u32 evt_id, u32 clnt_type, u32 clnt_id,
		void (*listner)(u32 evt_id,
			union auddev_evt_data *evt_payload,
			void *private_data),
		void *private_data);
int auddev_unregister_evt_listner(u32 clnt_type, u32 clnt_id);
void mixer_post_event(u32 evt_id, u32 dev_id);
void broadcast_event(u32 evt_id, u32 dev_id, u32 session_id);
int msm_snddev_request_freq(int *freq, u32 session_id,
			u32 capability, u32 clnt_type);
int msm_snddev_withdraw_freq(u32 session_id,
			u32 capability, u32 clnt_type);
int msm_device_is_voice(int dev_id);
int msm_get_voc_freq(int *tx_freq, int *rx_freq);
int msm_snddev_get_enc_freq(int session_id);
int msm_set_voice_vol(int dir, s32 volume);
int msm_set_voice_mute(int dir, int mute);
int msm_get_call_state(void);
int msm_get_voice_state(void);
#endif
