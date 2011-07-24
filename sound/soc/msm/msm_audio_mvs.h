/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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


#ifndef __MSM_AUDIO_MVS_H
#define __MSM_AUDIO_MVS_H
#include <linux/msm_audio.h>
#include <linux/wakelock.h>
#include <mach/msm_rpcrouter.h>
#include <mach/debug_mm.h>
#include <linux/slab.h>


#define AUDIO_GET_MVS_CONFIG _IOW(AUDIO_IOCTL_MAGIC, \
    (AUDIO_MAX_COMMON_IOCTL_NUM + 0), unsigned)
#define AUDIO_SET_MVS_CONFIG _IOR(AUDIO_IOCTL_MAGIC, \
 (AUDIO_MAX_COMMON_IOCTL_NUM + 1), unsigned)
#define AUDIO_SET_SCR_CONFIG _IOR(AUDIO_IOCTL_MAGIC, \
 (AUDIO_MAX_COMMON_IOCTL_NUM + 2), unsigned)
#define AUDIO_SET_DTX_CONFIG _IOR(AUDIO_IOCTL_MAGIC, \
 (AUDIO_MAX_COMMON_IOCTL_NUM + 3), unsigned)
/* MVS modes */
#define MVS_MODE_LINEAR_PCM 9

#define MVS_PROG 0x30000014
#define MVS_VERS 0x00030001

#define MVS_CLIENT_ID_VOIP 0x00000003	/* MVS_CLIENT_VOIP */

#define MVS_ACQUIRE_PROC 4
#define MVS_ENABLE_PROC 5
#define MVS_RELEASE_PROC 6
#define MVS_SET_PCM_MODE_PROC 9

#define MVS_EVENT_CB_TYPE_PROC 1
#define MVS_PACKET_UL_FN_TYPE_PROC 2
#define MVS_PACKET_DL_FN_TYPE_PROC 3

#define MVS_CB_FUNC_ID 0xAAAABBBB
#define MVS_UL_CB_FUNC_ID 0xBBBBCCCC
#define MVS_DL_CB_FUNC_ID 0xCCCCDDDD

/* MVS frame modes */

#define MVS_FRAME_MODE_PCM_UL 13
#define MVS_FRAME_MODE_PCM_DL 14

/* MVS context */
#define MVS_PKT_CONTEXT_ISR 0x00000001

/* Max voc packet size */
#define MVS_MAX_VOC_PKT_SIZE 320

#define MVS_MAX_Q_LEN  8
#define RPC_TYPE_REQUEST 0
#define RPC_TYPE_REPLY 1

#define RPC_STATUS_FAILURE 0
#define RPC_STATUS_SUCCESS 1
#define RPC_STATUS_REJECT 1


#define RPC_COMMON_HDR_SZ       (sizeof(uint32_t) * 2)
#define RPC_REQUEST_HDR_SZ      (sizeof(struct rpc_request_hdr))
#define RPC_REPLY_HDR_SZ        (sizeof(uint32_t) * 3)


enum audio_mvs_state_type { AUDIO_MVS_CLOSED, AUDIO_MVS_OPENED,
	AUDIO_MVS_PREPARING, AUDIO_MVS_ACQUIRE, AUDIO_MVS_ENABLED,
	AUDIO_MVS_CLOSING
};

enum audio_mvs_event_type { AUDIO_MVS_COMMAND, AUDIO_MVS_MODE,
	AUDIO_MVS_NOTIFY
};

enum audio_mvs_cmd_status_type { AUDIO_MVS_CMD_FAILURE, AUDIO_MVS_CMD_BUSY,
	AUDIO_MVS_CMD_SUCCESS
};

enum audio_mvs_mode_status_type { AUDIO_MVS_MODE_NOT_AVAIL,
	AUDIO_MVS_MODE_INIT, AUDIO_MVS_MODE_READY
};

enum audio_mvs_pkt_status_type { AUDIO_MVS_PKT_NORMAL, AUDIO_MVS_PKT_FAST,
	AUDIO_MVS_PKT_SLOW
};

struct rpc_audio_mvs_acquire_args {
	uint32_t client_id;
	uint32_t cb_func_id;
};

struct audio_mvs_acquire_msg {
	struct rpc_request_hdr rpc_hdr;
	struct rpc_audio_mvs_acquire_args acquire_args;
};

struct rpc_audio_mvs_enable_args {
	uint32_t client_id;
	uint32_t mode;
	uint32_t ul_cb_func_id;
	uint32_t dl_cb_func_id;
	uint32_t context;
};

struct audio_mvs_enable_msg {
	struct rpc_request_hdr rpc_hdr;
	struct rpc_audio_mvs_enable_args enable_args;
};

struct audio_mvs_release_msg {
	struct rpc_request_hdr rpc_hdr;
	uint32_t client_id;
};

struct audio_mvs_set_pcm_mode_msg {
	struct rpc_request_hdr rpc_hdr;
	uint32_t pcm_mode;
};

struct audio_mvs_set_pcmwb_mode_msg {
	struct rpc_request_hdr rpc_hdr;
	uint32_t pcmwb_mode;
};

struct audio_mvs_buffer {
	uint8_t *voc_pkt;
	uint32_t len;
};

union audio_mvs_event_data {
	struct mvs_ev_command_type {
		uint32_t event;
		uint32_t client_id;
		uint32_t cmd_status;
	} mvs_ev_command_type;

	struct mvs_ev_mode_type {
		uint32_t event;
		uint32_t client_id;
		uint32_t mode_status;
		uint32_t mode;
	} mvs_ev_mode_type;

	struct mvs_ev_notify_type {
		uint32_t event;
		uint32_t client_id;
		uint32_t buf_dir;
		uint32_t max_frames;
	} mvs_ev_notify_type;
};

struct audio_mvs_cb_func_args {
	uint32_t cb_func_id;
	uint32_t valid_ptr;
	uint32_t event;
	union audio_mvs_event_data event_data;
};

struct audio_mvs_frame_info_hdr {
	uint32_t frame_mode;
	uint32_t mvs_mode;
	uint32_t buf_free_cnt;
};

struct audio_mvs_ul_cb_func_args {
	uint32_t cb_func_id;
	uint32_t pkt_len;
	uint32_t voc_pkt[MVS_MAX_VOC_PKT_SIZE / 4];

	uint32_t valid_ptr;

	uint32_t frame_mode;
	uint32_t frame_mode_ignore;

	struct audio_mvs_frame_info_hdr frame_info_hdr;

	uint32_t pcm_frame;
	uint32_t pcm_mode;

	uint32_t pkt_len_ignore;
};

struct audio_mvs_ul_reply {
	struct rpc_reply_hdr reply_hdr;
	uint32_t valid_pkt_status_ptr;
	uint32_t pkt_status;
};

struct audio_mvs_dl_cb_func_args {
	uint32_t cb_func_id;
	uint32_t valid_ptr;

	uint32_t frame_mode;
	uint32_t frame_mode_ignore;

	struct audio_mvs_frame_info_hdr frame_info_hdr;

	uint32_t pcm_frame;
	uint32_t pcm_mode;

};

struct audio_mvs_dl_reply {
	struct rpc_reply_hdr reply_hdr;
	uint32_t voc_pkt[MVS_MAX_VOC_PKT_SIZE / 4];
	uint32_t valid_frame_info_ptr;

	uint32_t frame_mode;
	uint32_t frame_mode_again;

	struct audio_mvs_frame_info_hdr frame_info_hdr;

	uint32_t pcm_frame;
	uint32_t pcm_mode;

	uint32_t valid_pkt_status_ptr;
	uint32_t pkt_status;
};

struct audio_mvs_info_type {
	enum audio_mvs_state_type state;
	uint32_t frame_mode;
	uint32_t mvs_mode;
	uint32_t buf_free_cnt;
	uint32_t pcm_frame;
	uint32_t pcm_mode;
	uint32_t out_sample_rate;
	uint32_t out_channel_mode;
	uint32_t out_weight;
	uint32_t out_buffer_size;
	int dl_play;
	struct msm_rpc_endpoint *rpc_endpt;
	uint32_t rpc_prog;
	uint32_t rpc_ver;
	uint32_t rpc_status;

	unsigned int pcm_size;
	unsigned int pcm_count;
	unsigned int pcm_playback_irq_pos;	/* IRQ position */
	unsigned int pcm_playback_buf_pos;	/* position in buffer */

	unsigned int pcm_capture_size;
	unsigned int pcm_capture_count;
	unsigned int pcm_capture_irq_pos;	/* IRQ position */
	unsigned int pcm_capture_buf_pos;	/* position in buffer */

	uint32_t samp_rate;
	uint32_t channel_mode;

	uint8_t *mem_chunk;
	struct snd_pcm_substream *playback_substream;
	struct snd_pcm_substream *capture_substream;

	struct audio_mvs_buffer in[MVS_MAX_Q_LEN];
	uint32_t in_read;
	uint32_t in_write;

	struct audio_mvs_buffer out[MVS_MAX_Q_LEN];
	uint32_t out_read;
	uint32_t out_write;

	struct task_struct *task;

	wait_queue_head_t wait;
	wait_queue_head_t prepare_wait;
	wait_queue_head_t out_wait;
	wait_queue_head_t in_wait;


	struct mutex lock;
	struct mutex prepare_lock;
	struct mutex in_lock;
	struct mutex out_lock;

	struct wake_lock suspend_lock;
	struct wake_lock idle_lock;
	struct timer_list timer;
	unsigned long expiry;
	int ack_dl_count;
	int ack_ul_count;
	int prepare_ack;
	int playback_start;
	int capture_start;
	unsigned long expiry_delta;
	int mvs_enable;
	int playback_enable;
	int capture_enable;
	int instance;

};

enum msm_audio_pcm_frame_type {
	MVS_AMR_SPEECH_GOOD,	/* Good speech frame              */
	MVS_AMR_SPEECH_DEGRADED,	/* Speech degraded                */
	MVS_AMR_ONSET,		/* onset                          */
	MVS_AMR_SPEECH_BAD,	/* Corrupt speech frame (bad CRC) */
	MVS_AMR_SID_FIRST,	/* First silence descriptor       */
	MVS_AMR_SID_UPDATE,	/* Comfort noise frame            */
	MVS_AMR_SID_BAD,	/* Corrupt SID frame (bad CRC)    */
	MVS_AMR_NO_DATA,	/* Nothing to transmit            */
	MVS_AMR_SPEECH_LOST,	/* downlink speech lost           */
};

enum msm_audio_dtx_mode_type { MVS_DTX_OFF, MVS_DTX_ON
};

struct msm_audio_mvs_config {
	uint32_t mvs_mode;
	uint32_t bit_rate;
};
extern struct snd_soc_dai msm_mvs_dais[2];
extern struct snd_soc_codec_device soc_codec_dev_msm_mvs;
extern struct snd_soc_platform msm_mvs_soc_platform;
#endif /* __MSM_AUDIO_MVS_H */
