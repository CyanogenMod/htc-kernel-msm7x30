/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __MACH_QDSP5_V2_SNDDEV_ICODEC_H
#define __MACH_QDSP5_V2_SNDDEV_ICODEC_H
#include <mach/qdsp5v2/adie_marimba.h>
#include <mach/qdsp5v2/audio_def.h>
#include <../pmic.h>

/* Context for each internal codec sound device */
struct snddev_icodec_state {
    struct snddev_icodec_data *data;
    struct adie_codec_path *adie_path;
    u32 sample_rate;
    u32 enabled;
};

struct snddev_icodec_data {
	u32 capability; /* RX or TX */
	const char *name;
	u32 copp_id; /* audpp routing */
	u32 acdb_id; /* Audio Cal purpose */
	/* Adie profile */
	struct adie_codec_dev_profile *profile;
	/* Afe setting */
	u8 channel_mode;
	enum hsed_controller *pmctl_id; /* tx only enable mic bias */
	u32 pmctl_id_sz;
	u32 default_sample_rate;
	void (*pamp_on) (int on);
	u32 dev_vol_type;
	u32 vol_idx;
};

struct q5v2audio_analog_ops {
	void (*speaker_enable)(int en);
	void (*headset_enable)(int en);
	void (*handset_enable)(int en);
	void (*bt_sco_enable)(int en);
	void (*headset_speaker_enable)(int en);
	void (*int_mic_enable)(int en);
	void (*ext_mic_enable)(int en);
	void (*usb_headset_enable)(int en);
	void (*fm_headset_enable)(int en);
	void (*fm_speaker_enable)(int en);
};

void htc_7x30_register_analog_ops(struct q5v2audio_analog_ops *ops);

struct q5v2audio_icodec_ops {
	int (*support_aic3254) (void);
};

void htc_7x30_register_icodec_ops(struct q5v2audio_icodec_ops *ops);
#endif
