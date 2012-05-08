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
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/mfd/msm-adie-codec.h>
#include <mach/qdsp5v2_2x/snddev_icodec.h>
#include <mach/qdsp5v2_2x/aux_pcm.h>
#include <mach/qdsp5v2_2x/snddev_ecodec.h>
#include <mach/qdsp5v2_2x/audio_dev_ctl.h>
#include <mach/qdsp5v2_2x/snddev_mi2s.h>
#include <mach/qdsp5v2_2x/mi2s.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/qdsp5v2_2x/audio_acdb_def.h>

#include <mach/qdsp5v2_2x/marimba_profile.h>
#include <linux/spi/spi_aic3254.h>
#ifdef CONFIG_TIMPANI_CODEC
#include "timpani_profile_8x60.h"
#endif

/* define the value for BT_SCO */
#define BT_SCO_PCM_CTL_VAL (PCM_CTL__RPCM_WIDTH__LINEAR_V |\
				PCM_CTL__TPCM_WIDTH__LINEAR_V)
#define BT_SCO_DATA_FORMAT_PADDING (DATA_FORMAT_PADDING_INFO__RPCM_FORMAT_V |\
				DATA_FORMAT_PADDING_INFO__TPCM_FORMAT_V)
#define BT_SCO_AUX_CODEC_INTF   AUX_CODEC_INTF_CTL__PCMINTF_DATA_EN_V

static struct q5v2audio_analog_ops default_audio_ops;
static struct q5v2audio_analog_ops *audio_ops = &default_audio_ops;

void speaker_enable(int en)
{
	if (audio_ops->speaker_enable)
		audio_ops->speaker_enable(en);
}

void headset_enable(int en)
{
	if (audio_ops->headset_enable)
		audio_ops->headset_enable(en);
}

void handset_enable(int en)
{
	if (audio_ops->handset_enable)
		audio_ops->handset_enable(en);
}

void headset_speaker_enable(int en)
{
	if (audio_ops->headset_speaker_enable)
		audio_ops->headset_speaker_enable(en);
}

void int_mic_enable(int en)
{
	if (audio_ops->int_mic_enable)
		audio_ops->int_mic_enable(en);
}

void ext_mic_enable(int en)
{
	if (audio_ops->ext_mic_enable)
		audio_ops->ext_mic_enable(en);
}

void usb_headset_enable(int en)
{
	if (audio_ops->usb_headset_enable)
		audio_ops->usb_headset_enable(en);
}

void fm_headset_enable(int en)
{
	if (audio_ops->fm_headset_enable)
		audio_ops->fm_headset_enable(en);
}

void fm_speaker_enable(int en)
{
	if (audio_ops->fm_speaker_enable)
		audio_ops->fm_speaker_enable(en);
}

void qtr_headset_enable(int en)
{
	if (audio_ops->qtr_headset_enable)
		audio_ops->qtr_headset_enable(en);
}

static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions[] =
#ifdef CONFIG_TIMPANI_CODEC
	EAR_PRI_MONO_8000_OSR_256;
#else
	HANDSET_RX_48000_OSR_256;
#endif

static struct adie_codec_hwsetting_entry iearpiece_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_settings),
};

static struct snddev_icodec_data snddev_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = handset_enable,
	.vol_idx = Q5V2_HW_HANDSET,
	.aic3254_id = PLAYBACK_RECEIVER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_RECEIVER,
	.default_aic3254_id = PLAYBACK_RECEIVER
};

static struct platform_device msm_iearpiece_device = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data },
};

static struct adie_codec_action_unit imic_8KHz_osr256_actions[] =
	HANDSET_TX_8000_OSR_256;

static struct adie_codec_action_unit imic_16KHz_osr256_actions[] =
	HANDSET_TX_16000_OSR_256;

static struct adie_codec_action_unit imic_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry imic_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings,
	.setting_sz = ARRAY_SIZE(imic_settings),
};

static struct snddev_icodec_data snddev_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.aic3254_id = VOICERECORD_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_RECEIVER,
	.default_aic3254_id = VOICERECORD_IMIC
};

static struct platform_device msm_imic_device = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data },
};

#ifdef CONFIG_TIMPANI_CODEC
static struct adie_codec_action_unit headset_ab_cpls_48KHz_osr256_actions[] =
	HEADSET_AB_CPLS_48000_OSR_256;

static struct adie_codec_hwsetting_entry headset_ab_cpls_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_ab_cpls_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_ab_cpls_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile headset_ab_cpls_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_ab_cpls_settings,
	.setting_sz = ARRAY_SIZE(headset_ab_cpls_settings),
};

#else

static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings),
};
#endif

static struct snddev_mi2s_data snddev_mi2s_stereo_rx_data = {
	.capability = SNDDEV_CAP_RX ,
	.name = "hdmi_stereo_rx",
	.copp_id = 3,
	.acdb_id = ACDB_ID_HDMI,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_0,
	.default_sample_rate = 48000,
};

static struct platform_device msm_snddev_mi2s_stereo_rx_device = {
	.name = "snddev_mi2s",
	.id = 25,
	.dev = { .platform_data = &snddev_mi2s_stereo_rx_data },
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
#ifdef CONFIG_TIMPANI_CODEC
	.profile = &headset_ab_cpls_profile,
#else
	.profile = &ihs_stereo_rx_profile,
#endif
	.channel_mode = 2,
	.default_sample_rate = 48000,
	/* change to raise ncp power. capless need ncp bias. */
	.pamp_on = headset_enable,
	.vol_idx = Q5V2_HW_HEADSET,
	.aic3254_id = PLAYBACK_HEADSET,
	.aic3254_voc_id = CALL_DOWNLINK_EMIC_HEADSET,
	.default_aic3254_id = PLAYBACK_HEADSET,
	.pre_pamp_on = qtr_headset_enable
};

static struct platform_device msm_ihs_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data },
};

static struct adie_codec_action_unit ihs_mono_rx_48KHz_osr256_actions[] =
	HEADSET_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MONO,
	.profile = &ihs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.vol_idx = Q5V2_HW_HEADSET

};

static struct platform_device msm_ihs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 3,
	.dev = { .platform_data = &snddev_ihs_mono_rx_data },
};

static struct adie_codec_action_unit ihs_mono_tx_8KHz_osr256_actions[] =
	HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_16KHz_osr256_actions[] =
	HEADSET_MONO_TX_16000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions[] =
	HEADSET_MONO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihs_mono_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = ext_mic_enable,
	.aic3254_id = VOICERECORD_EMIC,
	.aic3254_voc_id = CALL_UPLINK_EMIC_HEADSET,
	.default_aic3254_id = VOICERECORD_EMIC
};

static struct platform_device msm_ihs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data },
};

#if 1 /* add for FM recording */
static struct adie_codec_action_unit fm_ihs_mono_tx_48KHz_osr256_actions[] =
	FM_HEADSET_MONO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry fm_ihs_mono_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = fm_ihs_mono_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fm_ihs_mono_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile fm_ihs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = fm_ihs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(fm_ihs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_fm_ihs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "fm_headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &fm_ihs_mono_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.aic3254_id = VOICERECORD_EMIC,
	.aic3254_voc_id = CALL_UPLINK_EMIC_HEADSET,
	.default_aic3254_id = VOICERECORD_EMIC
};

static struct platform_device msm_fm_ihs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 28,
	.dev = { .platform_data = &snddev_fm_ihs_mono_tx_data },
};
#endif

static struct adie_codec_action_unit ifmradio_handset_osr64_actions[] =
	FM_HANDSET_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_handset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_handset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_handset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_handset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_handset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_handset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_handset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_handset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_handset_device = {
	.name = "snddev_icodec",
	.id = 7,
	.dev = { .platform_data = &snddev_ifmradio_handset_data },
};

#ifdef CONFIG_TIMPANI_CODEC
static struct adie_codec_action_unit ispkr_stereo_48KHz_osr256_actions[] =
	SPEAKER_PRI_STEREO_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispkr_stereo_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispkr_stereo_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispkr_stereo_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispkr_stereo_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispkr_stereo_settings,
	.setting_sz = ARRAY_SIZE(ispkr_stereo_settings),
};
#else
static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions[] =
	SPEAKER_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings),
};
#endif

static struct snddev_icodec_data snddev_ispeaker_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_rx",
	.copp_id = 0,
#ifdef CONFIG_TIMPANI_CODEC
	.acdb_id = 7,
	.profile = &ispkr_stereo_profile,
#else
	.acdb_id = ACDB_ID_SPKR_PHONE_MONO,
	.profile = &ispeaker_rx_profile,
#endif
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = speaker_enable,
	.vol_idx = Q5V2_HW_SPEAKER,
	.aic3254_id = PLAYBACK_SPEAKER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_SPEAKER,
	.default_aic3254_id = PLAYBACK_SPEAKER
};

static struct platform_device msm_ispeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data },

};

static struct adie_codec_action_unit ifmradio_speaker_osr64_actions[] =
	AUXPGA_SPEAKER_RX;

static struct adie_codec_hwsetting_entry ifmradio_speaker_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ifmradio_speaker_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_speaker_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_speaker_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_speaker_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_speaker_settings),
};

static struct snddev_icodec_data snddev_ifmradio_speaker_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_speaker_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_speaker_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = fm_speaker_enable,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
	.aic3254_id = FM_OUT_SPEAKER,
	.aic3254_voc_id = FM_OUT_SPEAKER
};

static struct platform_device msm_ifmradio_speaker_device = {
	.name = "snddev_icodec",
	.id = 9,
	.dev = { .platform_data = &snddev_ifmradio_speaker_data },
};

static struct adie_codec_action_unit ifmradio_headset_osr64_actions[] =
	AUXPGA_HEADSET_STEREO_RX_CAPLESS;

static struct adie_codec_hwsetting_entry ifmradio_headset_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ifmradio_headset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_headset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_headset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ifmradio_headset_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	/* change to raise ncp power. capless need ncp bias. */
	.pamp_on = headset_enable,
	.vol_idx = Q5V2_HW_HEADSET,
	.aic3254_id = FM_OUT_HEADSET,
	.aic3254_voc_id = FM_OUT_HEADSET,
	.pre_pamp_on = qtr_headset_enable
};

static struct platform_device msm_ifmradio_headset_device = {
	.name = "snddev_icodec",
	.id = 10,
	.dev = { .platform_data = &snddev_ifmradio_headset_data },
};


static struct snddev_ecodec_data snddev_bt_sco_earpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.vol_idx = Q5V2_HW_BT_SCO
};

static struct snddev_ecodec_data snddev_bt_sco_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_MIC,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};

struct platform_device msm_bt_sco_earpiece_device = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data },
};

struct platform_device msm_bt_sco_mic_device = {
	.name = "msm_snddev_ecodec",
	.id = 1,
	.dev = { .platform_data = &snddev_bt_sco_mic_data },
};

static struct adie_codec_action_unit itty_hs_mono_tx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_hwsetting_entry itty_hs_mono_tx_settings[] = {
	/* 8KHz, 16KHz, 48KHz TTY Tx devices can shared same set of actions */
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = itty_hs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_MIC,
	.profile = &itty_hs_mono_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.pamp_on = ext_mic_enable,
};

static struct platform_device msm_itty_hs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 16,
	.dev = { .platform_data = &snddev_itty_hs_mono_tx_data },
};

static struct adie_codec_action_unit itty_hs_mono_rx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_8000_OSR_256;

static struct adie_codec_action_unit itty_hs_mono_rx_16KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_16000_OSR_256;

static struct adie_codec_action_unit itty_hs_mono_rx_48KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_48000_OSR_256;

static struct adie_codec_hwsetting_entry itty_hs_mono_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_rx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = itty_hs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_SPKR,
	.profile = &itty_hs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = headset_enable,
	.vol_idx = Q5V2_HW_TTY,
	.pre_pamp_on = qtr_headset_enable
};

static struct platform_device msm_itty_hs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 17,
	.dev = { .platform_data = &snddev_itty_hs_mono_rx_data },
};

static struct adie_codec_action_unit ispeaker_tx_8KHz_osr256_actions[] =
	SPEAKER_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions[] =
	SPEAKER_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.aic3254_id = VOICERECORD_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_SPEAKER,
	.default_aic3254_id = VOICERECORD_IMIC
};

static struct platform_device msm_ispeaker_tx_device = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data },
};

static struct adie_codec_action_unit ihs_ispk_stereo_rx_48KHz_osr256_actions[] =
	SPEAKER_HEADSET_STEREO_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ispk_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ispk_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ispk_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_ispk_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ispk_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ispk_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ispk_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,
	.profile = &ihs_ispk_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = headset_speaker_enable,
	.vol_idx = Q5V2_HW_HS_SPKR,
	.aic3254_id = RING_HEADSET_SPEAKER,
	.aic3254_voc_id = RING_HEADSET_SPEAKER,
	.default_aic3254_id = RING_HEADSET_SPEAKER,
	.pre_pamp_on = qtr_headset_enable
};

static struct platform_device msm_iheadset_ispeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_ispk_stereo_rx_data },
};

static struct adie_codec_action_unit iusb_headset_rx_48KHz_osr256_actions[] =
	USB_HEADSET_STEREO_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry iusb_headset_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iusb_headset_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iusb_headset_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iusb_headset_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iusb_headset_rx_settings,
	.setting_sz = ARRAY_SIZE(iusb_headset_rx_settings),
};

static struct snddev_icodec_data snddev_iusb_headset_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "usb_headset_stereo_rx",
	.copp_id = 0,
	/*.acdb_id = 8,*/
	.acdb_id = ACDB_ID_SPKR_PHONE_MONO,
	.profile = &iusb_headset_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = usb_headset_enable,
	.vol_idx = Q5V2_HW_USB_HS
};

static struct platform_device msm_iusb_headset_rx_device = {
	.name = "snddev_icodec",
	.id = 22,
	.dev = { .platform_data = &snddev_iusb_headset_rx_data },
};

static struct adie_codec_action_unit ihac_rx_48KHz_osr256_actions[] =
	SPEAKER_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihac_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihac_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihac_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihac_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihac_rx_settings,
	.setting_sz = ARRAY_SIZE(ihac_rx_settings),
};

static struct snddev_icodec_data snddev_ihac_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "hac_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ihac_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = handset_enable,
	.vol_idx = Q5V2_HW_HAC
};

static struct platform_device msm_ihac_rx_device = {
	.name = "snddev_icodec",
	.id = 23,
	.dev = { .platform_data = &snddev_ihac_rx_data },
};

static struct adie_codec_action_unit ialt_rx_48KHz_osr256_actions[] =
	SPEAKER_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ialt_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ialt_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ialt_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ialt_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ialt_rx_settings,
	.setting_sz = ARRAY_SIZE(ialt_rx_settings),
};

static struct snddev_icodec_data snddev_ialt_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "alt_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ialt_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = speaker_enable,
	.vol_idx = Q5V2_HW_SPEAKER
};

static struct platform_device msm_ialt_rx_device = {
	.name = "snddev_icodec",
	.id = 24,
	.dev = { .platform_data = &snddev_ialt_rx_data },
};

static struct adie_codec_action_unit ivr_mic_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ivr_mic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ivr_mic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ivr_mic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ivr_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ivr_mic_settings,
	.setting_sz = ARRAY_SIZE(ivr_mic_settings),
};

static struct snddev_icodec_data snddev_ivr_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_vr_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &ivr_mic_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
};

static struct platform_device msm_ivr_mic_device = {
	.name = "snddev_icodec",
	.id = 25,
	.dev = { .platform_data = &snddev_ivr_mic_data },
};

static struct adie_codec_action_unit idual_mic_48KHz_osr256_actions[] =
	DUAL_MIC_STEREO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_settings),
};

static struct snddev_icodec_data snddev_idual_mic_endfire_real_stereo_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "dual_mic_stereo_tx",
	.copp_id = 0,
	.acdb_id = 6,
	.profile = &idual_mic_profile,
	.channel_mode = REAL_STEREO_CHANNEL_MODE,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.aic3254_id = VIDEORECORD_IMIC,
	.aic3254_voc_id = VOICERECORD_EMIC, /* FIX ME */
	.default_aic3254_id = VIDEORECORD_IMIC
};

static struct platform_device msm_real_stereo_tx_device = {
	.name = "snddev_icodec",
	.id = 26,
	.dev = { .platform_data = &snddev_idual_mic_endfire_real_stereo_data },
};

static struct adie_codec_action_unit ihs_vr_mic_48KHz_osr256_actions[] =
	HEADSET_MONO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_vr_mic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_vr_mic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_vr_mic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_vr_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_vr_mic_settings,
	.setting_sz = ARRAY_SIZE(ihs_vr_mic_settings),
};

static struct snddev_icodec_data snddev_ihs_vr_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_vr_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_vr_mic_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = ext_mic_enable,
};

static struct platform_device msm_ihs_vr_mic_device = {
	.name = "snddev_icodec",
	.id = 27,
	.dev = { .platform_data = &snddev_ihs_vr_mic_data },
};

static struct platform_device *snd_devices_surf[] __initdata = {
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	&msm_ispeaker_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device,
	&msm_iheadset_ispeaker_rx_device,
	&msm_iusb_headset_rx_device,
	&msm_ihac_rx_device,
	&msm_ialt_rx_device,
	&msm_ivr_mic_device,
	&msm_real_stereo_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_ihs_vr_mic_device,
	&msm_fm_ihs_mono_tx_device
};

void htc_7x30_register_analog_ops(struct q5v2audio_analog_ops *ops)
{
	audio_ops = ops;
}

void __init msm_snddev_init(void)
{
	platform_add_devices(snd_devices_surf,
	ARRAY_SIZE(snd_devices_surf));
}
