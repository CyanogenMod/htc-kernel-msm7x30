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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/marimba-codec.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>
#include <mach/qdsp5v2_1x/snddev_icodec.h>
#include <mach/qdsp5v2_1x/audio_dev_ctl.h>
#include <mach/qdsp5v2_1x/audio_interct.h>
#include <mach/qdsp5v2_1x/mi2s.h>
#include <mach/qdsp5v2_1x/afe.h>
#include <mach/qdsp5v2_1x/lpa.h>
#include <mach/vreg.h>
#include <mach/debug_audio_mm.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <mach/qdsp5v2_1x/audio_acdb_def.h>

#include <mach/qdsp5v2_1x/marimba_profile.h>
#include <asm/mach-types.h>

#define SNDDEV_ICODEC_PCM_SZ 32 /* 16 bit / sample stereo mode */
#define SNDDEV_ICODEC_MUL_FACTOR 3 /* Multi by 8 Shift by 3  */
#define SNDDEV_ICODEC_CLK_RATE(freq) \
	(((freq) * (SNDDEV_ICODEC_PCM_SZ)) << (SNDDEV_ICODEC_MUL_FACTOR))

static struct q5v2audio_icodec_ops default_audio_ops;
static struct q5v2audio_icodec_ops * audio_ops = &default_audio_ops;
static int support_aic3254 = 0;

#ifdef CONFIG_DEBUG_FS
static struct adie_codec_action_unit debug_rx_actions[] = {
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_OFF},
	{ ADIE_CODEC_ACTION_DELAY_WAIT, 0xbb8},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x02, 0x02)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x02, 0x00)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_READY },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x24, 0x6F, 0x44)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x04, 0x5F, 0xBC)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x81, 0xFF, 0x4E)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x25, 0x0F, 0x0E)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x26, 0xfc, 0xfc)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x36, 0xc0, 0x80)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x3A, 0xFF, 0x2B)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x3d, 0xFF, 0xD5)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x21, 0x21)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x80, 0x80)},
	{ ADIE_CODEC_ACTION_DELAY_WAIT,  0x2710},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x40, 0x40)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x84, 0xff, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x05, 0x04)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_ANALOG_READY},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8a, 0x01, 0x01)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x36, 0xc0, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x40, 0x00)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED,  ADIE_CODEC_ANALOG_OFF},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x80, 0x00)}
};

static struct adie_codec_action_unit debug_tx_lb_actions[] = {
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_OFF },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x01, 0x01)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x01, 0x00) },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x30, 0x30)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x11, 0xfc, 0xfc)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x13, 0xfc, 0x58)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x14, 0xff, 0x65)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x15, 0xff, 0x64)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x82, 0xff, 0x5C)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_READY },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x0D, 0xF0, 0xd0)},
	{ ADIE_CODEC_ACTION_DELAY_WAIT, 0xbb8},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x14, 0x14)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x86, 0xff, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x50, 0x40)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x91, 0xFF, 0x01)}, /* Start loop back */
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_ANALOG_READY},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x10, 0x30)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x0D, 0xFF, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x14, 0x00)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_ANALOG_OFF},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x11, 0xff, 0x00)}
};

static struct adie_codec_action_unit debug_tx_actions[] = {
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_OFF },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x01, 0x01)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x01, 0x00) },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x30, 0x30)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x11, 0xfc, 0xfc)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x13, 0xfc, 0x58)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x14, 0xff, 0x65)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x15, 0xff, 0x64)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x82, 0xff, 0x5C)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_READY },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x0D, 0xF0, 0xd0)},
	{ ADIE_CODEC_ACTION_DELAY_WAIT, 0xbb8},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x14, 0x14)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x86, 0xff, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x50, 0x40)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_ANALOG_READY},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x10, 0x30)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x0D, 0xFF, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x14, 0x00)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_ANALOG_OFF},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x11, 0xff, 0x00)}
};

static struct adie_codec_hwsetting_entry debug_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = debug_rx_actions,
		.action_sz = ARRAY_SIZE(debug_rx_actions),
	}
};

static struct adie_codec_hwsetting_entry debug_tx_lb_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = debug_tx_lb_actions,
		.action_sz = ARRAY_SIZE(debug_tx_lb_actions),
	}
};

static struct adie_codec_hwsetting_entry debug_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = debug_tx_actions,
		.action_sz = ARRAY_SIZE(debug_tx_actions),
	}
};

static struct adie_codec_dev_profile debug_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = debug_rx_settings,
	.setting_sz = ARRAY_SIZE(debug_rx_settings),
};

static struct adie_codec_dev_profile debug_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = debug_tx_settings,
	.setting_sz = ARRAY_SIZE(debug_tx_settings),
};

static struct adie_codec_dev_profile debug_tx_lb_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = debug_tx_lb_settings,
	.setting_sz = ARRAY_SIZE(debug_tx_lb_settings),
};
#endif /* CONFIG_DEBUG_FS */

/* Global state for the driver */
struct snddev_icodec_drv_state {
	struct mutex rx_lock;
	struct mutex tx_lock;
	u32 rx_active; /* ensure one rx device at a time */
	u32 tx_active; /* ensure one tx device at a time */
	struct clk *rx_mclk;
	struct clk *rx_sclk;
	struct clk *tx_mclk;
	struct clk *tx_sclk;
	struct clk *lpa_codec_clk;
	struct clk *lpa_core_clk;
	struct clk *lpa_p_clk;
	struct lpa_drv *lpa;

	struct wake_lock rx_idlelock;
	struct wake_lock tx_idlelock;
};

static struct snddev_icodec_drv_state snddev_icodec_drv;

static int snddev_icodec_open_rx(struct snddev_icodec_state *icodec)
{
	int trc;
	struct msm_afe_config afe_config;
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;
	struct lpa_codec_config lpa_config;

	wake_lock(&drv->rx_idlelock);

	/* enable MI2S RX master block */
	/* enable MI2S RX bit clock */
	trc = clk_set_rate(drv->rx_mclk,
			SNDDEV_ICODEC_CLK_RATE(icodec->sample_rate));
	if (IS_ERR_VALUE(trc))
		goto error_invalid_freq;

	clk_enable(drv->rx_mclk);
	clk_enable(drv->rx_sclk);
	/* clk_set_rate(drv->lpa_codec_clk, 1); */ /* Remove if use pcom */
	clk_enable(drv->lpa_p_clk);
	clk_enable(drv->lpa_codec_clk);
	clk_enable(drv->lpa_core_clk);

	/* Enable LPA sub system
	 */
	drv->lpa = lpa_get();
	if (!drv->lpa)
		goto error_lpa;
	lpa_config.sample_rate = icodec->sample_rate;
	lpa_config.sample_width = 16;
	lpa_config.output_interface = LPA_OUTPUT_INTF_WB_CODEC;
	lpa_config.num_channels = icodec->data->channel_mode;
	lpa_cmd_codec_config(drv->lpa, &lpa_config);

	/* Set audio interconnect reg to LPA */
	audio_interct_codec(AUDIO_INTERCT_LPA);

	/* Set MI2S */
	mi2s_set_codec_output_path((icodec->data->channel_mode == 2 ?
	MI2S_CHAN_STEREO : MI2S_CHAN_MONO_PACKED), WT_16_BIT);

	if (!support_aic3254 ||
		 !strcmp(icodec->data->name, "usb_headset_stereo_rx")) {
		/* Configure ADIE */
		trc = adie_codec_open(icodec->data->profile, &icodec->adie_path);
		if (IS_ERR_VALUE(trc))
			goto error_adie;
		/* OSR default to 256, can be changed for power optimization
		 * If OSR is to be changed, need clock API for setting the divider
		 */
		adie_codec_setpath(icodec->adie_path, icodec->sample_rate, 256);
	}

	/* Start AFE */
	afe_config.sample_rate = icodec->sample_rate / 1000;
	afe_config.channel_mode = icodec->data->channel_mode;
	afe_config.volume = AFE_VOLUME_UNITY;
	trc = afe_enable(AFE_HW_PATH_CODEC_RX, &afe_config);
	if (IS_ERR_VALUE(trc))
		goto error_afe;
	lpa_cmd_enable_codec(drv->lpa, 1);
	if (!support_aic3254 ||
		 !strcmp(icodec->data->name, "usb_headset_stereo_rx")) {
		/* Enable ADIE */
		if (adie_codec_proceed_stage(icodec->adie_path,
					ADIE_CODEC_DIGITAL_READY) ) {
			icodec->adie_path->profile = NULL;
			goto error_adie;
		}

		if (adie_codec_proceed_stage(icodec->adie_path,
					ADIE_CODEC_DIGITAL_ANALOG_READY) ) {
			icodec->adie_path->profile = NULL;
			goto error_adie;
		}
	}

	/* Enable power amplifier */
	if (icodec->data->pamp_on)
		icodec->data->pamp_on(1);

	icodec->enabled = 1;

	wake_unlock(&drv->rx_idlelock);
	return 0;

error_afe:
	if (!support_aic3254 ||
		 !strcmp(icodec->data->name, "usb_headset_stereo_rx")) {
		adie_codec_close(icodec->adie_path);
		icodec->adie_path = NULL;
	}
error_adie:
	lpa_put(drv->lpa);
error_lpa:
	clk_disable(drv->lpa_p_clk);
	clk_disable(drv->lpa_codec_clk);
	clk_disable(drv->lpa_core_clk);
	clk_disable(drv->rx_sclk);
	clk_disable(drv->rx_mclk);
error_invalid_freq:

	pr_aud_err("%s: encounter error\n", __func__);

	wake_unlock(&drv->rx_idlelock);
	return -ENODEV;
}

static int snddev_icodec_open_tx(struct snddev_icodec_state *icodec)
{
	int trc;
	struct msm_afe_config afe_config;
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;

	wake_lock(&drv->tx_idlelock);

	/* Reuse pamp_on for TX platform-specific setup  */
	if (icodec->data->pamp_on)
		icodec->data->pamp_on(1);

	/* enable MI2S TX master block */
	/* enable MI2S TX bit clock */
	trc = clk_set_rate(drv->tx_mclk,
		SNDDEV_ICODEC_CLK_RATE(icodec->sample_rate));
	if (IS_ERR_VALUE(trc))
		goto error_invalid_freq;
	clk_enable(drv->tx_mclk);
	clk_enable(drv->tx_sclk);

	/* Set MI2S */
	mi2s_set_codec_input_path((icodec->data->channel_mode == 2 ?
	MI2S_CHAN_STEREO : MI2S_CHAN_MONO_RAW), WT_16_BIT);
	/* Configure ADIE */
	trc = adie_codec_open(icodec->data->profile, &icodec->adie_path);
	if (IS_ERR_VALUE(trc))
		goto error_adie;
	/* Enable ADIE */
	adie_codec_setpath(icodec->adie_path, icodec->sample_rate, 256);
	if (adie_codec_proceed_stage(icodec->adie_path,
			ADIE_CODEC_DIGITAL_READY)) {
		icodec->adie_path->profile = NULL;
		goto error_adie;
	}
	if (adie_codec_proceed_stage(icodec->adie_path,
			ADIE_CODEC_DIGITAL_ANALOG_READY) ) {
		icodec->adie_path->profile = NULL;
		goto error_adie;
	}

	/* Start AFE */
	afe_config.sample_rate = icodec->sample_rate / 1000;
	afe_config.channel_mode = icodec->data->channel_mode;
	afe_config.volume = AFE_VOLUME_UNITY;
	trc = afe_enable(AFE_HW_PATH_CODEC_TX, &afe_config);
	if (IS_ERR_VALUE(trc))
		goto error_afe;


	icodec->enabled = 1;

	wake_unlock(&drv->tx_idlelock);
	return 0;

error_afe:
	adie_codec_close(icodec->adie_path);
	icodec->adie_path = NULL;
error_adie:
	clk_disable(drv->tx_sclk);
	clk_disable(drv->tx_mclk);
error_invalid_freq:

	if (icodec->data->pamp_on)
		icodec->data->pamp_on(0);

	pr_aud_err("%s: encounter error\n", __func__);

	wake_unlock(&drv->tx_idlelock);
	return -ENODEV;
}

static int snddev_icodec_close_rx(struct snddev_icodec_state *icodec)
{
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;

	wake_lock(&drv->rx_idlelock);

	/* Disable power amplifier */
	if (icodec->data->pamp_on)
		icodec->data->pamp_on(0);

	if (!support_aic3254 ||
		 !strcmp(icodec->data->name, "usb_headset_stereo_rx")) {
		/* Disable ADIE */
		adie_codec_proceed_stage(icodec->adie_path,
						ADIE_CODEC_DIGITAL_OFF);
		adie_codec_close(icodec->adie_path);
		icodec->adie_path = NULL;
	}

	afe_disable(AFE_HW_PATH_CODEC_RX);

	/* Disable LPA Sub system */
	lpa_cmd_enable_codec(drv->lpa, 0);
	lpa_put(drv->lpa);

	/* Disable LPA clocks */
	clk_disable(drv->lpa_p_clk);
	clk_disable(drv->lpa_codec_clk);
	clk_disable(drv->lpa_core_clk);

	/* Disable MI2S RX master block */
	/* Disable MI2S RX bit clock */
	clk_disable(drv->rx_sclk);
	clk_disable(drv->rx_mclk);
	icodec->enabled = 0;

	wake_unlock(&drv->rx_idlelock);
	return 0;
}

static int snddev_icodec_close_tx(struct snddev_icodec_state *icodec)
{
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;

	wake_lock(&drv->tx_idlelock);

	afe_disable(AFE_HW_PATH_CODEC_TX);

	/* Disable ADIE */
	adie_codec_proceed_stage(icodec->adie_path, ADIE_CODEC_DIGITAL_OFF);
	adie_codec_close(icodec->adie_path);
	icodec->adie_path = NULL;

	/* Disable MI2S TX master block */
	/* Disable MI2S TX bit clock */
	clk_disable(drv->tx_sclk);
	clk_disable(drv->tx_mclk);

	/* Reuse pamp_off for TX platform-specific setup  */
	if (icodec->data->pamp_on)
		icodec->data->pamp_on(0);

	icodec->enabled = 0;

	wake_unlock(&drv->tx_idlelock);
	return 0;
}

static int snddev_icodec_set_device_volume_impl(
		struct msm_snddev_info *dev_info, u32 volume)
{
	struct snddev_icodec_state *icodec;
	u8 afe_path_id;

	int rc = 0;

	icodec = dev_info->private_data;

	if (icodec->data->capability & SNDDEV_CAP_RX) {
		if (support_aic3254)
			return rc;
		afe_path_id = AFE_HW_PATH_CODEC_RX;
	} else
		afe_path_id = AFE_HW_PATH_CODEC_TX;

	if (icodec->data->dev_vol_type & SNDDEV_DEV_VOL_DIGITAL) {

		rc = adie_codec_set_device_digital_volume(icodec->adie_path,
				icodec->data->channel_mode, volume);
		if (rc < 0) {
			pr_aud_err("%s: unable to set_device_digital_volume for"
				"%s volume in percentage = %u\n",
				__func__, dev_info->name, volume);
			return rc;
		}

	} else if (icodec->data->dev_vol_type & SNDDEV_DEV_VOL_ANALOG) {
		rc = adie_codec_set_device_analog_volume(icodec->adie_path,
				icodec->data->channel_mode, volume);
		if (rc < 0) {
			pr_aud_err("%s: unable to set_device_analog_volume for"
				"%s volume in percentage = %u\n",
				__func__, dev_info->name, volume);
			return rc;
		}
	} else {
		pr_aud_err("%s: Invalid device volume control\n", __func__);
		return -EPERM;
	}
	return rc;
}

static int snddev_icodec_open(struct msm_snddev_info *dev_info)
{
	int rc = 0;
	struct snddev_icodec_state *icodec;
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;

	if (!dev_info) {
		rc = -EINVAL;
		goto error;
	}

	icodec = dev_info->private_data;
	pr_aud_info("snddev_icodec_open: device %s\n", dev_info->name);

	if (icodec->data->capability & SNDDEV_CAP_RX) {
		mutex_lock(&drv->rx_lock);
		if (drv->rx_active) {
			mutex_unlock(&drv->rx_lock);
			rc = -EBUSY;
			goto error;
		}
		rc = snddev_icodec_open_rx(icodec);

		if (!IS_ERR_VALUE(rc)) {
			drv->rx_active = 1;
			if ((icodec->data->dev_vol_type & (
				SNDDEV_DEV_VOL_DIGITAL |
				SNDDEV_DEV_VOL_ANALOG)))
				snddev_icodec_set_device_volume_impl(
						dev_info, dev_info->dev_volume);
		} else {
			pr_aud_info("snddev_icodec_open failed. %s\n", dev_info->name);
			mutex_unlock(&drv->rx_lock);
			return rc;
		}
		mutex_unlock(&drv->rx_lock);
	} else {
		mutex_lock(&drv->tx_lock);
		if (drv->tx_active) {
			mutex_unlock(&drv->tx_lock);
			rc = -EBUSY;
			goto error;
		}
		rc = snddev_icodec_open_tx(icodec);

		if (!IS_ERR_VALUE(rc)) {
			drv->tx_active = 1;
			if ((icodec->data->dev_vol_type & (
				SNDDEV_DEV_VOL_DIGITAL |
				SNDDEV_DEV_VOL_ANALOG)))
				snddev_icodec_set_device_volume_impl(
						dev_info, dev_info->dev_volume);
		} else {
			pr_aud_info("snddev_icodec_open failed. %s\n", dev_info->name);
			mutex_unlock(&drv->tx_lock);
			return rc;
		}
		mutex_unlock(&drv->tx_lock);
	}
error:
	return rc;
}

static int snddev_icodec_close(struct msm_snddev_info *dev_info)
{
	int rc = 0;
	struct snddev_icodec_state *icodec;
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;
	if (!dev_info) {
		rc = -EINVAL;
		goto error;
	}

	icodec = dev_info->private_data;
	pr_aud_info("snddev_icodec_close: device %s\n", dev_info->name);

	if (icodec->data->capability & SNDDEV_CAP_RX) {
		mutex_lock(&drv->rx_lock);
		if (!drv->rx_active) {
			mutex_unlock(&drv->rx_lock);
			rc = -EPERM;
			goto error;
		}
		rc = snddev_icodec_close_rx(icodec);
		if (!IS_ERR_VALUE(rc))
			drv->rx_active = 0;
		mutex_unlock(&drv->rx_lock);
	} else {
		mutex_lock(&drv->tx_lock);
		if (!drv->tx_active) {
			mutex_unlock(&drv->tx_lock);
			rc = -EPERM;
			goto error;
		}
		rc = snddev_icodec_close_tx(icodec);
		if (!IS_ERR_VALUE(rc))
			drv->tx_active = 0;
		mutex_unlock(&drv->tx_lock);
	}

error:
	return rc;
}

static int snddev_icodec_check_freq(u32 req_freq)
{
	int rc = -EINVAL;

	if ((req_freq != 0) && (req_freq >= 8000) && (req_freq <= 48000)) {
		if ((req_freq == 8000) || (req_freq == 11025) ||
			(req_freq == 12000) || (req_freq == 16000) ||
			(req_freq == 22050) || (req_freq == 24000) ||
			(req_freq == 32000) || (req_freq == 44100) ||
			(req_freq == 48000)) {
				rc = 0;
		} else
			pr_aud_info("%s: Unsupported Frequency:%d\n", __func__,
								req_freq);
		}
		return rc;
}

static int snddev_icodec_set_freq(struct msm_snddev_info *dev_info, u32 rate)
{
	int rc;
	struct snddev_icodec_state *icodec;

	if (!dev_info) {
		rc = -EINVAL;
		goto error;
	}

	icodec = dev_info->private_data;
	if (adie_codec_freq_supported(icodec->data->profile, rate) != 0) {
		rc = -EINVAL;
		goto error;
	} else {
		if (snddev_icodec_check_freq(rate) != 0) {
			rc = -EINVAL;
			goto error;
		} else
			icodec->sample_rate = rate;
	}

	if (icodec->enabled) {
		snddev_icodec_close(dev_info);
		snddev_icodec_open(dev_info);
	}

	return icodec->sample_rate;

error:
	return rc;
}

static int snddev_icodec_enable_sidetone(struct msm_snddev_info *dev_info,
	u32 enable)
{
	int rc = 0;
	struct snddev_icodec_state *icodec;
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;

	if (!dev_info) {
		MM_AUD_ERR("invalid dev_info\n");
		rc = -EINVAL;
		goto error;
	}

	icodec = dev_info->private_data;

	if (icodec->data->capability & SNDDEV_CAP_RX) {
		mutex_lock(&drv->rx_lock);
		if (!drv->rx_active || !dev_info->opened) {
			MM_AUD_ERR("dev not active\n");
			rc = -EPERM;
			mutex_unlock(&drv->rx_lock);
			goto error;
		}
		rc = adie_codec_enable_sidetone(icodec->adie_path, enable);
		mutex_unlock(&drv->rx_lock);
	} else {
		rc = -EINVAL;
		MM_AUD_ERR("rx device only\n");
	}

error:
	return rc;

}

int snddev_icodec_set_device_volume(struct msm_snddev_info *dev_info,
		u32 volume)
{
	struct snddev_icodec_state *icodec;
	struct mutex *lock;
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;
	int rc = -EPERM;

	if (!dev_info) {
		pr_aud_info("%s : device not intilized.\n", __func__);
		return  -EINVAL;
	}

	icodec = dev_info->private_data;

	if (!(icodec->data->dev_vol_type & (SNDDEV_DEV_VOL_DIGITAL
				| SNDDEV_DEV_VOL_ANALOG))) {

		pr_aud_info("%s : device %s does not support device volume "
				"control.", __func__, dev_info->name);
		return -EPERM;
	}
	dev_info->dev_volume =  volume;

	if (icodec->data->capability & SNDDEV_CAP_RX)
		lock = &drv->rx_lock;
	else
		lock = &drv->tx_lock;

	mutex_lock(lock);

	rc = snddev_icodec_set_device_volume_impl(dev_info,
			dev_info->dev_volume);
	mutex_unlock(lock);
	return rc;
}

void htc_7x30_register_icodec_ops(struct q5v2audio_icodec_ops *ops)
{
	audio_ops = ops;
}

static int snddev_icodec_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct snddev_icodec_data *pdata;
	struct msm_snddev_info *dev_info;
	struct snddev_icodec_state *icodec;
	static int first_time = 1;

	if (!pdev || !pdev->dev.platform_data) {
		printk(KERN_ALERT "Invalid caller \n");
		rc = -1;
		goto error;
	}
	pdata = pdev->dev.platform_data;
	if ((pdata->capability & SNDDEV_CAP_RX) &&
	   (pdata->capability & SNDDEV_CAP_TX)) {
		pr_aud_err("%s: invalid device data either RX or TX\n", __func__);
		goto error;
	}
	icodec = kzalloc(sizeof(struct snddev_icodec_state), GFP_KERNEL);
	if (!icodec) {
		rc = -ENOMEM;
		goto error;
	}
	dev_info = kmalloc(sizeof(struct msm_snddev_info), GFP_KERNEL);
	if (!dev_info) {
		kfree(icodec);
		rc = -ENOMEM;
		goto error;
	}

	dev_info->name = pdata->name;
	dev_info->copp_id = pdata->copp_id;
	dev_info->acdb_id = pdata->acdb_id;
	dev_info->private_data = (void *) icodec;
	dev_info->dev_ops.open = snddev_icodec_open;
	dev_info->dev_ops.close = snddev_icodec_close;
	dev_info->dev_ops.set_freq = snddev_icodec_set_freq;
	dev_info->dev_ops.set_device_volume = snddev_icodec_set_device_volume;
	dev_info->capability = pdata->capability;
	dev_info->opened = 0;
	msm_snddev_register(dev_info);
	icodec->data = pdata;
	icodec->sample_rate = pdata->default_sample_rate;
	dev_info->sample_rate = pdata->default_sample_rate;

	if (first_time) {
		if (audio_ops->support_aic3254) {
			support_aic3254 = audio_ops->support_aic3254();
			pr_aud_info("%s: support_aic3254 = %d\n",
				__func__, support_aic3254);
		}
		first_time = 0;
	}

	if (pdata->capability & SNDDEV_CAP_RX) {
		dev_info->vol_idx = pdata->vol_idx;
		if (support_aic3254)
			dev_info->dev_ops.enable_sidetone = NULL;
		else
			dev_info->dev_ops.enable_sidetone =
				snddev_icodec_enable_sidetone;
	} else {
		dev_info->dev_ops.enable_sidetone = NULL;
	}

error:
	return rc;
}

static int snddev_icodec_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver snddev_icodec_driver = {
  .probe = snddev_icodec_probe,
  .remove = snddev_icodec_remove,
  .driver = { .name = "snddev_icodec" }
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_sdev_dent;
static struct dentry *debugfs_afelb;
static struct dentry *debugfs_adielb;
static struct adie_codec_path *debugfs_rx_adie;
static struct adie_codec_path *debugfs_tx_adie;

static int snddev_icodec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	pr_aud_info("snddev_icodec: debug intf %s\n", (char *) file->private_data);
	return 0;
}

static void debugfs_adie_loopback(u32 loop)
{
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;

	if (loop) {

		/* enable MI2S RX master block */
		/* enable MI2S RX bit clock */
		clk_set_rate(drv->rx_mclk,
			SNDDEV_ICODEC_CLK_RATE(8000));
		clk_enable(drv->rx_mclk);
		clk_enable(drv->rx_sclk);

		pr_aud_info("%s: configure ADIE RX path\n", __func__);
		/* Configure ADIE */
		adie_codec_open(&debug_rx_profile, &debugfs_rx_adie);
		adie_codec_setpath(debugfs_rx_adie, 8000, 256);
		adie_codec_proceed_stage(debugfs_rx_adie,
		ADIE_CODEC_DIGITAL_ANALOG_READY);

		pr_aud_info("%s: Enable Handset Mic bias\n", __func__);
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_PWM_TCXO);
		/* enable MI2S TX master block */
		/* enable MI2S TX bit clock */
		clk_set_rate(drv->tx_mclk,
			SNDDEV_ICODEC_CLK_RATE(8000));
		clk_enable(drv->tx_mclk);
		clk_enable(drv->tx_sclk);

		pr_aud_info("%s: configure ADIE TX path\n", __func__);
		/* Configure ADIE */
		adie_codec_open(&debug_tx_lb_profile, &debugfs_tx_adie);
		adie_codec_setpath(debugfs_tx_adie, 8000, 256);
		adie_codec_proceed_stage(debugfs_tx_adie,
		ADIE_CODEC_DIGITAL_ANALOG_READY);
	} else {
		/* Disable ADIE */
		adie_codec_proceed_stage(debugfs_rx_adie,
		ADIE_CODEC_DIGITAL_OFF);
		adie_codec_close(debugfs_rx_adie);
		adie_codec_proceed_stage(debugfs_tx_adie,
		ADIE_CODEC_DIGITAL_OFF);
		adie_codec_close(debugfs_tx_adie);

		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);

		/* Disable MI2S RX master block */
		/* Disable MI2S RX bit clock */
		clk_disable(drv->rx_sclk);
		clk_disable(drv->rx_mclk);

		/* Disable MI2S TX master block */
		/* Disable MI2S TX bit clock */
		clk_disable(drv->tx_sclk);
		clk_disable(drv->tx_mclk);
	}
}

static void debugfs_afe_loopback(u32 loop)
{
	int trc;
	struct msm_afe_config afe_config;
	struct snddev_icodec_drv_state *drv = &snddev_icodec_drv;

	if (loop) {

		/* enable MI2S RX master block */
		/* enable MI2S RX bit clock */
		trc = clk_set_rate(drv->rx_mclk,
		SNDDEV_ICODEC_CLK_RATE(8000));
		if (IS_ERR_VALUE(trc))
			pr_aud_err("%s: failed to set clk rate\n", __func__);
		clk_enable(drv->rx_mclk);
		clk_enable(drv->rx_sclk);
		clk_enable(drv->lpa_codec_clk);
		clk_enable(drv->lpa_core_clk);
		clk_enable(drv->lpa_p_clk);
		/* Set audio interconnect reg to ADSP */
		audio_interct_codec(AUDIO_INTERCT_ADSP);
		/* Set MI2S */
		mi2s_set_codec_output_path(0, WT_16_BIT);
		pr_aud_info("%s: configure ADIE RX path\n", __func__);
		/* Configure ADIE */
		adie_codec_open(&debug_rx_profile, &debugfs_rx_adie);
		adie_codec_setpath(debugfs_rx_adie, 8000, 256);
		afe_config.sample_rate = 8;
		afe_config.channel_mode = 1;
		afe_config.volume = AFE_VOLUME_UNITY;
		pr_aud_info("%s: enable afe\n", __func__);
		trc = afe_enable(AFE_HW_PATH_CODEC_RX, &afe_config);
		if (IS_ERR_VALUE(trc))
			pr_aud_err("%s: fail to enable afe rx\n", __func__);
		adie_codec_proceed_stage(debugfs_rx_adie,
		ADIE_CODEC_DIGITAL_ANALOG_READY);

		pr_aud_info("%s: Enable Handset Mic bias\n", __func__);
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_PWM_TCXO);
		/* enable MI2S TX master block */
		/* enable MI2S TX bit clock */
		clk_set_rate(drv->tx_mclk,
			SNDDEV_ICODEC_CLK_RATE(8000));
		clk_enable(drv->tx_mclk);
		clk_enable(drv->tx_sclk);
		/* Set MI2S */
		mi2s_set_codec_input_path(0, WT_16_BIT);
		pr_aud_info("%s: configure ADIE TX path\n", __func__);
		/* Configure ADIE */
		adie_codec_open(&debug_tx_profile, &debugfs_tx_adie);
		adie_codec_setpath(debugfs_tx_adie, 8000, 256);
		adie_codec_proceed_stage(debugfs_tx_adie,
		ADIE_CODEC_DIGITAL_ANALOG_READY);
		/* Start AFE */
		afe_config.sample_rate = 0x8;
		afe_config.channel_mode = 1;
		afe_config.volume = AFE_VOLUME_UNITY;
		trc = afe_enable(AFE_HW_PATH_CODEC_TX, &afe_config);
		if (IS_ERR_VALUE(trc))
			pr_aud_err("%s: failed to enable AFE TX\n", __func__);
	} else {
		/* Disable ADIE */
		adie_codec_proceed_stage(debugfs_rx_adie,
		ADIE_CODEC_DIGITAL_OFF);
		adie_codec_close(debugfs_rx_adie);
		adie_codec_proceed_stage(debugfs_tx_adie,
		ADIE_CODEC_DIGITAL_OFF);
		adie_codec_close(debugfs_tx_adie);

		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);

		/* Disable MI2S RX master block */
		/* Disable MI2S RX bit clock */
		clk_disable(drv->rx_sclk);
		clk_disable(drv->rx_mclk);

		/* Disable MI2S TX master block */
		/* Disable MI2S TX bit clock */
		clk_disable(drv->tx_sclk);
		clk_disable(drv->tx_mclk);
	}
}

static ssize_t snddev_icodec_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *lb_str = filp->private_data;
	char cmd;

	if (get_user(cmd, ubuf))
		return -EFAULT;

	pr_aud_info("%s: %s %c\n", __func__, lb_str, cmd);

	if (!strcmp(lb_str, "adie_loopback")) {
		switch (cmd) {
		case '1':
			debugfs_adie_loopback(1);
			break;
		case '0':
			debugfs_adie_loopback(0);
			break;
		}
	} else if (!strcmp(lb_str, "afe_loopback")) {
		switch (cmd) {
		case '1':
			debugfs_afe_loopback(1);
			break;
		case '0':
			debugfs_afe_loopback(0);
			break;
		}
	}

	return cnt;
}

static const struct file_operations snddev_icodec_debug_fops = {
	.open = snddev_icodec_debug_open,
	.write = snddev_icodec_debug_write
};
#endif

int update_aic3254_info(struct aic3254_info *info)
{
	/* do not support this function at 1x codebase temporarily */
	return 0;
}

static int __init snddev_icodec_init(void)
{
	s32 rc;
	struct snddev_icodec_drv_state *icodec_drv = &snddev_icodec_drv;

	rc = platform_driver_register(&snddev_icodec_driver);
	if (IS_ERR_VALUE(rc))
		goto error_platform_driver;
	icodec_drv->rx_mclk = clk_get(NULL, "mi2s_codec_rx_m_clk");
	if (IS_ERR(icodec_drv->rx_mclk))
		goto error_rx_mclk;
	icodec_drv->rx_sclk = clk_get(NULL, "mi2s_codec_rx_s_clk");
	if (IS_ERR(icodec_drv->rx_sclk))
		goto error_rx_sclk;
	icodec_drv->tx_mclk = clk_get(NULL, "mi2s_codec_tx_m_clk");
	if (IS_ERR(icodec_drv->tx_mclk))
		goto error_tx_mclk;
	icodec_drv->tx_sclk = clk_get(NULL, "mi2s_codec_tx_s_clk");
	if (IS_ERR(icodec_drv->tx_sclk))
		goto error_tx_sclk;
	icodec_drv->lpa_codec_clk = clk_get(NULL, "lpa_codec_clk");
	if (IS_ERR(icodec_drv->lpa_codec_clk))
		goto error_lpa_codec_clk;
	icodec_drv->lpa_core_clk = clk_get(NULL, "lpa_core_clk");
	if (IS_ERR(icodec_drv->lpa_core_clk))
		goto error_lpa_core_clk;
	icodec_drv->lpa_p_clk = clk_get(NULL, "lpa_pclk");
	if (IS_ERR(icodec_drv->lpa_p_clk))
		goto error_lpa_p_clk;

#ifdef CONFIG_DEBUG_FS
	debugfs_sdev_dent = debugfs_create_dir("snddev_icodec", 0);
	if (!IS_ERR(debugfs_sdev_dent)) {
		debugfs_afelb = debugfs_create_file("afe_loopback",
		S_IFREG | S_IRUGO, debugfs_sdev_dent,
		(void *) "afe_loopback", &snddev_icodec_debug_fops);
		debugfs_adielb = debugfs_create_file("adie_loopback",
		S_IFREG | S_IRUGO, debugfs_sdev_dent,
		(void *) "adie_loopback", &snddev_icodec_debug_fops);
	}
#endif
	mutex_init(&icodec_drv->rx_lock);
	mutex_init(&icodec_drv->tx_lock);
	icodec_drv->rx_active = 0;
	icodec_drv->tx_active = 0;
	icodec_drv->lpa = NULL;
	wake_lock_init(&icodec_drv->tx_idlelock, WAKE_LOCK_IDLE,
			"snddev_tx_idle");
	wake_lock_init(&icodec_drv->rx_idlelock, WAKE_LOCK_IDLE,
			"snddev_rx_idle");
	return 0;

error_lpa_p_clk:
	clk_put(icodec_drv->lpa_core_clk);
error_lpa_core_clk:
	clk_put(icodec_drv->lpa_codec_clk);
error_lpa_codec_clk:
	clk_put(icodec_drv->tx_sclk);
error_tx_sclk:
	clk_put(icodec_drv->tx_mclk);
error_tx_mclk:
	clk_put(icodec_drv->rx_sclk);
error_rx_sclk:
	clk_put(icodec_drv->rx_mclk);
error_rx_mclk:
	platform_driver_unregister(&snddev_icodec_driver);
error_platform_driver:

	pr_aud_err("%s: encounter error\n", __func__);
	return -ENODEV;
}

static void __exit snddev_icodec_exit(void)
{
	struct snddev_icodec_drv_state *icodec_drv = &snddev_icodec_drv;

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(debugfs_afelb);
	debugfs_remove(debugfs_adielb);
	debugfs_remove(debugfs_sdev_dent);
#endif
	platform_driver_unregister(&snddev_icodec_driver);

	clk_put(icodec_drv->rx_sclk);
	clk_put(icodec_drv->rx_mclk);
	clk_put(icodec_drv->tx_sclk);
	clk_put(icodec_drv->tx_mclk);
	return;
}

module_init(snddev_icodec_init);
module_exit(snddev_icodec_exit);

MODULE_DESCRIPTION("ICodec Sound Device driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("Dual BSD/GPL");
