/* linux/arch/arm/mach-msm/board-glacier-audio.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include "board-glacier.h"
#include <mach/qdsp5v2_1x/snddev_icodec.h>
#include <mach/qdsp5v2_1x/snddev_ecodec.h>
#include <mach/qdsp5v2_1x/audio_def.h>
#include <mach/qdsp5v2_1x/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <linux/delay.h>

static struct mutex bt_sco_lock;

static struct q5v2_hw_info_percentage q5v2_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-1600, -1100, -600, -200, 200, 400, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-1600, -1100, -600, -200, 200, 400, 0, 0, 0, 0},
	},
	[Q5V2_HW_HEADSET] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-1100, -700, -300, 100, 500, 900, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-1100, -700, -300, 100, 500, 900, 0, 0, 0, 0},
	},
	[Q5V2_HW_SPEAKER] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-400, -100, 200, 500, 800, 1100, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-400, -100, 200, 500, 800, 1100, 0, 0, 0, 0},
	},
	[Q5V2_HW_BT_SCO] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-1500, -1200, -900, -600, -300, 0, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-1500, -1200, -900, -600, -300, 0, 0, 0, 0, 0},
	},
	[Q5V2_HW_TTY] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	},
	[Q5V2_HW_HS_SPKR] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-2000, -1700, -1400, -1100, -800, -500, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-2000, -1700, -1400, -1100, -800, -500, 0, 0, 0, 0},
	},
	[Q5V2_HW_USB_HS] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-500, -200, 100, 400, 700, 1000, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-500, -200, 100, 400, 700, 1000, 0, 0, 0, 0},
	},
	[Q5V2_HW_HAC] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-500, -200, 100, 400, 700, 1000, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-500, -200, 100, 400, 700, 1000, 0, 0, 0, 0},
	},
};

static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_OUT, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_IN, 1, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_SYNC, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_CLK, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_OUT, 0, GPIO_OUTPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_IN, 0, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_SYNC, 0, GPIO_OUTPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(GLACIER_GPIO_BT_PCM_CLK, 0, GPIO_OUTPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
};

void glacier_snddev_poweramp_on(int en)
{
	if (en) 
		mdelay(30);
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(GLACIER_AUD_SPK_ENO), en);
}

void glacier_snddev_hsed_pamp_on(int en)
{
	struct vreg *vreg_ncp;
	int ret;
	vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(vreg_ncp)) {
		pr_aud_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(vreg_ncp));
		return;
	}

	if (en) {
		mdelay(60);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(GLACIER_AUD_HP_EN), 1);
		ret = vreg_enable(vreg_ncp);
		if (ret)
			pr_aud_err("%s: vreg_enable failed (%d)\n", __func__, ret);
	} else {
		ret = vreg_disable(vreg_ncp);
		if (ret)
			pr_aud_err("%s: vreg_disable failed (%d)\n", __func__, ret);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(GLACIER_AUD_HP_EN), 0);
	}
}

void glacier_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
}

void glacier_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_aud_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1)
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(GLACIER_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(GLACIER_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(GLACIER_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

void glacier_snddev_hs_spk_pamp_on(int en)
{
	glacier_snddev_poweramp_on(en);
	glacier_snddev_hsed_pamp_on(en);
}

void glacier_snddev_imic_pamp_on(int en)
{
	unsigned int engineerID = glacier_get_engineerid();
	pr_aud_info("%s: %d\n", __func__, en);
	if (en)
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
	else
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);

	/* enable/disable back mic */
	if ((engineerID & 0x4) == 0) {
		if (en)
			pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_ALWAYS);
		else
			pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_OFF);
	}
}

int glacier_get_rx_vol(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info_percentage *info;
	int vol;

	info = &q5v2_audio_hw[hw];

	level = (level > 100)? 100 : ((level < 0) ? 0 : level);
	vol = info->gain[network][(uint32_t)((info->max_step - 1) * level / 100)];

	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}

void glacier_mic_bias_enable(int en, int shift)
{
	pr_aud_info("%s: %d\n", __func__, en);

	if (en)
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_ALWAYS);
	else
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_OFF);
}

int glacier_support_audience(void)
{
	unsigned int engineerID = glacier_get_engineerid();
	pr_aud_info("%s: engineerid: %x", __func__, engineerID);
	/*Bit2:
	0: with audience.
	1: without audience*/
	return engineerID & 0x4 ? 0 : 1;
}

int glacier_support_back_mic(void)
{
	return glacier_support_audience();
}

void glacier_mic_disable(int mic)
{
	switch (mic) {
	case 0: /* main mic */
		pr_aud_info("%s: disable main mic\n", __func__);
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
		break;
	case 1: /* back mic */
		pr_aud_info("%s: disable back mic\n", __func__);
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_OFF);
		break;
	default:
		break;
	}
}

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= glacier_snddev_poweramp_on,
	.headset_enable	= glacier_snddev_hsed_pamp_on,
	.handset_enable	= glacier_snddev_receiver_pamp_on,
	.bt_sco_enable	= glacier_snddev_bt_sco_pamp_on,
	.headset_speaker_enable	= glacier_snddev_hs_spk_pamp_on,
	.bt_sco_enable = glacier_snddev_bt_sco_pamp_on,
	.int_mic_enable = glacier_snddev_imic_pamp_on,
	.fm_headset_enable = glacier_snddev_hsed_pamp_on,
	.fm_speaker_enable = glacier_snddev_poweramp_on,
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = glacier_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = glacier_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = glacier_mic_bias_enable,
	.support_audience = glacier_support_audience,
	.support_back_mic = glacier_support_back_mic,
	.mic_disable = glacier_mic_disable,
	.mute_headset_amp = glacier_snddev_hsed_pamp_on,
};

void __init glacier_audio_init(void)
{
	static struct pm8058_gpio audio_pwr = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.vin_sel        = 6,
	};

	mutex_init(&bt_sco_lock);
#ifdef CONFIG_MSM7KV2_1X_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	acoustic_register_ops(&acoustic);
#endif
	pm8058_gpio_config(GLACIER_AUD_SPK_ENO, &audio_pwr);
	pm8058_gpio_config(GLACIER_AUD_HP_EN, &audio_pwr);

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(GLACIER_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(GLACIER_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(GLACIER_GPIO_BT_PCM_CLK, 0);
	mutex_unlock(&bt_sco_lock);
}

