/* linux/arch/arm/mach-msm/board-speedy-audio.c
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
#include <linux/delay.h>
#include <mach/tpa2051d3.h>
#include <mach/gpio.h>
#include <mach/dal.h>
#include "board-speedy.h"
#include <mach/qdsp5v2_1x/snddev_icodec.h>
#include <mach/qdsp5v2_1x/snddev_ecodec.h>
#include <mach/qdsp5v2_1x/audio_def.h>
#include <mach/qdsp5v2_1x/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <mach/htc_acdb_7x30.h>

static struct mutex bt_sco_lock;
static struct mutex pm_lock;
static struct mutex vreg_lock;

#define SPEEDY_ACDB_SMEM_SIZE        (0xE000)

static struct q5v2_hw_info_percentage q5v2_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-1600, -1200, -800, -400, 0, 400, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-1600, -1200, -800, -400, 0, 400, 0, 0, 0, 0},
	},
	[Q5V2_HW_HEADSET] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-600, -300, 0, 300, 600, 900, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-600, -300, 0, 300, 600, 900, 0, 0, 0, 0},
	},
	[Q5V2_HW_SPEAKER] = {
		.max_step = 6,
		.gain[VOC_NB_INDEX] =
			{-1200, -800, -400, 0, 400, 800, 0, 0, 0, 0},
		.gain[VOC_WB_INDEX] =
			{-1200, -800, -400, 0, 400, 800, 0, 0, 0, 0}
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

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_OUT, 0, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_IN, 0, GPIO_INPUT,
			GPIO_PULL_UP, GPIO_2MA),
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_SYNC, 0, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_CLK, 0, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_OUT, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_IN, 1, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_SYNC, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPEEDY_GPIO_BT_PCM_CLK, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

void pm8058_spkr_gpio_cfg(int en) {
	static int pm8058_spkr_refcount;

	mutex_lock(&pm_lock);
	if (en) {
		if (++pm8058_spkr_refcount == 1) {
			gpio_set_value(
				PM8058_GPIO_PM_TO_SYS(SPEEDY_AUD_SPK_ENO), 1);
		}
	} else {
		if (--pm8058_spkr_refcount == 0) {
			gpio_set_value(
				PM8058_GPIO_PM_TO_SYS(SPEEDY_AUD_SPK_ENO), 0);
		}
	}
	mutex_unlock(&pm_lock);
}

int vreg_ncp_enable(int en) {
	static int vreg_ncp_refcount;
	struct vreg *vreg_ncp;
	int ret = 0;

	mutex_lock(&vreg_lock);
	vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(vreg_ncp)) {
		pr_aud_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(vreg_ncp));
		mutex_unlock(&vreg_lock);
		return -1;
	}

	if (en) {
		if (++vreg_ncp_refcount == 1)
			ret = vreg_enable(vreg_ncp);
	} else {
		if (--vreg_ncp_refcount == 0)
			ret = vreg_disable(vreg_ncp);
	}
	mutex_unlock(&vreg_lock);
	pr_info("%s(%d), ret %d\n", __func__, en, ret);
	return ret;
}

void speedy_snddev_poweramp_on(int en)
{
	pr_info("%s %d\n", __func__, en);
	if (en) {
		pm8058_spkr_gpio_cfg(1);
		mdelay(30);
		set_speaker_amp(1);
	} else {
		set_speaker_amp(0);
		pm8058_spkr_gpio_cfg(0);
	}
}

void speedy_snddev_hsed_pamp_on(int en)
{
	pr_info("%s %d\n", __func__, en);
	if (en) {
		vreg_ncp_enable(1);
		pm8058_spkr_gpio_cfg(1);
		mdelay(30);
		set_headset_amp(1);
	} else {
		set_headset_amp(0);
		pm8058_spkr_gpio_cfg(0);
		vreg_ncp_enable(0);
		}
}

void speedy_snddev_hs_spk_pamp_on(int en)
{
	pr_info("%s %d\n", __func__, en);

	if (en) {
		vreg_ncp_enable(1);
		pm8058_spkr_gpio_cfg(1);
		mdelay(30);
		set_speaker_headset_amp(1);
	} else {
		set_speaker_headset_amp(0);
		pm8058_spkr_gpio_cfg(0);
		vreg_ncp_enable(0);
	}
}

void speedy_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1)
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(SPEEDY_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(SPEEDY_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(SPEEDY_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

void speedy_snddev_receiver_pamp_on(int en)
{
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPEEDY_AUD_HANDSET_ENO), en);
}

void speedy_snddev_imic_pamp_on(int en)
{
	pr_info("%s: %d\n", __func__, en);

	if (en)
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
	else
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
}

int speedy_get_rx_vol(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info_percentage *info;
	int vol;

	info = &q5v2_audio_hw[hw];

	level = (level > 100)? 100 : ((level < 0) ? 0 : level);
	vol = info->gain[network][(uint32_t)((info->max_step - 1) * level / 100)];

	pr_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}

void speedy_mic_bias_enable(int en, int shift)
{
	pr_info("%s: %d\n", __func__, en);

	if (en)
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_ALWAYS);
	else
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_OFF);
}

uint32_t speedy_get_smem_size(void)
{
	return SPEEDY_ACDB_SMEM_SIZE;
}

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= speedy_snddev_poweramp_on,
	.headset_enable	= speedy_snddev_hsed_pamp_on,
	.handset_enable	= speedy_snddev_receiver_pamp_on,
	.bt_sco_enable	= speedy_snddev_bt_sco_pamp_on,
	.headset_speaker_enable	= speedy_snddev_hs_spk_pamp_on,
	.int_mic_enable = speedy_snddev_imic_pamp_on,
	.fm_headset_enable = speedy_snddev_hsed_pamp_on,
	.fm_speaker_enable = speedy_snddev_poweramp_on,
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = speedy_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = speedy_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = speedy_mic_bias_enable,
};

static struct acdb_ops acdb = {
	.get_smem_size = speedy_get_smem_size,
};

void __init speedy_audio_init(void)
{
	struct pm8058_gpio audio_pwr = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 6,	/* LDO5 2.85 V */
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	struct pm8058_gpio tpa2051_pwr = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 6,      /* S3 1.8 V */
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	mutex_init(&bt_sco_lock);
	mutex_init(&pm_lock);
	mutex_init(&vreg_lock);
#ifdef CONFIG_MSM7KV2_1X_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	acoustic_register_ops(&acoustic);
	acdb_register_ops(&acdb);
#endif
	pm8058_gpio_config(SPEEDY_AUD_HANDSET_ENO, &audio_pwr);
	pm8058_gpio_config(SPEEDY_AUD_SPK_ENO, &tpa2051_pwr);

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(SPEEDY_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(SPEEDY_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(SPEEDY_GPIO_BT_PCM_CLK, 0);
	mutex_unlock(&bt_sco_lock);
}


