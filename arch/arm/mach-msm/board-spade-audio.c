/* linux/arch/arm/mach-msm/board-spade-audio.c
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

#include <mach/gpio.h>
#include <mach/dal.h>
#include "board-spade.h"
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/qdsp5v2/audio_def.h>
#include <mach/qdsp5v2/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <linux/spi/spi_aic3254.h>

static struct mutex bt_sco_lock;
static struct mutex mic_lock;
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)

static struct q5v2_hw_info q5v2_audio_hw[Q5V2_HW_COUNT] = {
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

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_OUT, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_IN, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_SYNC, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_CLK, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
};

static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_OUT, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_IN, 1, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_SYNC, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SPADE_GPIO_BT_PCM_CLK, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

void spade_snddev_poweramp_on(int en)
{
	static int first_time = 1;
	pr_info("%s %d\n", __func__, en);
	if (en) {
		if (first_time) {
			msleep(70);
			first_time = 0;
		} else
			msleep(30);

		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_SPK_ENO), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_SPK_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
	}
}

void spade_snddev_hsed_pamp_on(int en)
{
	pr_info("%s %d\n", __func__, en);
	if (en) {
		msleep(60);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void spade_snddev_hs_spk_pamp_on(int en)
{
	spade_snddev_poweramp_on(en);
	spade_snddev_hsed_pamp_on(en);
}

void spade_snddev_receiver_pamp_on(int en)
{
	pr_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_EP_EN), 1);
		mdelay(5);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_EP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
}

void spade_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1) {
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
		}
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(SPADE_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(SPADE_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(SPADE_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

/* power up internal/externnal mic shared GPIO */
void spade_mic_enable(int en, int shift)
{
	static int flag = 0;
	pr_info("%s: %d, shift %d\n", __func__, en, shift);

	mutex_lock(&mic_lock);
	if (en)
		flag |= 1 << shift;
	else
		flag &= ~(1 << shift);

	if (flag)
		gpio_set_value(SPADE_AUD_MIC_BIAS, 1);
	else
		gpio_set_value(SPADE_AUD_MIC_BIAS, 0);

	mutex_unlock(&mic_lock);
}

void spade_snddev_imic_pamp_on(int en)
{
	pr_info("%s %d\n", __func__, en);
	spade_mic_enable(en, 0);
}

void spade_snddev_emic_pamp_on(int en)
{
	pr_info("%s %d\n", __func__, en);
}

void spade_snddev_fmspk_pamp_on(int en)
{
	if (en) {
		msleep(30);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_SPK_ENO), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_SPK;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_SPK_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_SPK;
	}
}

void spade_snddev_fmhs_pamp_on(int en)
{
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_HS;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SPADE_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_HS;
	}
}

int spade_get_rx_vol(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info *info;
	int vol, maxv, minv;

	info = &q5v2_audio_hw[hw];
	maxv = info->max_gain[network];
	minv = info->min_gain[network];
	vol = minv + ((maxv - minv) * level) / 100;
	pr_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}

void spade_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			spade_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			spade_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			spade_snddev_receiver_pamp_on(en);
		if (curr_rx_mode & BIT_FM_SPK)
			spade_snddev_fmspk_pamp_on(en);
		if (curr_rx_mode & BIT_FM_HS)
			spade_snddev_fmhs_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}

int spade_support_aic3254(void)
{
	return 1;
}

int spade_support_back_mic(void)
{
	if (system_rev < 4)
		/* the stage before XE board */
		return 1;
	else
		return 0;
}

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= spade_snddev_poweramp_on,
	.headset_enable	= spade_snddev_hsed_pamp_on,
	.handset_enable	= spade_snddev_receiver_pamp_on,
	.headset_speaker_enable	= spade_snddev_hs_spk_pamp_on,
	.bt_sco_enable	= spade_snddev_bt_sco_pamp_on,
	.int_mic_enable = spade_snddev_imic_pamp_on,
	.ext_mic_enable = spade_snddev_emic_pamp_on,
	.fm_headset_enable = spade_snddev_fmhs_pamp_on,
	.fm_speaker_enable = spade_snddev_fmspk_pamp_on,
};

static struct q5v2audio_icodec_ops iops = {
	.support_aic3254 = spade_support_aic3254,
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = spade_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = spade_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = spade_mic_enable,
	.support_aic3254 = spade_support_aic3254,
	.support_back_mic = spade_support_back_mic,
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable = spade_rx_amp_enable,
	.panel_sleep_in = spade_panel_sleep_in
};

void __init spade_audio_init(void)
{
	struct pm8058_gpio audio_pwr = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.vin_sel        = 6,
	};

	mutex_init(&bt_sco_lock);
	mutex_init(&mic_lock);

#ifdef CONFIG_MSM7KV2_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_icodec_ops(&iops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	acoustic_register_ops(&acoustic);
#endif
	aic3254_register_ctl_ops(&cops);

	/* Init PMIC GPIO */
	pm8058_gpio_config(SPADE_AUD_EP_EN, &audio_pwr);
	pm8058_gpio_config(SPADE_AUD_SPK_ENO, &audio_pwr);
	pm8058_gpio_config(SPADE_AUD_HP_EN, &audio_pwr);

	/* Rest AIC3254 */
	gpio_set_value(SPADE_AUD_CODEC_RST, 0);
	mdelay(1);
	gpio_set_value(SPADE_AUD_CODEC_RST, 1);

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(SPADE_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(SPADE_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(SPADE_GPIO_BT_PCM_CLK, 0);
	mutex_unlock(&bt_sco_lock);
}
